// This weird pragma is required for the compiler to properly include the necessary namespaces.
#pragma comment(lib, "windowsapp")

#include "PeripheralWindows.h"
#include "CommonUtils.h"
#include "Utils.h"
#include "MtaManager.h"

#include "../common/CharacteristicBase.h"
#include "../common/DescriptorBase.h"
#include "../common/ServiceBase.h"
#include "simpleble/Characteristic.h"
#include "simpleble/Descriptor.h"
#include "simpleble/Service.h"

#include <simpleble/Exceptions.h>
#include <simpleble/Config.h>

#include "winrt/Windows.Foundation.Collections.h"
#include "winrt/Windows.Foundation.h"
#include "winrt/Windows.Storage.Streams.h"
#include "winrt/base.h"

#include <chrono>
#include <iostream>
#include <thread>

using namespace SimpleBLE;
using namespace SimpleBLE::WinRT;
using namespace std::chrono_literals;

PeripheralWindows::PeripheralWindows(BluetoothLEDevice device) {
    device_ = device;
    identifier_ = winrt::to_string(device.Name());
    address_ = _mac_address_to_str(device.BluetoothAddress());
    address_type_ = BluetoothAddressType::PUBLIC;

    // NOTE: We're assuming that the device is connectable, as this constructor is only called
    // when the device has paired in the past.
    connectable_ = true;
}

PeripheralWindows::PeripheralWindows(advertising_data_t advertising_data) {
    address_type_ = advertising_data.address_type;
    identifier_ = advertising_data.identifier;
    address_ = advertising_data.mac_address;
    rssi_ = advertising_data.rssi;
    tx_power_ = advertising_data.tx_power;
    manufacturer_data_ = advertising_data.manufacturer_data;
    service_data_ = advertising_data.service_data;
    connectable_ = advertising_data.connectable;
}

PeripheralWindows::~PeripheralWindows() {
    if (device_ != nullptr) {
        MtaManager::get().execute_sync([this]() {
            _teardown_connection_parameters_watch_locked_mta();
            if (connection_status_changed_token_) {
                device_.ConnectionStatusChanged(connection_status_changed_token_);
            }
        });
    }
}

void* PeripheralWindows::underlying() const {
    return reinterpret_cast<void*>(const_cast<BluetoothLEDevice*>(&device_));
}

SimpleBLE::BluetoothAddressType PeripheralWindows::address_type() { return address_type_; }

std::string PeripheralWindows::identifier() { return identifier_; }

BluetoothAddress PeripheralWindows::address() { return address_; }

int16_t PeripheralWindows::rssi() { return rssi_; }

int16_t PeripheralWindows::tx_power() { return tx_power_; }

uint16_t PeripheralWindows::mtu() {
    if (!is_connected()) return 0;

    // The value provided by the MaxPduSize includes an extra 3 bytes from the GATT header
    // which needs to be removed.
    return mtu_ - 3;
}

void PeripheralWindows::update_advertising_data(advertising_data_t advertising_data) {
    if (advertising_data.identifier != "") {
        identifier_ = advertising_data.identifier;
    }
    rssi_ = advertising_data.rssi;
    tx_power_ = advertising_data.tx_power;
    address_type_ = advertising_data.address_type;
    manufacturer_data_ = advertising_data.manufacturer_data;

    advertising_data.service_data.merge(service_data_);
    service_data_ = advertising_data.service_data;
}

bool PeripheralWindows::is_disconnect_pending() const noexcept {
    return SimpleBLE::Config::WinRT::use_deferred_disconnect &&
           connection_state_ == ConnectionState::Disconnecting;
}

void PeripheralWindows::connect() {
    constexpr uint32_t kConnectOperationTimeoutMs = 3000;
    if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
        if (connection_state_ == ConnectionState::Disconnecting) {
            throw SimpleBLE::Exception::OperationFailed("Device is still disconnecting");
        }
        connection_state_ = ConnectionState::Connecting;
    }

    MtaManager::get().execute_sync([this, kConnectOperationTimeoutMs]() {
        // get_paired_peripherals() already gives us a BluetoothLEDevice tied
        // to the owning radio. Keep that handle: resolving the same address
        // again can fail while Windows' advertisement watcher is recovering,
        // which made reconnect impossible even though the OS pairing record
        // was valid. Scanned peripherals do not have a device handle yet and
        // still need the address-based lookup.
        const bool reused_paired_handle = device_ != nullptr;
        if (!reused_paired_handle) {
            device_ = async_get_for(
                BluetoothLEDevice::FromBluetoothAddressAsync(_str_to_mac_address(address_)),
                kConnectOperationTimeoutMs);
        }

        // PRE-CONNECT: request ThroughputOptimized BEFORE GATT discovery
        // initiates the connection. The preference is stored by Windows
        // against this device handle and is used when the initial connection-
        // parameter negotiation occurs with the peripheral — as opposed to
        // requesting afterwards, which only triggers a *re-*negotiation that
        // the stack is free to ignore.
        //
        // NOTE: we intentionally do NOT pin a GattSession (via
        // GattSession::FromDeviceIdAsync + MaintainConnection(true)) here.
        // Pinning the session pre-connect caused GATT service discovery to
        // fail with status=Unreachable on second-launch reconnects: Windows
        // returned a session that was still in "MaintainConnection" mode from
        // the previous (now dead) process, and the underlying state machine
        // was stuck. service.Session().MaintainConnection(true) runs
        // post-discovery inside _attempt_connect() for each service, which is
        // both sufficient for the multi-device rate fix and safe on reconnect.
        // A DeviceInformation-derived paired handle can exist while the radio
        // is still restoring that device after wake. Requesting preferred
        // parameters on that dormant handle has caused Windows to leave the
        // following uncached GATT request pending. Let GATT establish the link
        // first; the post-connect path below applies the same preference.
        if (!reused_paired_handle) {
            try {
                preferred_connection_params_request_ = device_.RequestPreferredConnectionParameters(
                    BluetoothLEPreferredConnectionParameters::ThroughputOptimized());
                SIMPLEBLE_LOG_INFO(
                    fmt::format("[{}] pre-connect ThroughputOptimized request issued", address_));
            } catch (...) {
                SIMPLEBLE_LOG_WARN(
                    fmt::format("[{}] pre-connect ThroughputOptimized request THREW (not supported)",
                                address_));
            }
        }
    });

    // Attempt GATT service discovery via Uncached mode, which is the only mode
    // that actually forces a live read from the peripheral and therefore the
    // only one whose success guarantees a usable connection.
    //
    // Cached mode was previously used as a fallback, but it gives a FALSE
    // success on reconnect: Windows returns the services from its local cache
    // (populated at pairing time) WITHOUT requiring the link to be live, so
    // _attempt_connect() returns true with a populated gatt_map_, the connect
    // callback fires and the first write may even succeed briefly — but the
    // connection is tentative, the CCCD write needed to enable notifications
    // then fails ("notify subscribe failed"), and the device disconnects soon
    // after. Relying on Uncached + retries keeps the success signal honest.
    //
    // Between retries we do a handle-reset (close + re-open the
    // BluetoothLEDevice, drop the pinned session / preferred-params request,
    // and re-establish them) once — this clears stale state left behind by a
    // previous app process that didn't disconnect cleanly, which is the
    // dominant cause of the second-launch failure on certain BLE dongles.
    bool discovery_ok = false;
    constexpr int kMaxRetries = 3;
    // Each uncached query has a three-second bound. Together with these short
    // backoffs, an offline device cannot monopolize reconnect for a minute.
    constexpr int kBackoffMs[kMaxRetries - 1] = {150, 300};
    for (int i = 0; i < kMaxRetries; i++) {
        if (_attempt_connect(false /* use_cached */, kConnectOperationTimeoutMs)) {
            discovery_ok = true;
            break;
        }
        if (i + 1 >= kMaxRetries) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(kBackoffMs[i]));

        // After 2 failures do a single handle-reset: close the current
        // BluetoothLEDevice and drop any preferred-params request, then
        // re-acquire them against a fresh handle. This clears stale state
        // that a previous (non-graceful) app exit may have left in Windows.
        if (i == 1) {
            SIMPLEBLE_LOG_WARN(
                fmt::format("[{}] GATT discovery failing; resetting device handle",
                            address_));
            try {
                MtaManager::get().execute_sync([this, kConnectOperationTimeoutMs]() {
                    _teardown_connection_parameters_watch_locked_mta();
                    if (device_ != nullptr) {
                        try { device_.Close(); } catch (...) {}
                        device_ = nullptr;
                    }
                    device_ = async_get_for(
                        BluetoothLEDevice::FromBluetoothAddressAsync(_str_to_mac_address(address_)),
                        kConnectOperationTimeoutMs);
                    if (device_ == nullptr) return;

                    // Re-issue ThroughputOptimized preference against the fresh
                    // handle. (No pre-connect GattSession pinning — see the
                    // comment at the top of connect() for why.)
                    try {
                        preferred_connection_params_request_ = device_.RequestPreferredConnectionParameters(
                            BluetoothLEPreferredConnectionParameters::ThroughputOptimized());
                    } catch (...) {}
                });
            } catch (...) {}
            // Extra settle time after the handle reset.
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
    }

    if (!discovery_ok) {
        SIMPLEBLE_LOG_WARN(
            fmt::format("[{}] GATT service discovery failed after all retries", address_));
    }

    if (discovery_ok && is_connected()) {
        MtaManager::get().execute_sync([this]() {
            connection_status_changed_token_ = device_.ConnectionStatusChanged(
                [this](const BluetoothLEDevice device, const auto args) {
                    if (device.ConnectionStatus() == BluetoothConnectionStatus::Disconnected) {
                        if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
                            connection_state_ = ConnectionState::Disconnected;

                            // Tear down the connection-parameters watch before clearing
                            // the services, so no more callbacks can touch us.
                            _teardown_connection_parameters_watch_locked_mta();

                            // Explicitly clean up WinRT service objects and clear the map
                            // on a spontaneous disconnect to prevent stale sessions leaking.
                            for (auto& [uuid, svc] : gatt_map_) {
                                if (svc.obj) svc.obj.Close();
                            }
                            gatt_map_.clear();

                            device_ = nullptr;
                        }
                        this->disconnection_cv_.notify_all();

                        SAFE_CALLBACK_CALL(this->callback_on_disconnected_);
                    }
                });

            // Subscribe to ConnectionParametersChanged so we can push back if
            // Windows extends our interval on a new-device connection. The
            // handler checks the observed interval and has a cooldown; blindly
            // issuing a new request for every event creates a request/event
            // feedback loop on some Windows BLE drivers.
            // (Available on Windows 10 version 2004 / 10.0.19041.0+.)
            try {
                connection_params_changed_token_ = device_.ConnectionParametersChanged(
                    [this](const BluetoothLEDevice& /*device*/, const auto& /*args*/) {
                        // Already on an MTA thread-pool thread → call the locked variant directly.
                        // But we may deadlock if we ran through execute_sync from the same MTA
                        // thread that raised the event. Using the non-locked trampoline is safer.
                        try {
                            _reapply_preferred_connection_parameters_locked_mta();
                        } catch (...) {}
                    });
            } catch (...) {
                // API may not exist on this Windows build; not fatal.
            }

            // One post-discovery check covers paired handles, for which the
            // pre-connect request is intentionally skipped.
            _reapply_preferred_connection_parameters_locked_mta();
        });

        if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
            connection_state_ = ConnectionState::Connected;
        }
        SAFE_CALLBACK_CALL(this->callback_on_connected_);
    } else {
        if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
            connection_state_ = ConnectionState::Disconnected;
        }
        // Release any preconnect state we pinned so the next retry starts clean
        // (pinned GattSession + ThroughputOptimized request). If we skip this,
        // Windows may hold the previous session open and the next FromDeviceIdAsync
        // returns a handle bound to a half-connected state.
        try {
            MtaManager::get().execute_sync([this]() {
                _teardown_connection_parameters_watch_locked_mta();
                if (device_ != nullptr) {
                    try { device_.Close(); } catch (...) {}
                }
                device_ = nullptr;
            });
        } catch (...) {}

        const std::string msg = discovery_ok
            ? "Failed to connect to device."
            : "GATT service discovery failed (device may be reachable but services could not be enumerated).";
        throw SimpleBLE::Exception::OperationFailed(msg);
    }
}

void PeripheralWindows::disconnect() {
    if (device_ == nullptr) {
        if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
            connection_state_ = ConnectionState::Disconnected;
        }
        return;
    }

    if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
        // Deferred path
        if (connection_state_ == ConnectionState::Disconnecting ||
            connection_state_ == ConnectionState::Disconnected) {
            return;
        }

        connection_state_ = ConnectionState::Disconnecting;

        // Explicitly close services (extra safety against stale WinRT objects)
        for (auto& [uuid, svc] : gatt_map_) {
            if (svc.obj) svc.obj.Close();
        }
        gatt_map_.clear();

        MtaManager::get().execute_sync([this]() {
            _teardown_connection_parameters_watch_locked_mta();
            device_.Close();   // fire-and-forget – the 3 s delay still happens in background
        });
    } else {
        // Blocking path
        gatt_map_.clear();
        MtaManager::get().execute_sync([this]() {
            _teardown_connection_parameters_watch_locked_mta();
            device_.Close();
        });

        std::unique_lock<std::mutex> lock(disconnection_mutex_);
        if (disconnection_cv_.wait_for(lock, 10s, [this] { return !this->is_connected(); })) {
            // Disconnection successful
        } else {
            SIMPLEBLE_LOG_ERROR("Disconnection failed");
            throw SimpleBLE::Exception::OperationFailed("Disconnection attempt was not acknowledged.");
        }

        device_ = nullptr;
    }
}

bool PeripheralWindows::is_connected() {
    if (device_ == nullptr) {
        return false;
    }

    if (SimpleBLE::Config::WinRT::use_deferred_disconnect) {
        if (connection_state_ == ConnectionState::Disconnecting ||
            connection_state_ == ConnectionState::Disconnected) {
            return false;
        }
    }

    return MtaManager::get().execute_sync<bool>([this]() {
        return device_.ConnectionStatus() == BluetoothConnectionStatus::Connected;
    });
}

bool PeripheralWindows::is_connectable() { return connectable_; }

bool PeripheralWindows::is_paired() { throw Exception::OperationNotSupported(); }

void PeripheralWindows::unpair() { throw Exception::OperationNotSupported(); }

SharedPtrVector<ServiceBase> PeripheralWindows::available_services() {
    SharedPtrVector<ServiceBase> service_list;
    for (auto& [service_uuid, service] : gatt_map_) {
        // Build the list of characteristics for the service.
        SharedPtrVector<CharacteristicBase> characteristic_list;
        for (auto& [characteristic_uuid, characteristic] : service.characteristics) {
            // Build the list of descriptors for the characteristic.
            SharedPtrVector<DescriptorBase> descriptor_list;
            for (auto& [descriptor_uuid, descriptor] : characteristic.descriptors) {
                descriptor_list.push_back(std::make_shared<DescriptorBase>(descriptor_uuid));
            }

            uint32_t properties = MtaManager::get().execute_sync<uint32_t>([&characteristic]() {
                return (uint32_t)characteristic.obj.CharacteristicProperties();
            });

            bool can_read = (properties & (uint32_t)GattCharacteristicProperties::Read) != 0;
            bool can_write_request = (properties & (uint32_t)GattCharacteristicProperties::Write) != 0;
            bool can_write_command = (properties & (uint32_t)GattCharacteristicProperties::WriteWithoutResponse) != 0;
            bool can_notify = (properties & (uint32_t)GattCharacteristicProperties::Notify) != 0;
            bool can_indicate = (properties & (uint32_t)GattCharacteristicProperties::Indicate) != 0;

            characteristic_list.push_back(
                std::make_shared<CharacteristicBase>(characteristic_uuid, descriptor_list, can_read, can_write_request,
                                                     can_write_command, can_notify, can_indicate));
        }
        service_list.push_back(std::make_shared<ServiceBase>(service_uuid, characteristic_list));
    }

    return service_list;
}

SharedPtrVector<ServiceBase> PeripheralWindows::advertised_services() {
    SharedPtrVector<ServiceBase> service_list;
    for (auto& [service_uuid, data] : service_data_) {
        service_list.push_back(std::make_shared<ServiceBase>(service_uuid, data));
    }

    return service_list;
}

std::map<uint16_t, ByteArray> PeripheralWindows::manufacturer_data() { return manufacturer_data_; }

ByteArray PeripheralWindows::read(BluetoothUUID const& service, BluetoothUUID const& characteristic) {
    GattCharacteristic gatt_characteristic = _fetch_characteristic(service, characteristic).obj;

    return MtaManager::get().execute_sync<ByteArray>([this, &gatt_characteristic]() {
        // Validate that the operation can be performed.
        uint32_t gatt_characteristic_prop = (uint32_t)gatt_characteristic.CharacteristicProperties();
        if ((gatt_characteristic_prop & (uint32_t)GattCharacteristicProperties::Read) == 0) {
            throw SimpleBLE::Exception::OperationNotSupported("read", guid_to_uuid(gatt_characteristic.Uuid()));
        }

        // Read the value.
        auto result = async_get(gatt_characteristic.ReadValueAsync(Devices::Bluetooth::BluetoothCacheMode::Uncached));
        if (result.Status() != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
        return ibuffer_to_bytearray(result.Value());
    });
}

void PeripheralWindows::write_request(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                                      ByteArray const& data) {
    GattCharacteristic gatt_characteristic = _fetch_characteristic(service, characteristic).obj;

    MtaManager::get().execute_sync([this, &gatt_characteristic, &data]() {
        // Validate that the operation can be performed.
        uint32_t gatt_characteristic_prop = (uint32_t)gatt_characteristic.CharacteristicProperties();
        if ((gatt_characteristic_prop & (uint32_t)GattCharacteristicProperties::Write) == 0) {
            throw SimpleBLE::Exception::OperationNotSupported("write_request", guid_to_uuid(gatt_characteristic.Uuid()));
        }

        // Convert the request data to a buffer.
        winrt::Windows::Storage::Streams::IBuffer buffer = bytearray_to_ibuffer(data);

        // Write the value.
        auto result = async_get(gatt_characteristic.WriteValueAsync(buffer, GattWriteOption::WriteWithResponse));
        if (result != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
    });
}

void PeripheralWindows::write_command(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                                      ByteArray const& data) {
    GattCharacteristic gatt_characteristic = _fetch_characteristic(service, characteristic).obj;

    MtaManager::get().execute_sync([this, &gatt_characteristic, &data]() {
        // Validate that the operation can be performed.
        uint32_t gatt_characteristic_prop = (uint32_t)gatt_characteristic.CharacteristicProperties();
        if ((gatt_characteristic_prop & (uint32_t)GattCharacteristicProperties::WriteWithoutResponse) == 0) {
            throw SimpleBLE::Exception::OperationNotSupported("write_command", guid_to_uuid(gatt_characteristic.Uuid()));
        }

        // Convert the request data to a buffer.
        winrt::Windows::Storage::Streams::IBuffer buffer = bytearray_to_ibuffer(data);

        // Write the value.
        auto result = async_get(gatt_characteristic.WriteValueAsync(buffer, GattWriteOption::WriteWithoutResponse));
        if (result != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
    });
}

void PeripheralWindows::notify(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                               std::function<void(ByteArray payload)> callback) {
    _subscribe(service, characteristic, std::move(callback), GattCharacteristicProperties::Notify,
               GattClientCharacteristicConfigurationDescriptorValue::Notify);
}

void PeripheralWindows::indicate(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                                 std::function<void(ByteArray payload)> callback) {
    _subscribe(service, characteristic, std::move(callback), GattCharacteristicProperties::Indicate,
               GattClientCharacteristicConfigurationDescriptorValue::Indicate);
}

void PeripheralWindows::unsubscribe(BluetoothUUID const& service, BluetoothUUID const& characteristic) {
    gatt_characteristic_t& gatt_characteristic_holder = _fetch_characteristic(service, characteristic);
    GattCharacteristic gatt_characteristic = gatt_characteristic_holder.obj;

    MtaManager::get().execute_sync([this, &gatt_characteristic, &gatt_characteristic_holder]() {
        if (gatt_characteristic_holder.value_changed_token) {
            // Unregister the callback.
            gatt_characteristic.ValueChanged(gatt_characteristic_holder.value_changed_token);
            gatt_characteristic_holder.value_changed_token = {0};
        }

        // Start the indication.
        auto result = async_get(gatt_characteristic.WriteClientCharacteristicConfigurationDescriptorWithResultAsync(
            GattClientCharacteristicConfigurationDescriptorValue::None));

        if (result.Status() != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
    });
}

ByteArray PeripheralWindows::read(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                                  BluetoothUUID const& descriptor) {
    GattDescriptor gatt_descriptor = _fetch_descriptor(service, characteristic, descriptor);

    return MtaManager::get().execute_sync<ByteArray>([this, &gatt_descriptor]() {
        // Read the value.
        auto result = async_get(gatt_descriptor.ReadValueAsync(Devices::Bluetooth::BluetoothCacheMode::Uncached));
        if (result.Status() != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
        return ibuffer_to_bytearray(result.Value());
    });
}

void PeripheralWindows::write(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                              BluetoothUUID const& descriptor, ByteArray const& data) {
    GattDescriptor gatt_descriptor = _fetch_descriptor(service, characteristic, descriptor);

    MtaManager::get().execute_sync([this, &gatt_descriptor, &data]() {
        // Convert the request data to a buffer.
        winrt::Windows::Storage::Streams::IBuffer buffer = bytearray_to_ibuffer(data);

        // Write the value.
        auto result = async_get(gatt_descriptor.WriteValueWithResultAsync(buffer));
        if (result.Status() != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
    });
}

void PeripheralWindows::set_callback_on_connected(std::function<void()> on_connected) {
    if (on_connected) {
        callback_on_connected_.load(std::move(on_connected));
    } else {
        callback_on_connected_.unload();
    }
}

void PeripheralWindows::set_callback_on_disconnected(std::function<void()> on_disconnected) {
    if (on_disconnected) {
        callback_on_disconnected_.load(std::move(on_disconnected));
    } else {
        callback_on_disconnected_.unload();
    }
}

// Private methods

void PeripheralWindows::_subscribe(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                                   std::function<void(ByteArray payload)> callback,
                                   GattCharacteristicProperties property,
                                   GattClientCharacteristicConfigurationDescriptorValue descriptor_value) {
    gatt_characteristic_t& gatt_characteristic_holder = _fetch_characteristic(service, characteristic);
    GattCharacteristic gatt_characteristic = gatt_characteristic_holder.obj;

    MtaManager::get().execute_sync([this, &gatt_characteristic, &gatt_characteristic_holder, callback, property, descriptor_value]() {
        // Validate that the operation can be performed.
        uint32_t gatt_characteristic_prop = (uint32_t)gatt_characteristic.CharacteristicProperties();
        if ((gatt_characteristic_prop & (uint32_t)property) == 0) {
            std::string operation = (property == GattCharacteristicProperties::Notify) ? "notify" : "indicate";
            throw SimpleBLE::Exception::OperationNotSupported(operation, guid_to_uuid(gatt_characteristic.Uuid()));
        }

        // If a notification for the given characteristic is already in progress, swap the callbacks.
        if (gatt_characteristic_holder.value_changed_token) {
            SIMPLEBLE_LOG_WARN("A notification for the given characteristic is already in progress. Swapping callbacks.");
            // Unregister the callback.
            gatt_characteristic.ValueChanged(gatt_characteristic_holder.value_changed_token);
            gatt_characteristic_holder.value_changed_token = {0};
        }

        gatt_characteristic_holder.value_changed_callback = [=](const GattCharacteristic& sender,
                                                                const GattValueChangedEventArgs& args) {
            // Convert the payload to a ByteArray.
            ByteArray payload = ibuffer_to_bytearray(args.CharacteristicValue());
            callback(payload);
        };

        // Register the callback.
        gatt_characteristic_holder.value_changed_token = gatt_characteristic.ValueChanged(
            gatt_characteristic_holder.value_changed_callback);

        // Start the notification.
        auto result = async_get(
            gatt_characteristic.WriteClientCharacteristicConfigurationDescriptorWithResultAsync(descriptor_value));

        if (result.Status() != GenericAttributeProfile::GattCommunicationStatus::Success) {
            throw SimpleBLE::Exception::OperationFailed();
        }
    });
}

bool PeripheralWindows::_attempt_connect(bool use_cached, uint32_t timeout_ms) {
    gatt_map_.clear();

    const BluetoothCacheMode cache_mode =
        use_cached ? BluetoothCacheMode::Cached : BluetoothCacheMode::Uncached;

    // Convert any WinRT exception into a `return false` with a WARN log so the
    // retry loop in connect() can escalate (and so we can see exactly what's
    // failing in the user's logs, which otherwise just show the final
    // "GATT service discovery failed after all retries" message).
    try {
        return MtaManager::get().execute_sync<bool>([this, cache_mode, use_cached, timeout_ms]() {
        if (device_ == nullptr) {
            SIMPLEBLE_LOG_WARN(
                fmt::format("[{}] _attempt_connect: device_ is null", address_));
            return false;
        }
        // We need to cache all services, characteristics and descriptors in the class, else
        // the underlying objects will be garbage collected.
        auto services_result = async_get_for(device_.GetGattServicesAsync(cache_mode), timeout_ms);
        if (services_result.Status() != GattCommunicationStatus::Success) {
            SIMPLEBLE_LOG_WARN(
                fmt::format("[{}] GetGattServicesAsync failed (cached={}): status={}",
                            address_, use_cached,
                            static_cast<int>(services_result.Status())));
            return false;
        }
        if (services_result.Services().Size() == 0) {
            SIMPLEBLE_LOG_WARN(
                fmt::format("[{}] GetGattServicesAsync returned 0 services (cached={})",
                            address_, use_cached));
            return false;
        }

        auto gatt_services = services_result.Services();
        for (GattDeviceService&& service : gatt_services) {
            // For each service...
            gatt_service_t gatt_service;
            gatt_service.obj = service;

            // Save the MTU size
            mtu_ = service.Session().MaxPduSize();

            // Request that Windows keep the GATT session active even when no
            // operations are in progress. Without this, Windows can let the
            // session go idle, which triggers a connection-parameter
            // renegotiation that degrades the notify rate.
            try {
                service.Session().MaintainConnection(true);
            } catch (...) {}

            // Fetch the service UUID
            std::string service_uuid = guid_to_uuid(service.Uuid());

            // Fetch the service characteristics (same cache mode as the
            // services fetch for consistency; without this a Cached services
            // fetch but Uncached characteristics fetch can re-fail mid-way).
            auto characteristics_result = async_get_for(service.GetCharacteristicsAsync(cache_mode), timeout_ms);
            if (characteristics_result.Status() != GattCommunicationStatus::Success) {
                SIMPLEBLE_LOG_WARN(
                    fmt::format("[{}] GetCharacteristicsAsync failed (cached={}, service={}): status={}",
                                address_, use_cached, service_uuid,
                                static_cast<int>(characteristics_result.Status())));
                return false;
            }

            // Load the characteristics into the service
            auto gatt_characteristics = characteristics_result.Characteristics();
            for (GattCharacteristic&& characteristic : gatt_characteristics) {
                // For each characteristic...
                gatt_characteristic_t gatt_characteristic;
                gatt_characteristic.obj = characteristic;

                // Fetch the characteristic UUID
                std::string characteristic_uuid = guid_to_uuid(characteristic.Uuid());

                // Fetch the characteristic descriptors (same cache mode).
                auto descriptors_result = async_get_for(characteristic.GetDescriptorsAsync(cache_mode), timeout_ms);
                if (descriptors_result.Status() != GattCommunicationStatus::Success) {
                    SIMPLEBLE_LOG_WARN(
                        fmt::format("[{}] GetDescriptorsAsync failed (cached={}, char={}): status={}",
                                    address_, use_cached, characteristic_uuid,
                                    static_cast<int>(descriptors_result.Status())));
                    return false;
                }

                // Load the descriptors into the characteristic
                auto gatt_descriptors = descriptors_result.Descriptors();
                for (GattDescriptor&& descriptor : gatt_descriptors) {
                    // For each descriptor...
                    gatt_descriptor_t gatt_descriptor;
                    gatt_descriptor.obj = descriptor;

                    // Fetch the descriptor UUID.
                    std::string descriptor_uuid = guid_to_uuid(descriptor.Uuid());

                    // Append the descriptor to the characteristic.
                    gatt_characteristic.descriptors.emplace(descriptor_uuid, std::move(gatt_descriptor));
                }

                // Append the characteristic to the service.
                gatt_service.characteristics.emplace(characteristic_uuid, std::move(gatt_characteristic));
            }

            // Append the service to the map.
            gatt_map_.emplace(service_uuid, std::move(gatt_service));
        }

        return true;
        });
    } catch (const SimpleBLE::Exception::WinRTException& e) {
        SIMPLEBLE_LOG_WARN(
            fmt::format("[{}] _attempt_connect WinRT exception (cached={}): {}",
                        address_, use_cached, e.what()));
        return false;
    } catch (const std::exception& e) {
        SIMPLEBLE_LOG_WARN(
            fmt::format("[{}] _attempt_connect std::exception (cached={}): {}",
                        address_, use_cached, e.what()));
        return false;
    } catch (...) {
        SIMPLEBLE_LOG_WARN(
            fmt::format("[{}] _attempt_connect unknown exception (cached={})",
                        address_, use_cached));
        return false;
    }
}

// ─── Connection-parameter maintenance (Windows multi-device notify-rate fix) ───
//
// When a new BLE device connects, the Windows BLE stack renegotiates the
// connection interval on already-connected devices, often extending it from
// ~7.5 ms to ~67 ms — which collapses their notify rate from 100 Hz to 15 Hz.
// Windows may replace the preference during a later negotiation. Re-assert on
// its change event, but rate-limit requests because some drivers emit that event
// in response to RequestPreferredConnectionParameters itself.

void PeripheralWindows::_reapply_preferred_connection_parameters_locked_mta() {
    if (device_ == nullptr) return;

    const auto now = std::chrono::steady_clock::now();
    if (last_connection_parameter_request_.time_since_epoch() !=
            std::chrono::steady_clock::duration::zero() &&
        now - last_connection_parameter_request_ <
            std::chrono::milliseconds(kConnectionParameterRetryCooldownMs)) {
        return;
    }
    last_connection_parameter_request_ = now;

    // Reassert MaintainConnection(true) on the persistent session. It's a
    // no-op if already set, but catches the case where Windows silently
    // cleared it during a new-device negotiation.
    try {
        if (gatt_session_ != nullptr) {
            gatt_session_.MaintainConnection(true);
        }
    } catch (...) {}

    // Diagnostic: distinguish between "request accepted by Windows" (no throw)
    // and "request rejected" (throw). If the request is accepted but the notify
    // rate stays at 15 Hz, the BLE adapter firmware itself is rejecting tighter
    // intervals (= hardware limitation, not fixable in software).
    try {
        preferred_connection_params_request_ = device_.RequestPreferredConnectionParameters(
            BluetoothLEPreferredConnectionParameters::ThroughputOptimized());
        // SIMPLEBLE_LOG_INFO(
        //     fmt::format("[{}] reapply ThroughputOptimized: request accepted by Windows",
        //                 address_));
    } catch (const winrt::hresult_error& e) {
        // SIMPLEBLE_LOG_WARN(
        //     fmt::format("[{}] reapply ThroughputOptimized: hresult_error 0x{:08x} - {}",
        //                 address_, (uint32_t)e.code().value, winrt::to_string(e.message())));
    } catch (...) {
        // SIMPLEBLE_LOG_WARN(
        //     fmt::format("[{}] reapply ThroughputOptimized: unknown exception", address_));
    }
}

void PeripheralWindows::_teardown_connection_parameters_watch_locked_mta() {
    if (connection_params_changed_token_ && device_ != nullptr) {
        try {
            device_.ConnectionParametersChanged(connection_params_changed_token_);
        } catch (...) {}
        connection_params_changed_token_ = {};
    }
    // Dropping the request object tells Windows we no longer need the
    // preference held. Safe to call repeatedly.
    preferred_connection_params_request_ = nullptr;
    last_connection_parameter_request_ = {};

    // Release the persistent GattSession. Releasing it also clears
    // MaintainConnection, letting Windows reclaim the resources.
    if (gatt_session_ != nullptr) {
        try {
            gatt_session_.MaintainConnection(false);
            gatt_session_.Close();
        } catch (...) {}
        gatt_session_ = nullptr;
    }
}

// ──────────────────────────────────────────────────────────────────────────

gatt_characteristic_t& PeripheralWindows::_fetch_characteristic(const BluetoothUUID& service_uuid,
                                                                const BluetoothUUID& characteristic_uuid) {
    if (gatt_map_.count(service_uuid) == 0) {
        throw SimpleBLE::Exception::ServiceNotFound(service_uuid);
    }

    if (gatt_map_[service_uuid].characteristics.count(characteristic_uuid) == 0) {
        throw SimpleBLE::Exception::CharacteristicNotFound(characteristic_uuid);
    }

    return gatt_map_[service_uuid].characteristics.at(characteristic_uuid);
}

GattDescriptor PeripheralWindows::_fetch_descriptor(const BluetoothUUID& service_uuid,
                                                    const BluetoothUUID& characteristic_uuid,
                                                    const BluetoothUUID& descriptor_uuid) {
    if (gatt_map_.count(service_uuid) == 0) {
        throw SimpleBLE::Exception::ServiceNotFound(service_uuid);
    }

    if (gatt_map_[service_uuid].characteristics.count(characteristic_uuid) == 0) {
        throw SimpleBLE::Exception::CharacteristicNotFound(characteristic_uuid);
    }

    if (gatt_map_[service_uuid].characteristics[characteristic_uuid].descriptors.count(descriptor_uuid) == 0) {
        throw SimpleBLE::Exception::DescriptorNotFound(descriptor_uuid);
    }

    return gatt_map_[service_uuid].characteristics[characteristic_uuid].descriptors.at(descriptor_uuid).obj;
}
