#pragma once

#include <simpleble/Exceptions.h>
#include <simpleble/Service.h>
#include <simpleble/Types.h>

#include "AdapterBaseTypes.h"
#include "PeripheralBase.h"

#include <kvn_safe_callback.hpp>

#include "winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h"
#include "winrt/Windows.Devices.Bluetooth.h"

#include <condition_variable>
#include <functional>
#include <atomic>
#include <map>
#include <memory>
#include <thread>

using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;

namespace SimpleBLE {

struct gatt_descriptor_t {
    GattDescriptor obj = nullptr;
};

struct gatt_characteristic_t {
    GattCharacteristic obj = nullptr;
    winrt::event_token value_changed_token;
    std::function<void(const GattCharacteristic& sender, const GattValueChangedEventArgs& args)> value_changed_callback;
    std::map<BluetoothUUID, gatt_descriptor_t> descriptors;
};

struct gatt_service_t {
    GattDeviceService obj = nullptr;
    std::map<BluetoothUUID, gatt_characteristic_t> characteristics;
};

class PeripheralWindows : public PeripheralBase {
  public:
    PeripheralWindows(BluetoothLEDevice device);
    PeripheralWindows(advertising_data_t advertising_data);
    virtual ~PeripheralWindows();

    virtual void* underlying() const override;

    virtual std::string identifier() override;
    virtual BluetoothAddress address() override;
    virtual SimpleBLE::BluetoothAddressType address_type() override;
    virtual int16_t rssi() override;
    virtual int16_t tx_power() override;
    virtual uint16_t mtu() override;

    virtual void connect() override;
    virtual void disconnect() override;
    virtual bool is_connected() override;
    virtual bool is_connectable() override;
    virtual bool is_paired() override;
    virtual void unpair() override;

    virtual std::vector<std::shared_ptr<ServiceBase>> available_services() override;
    virtual std::vector<std::shared_ptr<ServiceBase>> advertised_services() override;
    virtual std::map<uint16_t, ByteArray> manufacturer_data() override;

    // clang-format off
    virtual ByteArray read(BluetoothUUID const& service, BluetoothUUID const& characteristic) override;
    virtual void write_request(BluetoothUUID const& service, BluetoothUUID const& characteristic, ByteArray const& data) override;
    virtual void write_command(BluetoothUUID const& service, BluetoothUUID const& characteristic, ByteArray const& data) override;
    virtual void notify(BluetoothUUID const& service, BluetoothUUID const& characteristic, std::function<void(ByteArray payload)> callback) override;
    virtual void indicate(BluetoothUUID const& service, BluetoothUUID const& characteristic, std::function<void(ByteArray payload)> callback) override;
    virtual void unsubscribe(BluetoothUUID const& service, BluetoothUUID const& characteristic) override;

    virtual ByteArray read(BluetoothUUID const& service, BluetoothUUID const& characteristic, BluetoothUUID const& descriptor) override;
    virtual void write(BluetoothUUID const& service, BluetoothUUID const& characteristic, BluetoothUUID const& descriptor, ByteArray const& data) override;
    // clang-format on

    virtual void set_callback_on_connected(std::function<void()> on_connected) override;
    virtual void set_callback_on_disconnected(std::function<void()> on_disconnected) override;

    // Internal methods not exposed to the user.

    void update_advertising_data(advertising_data_t advertising_data);
    bool is_disconnect_pending() const noexcept;

  private:
    BluetoothLEDevice device_{nullptr};

    // NOTE: Calling device_.Name() or device_.BluetoothAddress() might
    // cause a crash on some devices.
    // This is because any operation on the object before it is connected will
    // initiate a connection, which can then cause further cascading failures.
    // See:
    // https://docs.microsoft.com/en-us/uwp/api/windows.devices.bluetooth.bluetoothledevice.frombluetoothaddressasync

    std::string identifier_;
    BluetoothAddress address_;
    SimpleBLE::BluetoothAddressType address_type_;
    int16_t rssi_ = INT16_MIN;
    int16_t tx_power_ = INT16_MIN;
    uint16_t mtu_;
    bool connectable_;
    winrt::event_token connection_status_changed_token_;
    // Set when ConnectionParametersChanged is subscribed (Windows 10 2004+).
    // Used to re-assert ThroughputOptimized whenever Windows renegotiates the
    // connection interval — which it does aggressively every time a new BLE
    // device connects, extending older devices' intervals from ~7.5 ms to
    // ~67 ms (drops notify rate from ~100 Hz to ~15 Hz).
    winrt::event_token connection_params_changed_token_;

    // Preferred-connection-parameters request. Must stay alive (keeping the
    // shared_ptr / WinRT handle) to keep the preference asserted with Windows.
    BluetoothLEPreferredConnectionParametersRequest preferred_connection_params_request_{nullptr};

    // Explicit long-lived GattSession held for this device. Created via
    // GattSession::FromDeviceIdAsync BEFORE GATT discovery so that we can assert
    // MaintainConnection(true) on the SAME session object that Windows will use
    // for the actual connection — as opposed to `service.Session()` which
    // returns a temporary wrapper whose MaintainConnection setting appears to
    // get reset when the wrapper is released.
    GattSession gatt_session_{nullptr};

    // Watchdog thread that periodically re-asserts ThroughputOptimized. Belt-
    // and-suspenders fallback in case ConnectionParametersChanged events are
    // delivered late or not at all on a given driver / Windows build.
    std::thread conn_param_watchdog_thread_;
    std::atomic<bool> conn_param_watchdog_running_{false};

    // Target max interval in milliseconds (ThroughputOptimized max is 15 ms).
    // When the observed ConnectionInterval exceeds this, we re-request.
    static constexpr int64_t kPreferredMaxIntervalMs = 20;

    // Re-request ThroughputOptimized connection parameters if the current
    // connection interval exceeds the target. Safe to call concurrently.
    // Must be invoked via MtaManager::execute_sync (accesses device_).
    void _reapply_preferred_connection_parameters_locked_mta();
    // Public-facing trampoline: submits the reapply task to MtaManager and
    // returns once it has executed.
    void _reapply_preferred_connection_parameters();
    // Unregister the ConnectionParametersChanged handler and release the
    // preferred-parameters request. Must run on MTA.
    void _teardown_connection_parameters_watch_locked_mta();
    // Start / stop the watchdog thread.
    void _conn_param_watchdog_start();
    void _conn_param_watchdog_stop();

    // Internal state for deferred (non-blocking) disconnect mode
    // Only active when Config::WinRT::use_deferred_disconnect == true
    enum class ConnectionState { Disconnected, Connecting, Connected, Disconnecting };
    std::atomic<ConnectionState> connection_state_{ConnectionState::Disconnected};

    std::condition_variable disconnection_cv_;
    std::mutex disconnection_mutex_;

    std::map<BluetoothUUID, gatt_service_t> gatt_map_;

    kvn::safe_callback<void()> callback_on_connected_;
    kvn::safe_callback<void()> callback_on_disconnected_;

    std::map<uint16_t, SimpleBLE::ByteArray> manufacturer_data_;
    std::map<BluetoothUUID, SimpleBLE::ByteArray> service_data_;

    // use_cached=true => pass BluetoothCacheMode::Cached to GetGattServicesAsync
    // etc. Paired devices have their service structure persisted by Windows, so
    // Cached mode often succeeds on reconnect where Uncached fails (observed
    // with certain third-party BLE dongles).
    bool _attempt_connect(bool use_cached = false);

    gatt_characteristic_t& _fetch_characteristic(const BluetoothUUID& service_uuid,
                                                 const BluetoothUUID& characteristic_uuid);

    GattDescriptor _fetch_descriptor(const BluetoothUUID& service_uuid, const BluetoothUUID& characteristic_uuid,
                                     const BluetoothUUID& descriptor_uuid);

    void _subscribe(BluetoothUUID const& service, BluetoothUUID const& characteristic,
                    std::function<void(ByteArray payload)> callback, GattCharacteristicProperties property,
                    GattClientCharacteristicConfigurationDescriptorValue descriptor_value);
};

}  // namespace SimpleBLE
