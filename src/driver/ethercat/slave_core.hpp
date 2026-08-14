#pragma once

#include <cstdint>

#include "slave_class.hpp"

namespace LibXR::EtherCAT
{

namespace Detail
{
struct CallbackBridge;
}

/**
 * SOES runtime core, analogous to LibXR::USB::DeviceCore.
 *
 * Construct this after the SlaveClass-derived device object. SOES uses global
 * state, so one SlaveCore can be active in an image at a time.
 */
class SlaveCore final
{
 public:
  struct Options
  {
    int watchdog_count = 0;
  };

  explicit SlaveCore(SlaveClass& slave);
  SlaveCore(SlaveClass& slave, const Options& options);

  SlaveCore(const SlaveCore&) = delete;
  SlaveCore& operator=(const SlaveCore&) = delete;
  SlaveCore(SlaveCore&&) = delete;
  SlaveCore& operator=(SlaveCore&&) = delete;

  ~SlaveCore();

  /** Dispatch a low-priority ESC interrupt event mask from the board driver. */
  void HandleInterrupt(uint32_t event_mask);

  [[nodiscard]] const Options& GetOptions() const;

 private:
  friend struct Detail::CallbackBridge;

  void DispatchSetDefaults();
  void DispatchPreStateChange(uint8_t& state, uint8_t& notification);
  void DispatchPostStateChange(uint8_t& state, uint8_t& notification);
  void DispatchApplication();
  void DispatchSafeOutputs();
  void DispatchInputs();
  void DispatchOutputs();
  void DispatchReceiveProcessData();
  void DispatchTransmitProcessData();
  void DispatchEnableInterrupt(uint32_t mask);
  void DispatchDisableInterrupt(uint32_t mask);
  void DispatchEepromEvent();
  [[nodiscard]] uint16_t DispatchCheckDistributedClock();
  [[nodiscard]] ErrorCode DispatchGetDeviceId(uint16_t& device_id);
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPreObjectDownload(
      const SlaveClass::ObjectAddress& object, const SlaveClass::ObjectBuffer& buffer);
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPostObjectDownload(
      const SlaveClass::ObjectAddress& object);
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPreObjectUpload(
      const SlaveClass::ObjectAddress& object, SlaveClass::ObjectBuffer& buffer);
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPostObjectUpload(
      const SlaveClass::ObjectAddress& object);

  static SlaveCore* active_;

  SlaveClass& slave_;
  Options options_{};
};

}  // namespace LibXR::EtherCAT
