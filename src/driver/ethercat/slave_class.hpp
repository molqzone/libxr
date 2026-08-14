#pragma once

#include <cstddef>
#include <cstdint>

#include "core/libxr_def.hpp"

namespace LibXR::EtherCAT
{

class SlaveCore;

namespace Detail
{
struct CallbackBridge;
}

/**
 * EtherCAT device-class base, analogous to LibXR::USB::DeviceClass.
 *
 * Derive from SlaveClass to implement application and board-driver hooks. A
 * SlaveCore constructed with this completed object owns SOES startup and IRQ
 * dispatch.
 */
class SlaveClass
{
 public:
  struct ObjectAddress
  {
    uint16_t index;
    uint8_t subindex;
    uint16_t flags;
  };

  struct ObjectBuffer
  {
    void* data;
    size_t size;
  };

  using ObjectAccessCode = uint32_t;

  virtual ~SlaveClass() = default;

 protected:
  virtual void OnSetDefaults() {}
  virtual void OnPreStateChange(uint8_t& state, uint8_t& notification)
  {
    (void)state;
    (void)notification;
  }
  virtual void OnPostStateChange(uint8_t& state, uint8_t& notification)
  {
    (void)state;
    (void)notification;
  }
  virtual void OnApplication() {}
  virtual void OnSafeOutputs() {}
  virtual void OnInputs() {}
  virtual void OnOutputs() {}
  virtual void OnReceiveProcessData() {}
  virtual void OnTransmitProcessData() {}
  virtual void OnEnableInterrupt(uint32_t mask) { (void)mask; }
  virtual void OnDisableInterrupt(uint32_t mask) { (void)mask; }
  virtual void OnEepromEvent() {}
  virtual uint16_t OnCheckDistributedClock() { return 0; }
  virtual ErrorCode OnGetDeviceId(uint16_t& device_id)
  {
    (void)device_id;
    return ErrorCode::NOT_SUPPORT;
  }
  virtual ObjectAccessCode OnPreObjectDownload(const ObjectAddress& object,
                                                const ObjectBuffer& buffer)
  {
    (void)object;
    (void)buffer;
    return 0;
  }
  virtual ObjectAccessCode OnPostObjectDownload(const ObjectAddress& object)
  {
    (void)object;
    return 0;
  }
  virtual ObjectAccessCode OnPreObjectUpload(const ObjectAddress& object,
                                              ObjectBuffer& buffer)
  {
    (void)object;
    (void)buffer;
    return 0;
  }
  virtual ObjectAccessCode OnPostObjectUpload(const ObjectAddress& object)
  {
    (void)object;
    return 0;
  }

 private:
  friend class SlaveCore;
};

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

  [[nodiscard]] const Options& GetOptions() const { return options_; }

 private:
  friend struct Detail::CallbackBridge;

  void DispatchSetDefaults() { slave_.OnSetDefaults(); }
  void DispatchPreStateChange(uint8_t& state, uint8_t& notification)
  {
    slave_.OnPreStateChange(state, notification);
  }
  void DispatchPostStateChange(uint8_t& state, uint8_t& notification)
  {
    slave_.OnPostStateChange(state, notification);
  }
  void DispatchApplication() { slave_.OnApplication(); }
  void DispatchSafeOutputs() { slave_.OnSafeOutputs(); }
  void DispatchInputs() { slave_.OnInputs(); }
  void DispatchOutputs() { slave_.OnOutputs(); }
  void DispatchReceiveProcessData() { slave_.OnReceiveProcessData(); }
  void DispatchTransmitProcessData() { slave_.OnTransmitProcessData(); }
  void DispatchEnableInterrupt(uint32_t mask) { slave_.OnEnableInterrupt(mask); }
  void DispatchDisableInterrupt(uint32_t mask) { slave_.OnDisableInterrupt(mask); }
  void DispatchEepromEvent() { slave_.OnEepromEvent(); }
  [[nodiscard]] uint16_t DispatchCheckDistributedClock()
  {
    return slave_.OnCheckDistributedClock();
  }
  [[nodiscard]] ErrorCode DispatchGetDeviceId(uint16_t& device_id)
  {
    return slave_.OnGetDeviceId(device_id);
  }
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPreObjectDownload(
      const SlaveClass::ObjectAddress& object, const SlaveClass::ObjectBuffer& buffer)
  {
    return slave_.OnPreObjectDownload(object, buffer);
  }
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPostObjectDownload(
      const SlaveClass::ObjectAddress& object)
  {
    return slave_.OnPostObjectDownload(object);
  }
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPreObjectUpload(
      const SlaveClass::ObjectAddress& object, SlaveClass::ObjectBuffer& buffer)
  {
    return slave_.OnPreObjectUpload(object, buffer);
  }
  [[nodiscard]] SlaveClass::ObjectAccessCode DispatchPostObjectUpload(
      const SlaveClass::ObjectAddress& object)
  {
    return slave_.OnPostObjectUpload(object);
  }

  inline static SlaveCore* active_ = nullptr;

  SlaveClass& slave_;
  Options options_{};
};

}  // namespace LibXR::EtherCAT

#include "detail/slave_class_impl.hpp"
