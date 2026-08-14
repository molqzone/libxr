#pragma once

#include <cstddef>
#include <cstdint>

#include "core/libxr_def.hpp"

namespace LibXR::EtherCAT
{

class SlaveCore;

/**
 * EtherCAT device-class base, analogous to LibXR::USB::DeviceClass.
 *
 * Derive from SlaveClass to implement application and board-driver hooks.
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

}  // namespace LibXR::EtherCAT
