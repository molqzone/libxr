#pragma once

#include <cstddef>
#include <cstdint>

#include "core/libxr_def.hpp"

namespace LibXR::EtherCAT
{

namespace Detail
{
struct CallbackBridge;
}

/**
 * C++ base class for one SOES EtherCAT slave instance.
 *
 * Derive from SlaveClass and override the protected hooks required by the
 * application or ESC driver. SOES uses global state, so one instance can be
 * active in an image at a time.
 */
class SlaveClass
{
 public:
  enum class ExecutionMode : uint8_t
  {
    POLLING,
    INTERRUPT,
  };

  struct Options
  {
    ExecutionMode execution_mode = ExecutionMode::POLLING;
    int watchdog_count = 0;
    bool skip_default_initialization = false;
  };

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

  SlaveClass() = default;
  explicit SlaveClass(const Options& options) : options_(options) {}

  SlaveClass(const SlaveClass&) = delete;
  SlaveClass& operator=(const SlaveClass&) = delete;
  SlaveClass(SlaveClass&&) = delete;
  SlaveClass& operator=(SlaveClass&&) = delete;

  virtual ~SlaveClass();

  [[nodiscard]] ErrorCode Initialize();
  [[nodiscard]] ErrorCode Poll();
  [[nodiscard]] ErrorCode Worker(uint32_t event_mask);
  [[nodiscard]] ErrorCode Process(uint8_t flags);
  [[nodiscard]] ErrorCode Run();

  [[nodiscard]] bool IsInitialized() const { return initialized_; }
  [[nodiscard]] Options& GetOptions() { return options_; }
  [[nodiscard]] const Options& GetOptions() const { return options_; }

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
  friend struct Detail::CallbackBridge;

  inline static SlaveClass* active_ = nullptr;

  Options options_{};
  bool initialized_ = false;
};

}  // namespace LibXR::EtherCAT

#include "detail/slave_class_impl.hpp"
