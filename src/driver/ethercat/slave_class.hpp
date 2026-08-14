#pragma once

#include <cstddef>
#include <cstdint>

#include "core/libxr_def.hpp"

namespace LibXR::EtherCAT
{

/**
 * Thin LibXR facade for one SOES EtherCAT slave instance.
 *
 * SOES keeps its slave state in global objects and exposes process-data callbacks
 * as global C symbols. Consequently an image can have one active SlaveClass. The
 * application or the hardware driver must provide cb_get_inputs() and
 * cb_set_outputs(), as well as the ESC callbacks in Configuration.
 */
class SlaveClass
{
 public:
  using Hook = void (*)();
  using StateChangeHook = void (*)(uint8_t* state, uint8_t* notification);
  using PreDownloadHook = uint32_t (*)(uint16_t index, uint8_t subindex, void* data,
                                       size_t size, uint16_t flags);
  using PostDownloadHook = uint32_t (*)(uint16_t index, uint8_t subindex, uint16_t flags);
  using PreUploadHook = uint32_t (*)(uint16_t index, uint8_t subindex, void* data,
                                     size_t* size, uint16_t flags);
  using PostUploadHook = uint32_t (*)(uint16_t index, uint8_t subindex, uint16_t flags);
  using InterruptHook = void (*)(uint32_t mask);
  using DcCheckHook = uint16_t (*)();
  using DeviceIdHook = int (*)(uint16_t* device_id);

  /** XRECAT-owned slave setup data. */
  struct Configuration
  {
    void* user_arg = nullptr;
    bool use_interrupt = false;
    int watchdog_count = 0;
    bool skip_default_initialization = false;

    Hook on_set_defaults = nullptr;
    StateChangeHook on_pre_state_change = nullptr;
    StateChangeHook on_post_state_change = nullptr;
    Hook on_application = nullptr;
    Hook on_safe_output = nullptr;
    PreDownloadHook on_pre_object_download = nullptr;
    PostDownloadHook on_post_object_download = nullptr;
    PreUploadHook on_pre_object_upload = nullptr;
    PostUploadHook on_post_object_upload = nullptr;
    Hook on_rxpdo_override = nullptr;
    Hook on_txpdo_override = nullptr;
    InterruptHook on_interrupt_enable = nullptr;
    InterruptHook on_interrupt_disable = nullptr;
    Hook on_eeprom_event = nullptr;
    DcCheckHook on_dc_check = nullptr;
    DeviceIdHook on_get_device_id = nullptr;
  };

  explicit SlaveClass(const Configuration& config) : config_(config) {}

  SlaveClass(const SlaveClass&) = delete;
  SlaveClass& operator=(const SlaveClass&) = delete;
  SlaveClass(SlaveClass&&) = delete;
  SlaveClass& operator=(SlaveClass&&) = delete;

  ~SlaveClass();

  [[nodiscard]] ErrorCode Initialize();
  [[nodiscard]] ErrorCode Poll();
  [[nodiscard]] ErrorCode Worker(uint32_t event_mask);
  [[nodiscard]] ErrorCode Process(uint8_t flags);
  [[nodiscard]] ErrorCode Run();

  [[nodiscard]] bool IsInitialized() const { return initialized_; }

  [[nodiscard]] Configuration& GetConfiguration() { return config_; }
  [[nodiscard]] const Configuration& GetConfiguration() const { return config_; }

  static SlaveClass* Active();

 private:
  inline static SlaveClass* active_ = nullptr;

  Configuration config_{};
  bool initialized_ = false;
};

}  // namespace LibXR::EtherCAT

#include "detail/slave_class_impl.hpp"
