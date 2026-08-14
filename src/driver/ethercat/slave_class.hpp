#pragma once

#include <cstddef>
#include <cstdint>

// SOES remains an application/driver dependency. XRECAT only wraps its slave
// lifecycle and does not provide an ESC port, object dictionary, or process data model.
#ifdef __cplusplus
extern "C" {
#endif
#include "ecat_slv.h"
#ifdef __cplusplus
}
#endif

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

  /** XRECAT-owned setup data; it is translated to SOES internally. */
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

  ~SlaveClass()
  {
    if (active_ == this)
    {
      active_ = nullptr;
    }
  }

  [[nodiscard]] ErrorCode Initialize()
  {
    if (initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    if (active_ != nullptr && active_ != this)
    {
      return ErrorCode::BUSY;
    }

    active_ = this;
    soes_config_ = MakeSoesConfiguration(config_);
    ecat_slv_init(&soes_config_);
    initialized_ = true;
    return ErrorCode::OK;
  }

  [[nodiscard]] ErrorCode Poll()
  {
    if (!initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    ecat_slv_poll();
    return ErrorCode::OK;
  }

  [[nodiscard]] ErrorCode Worker(uint32_t event_mask)
  {
    if (!initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    ecat_slv_worker(event_mask);
    return ErrorCode::OK;
  }

  [[nodiscard]] ErrorCode Process(uint8_t flags)
  {
    if (!initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    DIG_process(flags);
    return ErrorCode::OK;
  }

  [[nodiscard]] ErrorCode Run()
  {
    if (!initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    ecat_slv();
    return ErrorCode::OK;
  }

  [[nodiscard]] bool IsInitialized() const { return initialized_; }

  [[nodiscard]] Configuration& GetConfiguration() { return config_; }
  [[nodiscard]] const Configuration& GetConfiguration() const { return config_; }

  static SlaveClass* Active() { return active_; }

 private:
  static esc_cfg_t MakeSoesConfiguration(const Configuration& config)
  {
    esc_cfg_t soes_config{};
    soes_config.user_arg = config.user_arg;
    soes_config.use_interrupt = config.use_interrupt ? 1 : 0;
    soes_config.watchdog_cnt = config.watchdog_count;
    soes_config.skip_default_initialization = config.skip_default_initialization;
    soes_config.set_defaults_hook = config.on_set_defaults;
    soes_config.pre_state_change_hook = config.on_pre_state_change;
    soes_config.post_state_change_hook = config.on_post_state_change;
    soes_config.application_hook = config.on_application;
    soes_config.safeoutput_override = config.on_safe_output;
    soes_config.pre_object_download_hook = config.on_pre_object_download;
    soes_config.post_object_download_hook = config.on_post_object_download;
    soes_config.pre_object_upload_hook = config.on_pre_object_upload;
    soes_config.post_object_upload_hook = config.on_post_object_upload;
    soes_config.rxpdo_override = config.on_rxpdo_override;
    soes_config.txpdo_override = config.on_txpdo_override;
    soes_config.esc_hw_interrupt_enable = config.on_interrupt_enable;
    soes_config.esc_hw_interrupt_disable = config.on_interrupt_disable;
    soes_config.esc_hw_eep_handler = config.on_eeprom_event;
    soes_config.esc_check_dc_handler = config.on_dc_check;
    soes_config.get_device_id = config.on_get_device_id;
    return soes_config;
  }

  inline static SlaveClass* active_ = nullptr;

  Configuration config_{};
  esc_cfg_t soes_config_{};
  bool initialized_ = false;
};

}  // namespace LibXR::EtherCAT
