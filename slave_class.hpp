#pragma once

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
 * cb_set_outputs(), as well as the ESC callbacks in esc_cfg_t.
 */
class SlaveClass
{
 public:
  explicit SlaveClass(const esc_cfg_t& config) : config_(config) {}

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
    ecat_slv_init(&config_);
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

  [[nodiscard]] esc_cfg_t& Config() { return config_; }
  [[nodiscard]] const esc_cfg_t& Config() const { return config_; }

  static SlaveClass* Active() { return active_; }

 private:
  inline static SlaveClass* active_ = nullptr;

  esc_cfg_t config_{};
  bool initialized_ = false;
};

}  // namespace LibXR::EtherCAT
