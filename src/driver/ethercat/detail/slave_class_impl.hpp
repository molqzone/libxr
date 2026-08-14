#pragma once

// This file is an implementation detail of slave_class.hpp. It is the only
// place where XRECAT translates its public Configuration to SOES types.
#ifdef __cplusplus
extern "C" {
#endif
#include "ecat_slv.h"
#ifdef __cplusplus
}
#endif

namespace LibXR::EtherCAT::Detail
{

inline esc_cfg_t soes_configuration{};

inline esc_cfg_t MakeSoesConfiguration(const SlaveClass::Configuration& config)
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

}  // namespace LibXR::EtherCAT::Detail

namespace LibXR::EtherCAT
{

inline SlaveClass::~SlaveClass()
{
  if (active_ == this)
  {
    active_ = nullptr;
  }
}

inline ErrorCode SlaveClass::Initialize()
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
  Detail::soes_configuration = Detail::MakeSoesConfiguration(config_);
  ecat_slv_init(&Detail::soes_configuration);
  initialized_ = true;
  return ErrorCode::OK;
}

inline ErrorCode SlaveClass::Poll()
{
  if (!initialized_)
  {
    return ErrorCode::STATE_ERR;
  }
  ecat_slv_poll();
  return ErrorCode::OK;
}

inline ErrorCode SlaveClass::Worker(uint32_t event_mask)
{
  if (!initialized_)
  {
    return ErrorCode::STATE_ERR;
  }
  ecat_slv_worker(event_mask);
  return ErrorCode::OK;
}

inline ErrorCode SlaveClass::Process(uint8_t flags)
{
  if (!initialized_)
  {
    return ErrorCode::STATE_ERR;
  }
  DIG_process(flags);
  return ErrorCode::OK;
}

inline ErrorCode SlaveClass::Run()
{
  if (!initialized_)
  {
    return ErrorCode::STATE_ERR;
  }
  ecat_slv();
  return ErrorCode::OK;
}

inline SlaveClass* SlaveClass::Active()
{
  return active_;
}

}  // namespace LibXR::EtherCAT
