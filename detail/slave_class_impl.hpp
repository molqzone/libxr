#pragma once

// This file is an implementation detail of slave_class.hpp. It owns the C
// callback bridge and the translation to the SOES stack configuration.
#ifdef __cplusplus
extern "C" {
#endif
#include "ecat_slv.h"
#ifdef __cplusplus
}
#endif

namespace LibXR::EtherCAT::Detail
{

struct CallbackBridge
{
  using ObjectAccessCode = SlaveClass::ObjectAccessCode;
  using ObjectBuffer = SlaveClass::ObjectBuffer;

  static SlaveClass* Active() { return SlaveClass::active_; }

  static void SetDefaults()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnSetDefaults();
    }
  }

  static void PreStateChange(uint8_t* state, uint8_t* notification)
  {
    if (auto* slave = Active(); slave != nullptr && state != nullptr && notification != nullptr)
    {
      slave->OnPreStateChange(*state, *notification);
    }
  }

  static void PostStateChange(uint8_t* state, uint8_t* notification)
  {
    if (auto* slave = Active(); slave != nullptr && state != nullptr && notification != nullptr)
    {
      slave->OnPostStateChange(*state, *notification);
    }
  }

  static void Application()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnApplication();
    }
  }

  static void SafeOutputs()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnSafeOutputs();
    }
  }

  static void Inputs()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnInputs();
    }
  }

  static void Outputs()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnOutputs();
    }
  }

  static ObjectAccessCode PreObjectDownload(uint16_t index, uint8_t subindex, void* data,
                                             size_t size, uint16_t flags)
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      return slave->OnPreObjectDownload({index, subindex, flags}, {data, size});
    }
    return 0;
  }

  static ObjectAccessCode PostObjectDownload(uint16_t index, uint8_t subindex, uint16_t flags)
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      return slave->OnPostObjectDownload({index, subindex, flags});
    }
    return 0;
  }

  static ObjectAccessCode PreObjectUpload(uint16_t index, uint8_t subindex, void* data,
                                           size_t* size, uint16_t flags)
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      ObjectBuffer buffer{data, size == nullptr ? 0u : *size};
      const ObjectAccessCode code =
          slave->OnPreObjectUpload({index, subindex, flags}, buffer);
      if (size != nullptr)
      {
        *size = buffer.size;
      }
      return code;
    }
    return 0;
  }

  static ObjectAccessCode PostObjectUpload(uint16_t index, uint8_t subindex, uint16_t flags)
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      return slave->OnPostObjectUpload({index, subindex, flags});
    }
    return 0;
  }

  static void ReceiveProcessData()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnReceiveProcessData();
    }
  }

  static void TransmitProcessData()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnTransmitProcessData();
    }
  }

  static void EnableInterrupt(uint32_t mask)
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnEnableInterrupt(mask);
    }
  }

  static void DisableInterrupt(uint32_t mask)
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnDisableInterrupt(mask);
    }
  }

  static void EepromEvent()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      slave->OnEepromEvent();
    }
  }

  static uint16_t CheckDistributedClock()
  {
    if (auto* slave = Active(); slave != nullptr)
    {
      return slave->OnCheckDistributedClock();
    }
    return 0;
  }

  static int GetDeviceId(uint16_t* device_id)
  {
    if (auto* slave = Active(); slave != nullptr && device_id != nullptr)
    {
      return static_cast<int>(slave->OnGetDeviceId(*device_id));
    }
    return static_cast<int>(ErrorCode::PTR_NULL);
  }
};

inline esc_cfg_t soes_configuration{};

inline esc_cfg_t MakeSoesConfiguration(const SlaveClass::Options& options)
{
  esc_cfg_t soes_config{};
  soes_config.user_arg = CallbackBridge::Active();
  soes_config.use_interrupt = 1;
  soes_config.watchdog_cnt = options.watchdog_count;
  soes_config.skip_default_initialization = options.skip_default_initialization;
  soes_config.set_defaults_hook = CallbackBridge::SetDefaults;
  soes_config.pre_state_change_hook = CallbackBridge::PreStateChange;
  soes_config.post_state_change_hook = CallbackBridge::PostStateChange;
  soes_config.application_hook = CallbackBridge::Application;
  soes_config.safeoutput_override = CallbackBridge::SafeOutputs;
  soes_config.pre_object_download_hook = CallbackBridge::PreObjectDownload;
  soes_config.post_object_download_hook = CallbackBridge::PostObjectDownload;
  soes_config.pre_object_upload_hook = CallbackBridge::PreObjectUpload;
  soes_config.post_object_upload_hook = CallbackBridge::PostObjectUpload;
  soes_config.rxpdo_override = CallbackBridge::ReceiveProcessData;
  soes_config.txpdo_override = CallbackBridge::TransmitProcessData;
  soes_config.esc_hw_interrupt_enable = CallbackBridge::EnableInterrupt;
  soes_config.esc_hw_interrupt_disable = CallbackBridge::DisableInterrupt;
  soes_config.esc_hw_eep_handler = CallbackBridge::EepromEvent;
  soes_config.esc_check_dc_handler = CallbackBridge::CheckDistributedClock;
  soes_config.get_device_id = CallbackBridge::GetDeviceId;
  return soes_config;
}

}  // namespace LibXR::EtherCAT::Detail

#if defined(__GNUC__) || defined(__clang__)
#define LIBXR_ETHERCAT_DETAIL_USED __attribute__((used))
#else
#define LIBXR_ETHERCAT_DETAIL_USED
#endif

extern "C" inline LIBXR_ETHERCAT_DETAIL_USED void cb_get_inputs()
{
  LibXR::EtherCAT::Detail::CallbackBridge::Inputs();
}

extern "C" inline LIBXR_ETHERCAT_DETAIL_USED void cb_set_outputs()
{
  LibXR::EtherCAT::Detail::CallbackBridge::Outputs();
}

#undef LIBXR_ETHERCAT_DETAIL_USED

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
  Detail::soes_configuration = Detail::MakeSoesConfiguration(options_);
  ecat_slv_init(&Detail::soes_configuration);
  initialized_ = true;
  return ErrorCode::OK;
}

inline ErrorCode SlaveClass::HandleInterrupt(uint32_t event_mask)
{
  if (!initialized_)
  {
    return ErrorCode::STATE_ERR;
  }
  ecat_slv_worker(event_mask);
  return ErrorCode::OK;
}

}  // namespace LibXR::EtherCAT
