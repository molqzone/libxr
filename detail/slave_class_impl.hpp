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

  static SlaveCore* Active() { return SlaveCore::active_; }

  static void SetDefaults()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchSetDefaults();
    }
  }

  static void PreStateChange(uint8_t* state, uint8_t* notification)
  {
    if (auto* core = Active(); core != nullptr && state != nullptr && notification != nullptr)
    {
      core->DispatchPreStateChange(*state, *notification);
    }
  }

  static void PostStateChange(uint8_t* state, uint8_t* notification)
  {
    if (auto* core = Active(); core != nullptr && state != nullptr && notification != nullptr)
    {
      core->DispatchPostStateChange(*state, *notification);
    }
  }

  static void Application()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchApplication();
    }
  }

  static void SafeOutputs()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchSafeOutputs();
    }
  }

  static void Inputs()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchInputs();
    }
  }

  static void Outputs()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchOutputs();
    }
  }

  static ObjectAccessCode PreObjectDownload(uint16_t index, uint8_t subindex, void* data,
                                             size_t size, uint16_t flags)
  {
    if (auto* core = Active(); core != nullptr)
    {
      return core->DispatchPreObjectDownload({index, subindex, flags}, {data, size});
    }
    return 0;
  }

  static ObjectAccessCode PostObjectDownload(uint16_t index, uint8_t subindex, uint16_t flags)
  {
    if (auto* core = Active(); core != nullptr)
    {
      return core->DispatchPostObjectDownload({index, subindex, flags});
    }
    return 0;
  }

  static ObjectAccessCode PreObjectUpload(uint16_t index, uint8_t subindex, void* data,
                                           size_t* size, uint16_t flags)
  {
    if (auto* core = Active(); core != nullptr)
    {
      ObjectBuffer buffer{data, size == nullptr ? 0u : *size};
      const ObjectAccessCode code =
          core->DispatchPreObjectUpload({index, subindex, flags}, buffer);
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
    if (auto* core = Active(); core != nullptr)
    {
      return core->DispatchPostObjectUpload({index, subindex, flags});
    }
    return 0;
  }

  static void ReceiveProcessData()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchReceiveProcessData();
    }
  }

  static void TransmitProcessData()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchTransmitProcessData();
    }
  }

  static void EnableInterrupt(uint32_t mask)
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchEnableInterrupt(mask);
    }
  }

  static void DisableInterrupt(uint32_t mask)
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchDisableInterrupt(mask);
    }
  }

  static void EepromEvent()
  {
    if (auto* core = Active(); core != nullptr)
    {
      core->DispatchEepromEvent();
    }
  }

  static uint16_t CheckDistributedClock()
  {
    if (auto* core = Active(); core != nullptr)
    {
      return core->DispatchCheckDistributedClock();
    }
    return 0;
  }

  static int GetDeviceId(uint16_t* device_id)
  {
    if (auto* core = Active(); core != nullptr && device_id != nullptr)
    {
      return static_cast<int>(core->DispatchGetDeviceId(*device_id));
    }
    return static_cast<int>(ErrorCode::PTR_NULL);
  }
};

inline esc_cfg_t soes_configuration{};

inline esc_cfg_t MakeSoesConfiguration(SlaveCore& core)
{
  esc_cfg_t soes_config{};
  soes_config.user_arg = &core;
  soes_config.use_interrupt = 1;
  soes_config.watchdog_cnt = core.GetOptions().watchdog_count;
  soes_config.skip_default_initialization = false;
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

inline SlaveCore::SlaveCore(SlaveClass& slave) : SlaveCore(slave, Options{}) {}

inline SlaveCore::SlaveCore(SlaveClass& slave, const Options& options)
    : slave_(slave), options_(options)
{
  ASSERT(active_ == nullptr);
  active_ = this;
  Detail::soes_configuration = Detail::MakeSoesConfiguration(*this);
  ecat_slv_init(&Detail::soes_configuration);
}

inline SlaveCore::~SlaveCore()
{
  if (active_ == this)
  {
    active_ = nullptr;
  }
}

inline void SlaveCore::HandleInterrupt(uint32_t event_mask)
{
  ecat_slv_worker(event_mask);
}

}  // namespace LibXR::EtherCAT
