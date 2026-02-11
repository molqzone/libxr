#include "mspm0_adc.hpp"

#include <cstddef>
#include <cstdint>

#include "dl_dma.h"

using namespace LibXR;

namespace
{

constexpr uint32_t MSPM0_ADC_DMA_TRIGGER_INVALID = 0xFFFFFFFFU;
constexpr uint8_t MSPM0_ADC_DMA_CHANNEL_INVALID = 0xFF;
constexpr uint8_t MSPM0_ADC_DMA_SINGLE_SAMPLE_COUNT = 1U;

constexpr uint32_t get_dma_interrupt_mask(uint8_t channel_id)
{
  return (channel_id < 32U) ? (1UL << channel_id) : 0U;
}

uint32_t get_mem_result_interrupt_mask(DL_ADC12_MEM_IDX mem_idx)
{
  switch (mem_idx)
  {
    case DL_ADC12_MEM_IDX_0:
      return DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_1:
      return DL_ADC12_INTERRUPT_MEM1_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_2:
      return DL_ADC12_INTERRUPT_MEM2_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_3:
      return DL_ADC12_INTERRUPT_MEM3_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_4:
      return DL_ADC12_INTERRUPT_MEM4_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_5:
      return DL_ADC12_INTERRUPT_MEM5_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_6:
      return DL_ADC12_INTERRUPT_MEM6_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_7:
      return DL_ADC12_INTERRUPT_MEM7_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_8:
      return DL_ADC12_INTERRUPT_MEM8_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_9:
      return DL_ADC12_INTERRUPT_MEM9_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_10:
      return DL_ADC12_INTERRUPT_MEM10_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_11:
      return DL_ADC12_INTERRUPT_MEM11_RESULT_LOADED;
    default:
      ASSERT(false);
      return DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED;
  }
}

uint32_t get_seq_start_addr(DL_ADC12_MEM_IDX mem_idx)
{
  switch (mem_idx)
  {
    case DL_ADC12_MEM_IDX_0:
      return DL_ADC12_SEQ_START_ADDR_00;
    case DL_ADC12_MEM_IDX_1:
      return DL_ADC12_SEQ_START_ADDR_01;
    case DL_ADC12_MEM_IDX_2:
      return DL_ADC12_SEQ_START_ADDR_02;
    case DL_ADC12_MEM_IDX_3:
      return DL_ADC12_SEQ_START_ADDR_03;
    case DL_ADC12_MEM_IDX_4:
      return DL_ADC12_SEQ_START_ADDR_04;
    case DL_ADC12_MEM_IDX_5:
      return DL_ADC12_SEQ_START_ADDR_05;
    case DL_ADC12_MEM_IDX_6:
      return DL_ADC12_SEQ_START_ADDR_06;
    case DL_ADC12_MEM_IDX_7:
      return DL_ADC12_SEQ_START_ADDR_07;
    case DL_ADC12_MEM_IDX_8:
      return DL_ADC12_SEQ_START_ADDR_08;
    case DL_ADC12_MEM_IDX_9:
      return DL_ADC12_SEQ_START_ADDR_09;
    case DL_ADC12_MEM_IDX_10:
      return DL_ADC12_SEQ_START_ADDR_10;
    case DL_ADC12_MEM_IDX_11:
      return DL_ADC12_SEQ_START_ADDR_11;
    default:
      ASSERT(false);
      return DL_ADC12_SEQ_START_ADDR_00;
  }
}

uint32_t get_mem_result_dma_trigger_mask(DL_ADC12_MEM_IDX mem_idx)
{
  switch (mem_idx)
  {
    case DL_ADC12_MEM_IDX_0:
      return DL_ADC12_DMA_MEM0_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_1:
      return DL_ADC12_DMA_MEM1_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_2:
      return DL_ADC12_DMA_MEM2_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_3:
      return DL_ADC12_DMA_MEM3_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_4:
      return DL_ADC12_DMA_MEM4_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_5:
      return DL_ADC12_DMA_MEM5_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_6:
      return DL_ADC12_DMA_MEM6_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_7:
      return DL_ADC12_DMA_MEM7_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_8:
      return DL_ADC12_DMA_MEM8_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_9:
      return DL_ADC12_DMA_MEM9_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_10:
      return DL_ADC12_DMA_MEM10_RESULT_LOADED;
    case DL_ADC12_MEM_IDX_11:
      return DL_ADC12_DMA_MEM11_RESULT_LOADED;
    default:
      ASSERT(false);
      return DL_ADC12_DMA_MEM0_RESULT_LOADED;
  }
}

uint32_t get_adcdma_trigger(const ADC12_Regs* instance)
{
  const uintptr_t INSTANCE_ADDR = reinterpret_cast<uintptr_t>(instance);

#if defined(ADC0_BASE) && defined(DMA_ADC0_EVT_GEN_BD_TRIG)
  if (INSTANCE_ADDR == static_cast<uintptr_t>(ADC0_BASE))
  {
    return DMA_ADC0_EVT_GEN_BD_TRIG;
  }
#endif

#if defined(ADC1_BASE) && defined(DMA_ADC1_EVT_GEN_BD_TRIG)
  if (INSTANCE_ADDR == static_cast<uintptr_t>(ADC1_BASE))
  {
    return DMA_ADC1_EVT_GEN_BD_TRIG;
  }
#endif

  return MSPM0_ADC_DMA_TRIGGER_INVALID;
}

uint8_t resolve_dma_channel_id(uint32_t trigger)
{
  if (trigger == MSPM0_ADC_DMA_TRIGGER_INVALID)
  {
    return MSPM0_ADC_DMA_CHANNEL_INVALID;
  }

#if defined(DMA_BASE)
#if defined(DMA_SYS_N_DMA_CHANNEL)
  constexpr uint8_t DMA_CHANNEL_COUNT = static_cast<uint8_t>(DMA_SYS_N_DMA_CHANNEL);
#else
  constexpr uint8_t DMA_CHANNEL_COUNT = 8;
#endif

  for (uint8_t channel = 0; channel < DMA_CHANNEL_COUNT; ++channel)
  {
    if (DL_DMA_getTriggerType(DMA, channel) != DL_DMA_TRIGGER_TYPE_EXTERNAL)
    {
      continue;
    }

    if (DL_DMA_getTrigger(DMA, channel) == trigger)
    {
      return channel;
    }
  }
#endif

  return MSPM0_ADC_DMA_CHANNEL_INVALID;
}

}  // namespace

MSPM0ADC::MSPM0ADC(Resources res)
    : res_(res),
      scale_(0.0f),
      use_dma_(false),
      dma_channel_id_(DMA_CHANNEL_INVALID),
      dma_trigger_(MSPM0_ADC_DMA_TRIGGER_INVALID),
      dma_irq_mask_(0U),
      dma_src_addr_(0U),
      dma_mem_trigger_mask_(0U),
      dma_sample_(0U),
      mem_interrupt_mask_(0U),
      seq_start_addr_(0U)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.vref > 0.0f);
  ASSERT(DL_ADC12_isConversionsEnabled(res_.instance));

  mem_interrupt_mask_ = get_mem_result_interrupt_mask(res_.mem_idx);
  seq_start_addr_ = get_seq_start_addr(res_.mem_idx);
  dma_mem_trigger_mask_ = get_mem_result_dma_trigger_mask(res_.mem_idx);

  const uint32_t RESOLUTION = DL_ADC12_getResolution(res_.instance);
  float full_scale = 4095.0f;
  if (RESOLUTION == DL_ADC12_SAMP_CONV_RES_12_BIT)
  {
    full_scale = 4095.0f;
  }
  else if (RESOLUTION == DL_ADC12_SAMP_CONV_RES_10_BIT)
  {
    full_scale = 1023.0f;
  }
  else if (RESOLUTION == DL_ADC12_SAMP_CONV_RES_8_BIT)
  {
    full_scale = 255.0f;
  }
  scale_ = res_.vref / full_scale;

  DL_ADC12_stopConversion(res_.instance);
  DL_Common_updateReg(&res_.instance->ULLMEM.CTL1,
                      (DL_ADC12_SAMPLING_SOURCE_AUTO | DL_ADC12_TRIG_SRC_SOFTWARE),
                      (ADC12_CTL1_SAMPMODE_MASK | ADC12_CTL1_TRIGSRC_MASK));

  ASSERT(DL_ADC12_getSamplingSource(res_.instance) == DL_ADC12_SAMPLING_SOURCE_AUTO);
  ASSERT(DL_ADC12_getTriggerSource(res_.instance) == DL_ADC12_TRIG_SRC_SOFTWARE);

  DL_ADC12_setStartAddress(res_.instance, seq_start_addr_);
  DL_ADC12_enableInterrupt(res_.instance, mem_interrupt_mask_);
  DL_ADC12_clearInterruptStatus(res_.instance, mem_interrupt_mask_);
  DL_ADC12_enableConversions(res_.instance);

  use_dma_ = DL_ADC12_isDMAEnabled(res_.instance);
  if (!use_dma_)
  {
    return;
  }

#if !defined(DMA_BASE)
  ASSERT(false);
#else
  ASSERT(DL_ADC12_getEnabledDMATrigger(res_.instance, dma_mem_trigger_mask_) != 0U);
  ASSERT(DL_ADC12_getDMASampleCnt(res_.instance) == MSPM0_ADC_DMA_SINGLE_SAMPLE_COUNT);

  dma_trigger_ = get_adcdma_trigger(res_.instance);
  ASSERT(dma_trigger_ != MSPM0_ADC_DMA_TRIGGER_INVALID);

  dma_channel_id_ = resolve_dma_channel_id(dma_trigger_);
  ASSERT(dma_channel_id_ != DMA_CHANNEL_INVALID);

  dma_irq_mask_ = get_dma_interrupt_mask(dma_channel_id_);
  ASSERT(dma_irq_mask_ != 0U);

  ASSERT(DL_DMA_getTriggerType(DMA, dma_channel_id_) == DL_DMA_TRIGGER_TYPE_EXTERNAL);
  ASSERT(DL_DMA_getTrigger(DMA, dma_channel_id_) == dma_trigger_);
  ASSERT(DL_DMA_getTransferMode(DMA, dma_channel_id_) == DL_DMA_SINGLE_TRANSFER_MODE);
  ASSERT(DL_DMA_getExtendedMode(DMA, dma_channel_id_) == DL_DMA_NORMAL_MODE);
  ASSERT(DL_DMA_getSrcIncrement(DMA, dma_channel_id_) == DL_DMA_ADDR_UNCHANGED);
  ASSERT(DL_DMA_getDestIncrement(DMA, dma_channel_id_) == DL_DMA_ADDR_INCREMENT);
  ASSERT(DL_DMA_getSrcWidth(DMA, dma_channel_id_) == DL_DMA_WIDTH_WORD);
  ASSERT(DL_DMA_getDestWidth(DMA, dma_channel_id_) == DL_DMA_WIDTH_WORD);

  DL_DMA_setTrigger(DMA, dma_channel_id_, static_cast<uint8_t>(dma_trigger_),
                    DL_DMA_TRIGGER_TYPE_EXTERNAL);

  const uintptr_t MEMRES_ADDR = reinterpret_cast<uintptr_t>(
      &res_.instance->ULLMEM.MEMRES[static_cast<size_t>(res_.mem_idx)]);
  dma_src_addr_ = static_cast<uint32_t>(MEMRES_ADDR);

  DL_DMA_setSrcAddr(DMA, dma_channel_id_, dma_src_addr_);
  ASSERT(DL_DMA_getSrcAddr(DMA, dma_channel_id_) == dma_src_addr_);

  DL_DMA_setDestAddr(DMA, dma_channel_id_, reinterpret_cast<uint32_t>(&dma_sample_));
  DL_DMA_setTransferSize(DMA, dma_channel_id_, 1U);
  DL_DMA_clearInterruptStatus(DMA, dma_irq_mask_);
  DL_ADC12_clearDMATriggerStatus(res_.instance, dma_mem_trigger_mask_);
#endif
}

float MSPM0ADC::Read()
{
  if (use_dma_)
  {
    return ReadByDMA();
  }
  return ReadByPolling();
}

float MSPM0ADC::ReadByPolling()
{
  DL_ADC12_disableDMA(res_.instance);
  DL_ADC12_enableConversions(res_.instance);
  DL_ADC12_clearInterruptStatus(res_.instance, mem_interrupt_mask_);
  DL_ADC12_startConversion(res_.instance);
  while (DL_ADC12_getRawInterruptStatus(res_.instance, mem_interrupt_mask_) == 0U)
  {
  }
  const uint16_t raw = DL_ADC12_getMemResult(res_.instance, res_.mem_idx);
  DL_ADC12_clearInterruptStatus(res_.instance, mem_interrupt_mask_);
  return static_cast<float>(raw) * scale_;
}

float MSPM0ADC::ReadByDMA()
{
  DL_ADC12_enableConversions(res_.instance);
  DL_ADC12_clearInterruptStatus(res_.instance, DL_ADC12_INTERRUPT_DMA_DONE);
  DL_ADC12_clearDMATriggerStatus(res_.instance, dma_mem_trigger_mask_);

  DL_ADC12_setDMASamplesCnt(res_.instance, MSPM0_ADC_DMA_SINGLE_SAMPLE_COUNT);
  DL_ADC12_enableDMA(res_.instance);

  DL_DMA_disableChannel(DMA, dma_channel_id_);
  DL_DMA_setDestAddr(DMA, dma_channel_id_, reinterpret_cast<uint32_t>(&dma_sample_));
  DL_DMA_setTransferSize(DMA, dma_channel_id_, 1U);
  DL_DMA_clearInterruptStatus(DMA, dma_irq_mask_);
  DL_DMA_enableChannel(DMA, dma_channel_id_);

  DL_ADC12_startConversion(res_.instance);

  while (DL_ADC12_getRawInterruptStatus(res_.instance, DL_ADC12_INTERRUPT_DMA_DONE) == 0U)
  {
  }

  const uint16_t raw = static_cast<uint16_t>(dma_sample_ & 0xFFFFU);
  return static_cast<float>(raw) * scale_;
}
