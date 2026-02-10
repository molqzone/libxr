#include "mspm0_adc.hpp"

using namespace LibXR;

MSPM0ADC::MSPM0ADC(Resources res) : res_(res), scale_(0.0f)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.vref > 0.0f);

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
}

float MSPM0ADC::Read()
{
  DL_ADC12_clearInterruptStatus(res_.instance, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
  DL_ADC12_startConversion(res_.instance);

  uint32_t timeout = WAIT_TIMEOUT;
  while (timeout-- > 0)
  {
    if (DL_ADC12_getRawInterruptStatus(res_.instance,
                                       DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) != 0U)
    {
      const uint16_t RAW = DL_ADC12_getMemResult(res_.instance, res_.mem_idx);
      DL_ADC12_clearInterruptStatus(res_.instance,
                                    DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
      return static_cast<float>(RAW) * scale_;
    }
  }

  ASSERT(false);
  return 0.0f;
}

