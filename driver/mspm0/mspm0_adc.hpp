#pragma once

#include "adc.hpp"
#include "dl_adc12.h"
#include "ti_msp_dl_config.h"

namespace LibXR
{

class MSPM0ADC : public ADC
{
 public:
  struct Resources
  {
    ADC12_Regs* instance;
    DL_ADC12_MEM_IDX mem_idx;
    float vref;
  };

  explicit MSPM0ADC(Resources res);

  float Read() override;

 private:
  static constexpr uint32_t WAIT_TIMEOUT = 300000;

  Resources res_;
  float scale_;
};

#define MSPM0_ADC_INIT(name)                                                    \
  ::LibXR::MSPM0ADC::Resources{(name##_INST), (name##_ADCMEM_0),               \
                               static_cast<float>(name##_ADCMEM_0_REF_VOLTAGE_V)}

}  // namespace LibXR

