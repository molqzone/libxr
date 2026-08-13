#pragma once

#include "driver/adc.hpp"
#include "process_data.hpp"

namespace LibXR::EtherCAT
{

/** Sample a LibXR ADC into a floating-point TxPDO field. */
class AnalogInput final : public ProcessData
{
 public:
  AnalogInput(ADC& adc, float& pdo_value, float scale = 1.0F, float offset = 0.0F)
      : ProcessData(Direction::INPUT), adc_(adc), pdo_value_(pdo_value), scale_(scale), offset_(offset)
  {
  }

  void UpdateInput(bool) override { pdo_value_ = adc_.Read() * scale_ + offset_; }

 private:
  ADC& adc_;
  float& pdo_value_;
  float scale_;
  float offset_;
};

}  // namespace LibXR::EtherCAT
