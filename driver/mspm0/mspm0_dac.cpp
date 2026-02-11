#include "mspm0_dac.hpp"

#include <algorithm>

using namespace LibXR;

MSPM0DAC::MSPM0DAC(Resources res, float init_voltage)
    : instance_(res.instance), vref_(res.vref), resolution_(4095)
{
  ASSERT(instance_ != nullptr);
  ASSERT(vref_ > 0.0f);

  const uint32_t RES = instance_->CTL0 & DAC12_CTL0_RES_MASK;
  if (RES == DAC12_CTL0_RES__8BITS)
  {
    resolution_ = 255;
  }
  else
  {
    resolution_ = 4095;
  }

  const ErrorCode WRITE_ANS = Write(init_voltage);
  ASSERT(WRITE_ANS == ErrorCode::OK);

  DL_DAC12_enableOutputPin(instance_);
  DL_DAC12_enable(instance_);
  ASSERT(DL_DAC12_isEnabled(instance_));
}

ErrorCode MSPM0DAC::Write(float voltage)
{
  const float CLAMPED = std::clamp(voltage, 0.0f, vref_);
  const uint32_t CODE =
      static_cast<uint32_t>(CLAMPED / vref_ * static_cast<float>(resolution_));

  if (resolution_ <= 255)
  {
    DL_DAC12_output8(instance_, static_cast<uint8_t>(CODE));
  }
  else
  {
    DL_DAC12_output12(instance_, CODE);
  }

  return ErrorCode::OK;
}
