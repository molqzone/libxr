#pragma once

#include "process_data.hpp"
#include "driver/pwm.hpp"

namespace LibXR::EtherCAT
{

/** Apply a normalized RxPDO value to a LibXR PWM output. */
class PwmOutput final : public ProcessData
{
 public:
  explicit PwmOutput(PWM& pwm, const float& pdo_duty)
      : ProcessData(Direction::OUTPUT), pwm_(pwm), pdo_duty_(pdo_duty)
  {
  }

  void UpdateOutput(bool) override
  {
    float duty = pdo_duty_;
    if (duty < 0.0F)
    {
      duty = 0.0F;
    }
    else if (duty > 1.0F)
    {
      duty = 1.0F;
    }
    if (pwm_.SetDutyCycle(duty) == ErrorCode::OK)
    {
      (void)pwm_.Enable();
    }
  }

  void ApplySafeOutput(bool) override
  {
    (void)pwm_.SetDutyCycle(0.0F);
    (void)pwm_.Disable();
  }

 private:
  PWM& pwm_;
  const float& pdo_duty_;
};

}  // namespace LibXR::EtherCAT
