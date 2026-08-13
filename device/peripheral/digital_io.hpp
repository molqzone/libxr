#pragma once

#include <cstdint>

#include "driver/gpio.hpp"
#include "process_data.hpp"

namespace LibXR::EtherCAT
{

/** Map one GPIO input to a bit in a TxPDO byte. */
class DigitalInput final : public ProcessData
{
 public:
  DigitalInput(GPIO& gpio, uint8_t& pdo_value, uint8_t mask, bool active_high = true)
      : ProcessData(Direction::INPUT),
        gpio_(gpio),
        pdo_value_(pdo_value),
        mask_(mask),
        active_high_(active_high)
  {
  }

  void UpdateInput(bool) override
  {
    const bool asserted = gpio_.Read() == active_high_;
    if (asserted)
    {
      pdo_value_ |= mask_;
    }
    else
    {
      pdo_value_ &= static_cast<uint8_t>(~mask_);
    }
  }

 private:
  GPIO& gpio_;
  uint8_t& pdo_value_;
  uint8_t mask_;
  bool active_high_;
};

/** Map a bit in an RxPDO byte to one GPIO output. */
class DigitalOutput final : public ProcessData
{
 public:
  DigitalOutput(GPIO& gpio, const uint8_t& pdo_value, uint8_t mask, bool active_high = true)
      : ProcessData(Direction::OUTPUT),
        gpio_(gpio),
        pdo_value_(pdo_value),
        mask_(mask),
        active_high_(active_high)
  {
  }

  void UpdateOutput(bool) override
  {
    const bool asserted = (pdo_value_ & mask_) != 0U;
    gpio_.Write(asserted == active_high_);
  }

  void ApplySafeOutput(bool) override { gpio_.Write(!active_high_); }

 private:
  GPIO& gpio_;
  const uint8_t& pdo_value_;
  uint8_t mask_;
  bool active_high_;
};

}  // namespace LibXR::EtherCAT
