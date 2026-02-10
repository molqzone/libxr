#pragma once

#include "dl_i2c.h"
#include "i2c.hpp"
#include "ti_msp_dl_config.h"

namespace LibXR
{

class MSPM0I2C : public I2C
{
 public:
  struct Resources
  {
    I2C_Regs* instance;
    IRQn_Type irqn;
    uint32_t clock_freq;
    uint32_t default_bus_speed_hz;
    uint8_t index;
  };

  MSPM0I2C(Resources res, RawData stage_buffer, uint32_t irq_enable_min_size = 8,
           I2C::Configuration config = {100000});

  ErrorCode Read(uint16_t slave_addr, RawData read_data, ReadOperation& op) override;

  ErrorCode Write(uint16_t slave_addr, ConstRawData write_data, WriteOperation& op) override;

  ErrorCode MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                    ReadOperation& op,
                    MemAddrLength mem_addr_size = MemAddrLength::BYTE_8) override;

  ErrorCode MemWrite(uint16_t slave_addr, uint16_t mem_addr, ConstRawData write_data,
                     WriteOperation& op,
                     MemAddrLength mem_addr_size = MemAddrLength::BYTE_8) override;

  ErrorCode SetConfig(Configuration config) override;

  static void OnInterrupt(uint8_t index);

  static constexpr uint8_t ResolveIndex(IRQn_Type irqn)
  {
    switch (irqn)
    {
#if defined(I2C0_BASE)
      case I2C0_INT_IRQn:
        return 0;
#endif
#if defined(I2C1_BASE)
      case I2C1_INT_IRQn:
        return 1;
#endif
#if defined(I2C2_BASE)
      case I2C2_INT_IRQn:
        return 2;
#endif
#if defined(I2C3_BASE)
      case I2C3_INT_IRQn:
        return 3;
#endif
      default:
        return INVALID_INSTANCE_INDEX;
    }
  }

 private:
  enum class AsyncMode : uint8_t
  {
    NONE,
    TX,
    RX,
  };

  static constexpr uint8_t MAX_I2C_INSTANCES = 4;
  static constexpr uint8_t INVALID_INSTANCE_INDEX = 0xFF;

  ErrorCode WaitControllerIdle() const;

  ErrorCode WaitTransactionDone() const;

  ErrorCode WaitBusIdle() const;

  ErrorCode CheckControllerError() const;

  ErrorCode PollingWrite7(uint16_t addr7, const uint8_t* data, size_t size);

  ErrorCode PollingRead7(uint16_t addr7, uint8_t* data, size_t size);

  ErrorCode StartAsyncRead(uint16_t addr7, RawData read_data, ReadOperation& op);

  ErrorCode StartAsyncWrite(uint16_t addr7, ConstRawData write_data, WriteOperation& op);

  void HandleInterrupt();

  void FinishAsync(ErrorCode code);

  Resources res_;
  RawData stage_buffer_;
  uint32_t irq_enable_min_size_;
  volatile bool busy_ = false;
  AsyncMode async_mode_ = AsyncMode::NONE;
  uint8_t* async_rx_data_ = nullptr;
  const uint8_t* async_tx_data_ = nullptr;
  size_t async_total_ = 0;
  size_t async_progress_ = 0;
  volatile ErrorCode last_async_result_ = ErrorCode::OK;
  ReadOperation read_op_;
  WriteOperation write_op_;

  static MSPM0I2C* instance_map_[MAX_I2C_INSTANCES];
};

#define MSPM0_I2C_INIT(name, stage_addr, stage_size, irq_min_size)                          \
  ::LibXR::MSPM0I2C::Resources{name##_INST, name##_INST_INT_IRQN,                           \
                               static_cast<uint32_t>(CPUCLK_FREQ),                           \
                               static_cast<uint32_t>(name##_BUS_SPEED_HZ),                   \
                               ::LibXR::MSPM0I2C::ResolveIndex(name##_INST_INT_IRQN)},       \
      ::LibXR::RawData{(stage_addr), (stage_size)}, (irq_min_size)

}  // namespace LibXR
