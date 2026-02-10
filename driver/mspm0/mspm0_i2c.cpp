#include "mspm0_i2c.hpp"

#include <cstring>

using namespace LibXR;

namespace
{

constexpr uint32_t MSPM0_I2C_WAIT_IDLE_TIMEOUT = 300000;
constexpr uint32_t MSPM0_I2C_WAIT_BUS_TIMEOUT = 300000;
constexpr uint32_t MSPM0_I2C_WAIT_FIFO_TIMEOUT = 300000;
constexpr uint32_t MSPM0_I2C_MAX_ISR_ROUNDS = 32;

constexpr uint32_t MSPM0_I2C_ASYNC_INTERRUPT_MASK =
    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |
    DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER | DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
    DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | DL_I2C_INTERRUPT_CONTROLLER_NACK |
    DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST;

constexpr uint16_t MSPM0_I2C_MAX_TRANSFER_SIZE = 0x0FFF;

constexpr uint16_t mspm0_i2c_to_addr7(uint16_t slave_addr)
{
  return static_cast<uint16_t>((slave_addr >> 1) & 0x7F);
}

}  // namespace

MSPM0I2C* MSPM0I2C::instance_map_[MAX_I2C_INSTANCES] = {nullptr};

MSPM0I2C::MSPM0I2C(Resources res, RawData stage_buffer, uint32_t irq_enable_min_size,
                   I2C::Configuration config)
    : I2C(),
      res_(res),
      stage_buffer_(stage_buffer),
      irq_enable_min_size_(irq_enable_min_size)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.clock_freq > 0);
  ASSERT(res_.index < MAX_I2C_INSTANCES);
  ASSERT(instance_map_[res_.index] == nullptr);
  ASSERT(stage_buffer_.addr_ != nullptr);
  ASSERT(stage_buffer_.size_ > 0);

  instance_map_[res_.index] = this;

  NVIC_ClearPendingIRQ(res_.irqn);
  NVIC_EnableIRQ(res_.irqn);

  if (config.clock_speed == 0)
  {
    config.clock_speed = res_.default_bus_speed_hz;
  }
  const ErrorCode SET_CFG_ANS = SetConfig(config);
  ASSERT(SET_CFG_ANS == ErrorCode::OK);
}

ErrorCode MSPM0I2C::CheckControllerError() const
{
  const uint32_t STATUS = DL_I2C_getControllerStatus(res_.instance);
  if ((STATUS & DL_I2C_CONTROLLER_STATUS_ERROR) != 0 ||
      (STATUS & DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::WaitControllerIdle() const
{
  uint32_t timeout = MSPM0_I2C_WAIT_IDLE_TIMEOUT;
  while (timeout-- > 0)
  {
    if ((DL_I2C_getControllerStatus(res_.instance) & DL_I2C_CONTROLLER_STATUS_IDLE) != 0)
    {
      return ErrorCode::OK;
    }
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::BUSY;
}

ErrorCode MSPM0I2C::WaitTransactionDone() const
{
  uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
  while (timeout-- > 0)
  {
    if (DL_I2C_getTransactionCount(res_.instance) == 0)
    {
      return ErrorCode::OK;
    }
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::BUSY;
}

ErrorCode MSPM0I2C::WaitBusIdle() const
{
  uint32_t timeout = MSPM0_I2C_WAIT_BUS_TIMEOUT;
  while (timeout-- > 0)
  {
    if ((DL_I2C_getControllerStatus(res_.instance) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) ==
        0)
    {
      return ErrorCode::OK;
    }
    if (CheckControllerError() != ErrorCode::OK)
    {
      return ErrorCode::FAILED;
    }
  }
  return ErrorCode::BUSY;
}

ErrorCode MSPM0I2C::SetConfig(Configuration config)
{
  if (config.clock_speed == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  const uint32_t PERIOD_DEN = config.clock_speed * 10U;
  if (PERIOD_DEN == 0)
  {
    return ErrorCode::ARG_ERR;
  }

  uint32_t period_factor = res_.clock_freq / PERIOD_DEN;
  if (period_factor == 0)
  {
    return ErrorCode::NOT_SUPPORT;
  }
  if (period_factor > 128U)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  const uint8_t TIMER_PERIOD = static_cast<uint8_t>(period_factor - 1U);
  const DL_I2C_ClockConfig CLOCK_CONFIG = {
      .clockSel = DL_I2C_CLOCK_BUSCLK,
      .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
  };

  DL_I2C_disableController(res_.instance);
  DL_I2C_setClockConfig(res_.instance, &CLOCK_CONFIG);
  DL_I2C_resetControllerTransfer(res_.instance);
  DL_I2C_setTimerPeriod(res_.instance, TIMER_PERIOD);
  DL_I2C_setControllerTXFIFOThreshold(res_.instance, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
  DL_I2C_setControllerRXFIFOThreshold(res_.instance, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
  DL_I2C_disableControllerClockStretching(res_.instance);
  DL_I2C_disableInterrupt(res_.instance, 0xFFFFFFFFU);
  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);
  DL_I2C_enableController(res_.instance);

  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::PollingWrite7(uint16_t addr7, const uint8_t* data, size_t size)
{
  if (size == 0)
  {
    return ErrorCode::OK;
  }
  if (size > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  ErrorCode ans = WaitControllerIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

  size_t sent =
      DL_I2C_fillControllerTXFIFO(res_.instance, data, static_cast<uint16_t>(size));
  DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_TX,
                                 static_cast<uint16_t>(size));

  while (sent < size)
  {
    uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
    while (DL_I2C_getRawInterruptStatus(res_.instance,
                                        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER) == 0U)
    {
      if (CheckControllerError() != ErrorCode::OK)
      {
        return ErrorCode::FAILED;
      }
      if (timeout-- == 0)
      {
        return ErrorCode::BUSY;
      }
    }

    sent += DL_I2C_fillControllerTXFIFO(res_.instance, data + sent,
                                        static_cast<uint16_t>(size - sent));
  }

  ans = WaitBusIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  return CheckControllerError();
}

ErrorCode MSPM0I2C::PollingRead7(uint16_t addr7, uint8_t* data, size_t size)
{
  if (size == 0)
  {
    return ErrorCode::OK;
  }
  if (size > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  ErrorCode ans = WaitControllerIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);
  DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_RX,
                                 static_cast<uint16_t>(size));

  size_t received = 0;
  while (received < size)
  {
    uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
    while (DL_I2C_isControllerRXFIFOEmpty(res_.instance))
    {
      if (CheckControllerError() != ErrorCode::OK)
      {
        return ErrorCode::FAILED;
      }
      if (timeout-- == 0)
      {
        return ErrorCode::BUSY;
      }
    }

    while (!DL_I2C_isControllerRXFIFOEmpty(res_.instance) && received < size)
    {
      data[received++] = DL_I2C_receiveControllerData(res_.instance);
    }
  }

  ans = WaitBusIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  return CheckControllerError();
}

ErrorCode MSPM0I2C::StartAsyncWrite(uint16_t addr7, ConstRawData write_data,
                                    WriteOperation& op)
{
  if (write_data.size_ > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }
  if (write_data.size_ > stage_buffer_.size_)
  {
    return ErrorCode::ARG_ERR;
  }

  ErrorCode ans = WaitControllerIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  async_mode_ = AsyncMode::TX;
  memcpy(stage_buffer_.addr_, write_data.addr_, write_data.size_);
  async_tx_data_ = static_cast<const uint8_t*>(stage_buffer_.addr_);
  async_rx_data_ = nullptr;
  async_total_ = write_data.size_;
  async_progress_ = 0;
  last_async_result_ = ErrorCode::BUSY;
  write_op_ = op;
  busy_ = true;

  DL_I2C_disableInterrupt(res_.instance, 0xFFFFFFFFU);
  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

  async_progress_ = DL_I2C_fillControllerTXFIFO(res_.instance, async_tx_data_,
                                                static_cast<uint16_t>(async_total_));

  DL_I2C_enableInterrupt(res_.instance, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |
                                            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                            DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

  DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_TX,
                                 static_cast<uint16_t>(async_total_));

  op.MarkAsRunning();
  if (op.type == WriteOperation::OperationType::BLOCK)
  {
    const ErrorCode WAIT_ANS = op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
    if (WAIT_ANS != ErrorCode::OK)
    {
      return WAIT_ANS;
    }
    return last_async_result_;
  }
  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::StartAsyncRead(uint16_t addr7, RawData read_data, ReadOperation& op)
{
  if (read_data.size_ > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  ErrorCode ans = WaitControllerIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  async_mode_ = AsyncMode::RX;
  async_tx_data_ = nullptr;
  async_rx_data_ = static_cast<uint8_t*>(read_data.addr_);
  async_total_ = read_data.size_;
  async_progress_ = 0;
  last_async_result_ = ErrorCode::BUSY;
  read_op_ = op;
  busy_ = true;

  DL_I2C_disableInterrupt(res_.instance, 0xFFFFFFFFU);
  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);
  DL_I2C_enableInterrupt(res_.instance, DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                                            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                                            DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                            DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

  DL_I2C_startControllerTransfer(res_.instance, addr7, DL_I2C_CONTROLLER_DIRECTION_RX,
                                 static_cast<uint16_t>(async_total_));

  op.MarkAsRunning();
  if (op.type == ReadOperation::OperationType::BLOCK)
  {
    const ErrorCode WAIT_ANS = op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
    if (WAIT_ANS != ErrorCode::OK)
    {
      return WAIT_ANS;
    }
    return last_async_result_;
  }
  return ErrorCode::OK;
}

ErrorCode MSPM0I2C::Read(uint16_t slave_addr, RawData read_data, ReadOperation& op)
{
  if (busy_)
  {
    return ErrorCode::BUSY;
  }

  if (read_data.size_ == 0)
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  const uint16_t ADDR7 = mspm0_i2c_to_addr7(slave_addr);

  if (read_data.size_ > irq_enable_min_size_)
  {
    return StartAsyncRead(ADDR7, read_data, op);
  }

  ErrorCode ans =
      PollingRead7(ADDR7, static_cast<uint8_t*>(read_data.addr_), read_data.size_);
  if (op.type != ReadOperation::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }
  return ans;
}

ErrorCode MSPM0I2C::Write(uint16_t slave_addr, ConstRawData write_data,
                          WriteOperation& op)
{
  if (busy_)
  {
    return ErrorCode::BUSY;
  }

  if (write_data.size_ == 0)
  {
    if (op.type != WriteOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  const uint16_t ADDR7 = mspm0_i2c_to_addr7(slave_addr);

  if (write_data.size_ > irq_enable_min_size_)
  {
    return StartAsyncWrite(ADDR7, write_data, op);
  }

  ErrorCode ans = PollingWrite7(ADDR7, static_cast<const uint8_t*>(write_data.addr_),
                                write_data.size_);
  if (op.type != WriteOperation::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }
  return ans;
}

ErrorCode MSPM0I2C::MemWrite(uint16_t slave_addr, uint16_t mem_addr,
                             ConstRawData write_data, WriteOperation& op,
                             MemAddrLength mem_addr_size)
{
  if (busy_)
  {
    return ErrorCode::BUSY;
  }

  const size_t ADDR_SIZE = (mem_addr_size == MemAddrLength::BYTE_8) ? 1 : 2;
  const size_t TOTAL_SIZE = ADDR_SIZE + write_data.size_;
  if (TOTAL_SIZE > stage_buffer_.size_)
  {
    return ErrorCode::ARG_ERR;
  }

  auto* tx = static_cast<uint8_t*>(stage_buffer_.addr_);
  if (ADDR_SIZE == 1)
  {
    tx[0] = static_cast<uint8_t>(mem_addr & 0xFF);
  }
  else
  {
    tx[0] = static_cast<uint8_t>((mem_addr >> 8) & 0xFF);
    tx[1] = static_cast<uint8_t>(mem_addr & 0xFF);
  }
  if (write_data.size_ > 0)
  {
    memcpy(tx + ADDR_SIZE, write_data.addr_, write_data.size_);
  }

  ErrorCode ans = PollingWrite7(mspm0_i2c_to_addr7(slave_addr), tx, TOTAL_SIZE);

  if (op.type != WriteOperation::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }
  return ans;
}

ErrorCode MSPM0I2C::MemRead(uint16_t slave_addr, uint16_t mem_addr, RawData read_data,
                            ReadOperation& op, MemAddrLength mem_addr_size)
{
  if (busy_)
  {
    return ErrorCode::BUSY;
  }

  if (read_data.size_ > MSPM0_I2C_MAX_TRANSFER_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }
  if (read_data.size_ == 0)
  {
    if (op.type != ReadOperation::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  const uint16_t ADDR7 = mspm0_i2c_to_addr7(slave_addr);
  const size_t ADDR_SIZE = (mem_addr_size == MemAddrLength::BYTE_8) ? 1 : 2;
  auto* addr_bytes = static_cast<uint8_t*>(stage_buffer_.addr_);
  if (stage_buffer_.size_ < ADDR_SIZE)
  {
    return ErrorCode::ARG_ERR;
  }

  if (ADDR_SIZE == 1)
  {
    addr_bytes[0] = static_cast<uint8_t>(mem_addr & 0xFF);
  }
  else
  {
    addr_bytes[0] = static_cast<uint8_t>((mem_addr >> 8) & 0xFF);
    addr_bytes[1] = static_cast<uint8_t>(mem_addr & 0xFF);
  }

  ErrorCode ans = WaitControllerIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  DL_I2C_clearInterruptStatus(res_.instance, 0xFFFFFFFFU);

  const uint16_t ADDR_SENT = DL_I2C_fillControllerTXFIFO(
      res_.instance, addr_bytes, static_cast<uint16_t>(ADDR_SIZE));
  if (ADDR_SENT != ADDR_SIZE)
  {
    return ErrorCode::FAILED;
  }

  DL_I2C_startControllerTransferAdvanced(
      res_.instance, ADDR7, DL_I2C_CONTROLLER_DIRECTION_TX,
      static_cast<uint16_t>(ADDR_SIZE), DL_I2C_CONTROLLER_START_ENABLE,
      DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_ENABLE);

  ans = WaitTransactionDone();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }
  if (CheckControllerError() != ErrorCode::OK)
  {
    return ErrorCode::FAILED;
  }

  DL_I2C_startControllerTransferAdvanced(
      res_.instance, ADDR7, DL_I2C_CONTROLLER_DIRECTION_RX,
      static_cast<uint16_t>(read_data.size_), DL_I2C_CONTROLLER_START_ENABLE,
      DL_I2C_CONTROLLER_STOP_ENABLE, DL_I2C_CONTROLLER_ACK_ENABLE);

  size_t received = 0;
  auto* read_ptr = static_cast<uint8_t*>(read_data.addr_);
  while (received < read_data.size_)
  {
    uint32_t timeout = MSPM0_I2C_WAIT_FIFO_TIMEOUT;
    while (DL_I2C_isControllerRXFIFOEmpty(res_.instance))
    {
      if (CheckControllerError() != ErrorCode::OK)
      {
        return ErrorCode::FAILED;
      }
      if (timeout-- == 0)
      {
        return ErrorCode::BUSY;
      }
    }

    while (!DL_I2C_isControllerRXFIFOEmpty(res_.instance) && received < read_data.size_)
    {
      read_ptr[received++] = DL_I2C_receiveControllerData(res_.instance);
    }
  }

  ans = WaitBusIdle();
  if (ans != ErrorCode::OK)
  {
    return ans;
  }

  ans = CheckControllerError();
  if (op.type != ReadOperation::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }
  return ans;
}

void MSPM0I2C::FinishAsync(ErrorCode code)
{
  const AsyncMode LAST_MODE = async_mode_;
  last_async_result_ = code;

  DL_I2C_disableInterrupt(res_.instance, MSPM0_I2C_ASYNC_INTERRUPT_MASK);
  DL_I2C_clearInterruptStatus(res_.instance, MSPM0_I2C_ASYNC_INTERRUPT_MASK);

  if (code != ErrorCode::OK)
  {
    DL_I2C_resetControllerTransfer(res_.instance);
    DL_I2C_enableController(res_.instance);
  }

  busy_ = false;
  async_mode_ = AsyncMode::NONE;
  async_rx_data_ = nullptr;
  async_tx_data_ = nullptr;
  async_total_ = 0;
  async_progress_ = 0;

  if (LAST_MODE == AsyncMode::RX)
  {
    read_op_.UpdateStatus(true, code);
  }
  else if (LAST_MODE == AsyncMode::TX)
  {
    write_op_.UpdateStatus(true, code);
  }
}

void MSPM0I2C::HandleInterrupt()
{
  for (uint32_t round = 0; round < MSPM0_I2C_MAX_ISR_ROUNDS; ++round)
  {
    const DL_I2C_IIDX IIDX = DL_I2C_getPendingInterrupt(res_.instance);
    if (IIDX == DL_I2C_IIDX_NO_INT)
    {
      return;
    }

    switch (IIDX)
    {
      case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
        if (async_mode_ == AsyncMode::TX)
        {
          async_progress_ += DL_I2C_fillControllerTXFIFO(
              res_.instance, async_tx_data_ + async_progress_,
              static_cast<uint16_t>(async_total_ - async_progress_));
        }
        break;

      case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
        if (async_mode_ == AsyncMode::RX)
        {
          while (!DL_I2C_isControllerRXFIFOEmpty(res_.instance) &&
                 async_progress_ < async_total_)
          {
            async_rx_data_[async_progress_++] =
                DL_I2C_receiveControllerData(res_.instance);
          }
        }
        break;

      case DL_I2C_IIDX_CONTROLLER_TX_DONE:
        if (async_mode_ == AsyncMode::TX && async_progress_ == async_total_)
        {
          FinishAsync(CheckControllerError());
          return;
        }
        FinishAsync(ErrorCode::FAILED);
        return;

      case DL_I2C_IIDX_CONTROLLER_RX_DONE:
        if (async_mode_ == AsyncMode::RX)
        {
          while (!DL_I2C_isControllerRXFIFOEmpty(res_.instance) &&
                 async_progress_ < async_total_)
          {
            async_rx_data_[async_progress_++] =
                DL_I2C_receiveControllerData(res_.instance);
          }

          if (async_progress_ == async_total_)
          {
            FinishAsync(CheckControllerError());
          }
          else
          {
            FinishAsync(ErrorCode::FAILED);
          }
          return;
        }
        FinishAsync(ErrorCode::FAILED);
        return;

      case DL_I2C_IIDX_CONTROLLER_NACK:
      case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        FinishAsync(ErrorCode::FAILED);
        return;

      default:
        break;
    }
  }

  FinishAsync(ErrorCode::FAILED);
}

void MSPM0I2C::OnInterrupt(uint8_t index)
{
  if (index >= MAX_I2C_INSTANCES)
  {
    return;
  }

  MSPM0I2C* i2c = instance_map_[index];
  if (i2c == nullptr)
  {
    return;
  }

  i2c->HandleInterrupt();
}

#if defined(I2C0_BASE)
extern "C" void I2C0_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0I2C::OnInterrupt(0);
}
#endif

#if defined(I2C1_BASE)
extern "C" void I2C1_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0I2C::OnInterrupt(1);
}
#endif

#if defined(I2C2_BASE)
extern "C" void I2C2_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0I2C::OnInterrupt(2);
}
#endif

#if defined(I2C3_BASE)
extern "C" void I2C3_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0I2C::OnInterrupt(3);
}
#endif
