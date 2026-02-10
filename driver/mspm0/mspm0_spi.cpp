#include "mspm0_spi.hpp"

#include <cstring>

using namespace LibXR;

MSPM0SPI* MSPM0SPI::instance_map_[MAX_SPI_INSTANCES] = {nullptr};

MSPM0SPI::MSPM0SPI(Resources res, RawData dma_rx_buffer, RawData dma_tx_buffer,
                   uint32_t dma_enable_min_size, SPI::Configuration config)
    : SPI(dma_rx_buffer, dma_tx_buffer),
      res_(res),
      dma_enable_min_size_(dma_enable_min_size)
{
  ASSERT(res_.instance != nullptr);
  ASSERT(res_.clock_freq > 0);
  ASSERT(res_.index < MAX_SPI_INSTANCES);
  ASSERT(instance_map_[res_.index] == nullptr);
  ASSERT(dma_rx_buffer.addr_ != nullptr);
  ASSERT(dma_tx_buffer.addr_ != nullptr);
  ASSERT(dma_rx_buffer.size_ > 0);
  ASSERT(dma_tx_buffer.size_ > 0);

  instance_map_[res_.index] = this;

  NVIC_ClearPendingIRQ(res_.irqn);
  NVIC_EnableIRQ(res_.irqn);

  const ErrorCode SET_CFG_ANS = SetConfig(config);
  ASSERT(SET_CFG_ANS == ErrorCode::OK);
}

ErrorCode MSPM0SPI::SetConfig(SPI::Configuration config)
{
  DL_SPI_FRAME_FORMAT frame_format = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0;
  if (config.clock_polarity == ClockPolarity::LOW)
  {
    frame_format = (config.clock_phase == ClockPhase::EDGE_1)
                       ? DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0
                       : DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA1;
  }
  else
  {
    frame_format = (config.clock_phase == ClockPhase::EDGE_1)
                       ? DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA0
                       : DL_SPI_FRAME_FORMAT_MOTO4_POL1_PHA1;
  }

  const uint32_t DIV = SPI::PrescalerToDiv(config.prescaler);
  if (DIV < 2 || DIV > 512 || (DIV & 0x1) != 0)
  {
    return ErrorCode::NOT_SUPPORT;
  }

  const uint32_t SCR = (DIV >> 1) - 1;

  DL_SPI_disable(res_.instance);
  DL_SPI_setFrameFormat(res_.instance, frame_format);
  DL_SPI_setBitRateSerialClockDivider(res_.instance, SCR);
  DL_SPI_enable(res_.instance);

  GetConfig() = config;
  return ErrorCode::OK;
}

uint32_t MSPM0SPI::GetMaxBusSpeed() const { return res_.clock_freq; }

SPI::Prescaler MSPM0SPI::GetMaxPrescaler() const { return SPI::Prescaler::DIV_512; }

ErrorCode MSPM0SPI::PollingTransfer(uint8_t* rx, const uint8_t* tx, uint32_t len)
{
  if (len == 0)
  {
    return ErrorCode::OK;
  }

  for (uint32_t i = 0; i < len; ++i)
  {
    while (DL_SPI_isTXFIFOFull(res_.instance))
    {
    }
    const uint8_t TX_BYTE = (tx == nullptr) ? 0 : tx[i];
    DL_SPI_transmitData8(res_.instance, TX_BYTE);

    uint8_t rx_byte = 0;
    while (!DL_SPI_receiveDataCheck8(res_.instance, &rx_byte))
    {
    }

    if (rx != nullptr)
    {
      rx[i] = rx_byte;
    }
  }

  while (DL_SPI_isBusy(res_.instance))
  {
  }

  return ErrorCode::OK;
}

bool MSPM0SPI::DmaBusy() const
{
  if (busy_)
  {
    return true;
  }

  return DL_DMA_isChannelEnabled(DMA, res_.dma_rx_channel) ||
         DL_DMA_isChannelEnabled(DMA, res_.dma_tx_channel);
}

void MSPM0SPI::StartDmaDuplex(uint32_t count)
{
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  DL_DMA_setSrcAddr(DMA, res_.dma_rx_channel,
                    reinterpret_cast<uint32_t>(&res_.instance->RXDATA));
  DL_DMA_setDestAddr(DMA, res_.dma_rx_channel, reinterpret_cast<uint32_t>(rx.addr_));
  DL_DMA_setTransferSize(DMA, res_.dma_rx_channel, count);

  DL_DMA_setSrcAddr(DMA, res_.dma_tx_channel, reinterpret_cast<uint32_t>(tx.addr_));
  DL_DMA_setDestAddr(DMA, res_.dma_tx_channel,
                     reinterpret_cast<uint32_t>(&res_.instance->TXDATA));
  DL_DMA_setTransferSize(DMA, res_.dma_tx_channel, count);

  DL_DMA_enableChannel(DMA, res_.dma_rx_channel);
  DL_DMA_enableChannel(DMA, res_.dma_tx_channel);
}

void MSPM0SPI::StopDma()
{
  DL_DMA_disableChannel(DMA, res_.dma_tx_channel);
  DL_DMA_disableChannel(DMA, res_.dma_rx_channel);
}

ErrorCode MSPM0SPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                 OperationRW& op)
{
  const uint32_t NEED = static_cast<uint32_t>(max(read_data.size_, write_data.size_));

  if (NEED == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  ASSERT(rx.size_ >= NEED);
  ASSERT(tx.size_ >= NEED);

  auto* tx_bytes = static_cast<uint8_t*>(tx.addr_);
  if (write_data.size_ > 0)
  {
    memcpy(tx_bytes, write_data.addr_, write_data.size_);
  }
  if (write_data.size_ < NEED)
  {
    memset(tx_bytes + write_data.size_, 0, NEED - write_data.size_);
  }

  if (NEED > dma_enable_min_size_)
  {
    mem_read_ = false;
    read_buff_ = read_data;
    rw_op_ = op;
    busy_ = true;

    StartDmaDuplex(NEED);

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  ErrorCode ans = PollingTransfer(static_cast<uint8_t*>(rx.addr_), tx_bytes, NEED);

  if (ans == ErrorCode::OK && read_data.size_ > 0)
  {
    memcpy(read_data.addr_, rx.addr_, read_data.size_);
  }

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }

  return ans;
}

ErrorCode MSPM0SPI::Transfer(size_t size, OperationRW& op)
{
  if (size == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  ASSERT(rx.size_ >= size);
  ASSERT(tx.size_ >= size);

  if (size > dma_enable_min_size_)
  {
    mem_read_ = false;
    read_buff_ = {nullptr, 0};
    rw_op_ = op;
    busy_ = true;

    StartDmaDuplex(static_cast<uint32_t>(size));

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  ErrorCode ans =
      PollingTransfer(static_cast<uint8_t*>(rx.addr_),
                      static_cast<const uint8_t*>(tx.addr_), static_cast<uint32_t>(size));

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }

  return ans;
}

ErrorCode MSPM0SPI::MemRead(uint16_t reg, RawData read_data, OperationRW& op)
{
  const uint32_t NEED_READ = static_cast<uint32_t>(read_data.size_);
  if (NEED_READ == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();

  ASSERT(rx.size_ >= (NEED_READ + 1));
  ASSERT(tx.size_ >= (NEED_READ + 1));

  auto* tx_bytes = static_cast<uint8_t*>(tx.addr_);
  tx_bytes[0] = static_cast<uint8_t>(reg | 0x80);
  memset(tx_bytes + 1, 0, NEED_READ);

  const uint32_t TOTAL = NEED_READ + 1;

  if (TOTAL > dma_enable_min_size_)
  {
    mem_read_ = true;
    read_buff_ = read_data;
    rw_op_ = op;
    busy_ = true;

    StartDmaDuplex(TOTAL);

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  ErrorCode ans = PollingTransfer(static_cast<uint8_t*>(rx.addr_), tx_bytes, TOTAL);

  if (ans == ErrorCode::OK)
  {
    auto* rx_bytes = static_cast<uint8_t*>(rx.addr_);
    memcpy(read_data.addr_, rx_bytes + 1, NEED_READ);
  }

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }

  return ans;
}

ErrorCode MSPM0SPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW& op)
{
  const uint32_t NEED_WRITE = static_cast<uint32_t>(write_data.size_);
  if (NEED_WRITE == 0)
  {
    if (op.type != OperationRW::OperationType::BLOCK)
    {
      op.UpdateStatus(false, ErrorCode::OK);
    }
    return ErrorCode::OK;
  }

  if (DmaBusy())
  {
    return ErrorCode::BUSY;
  }

  RawData tx = GetTxBuffer();
  ASSERT(tx.size_ >= (NEED_WRITE + 1));

  auto* tx_bytes = static_cast<uint8_t*>(tx.addr_);
  tx_bytes[0] = static_cast<uint8_t>(reg & 0x7F);
  memcpy(tx_bytes + 1, write_data.addr_, NEED_WRITE);

  const uint32_t TOTAL = NEED_WRITE + 1;

  if (TOTAL > dma_enable_min_size_)
  {
    mem_read_ = false;
    read_buff_ = {nullptr, 0};
    rw_op_ = op;
    busy_ = true;

    StartDmaDuplex(TOTAL);

    op.MarkAsRunning();
    if (op.type == OperationRW::OperationType::BLOCK)
    {
      return op.data.sem_info.sem->Wait(op.data.sem_info.timeout);
    }
    return ErrorCode::OK;
  }

  RawData rx = GetRxBuffer();
  ErrorCode ans = PollingTransfer(static_cast<uint8_t*>(rx.addr_), tx_bytes, TOTAL);

  SwitchBuffer();

  if (op.type != OperationRW::OperationType::BLOCK)
  {
    op.UpdateStatus(false, ans);
  }

  return ans;
}

void MSPM0SPI::OnInterrupt(uint8_t index)
{
  if (index >= MAX_SPI_INSTANCES)
  {
    return;
  }

  MSPM0SPI* spi = instance_map_[index];
  if (spi == nullptr)
  {
    return;
  }

  spi->HandleInterrupt();
}

void MSPM0SPI::HandleInterrupt()
{
  switch (DL_SPI_getPendingInterrupt(res_.instance))
  {
    case DL_SPI_IIDX_DMA_DONE_RX:
    {
      StopDma();

      if (read_buff_.size_ > 0)
      {
        RawData rx = GetRxBuffer();
        auto* rx_bytes = static_cast<uint8_t*>(rx.addr_);
        if (mem_read_)
        {
          memcpy(read_buff_.addr_, rx_bytes + 1, read_buff_.size_);
        }
        else
        {
          memcpy(read_buff_.addr_, rx_bytes, read_buff_.size_);
        }
        read_buff_.size_ = 0;
      }

      SwitchBuffer();
      busy_ = false;
      rw_op_.UpdateStatus(true, ErrorCode::OK);
      break;
    }

    case DL_SPI_IIDX_DMA_DONE_TX:
      break;

    case DL_SPI_IIDX_TX_UNDERFLOW:
    case DL_SPI_IIDX_RX_OVERFLOW:
    case DL_SPI_IIDX_RX_TIMEOUT:
    case DL_SPI_IIDX_PARITY_ERROR:
      StopDma();
      busy_ = false;
      rw_op_.UpdateStatus(true, ErrorCode::FAILED);
      break;

    default:
      break;
  }
}

#if defined(SPI0_BASE)
extern "C" void SPI0_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0SPI::OnInterrupt(0);
}
#endif

#if defined(SPI1_BASE)
extern "C" void SPI1_IRQHandler(void)  // NOLINT
{
  LibXR::MSPM0SPI::OnInterrupt(1);
}
#endif
