#include "linux_spi.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "libxr_def.hpp"

namespace LibXR
{

LinuxSPI::LinuxSPI(const char *device, RawData rx_buffer, RawData tx_buffer,
                   Configuration config)
    : SPI(rx_buffer, tx_buffer), device_(device == nullptr ? "" : device)
{
  if (device_.empty())
  {
    return;
  }

  fd_ = open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0)
  {
    return;
  }

  /* The device tree ceiling is what SPI_IOC_WR_MAX_SPEED_HZ will accept, so it is
   * also the source clock for the prescaler maths. */
  uint32_t max_speed = 0u;
  if (ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed) < 0 || max_speed == 0u)
  {
    max_speed = 0u;
  }
  max_bus_speed_ = max_speed;

  /* 8 bits per word is the only width the protocol uses; keep it explicit rather
   * than inheriting whatever the device node defaults to. */
  uint8_t bits = 8u;
  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
  {
    close(fd_);
    fd_ = -1;
    return;
  }

  if (SetConfig(config) != ErrorCode::OK)
  {
    close(fd_);
    fd_ = -1;
  }
}

ErrorCode LinuxSPI::SetConfig(Configuration config)
{
  if (fd_ < 0)
  {
    return ErrorCode::INIT_ERR;
  }

  /* Mode = (CPOL << 1) | CPHA.  The SG200X transport used CPOL = LOW with
   * CPHA = EDGE_2, i.e. SPI mode 1. */
  mode_ = static_cast<uint8_t>((static_cast<uint8_t>(config.clock_polarity) << 1) |
                               static_cast<uint8_t>(config.clock_phase));
  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0)
  {
    return ErrorCode::FAILED;
  }

  uint32_t speed = max_bus_speed_;
  if (config.prescaler != Prescaler::UNKNOWN)
  {
    const uint32_t div = PrescalerToDiv(config.prescaler);
    if (div == 0u)
    {
      return ErrorCode::ARG_ERR;
    }
    speed = max_bus_speed_ / div;
  }
  if (speed == 0u)
  {
    /* The prescaler divided the ceiling down to nothing; fall back to it. */
    speed = max_bus_speed_;
  }

  bus_speed_ = speed;
  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &bus_speed_) < 0)
  {
    return ErrorCode::FAILED;
  }

  GetConfig() = config;
  return ErrorCode::OK;
}

ErrorCode LinuxSPI::ReadAndWrite(RawData read_data, ConstRawData write_data,
                                 OperationRW &op, bool in_isr)
{
  ErrorCode result = ErrorCode::OK;

  if (fd_ < 0)
  {
    result = ErrorCode::INIT_ERR;
  }
  else if (in_isr)
  {
    /* spidev is a blocking ioctl; it cannot be driven from an ISR. */
    result = ErrorCode::NOT_SUPPORT;
  }
  else
  {
    const bool has_tx = write_data.addr_ != nullptr && write_data.size_ != 0u;
    const bool has_rx = read_data.addr_ != nullptr && read_data.size_ != 0u;

    if (!has_tx && !has_rx)
    {
      result = ErrorCode::ARG_ERR;
    }
    else if (has_tx && has_rx && read_data.size_ != write_data.size_)
    {
      /* SPI_IOC_MESSAGE exchanges equal lengths; a mismatch would silently
       * truncate one direction. */
      result = ErrorCode::SIZE_ERR;
    }
    else
    {
      struct spi_ioc_transfer transfer;
      memset(&transfer, 0, sizeof(transfer));
      transfer.tx_buf = has_tx ? reinterpret_cast<unsigned long>(write_data.addr_) : 0ul;
      transfer.rx_buf = has_rx ? reinterpret_cast<unsigned long>(read_data.addr_) : 0ul;
      transfer.len = static_cast<uint32_t>(has_tx ? write_data.size_ : read_data.size_);
      transfer.speed_hz = bus_speed_;
      transfer.bits_per_word = 8u;
      transfer.delay_usecs = 0u;
      transfer.cs_change = 0u;

      const int rc = ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer);
      result = (rc >= 1) ? ErrorCode::OK : ErrorCode::FAILED;
    }
  }

  op.UpdateStatus(in_isr, result);
  return result;
}

ErrorCode LinuxSPI::Transfer(size_t size, OperationRW &op, bool in_isr)
{
  RawData rx = GetRxBuffer();
  RawData tx = GetTxBuffer();
  if (size > rx.size_ || size > tx.size_)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  return ReadAndWrite(RawData(rx.addr_, size), ConstRawData(tx.addr_, size), op, in_isr);
}

ErrorCode LinuxSPI::MemWrite(uint16_t reg, ConstRawData write_data, OperationRW &op,
                             bool in_isr)
{
  if (reg > 0xFFu)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  if (write_data.size_ != 0u && write_data.addr_ == nullptr)
  {
    op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    return ErrorCode::ARG_ERR;
  }

  const size_t total = 1u + write_data.size_;
  RawData staging = GetTxBuffer();
  if (total > staging.size_)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }

  uint8_t *const staged = static_cast<uint8_t *>(staging.addr_);
  staged[0] = static_cast<uint8_t>(reg & 0x7Fu);
  if (write_data.size_ != 0u)
  {
    memcpy(staged + 1u, write_data.addr_, write_data.size_);
  }

  /* Transmit-only: the command and payload stay in one chip-select window. */
  return ReadAndWrite(RawData(nullptr, 0u), ConstRawData(staged, total), op, in_isr);
}

ErrorCode LinuxSPI::MemRead(uint16_t reg, RawData read_data, OperationRW &op, bool in_isr)
{
  if (reg > 0xFFu)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }
  if (read_data.size_ != 0u && read_data.addr_ == nullptr)
  {
    op.UpdateStatus(in_isr, ErrorCode::ARG_ERR);
    return ErrorCode::ARG_ERR;
  }
  if (read_data.size_ == 0u)
  {
    op.UpdateStatus(in_isr, ErrorCode::OK);
    return ErrorCode::OK;
  }

  const size_t total = 1u + read_data.size_;
  RawData staging_tx = GetTxBuffer();
  RawData staging_rx = GetRxBuffer();
  if (total > staging_tx.size_ || total > staging_rx.size_)
  {
    op.UpdateStatus(in_isr, ErrorCode::SIZE_ERR);
    return ErrorCode::SIZE_ERR;
  }

  uint8_t *const tx = static_cast<uint8_t *>(staging_tx.addr_);
  tx[0] = static_cast<uint8_t>(reg | 0x80u);
  memset(tx + 1u, 0, read_data.size_);

  /* One duplex transaction: the command byte goes out while the reply is clocked
   * in, so the slave keeps its command state across the whole read. */
  const ErrorCode result = ReadAndWrite(RawData(staging_rx.addr_, total),
                                        ConstRawData(tx, total), op, in_isr);
  if (result == ErrorCode::OK)
  {
    memcpy(read_data.addr_, static_cast<uint8_t *>(staging_rx.addr_) + 1u,
           read_data.size_);
  }
  return result;
}

}  // namespace LibXR
