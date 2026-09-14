#pragma once

#include <cstdint>
#include <string>

#include "spi.hpp"

namespace LibXR
{

/**
 * @class LinuxSPI
 * @brief 基于 spidev 的 Linux SPI 后端 / Linux SPI backend over spidev.
 *
 * 通过 `/dev/spidevX.Y` 的 ioctl 接口实现 `LibXR::SPI`。传输是同步的：`ioctl`
 * 返回时数据已交换完毕，因此没有中断或 DMA 完成回调，也不需要 cache 维护
 * （内核负责 dma_map/unmap，映射一致性由内核保证）。
 *
 * Backed by the `/dev/spidevX.Y` ioctl interface.  Transfers are synchronous --
 * `ioctl` returns once the bytes have been exchanged -- so there is no interrupt
 * or DMA completion path and no cache maintenance to do.
 *
 * 分频语义与框架一致：`实际速率 = GetMaxBusSpeed() / PrescalerToDiv(prescaler)`。
 * 这里 `GetMaxBusSpeed()` 取设备树中 `spi-max-frequency` 给出的上限，所以
 * `Prescaler::UNKNOWN` 表示"用上限"，`DIV_1` 也表示上限，`DIV_2` 表示一半，以此
 * 类推。这与 SG200X 后端把 ×2 折进 `GetMaxBusSpeed()` 的做法得到同一套结果。
 *
 * Prescaler semantics follow the framework: the resulting speed is
 * `GetMaxBusSpeed() / PrescalerToDiv(prescaler)`, with `GetMaxBusSpeed()` taken
 * from the device tree `spi-max-frequency` ceiling.
 */
class LinuxSPI : public SPI
{
 public:
  /**
   * @brief 打开一个 spidev 设备并应用配置 / Opens a spidev device and applies config.
   * @param device 设备节点路径，例如 "/dev/spidev0.0"。Device node, e.g. "/dev/spidev0.0".
   * @param rx_buffer 内部接收缓冲区 / Internal receive buffer.
   * @param tx_buffer 内部发送缓冲区 / Internal transmit buffer.
   * @param config 初始配置 / Initial configuration.
   */
  LinuxSPI(const char *device, RawData rx_buffer, RawData tx_buffer, Configuration config);

  /* 有意不声明析构：LibXR 的驱动按 static 生命周期使用，SPI 基类也没有虚析构，
   * 而这个类只会在进程生命周期内构造一次，fd 由进程退出回收。
   * No destructor on purpose: LibXR drivers are static-lifetime objects, the SPI
   * base has no virtual destructor, and this one is constructed once per process
   * so the fd is reclaimed at exit. */

  LinuxSPI(const LinuxSPI &) = delete;
  LinuxSPI &operator=(const LinuxSPI &) = delete;

  /**
   * @brief 全双工/半双工传输 / Full- or half-duplex transfer.
   *
   * 同步执行：返回时数据已就绪。全双工要求读写长度相等（spidev 的
   * `SPI_IOC_MESSAGE` 就是等长交换）。`in_isr` 为真时返回 `NOT_SUPPORT`。
   *
   * Runs synchronously; full duplex requires equal read and write lengths, which
   * is what `SPI_IOC_MESSAGE` performs.  `in_isr` yields `NOT_SUPPORT`.
   */
  ErrorCode ReadAndWrite(RawData read_data, ConstRawData write_data, OperationRW &op,
                         bool in_isr = false) override;

  /**
   * @brief 用内部收发缓冲区做一次等长全双工传输 / Equal-length duplex transfer using
   * the internal buffers.
   */
  ErrorCode Transfer(size_t size, OperationRW &op, bool in_isr = false) override;

  /**
   * @brief 寄存器写 / Register write.
   *
   * 与 SG200X 后端同一套约定：先发一个命令字节 `reg & 0x7F`，再发数据，整段保持在
   * 同一次片选内。
   *
   * Same convention as the SG200X backend: one command byte `reg & 0x7F` followed
   * by the payload, all inside a single chip-select window.
   */
  ErrorCode MemWrite(uint16_t reg, ConstRawData write_data, OperationRW &op,
                     bool in_isr = false) override;

  /**
   * @brief 寄存器读 / Register read.
   *
   * 命令字节 `reg | 0x80`，其后是读出的数据；命令与数据必须在同一次片选内完成，
   * 否则从设备会复位它的命令状态机。用内部缓冲区中转，再把数据拷回调用方。
   *
   * Command byte `reg | 0x80` followed by the payload, kept in one chip-select
   * window so the slave does not reset its command state machine.  Staged through
   * the internal buffers.
   */
  ErrorCode MemRead(uint16_t reg, RawData read_data, OperationRW &op,
                    bool in_isr = false) override;

  ErrorCode SetConfig(Configuration config) override;

  uint32_t GetMaxBusSpeed() const override { return max_bus_speed_; }

  Prescaler GetMaxPrescaler() const override { return Prescaler::DIV_16384; }

  /// @brief 设备是否可用 / Whether the device is usable.
  [[nodiscard]] bool IsValid() const { return fd_ >= 0; }

  /// @brief 当前请求的总线速率（Hz）/ Current requested bus speed in Hz.
  [[nodiscard]] uint32_t GetBusSpeed() const { return bus_speed_; }

  /// @brief 设备节点路径 / Device node path.
  [[nodiscard]] const char *GetDevice() const { return device_.c_str(); }

 private:
  std::string device_;
  int fd_ = -1;
  uint32_t max_bus_speed_ = 0u;
  uint32_t bus_speed_ = 0u;
  uint8_t mode_ = 0u;
};

}  // namespace LibXR
