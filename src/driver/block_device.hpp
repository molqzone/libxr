#pragma once

#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"
#include "libxr_type.hpp"

namespace LibXR
{

/**
 * @class BlockDevice
 * @brief 跨平台块设备接口（512 字节扇区语义，与 SPI/I2C 同层的驱动抽象）。
 *        Cross-platform block device interface (512-byte sector semantics).
 *
 * 上层文件系统只依赖本接口，不依赖任何平台 SDK 类型。
 * Filesystems depend on this interface only; no platform SDK types leak.
 *
 * @note 本接口对齐 IDF `esp_blockdev_ops_t` 与 Linux `block_device_operations`
 *       的裁剪语义（read/write/trim/sync 合一）。
 */
class BlockDevice
{
 public:
  virtual ~BlockDevice() = default;

  /**
   * @brief 扇区大小（字节）。FAT 设计假定为 512。
   *        Sector size in bytes. FAT design assumes 512.
   */
  [[nodiscard]] virtual uint32_t SectorSize() const = 0;

  /**
   * @brief 总扇区数 / Total sector count.
   */
  [[nodiscard]] virtual uint64_t SectorCount() const = 0;

  /**
   * @brief 读扇区 / Read sectors.
   * @param lba 起始逻辑块 / Starting LBA
   * @param count 扇区数 / Sector count
   * @param buf 目标缓冲区（≥ count * SectorSize()）/ Destination buffer
   */
  virtual ErrorCode Read(uint64_t lba, uint32_t count, RawData buf) = 0;

  /**
   * @brief 写扇区 / Write sectors.
   * @param lba 起始逻辑块 / Starting LBA
   * @param count 扇区数 / Sector count
   * @param buf 源缓冲区（≥ count * SectorSize()）/ Source buffer
   */
  virtual ErrorCode Write(uint64_t lba, uint32_t count, ConstRawData buf) = 0;

  /**
   * @brief  Trim/discard 提示 / Trim(discard) hint.
   * @param lba 起始逻辑块 / Starting LBA
   * @param count 扇区数 / Sector count
   * @return 不支持返回 NOT_SUPPORT，上层据此禁用 / NOT_SUPPORT if unsupported
   */
  virtual ErrorCode Trim(uint64_t lba, uint32_t count) = 0;

  /**
   * @brief 刷写设备侧缓存 / Flush device-side caches.
   * @return 无缓存实现返回 OK / Return OK when there is no cache
   */
  virtual ErrorCode Sync() = 0;
};

}  // namespace LibXR
