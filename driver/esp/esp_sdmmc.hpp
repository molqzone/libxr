#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_def.hpp"
#include "esp_sdmmc_host.hpp"
#include "libxr_type.hpp"
#include "sdmmc_cmd.h"

namespace LibXR
{

/**
 * @class ESP32SDMMCCard
 * @brief SD 存储卡管理：枚举、卡片信息与扇区读写（ESP32-P4）。
 *        SD memory card management: enumeration, card info, sector I/O.
 *
 * M1：枚举走 IDF 协议层（`sdmmc_card_init`），卡片信息落入 LibXR 自有结构，
 * 不向上层暴露 IDF 类型。空槽/无响应不 fail-fast（可移动介质），
 * 置 `Present()==false`，对卡访问返回 `NOT_FOUND`（设计文档 §4.2/§7）。
 * M1 enumerates via the IDF protocol layer; card info is stored in a
 * LibXR-owned struct. An empty slot is not a hard failure.
 */
class ESP32SDMMCCard
{
 public:
  /**
   * @struct Info
   * @brief 卡片身份与能力（来自 CID/CSD）/ Card identity and capabilities.
   */
  struct Info
  {
    uint8_t mfg_id = 0;           ///< 厂商 ID / Manufacturer ID
    char product_name[6] = {};    ///< 产品名（CID 前 5 字符）/ Product name
    uint32_t serial = 0;          ///< 序列号 / Serial number
    uint16_t date_year = 0;       ///< 生产年 / Manufacturing year
    uint16_t date_month = 0;      ///< 生产月 / Manufacturing month
    uint32_t spec_version = 0;    ///< SD 规范版本 / SD spec version
    uint64_t capacity_bytes = 0;  ///< 容量（字节）/ Capacity in bytes
    uint32_t sector_size = 512;   ///< 扇区大小（字节）/ Sector size
    uint32_t max_clock_khz = 0;   ///< 协商后最高频率 / Negotiated max clock
    uint8_t bus_width = 1;        ///< 逻辑位宽（1/4/8）/ Logical bus width
  };

  /**
   * @brief 构造并枚举 SD 卡 / Construct and enumerate the SD card.
   * @param host 已初始化的 SD 主机 / An initialized SD host
   *
   * 枚举失败（无卡）不 fail-fast：对象保持可用，`Present()` 为 false。
   * Enumeration failure (no card) is not fatal: the object stays usable.
   */
  explicit ESP32SDMMCCard(ESP32SDMMCHost& host);

  /**
   * @brief 卡在位且已完成枚举 / Card present and enumerated.
   */
  bool Present() const { return present_; }

  /**
   * @brief 获取卡片信息（未在位时内容为零）/ Get card info (zeroed if absent).
   */
  const Info& GetInfo() const { return info_; }

  /**
   * @brief 读扇区 / Read sectors.
   * @param lba 起始逻辑块 / Starting LBA
   * @param count 扇区数 / Sector count
   * @param buf 目标缓冲区（≥ count * sector_size）/ Destination buffer
   */
  ErrorCode ReadSectors(uint32_t lba, size_t count, RawData buf);

  /**
   * @brief 写扇区 / Write sectors.
   * @param lba 起始逻辑块 / Starting LBA
   * @param count 扇区数 / Sector count
   * @param buf 源缓冲区（≥ count * sector_size）/ Source buffer
   */
  ErrorCode WriteSectors(uint32_t lba, size_t count, ConstRawData buf);

  /**
   * @brief 擦除扇区 / Erase sectors.
   * @param lba_start 起始逻辑块 / Starting LBA
   * @param count 扇区数 / Sector count
   */
  ErrorCode EraseSectors(uint32_t lba_start, size_t count);

  /**
   * @brief 是否支持 discard（CMD38 trim）/ Discard (trim) capability.
   * @note IDF 协议层入口要求非 const；故不标 const。
   *       The IDF entry requires a non-const pointer, hence not const.
   */
  bool CanDiscard();

  /**
   * @brief 是否支持 eMMC sanitize / MMC sanitize capability.
   */
  bool CanTrim();

 private:
  ESP32SDMMCHost& host_;
  sdmmc_card_t card_ = {};
  Info info_;
  bool present_ = false;
};

}  // namespace LibXR
