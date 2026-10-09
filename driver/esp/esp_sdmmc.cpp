#include "esp_sdmmc.hpp"

#include <cstring>

#include "esp_err.h"

namespace LibXR
{

ESP32SDMMCCard::ESP32SDMMCCard(ESP32SDMMCHost& host) : host_(host)
{
  const esp_err_t init_err = sdmmc_card_init(host_.GetIdfHost(), &card_);
  if (init_err != ESP_OK)
  {
    // 空槽/无响应：可移动介质，不 fail-fast。
    present_ = false;
    init_error_ = init_err;
    return;
  }
  present_ = true;
  init_error_ = ESP_OK;

  info_.mfg_id = static_cast<uint8_t>(card_.cid.mfg_id);
  std::memcpy(info_.product_name, card_.cid.name, 5);
  info_.product_name[5] = '\0';
  info_.serial = static_cast<uint32_t>(card_.cid.serial);

  // CID date 字段：[19:12] 年（以 2000 为基），[11:8] 月。
  info_.date_year = static_cast<uint16_t>(2000 + ((card_.cid.date >> 12) & 0xFF));
  info_.date_month = static_cast<uint16_t>((card_.cid.date >> 8) & 0x0F);

  // CSD v1/v2/v3 对应 SD spec 1.x (SDSC) / 2.x (SDHC|SDXC) / 3.x (SDUC)。
  info_.spec_version = static_cast<uint32_t>(card_.csd.csd_ver);
  info_.sector_size = static_cast<uint32_t>(card_.csd.sector_size);
  info_.capacity_bytes = static_cast<uint64_t>(card_.csd.capacity) * info_.sector_size;
  info_.max_clock_khz = card_.max_freq_khz;
  info_.bus_width =
      card_.log_bus_width <= 3 ? static_cast<uint8_t>(1U << card_.log_bus_width) : 1U;
}

ErrorCode ESP32SDMMCCard::ReadSectors(uint32_t lba, size_t count, RawData buf)
{
  if (!present_)
  {
    return ErrorCode::NOT_FOUND;
  }
  if (buf.addr_ == nullptr || count == 0U || buf.size_ < count * info_.sector_size)
  {
    return ErrorCode::ARG_ERR;
  }

  if (sdmmc_read_sectors(&card_, buf.addr_, lba, count) != ESP_OK)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode ESP32SDMMCCard::WriteSectors(uint32_t lba, size_t count, ConstRawData buf)
{
  if (!present_)
  {
    return ErrorCode::NOT_FOUND;
  }
  if (buf.addr_ == nullptr || count == 0U || buf.size_ < count * info_.sector_size)
  {
    return ErrorCode::ARG_ERR;
  }

  if (sdmmc_write_sectors(&card_, buf.addr_, lba, count) != ESP_OK)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

ErrorCode ESP32SDMMCCard::EraseSectors(uint32_t lba_start, size_t count)
{
  if (!present_)
  {
    return ErrorCode::NOT_FOUND;
  }
  if (count == 0U)
  {
    return ErrorCode::ARG_ERR;
  }

  if (sdmmc_erase_sectors(&card_, lba_start, count, SDMMC_ERASE_ARG) != ESP_OK)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

bool ESP32SDMMCCard::CanDiscard()
{
  if (!present_)
  {
    return false;
  }
  return sdmmc_can_discard(&card_) == ESP_OK;
}

bool ESP32SDMMCCard::CanTrim()
{
  if (!present_)
  {
    return false;
  }
  return sdmmc_can_trim(&card_) == ESP_OK;
}

}  // namespace LibXR
