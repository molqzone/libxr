#include "esp_sdmmc_host.hpp"

#include <cstring>

#include "driver/gpio.h"
#include "driver/sdmmc_default_configs.h"
#include "esp_err.h"
#include "esp_private/sdmmc_common.h"
#include "libxr_assert.hpp"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

namespace LibXR
{

ESP32SDMMCHost::ESP32SDMMCHost(const Config& config) : config_(config)
{
  host_ = sdmmc_host_t SDMMC_HOST_DEFAULT();

  host_.slot = config.slot;
  host_.max_freq_khz = static_cast<int>(config.freq_khz);

  if (config.ldo_chan_id >= 0)
  {
    sd_pwr_ctrl_ldo_config_t ldo_config = {};
    ldo_config.ldo_chan_id = config.ldo_chan_id;
    ASSERT(sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &host_.pwr_ctrl_handle) == ESP_OK);
  }

  ASSERT(sdmmc_host_init() == ESP_OK);

  sdmmc_slot_config_t slot_config = sdmmc_slot_config_t SDMMC_SLOT_CONFIG_DEFAULT();
  // 显式 4-bit：默认的“最大可能宽度”在 P4 上会解析为 8-bit，而板上只接 D0-D3。
  slot_config.width = 4;
  slot_config.clk = static_cast<gpio_num_t>(config.gpio_clk);
  slot_config.cmd = static_cast<gpio_num_t>(config.gpio_cmd);
  slot_config.d0 = static_cast<gpio_num_t>(config.gpio_d0);
  slot_config.d1 = static_cast<gpio_num_t>(config.gpio_d1);
  slot_config.d2 = static_cast<gpio_num_t>(config.gpio_d2);
  slot_config.d3 = static_cast<gpio_num_t>(config.gpio_d3);
  if (config.gpio_cd >= 0)
  {
    slot_config.cd = static_cast<gpio_num_t>(config.gpio_cd);
  }

  // host/slot 初始化失败按仓库机制 fail-fast（板级接线/参数错误）。
  ASSERT(sdmmc_host_init_slot(host_.slot, &slot_config) == ESP_OK);

  int real_khz = 0;
  if (sdmmc_host_get_real_freq(host_.slot, &real_khz) == ESP_OK)
  {
    real_freq_hz_ = static_cast<uint32_t>(real_khz) * 1000U;
  }
}

ErrorCode ESP32SDMMCHost::SetBusWidth(uint8_t width)
{
  if (sdmmc_host_set_bus_width(host_.slot, width) != ESP_OK)
  {
    return ErrorCode::ARG_ERR;
  }
  return ErrorCode::OK;
}

ErrorCode ESP32SDMMCHost::SetClockHz(uint32_t hz)
{
  if (sdmmc_host_set_card_clk(host_.slot, hz / 1000U) != ESP_OK)
  {
    return ErrorCode::FAILED;
  }

  int real_khz = 0;
  if (sdmmc_host_get_real_freq(host_.slot, &real_khz) == ESP_OK)
  {
    real_freq_hz_ = static_cast<uint32_t>(real_khz) * 1000U;
  }
  return ErrorCode::OK;
}

uint32_t ESP32SDMMCHost::GetRealClockHz() const { return real_freq_hz_; }

ErrorCode ESP32SDMMCHost::Transaction(Command& cmd, Data* data)
{
  sdmmc_command_t idf_cmd = {};
  idf_cmd.opcode = cmd.opcode;
  idf_cmd.arg = cmd.arg;

  switch (cmd.response_type)
  {
    case ResponseType::NONE:
      idf_cmd.flags = SCF_RSP_R0;
      break;
    case ResponseType::R1:
      idf_cmd.flags = SCF_RSP_R1;
      break;
    case ResponseType::R1B:
      idf_cmd.flags = SCF_RSP_R1B;
      break;
    case ResponseType::R2:
      idf_cmd.flags = SCF_RSP_R2;
      break;
    case ResponseType::R3:
      idf_cmd.flags = SCF_RSP_R3;
      break;
    case ResponseType::R6:
      idf_cmd.flags = SCF_RSP_R6;
      break;
    case ResponseType::R7:
      idf_cmd.flags = SCF_RSP_R7;
      break;
    default:
      return ErrorCode::ARG_ERR;
  }

  if (data != nullptr)
  {
    idf_cmd.flags |= SCF_CMD_ADTC;
    if (data->read)
    {
      idf_cmd.flags |= SCF_CMD_READ;
    }
    idf_cmd.data = data->buffer;
    idf_cmd.datalen = data->length;
    idf_cmd.buflen = data->length;
    idf_cmd.blklen = data->block_size;
  }

  // IDF 主机要求非零超时（协议层自己也会兜底为 1000ms）；写命令放宽到 5s。
  idf_cmd.timeout_ms = (data != nullptr && !data->read) ? SDMMC_WRITE_CMD_TIMEOUT_MS
                                                        : SDMMC_DEFAULT_CMD_TIMEOUT_MS;

  const esp_err_t err = sdmmc_host_do_transaction(host_.slot, &idf_cmd);
  if (err == ESP_ERR_TIMEOUT)
  {
    return ErrorCode::TIMEOUT;
  }
  if (err != ESP_OK)
  {
    return ErrorCode::FAILED;
  }

  // v6.1 host 把卡级错误（响应超时/CRC 错等）写在 cmd.error，返回值只表示
  // 事务流程是否走完；协议层同样以 cmd->error 为准。
  if (idf_cmd.error == ESP_ERR_TIMEOUT)
  {
    return ErrorCode::TIMEOUT;
  }
  if (idf_cmd.error != ESP_OK)
  {
    return ErrorCode::FAILED;
  }

  std::memcpy(cmd.response, &idf_cmd.response, sizeof(cmd.response));
  return ErrorCode::OK;
}

bool ESP32SDMMCHost::CardPresent() const
{
  if (config_.gpio_cd < 0)
  {
    // 无 CD 引脚：恒报告在位，由卡层依据响应判定。
    return true;
  }
  return gpio_get_level(static_cast<gpio_num_t>(config_.gpio_cd)) == 0;
}

ErrorCode ESP32SDMMCHost::EnableCardInterrupt(bool enable)
{
  if (!enable)
  {
    return ErrorCode::OK;
  }
  if (sdmmc_host_io_int_enable(host_.slot) != ESP_OK)
  {
    return ErrorCode::FAILED;
  }
  return ErrorCode::OK;
}

}  // namespace LibXR
