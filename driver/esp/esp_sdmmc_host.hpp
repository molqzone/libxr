#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/sdmmc_host.h"
#include "sd_host.hpp"

namespace LibXR
{

/**
 * @class ESP32SDMMCHost
 * @brief 基于 ESP-IDF SDMMC 主机驱动的主机实现（ESP32-P4）。
 *        SD host implementation on top of the ESP-IDF SDMMC host driver.
 *
 * 职责：把 SDHost 接口的总线操作翻译成 IDF 调用，并持有 host/slot 初始化。
 * Translates SDHost bus operations into IDF calls and owns host/slot
 * initialization.
 *
 * @note M1 使用 IDF 旧版 ops 表 API（`driver/sdmmc_host.h`），因为协议层
 *       `sdmmc_card_init()` 只接受该结构；M2 的原生状态机将迁移到 v6.1 新的
 *       handle 式 API（`driver/sd_host_sdmmc.h`）。
 *       M1 uses the legacy ops-table API because the protocol layer only
 *       accepts it; M2's native state machine moves to the new handle API.
 */
class ESP32SDMMCHost : public SDHost
{
 public:
  /**
   * @struct Config
   * @brief 主机与 slot 配置 / Host and slot configuration.
   *
   * 默认值为 Waveshare ESP32-P4-Pico 的 4-bit 接线。
   * Defaults match the Waveshare ESP32-P4-Pico 4-bit wiring.
   */
  struct Config
  {
    int slot = 0;               ///< slot 号（P4 有 2 个）/ Slot index
    int gpio_clk = 43;          ///< CLK 引脚 / CLK GPIO
    int gpio_cmd = 44;          ///< CMD 引脚 / CMD GPIO
    int gpio_d0 = 39;           ///< D0 引脚 / D0 GPIO
    int gpio_d1 = 40;           ///< D1 引脚 / D1 GPIO
    int gpio_d2 = 41;           ///< D2 引脚 / D2 GPIO
    int gpio_d3 = 42;           ///< D3 引脚 / D3 GPIO
    int gpio_cd = -1;           ///< 卡检测引脚，-1=禁用 / Card detect, -1=disabled
    uint32_t freq_khz = 20000;  ///< 目标频率（kHz）/ Target frequency
    // ESP32-P4 的 SD 卡槽由片上 LDO 通道供电（Pico/EV 板为 LDO_VO4=4），
    // 不配置则卡不上电、总线静默；外部供电的板子设为 -1。
    int ldo_chan_id = 4;
  };

  /**
   * @brief 构造并完成 host/slot 初始化 / Construct and initialize host/slot.
   *
   * host/slot 初始化属于硬错误（板级接线/参数错误），按仓库机制 fail-fast。
   * Host/slot init failure is a hard error (board wiring/config); it fails
   * fast through the repository's established mechanism.
   */
  explicit ESP32SDMMCHost(const Config& config);

  /**
   * @brief 以默认配置（板级默认接线）构造 / Construct with board defaults.
   */
  ESP32SDMMCHost() : ESP32SDMMCHost(Config{}) {}

  ErrorCode SetBusWidth(uint8_t width) override;

  ErrorCode SetClockHz(uint32_t hz) override;

  uint32_t GetRealClockHz() const override;

  ErrorCode Transaction(Command& cmd, Data* data) override;

  bool CardPresent() const override;

  ErrorCode EnableCardInterrupt(bool enable) override;

  /**
   * @brief 供 IDF 协议层使用的 host 结构（M1）。
   *        IDF protocol-layer host structure (M1).
   *
   * `sdmmc_card_init()` 等协议层入口只接受该结构；返回指针仅用于协议层调用，
   * 生命周期归本对象所有。
   * The protocol layer only accepts this structure; the returned pointer is
   * for protocol-layer calls and is owned by this object.
   */
  const sdmmc_host_t* GetIdfHost() const { return &host_; }

 private:
  sdmmc_host_t host_;
  Config config_;
  uint32_t real_freq_hz_ = 0;
};

}  // namespace LibXR
