#pragma once

#include <cstddef>
#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR
{

/**
 * @class SDHost
 * @brief SD 主机控制器抽象接口（跨平台）。
 *        Cross-platform SD host controller interface.
 *
 * 角色定位对齐 Linux `mmc_host_ops` / Zephyr `sdhc.h`：主机只负责命令/数据
 * 事务与总线参数协商，协议状态机（枚举、CSD 解析等）位于上层。
 * Mirrors Linux `mmc_host_ops` / Zephyr `sdhc.h`: the host only transports
 * commands/data and negotiates bus parameters; the protocol state machine
 * lives one layer above.
 *
 * @note 本接口不出现任何平台 SDK 类型（LibXR 跨平台接口约束）。
 *       No platform SDK types appear in this interface (LibXR rule).
 */
class SDHost
{
 public:
  /**
   * @enum ResponseType
   * @brief SD 响应类型 / SD response type.
   */
  enum class ResponseType : uint8_t
  {
    NONE = 0,  ///< 无响应 / No response
    R1 = 1,    ///< 常规状态响应 / Normal status response
    R1B = 2,   ///< 带 busy 的响应 / Response with busy signal
    R2 = 3,    ///< CID/CSD，136-bit / CID/CSD, 136 bits
    R3 = 4,    ///< OCR / Operating Conditions Register
    R6 = 5,    ///< 发布的 RCA / Published RCA
    R7 = 6,    ///< 接口条件 / Interface condition
  };

  /**
   * @struct Command
   * @brief 待发送的 SD 命令 / SD command to send.
   */
  struct Command
  {
    uint8_t opcode = 0;                               ///< 命令索引 / Command index
    uint32_t arg = 0;                                 ///< 命令参数 / Command argument
    ResponseType response_type = ResponseType::NONE;  ///< 期望响应类型 / Response type
    uint32_t response[4] = {};  ///< 响应负载（R2 占 4 字）/ Response payload
  };

  /**
   * @struct Data
   * @brief 命令附带的数据传输（无则为 nullptr）/ Optional data phase.
   */
  struct Data
  {
    uint8_t* buffer = nullptr;  ///< 数据缓冲区 / Data buffer
    size_t length = 0;          ///< 总字节数 / Total bytes
    uint16_t block_size = 512;  ///< 块大小（字节）/ Block size in bytes
    uint16_t block_count = 1;   ///< 块数量 / Number of blocks
    bool read = false;  ///< true=读（卡->主机），false=写 / true=read, false=write
  };

  SDHost() = default;
  virtual ~SDHost() = default;

  SDHost(const SDHost&) = delete;
  SDHost& operator=(const SDHost&) = delete;

  /**
   * @brief 设置总线位宽 / Set bus width.
   * @param width 位宽：1 / 4 / 8 / Width in bits
   * @return ErrorCode 操作结果 / Error code
   */
  virtual ErrorCode SetBusWidth(uint8_t width) = 0;

  /**
   * @brief 设置时钟频率 / Set clock frequency.
   * @param hz 目标频率（Hz）/ Target frequency in Hz
   * @return ErrorCode 操作结果 / Error code
   */
  virtual ErrorCode SetClockHz(uint32_t hz) = 0;

  /**
   * @brief 获取实际时钟频率 / Get the real clock frequency.
   * @return 实际频率（Hz）/ Real frequency in Hz
   */
  virtual uint32_t GetRealClockHz() const = 0;

  /**
   * @brief 执行一次命令（可附带数据）/ Execute one command with optional data.
   * @param cmd 命令与响应 / Command and response
   * @param data 数据相位，可为 nullptr / Data phase, may be nullptr
   * @return ErrorCode 操作结果 / Error code
   */
  virtual ErrorCode Transaction(Command& cmd, Data* data) = 0;

  /**
   * @brief 卡在位查询 / Card presence query.
   * @return true=在位，false=未插入 / true=present, false=absent
   *
   * @note 无卡检测（CD）引脚的主机应恒返回 true，由协议层判定卡是否响应。
   *       Hosts without a card-detect pin should always return true and let
   *       the protocol layer decide by card response.
   */
  virtual bool CardPresent() const = 0;

  /**
   * @brief 使能/禁止 SDIO 中断（SDIO 卡用，存储卡恒 OK）/ SDIO interrupt.
   * @param enable 是否使能 / Enable or disable
   * @return ErrorCode 操作结果 / Error code
   */
  virtual ErrorCode EnableCardInterrupt(bool enable) = 0;
};

}  // namespace LibXR
