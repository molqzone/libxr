#pragma once

#include <cstdint>

#include "libxr_def.hpp"

namespace LibXR::Debug
{
/**
 * @class Sdi
 * @brief WCH SDI（Serial Debug Interface）抽象基类。
 *
 * SDI 在 WCH-Link RV 语义中以 DMI（addr/data/op）事务形式暴露。
 * 本类定义了 SDI 链路控制与 DMI 读写辅助接口，便于 USB 协议层
 * （如 WCH-LinkRV 类）与具体硬件实现解耦。
 */
class Sdi
{
 public:
  /**
   * @struct TransferPolicy
   * @brief SDI 传输策略。
   */
  struct TransferPolicy
  {
    uint16_t busy_retry = 100;  ///< DMI busy 重试次数上限。
    uint8_t idle_cycles = 0;    ///< 每次传输尝试后插入的空闲周期。
  };

  /**
   * @enum Op
   * @brief DMI 操作类型。
   */
  enum class Op : uint8_t
  {
    NOP = 0x00,
    READ = 0x01,
    WRITE = 0x02,
  };

  /**
   * @enum Ack
   * @brief DMI 响应状态（与 WCH-Link RV `CMD=0x08` op 字段一致）。
   */
  enum class Ack : uint8_t
  {
    OK = 0x00,
    RESERVED = 0x01,
    FAILED = 0x02,
    BUSY = 0x03,
    PROTOCOL = 0xFF,
  };

  /**
   * @struct Request
   * @brief DMI 请求包。
   */
  struct Request
  {
    uint8_t addr = 0;
    uint32_t data = 0;
    Op op = Op::NOP;
  };

  /**
   * @struct Response
   * @brief DMI 响应包。
   */
  struct Response
  {
    uint8_t addr = 0;
    uint32_t data = 0;
    Ack ack = Ack::PROTOCOL;
  };

  virtual ~Sdi() = default;

  Sdi(const Sdi&) = delete;
  Sdi& operator=(const Sdi&) = delete;

  void SetTransferPolicy(const TransferPolicy& policy) { policy_ = policy; }
  [[nodiscard]] const TransferPolicy& GetTransferPolicy() const { return policy_; }

  /**
   * @brief 设置调试时钟频率（可选实现）。
   */
  virtual ErrorCode SetClockHz(uint32_t hz) = 0;

  /**
   * @brief 关闭探针并释放资源。
   */
  virtual void Close() = 0;

  /**
   * @brief 线复位。
   */
  virtual ErrorCode LineReset() = 0;

  /**
   * @brief 进入 SDI 模式。
   */
  virtual ErrorCode EnterSdi() = 0;

  /**
   * @brief 执行一次 DMI 传输（不含 busy 重试）。
   */
  virtual ErrorCode Transfer(const Request& req, Response& resp) = 0;

  /**
   * @brief 插入空闲时钟周期（可选）。
   */
  virtual void IdleClocks(uint32_t cycles) { UNUSED(cycles); }

  /**
   * @brief 带 busy 重试的 DMI 传输封装。
   */
  ErrorCode TransferWithRetry(const Request& req, Response& resp)
  {
    ResetResponse(resp);

    uint16_t retry = 0;
    while (true)
    {
      const ErrorCode ec = Transfer(req, resp);
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::PROTOCOL;
        return ec;
      }

      if (policy_.idle_cycles != 0u)
      {
        IdleClocks(policy_.idle_cycles);
      }

      if (resp.ack != Ack::BUSY)
      {
        break;
      }

      if (retry >= policy_.busy_retry)
      {
        return ErrorCode::TIMEOUT;
      }
      ++retry;
    }

    return (resp.ack == Ack::OK) ? ErrorCode::OK : ErrorCode::FAILED;
  }

  /**
   * @brief DMI 读事务（带 busy 重试）。
   */
  ErrorCode DmiReadTxn(uint8_t addr, uint32_t& data, Ack& ack)
  {
    Response resp = {};
    const ErrorCode ec = TransferWithRetry({addr, 0u, Op::READ}, resp);
    ack = resp.ack;
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    data = resp.data;
    return ErrorCode::OK;
  }

  /**
   * @brief DMI 写事务（带 busy 重试）。
   */
  ErrorCode DmiWriteTxn(uint8_t addr, uint32_t data, Ack& ack)
  {
    Response resp = {};
    const ErrorCode ec = TransferWithRetry({addr, data, Op::WRITE}, resp);
    ack = resp.ack;
    return ec;
  }

 protected:
  Sdi() = default;

 private:
  static void ResetResponse(Response& resp)
  {
    resp.addr = 0u;
    resp.data = 0u;
    resp.ack = Ack::PROTOCOL;
  }

 private:
  TransferPolicy policy_ = {};
};

}  // namespace LibXR::Debug

