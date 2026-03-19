#pragma once

#include <array>
#include <cstdint>

#include "sdi.hpp"
#include "swd.hpp"

namespace LibXR::Debug
{
/**
 * @brief WCH SDI over two-wire clocked link implementation.
 *
 * This class uses a SWD-capable GPIO bit-bang backend to transmit SDI
 * request frames in MSB-first form:
 * - start(1) + addr(8) + data(32) for write
 * - start(1) + addr(8) + readback(32) for read
 *
 * Address mapping follows WCH SDI register convention:
 * - wire_addr[7:1] = DMI addr[6:0]
 * - wire_addr[0]   = rw (0=read, 1=write)
 */
template <typename SwdPort>
class SdiOverSwd final : public Sdi
{
 public:
  explicit SdiOverSwd(SwdPort& swd_port) : swd_(swd_port) {}
  ~SdiOverSwd() override = default;

  SdiOverSwd(const SdiOverSwd&) = delete;
  SdiOverSwd& operator=(const SdiOverSwd&) = delete;

  ErrorCode SetClockHz(uint32_t hz) override { return swd_.SetClockHz(hz); }

  void Close() override
  {
    online_ready_ = false;
    swd_.Close();
  }

  ErrorCode LineReset() override { return swd_.LineReset(); }

  ErrorCode EnterSdi() override
  {
    online_ready_ = false;

    auto ec = swd_.LineReset();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    // Keep SDI slave output enabled (same sequence used by WCH single-line toolchain).
    ec = WriteWordRaw(kOnlineCfgrShad, kOnlineEnableOutput);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = WriteWordRaw(kOnlineCfgrMask, kOnlineEnableOutput);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t capr = 0u;
    ec = ReadWordRaw(kOnlineCaprSta, capr);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    // Different chip families may return different CAPR values.
    if (capr == 0x00000000u || capr == 0xFFFFFFFFu)
    {
      return ErrorCode::FAILED;
    }

    online_ready_ = true;
    return ErrorCode::OK;
  }

  ErrorCode Transfer(const Request& req, Response& resp) override
  {
    resp.addr = req.addr;
    resp.data = 0u;
    resp.ack = Ack::FAILED;

    if (req.op == Op::NOP)
    {
      resp.ack = Ack::OK;
      return ErrorCode::OK;
    }

    if (!online_ready_)
    {
      const ErrorCode ec = EnterSdi();
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::PROTOCOL;
        return ec;
      }
    }

    if (req.op == Op::READ)
    {
      uint32_t data = 0u;
      const ErrorCode ec = ReadWordRaw(EncodeReadAddr(req.addr), data);
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::FAILED;
        return ec;
      }
      resp.data = data;
      resp.ack = Ack::OK;
      return ErrorCode::OK;
    }

    if (req.op == Op::WRITE)
    {
      const ErrorCode ec = WriteWordRaw(EncodeWriteAddr(req.addr), req.data);
      if (ec != ErrorCode::OK)
      {
        resp.ack = Ack::FAILED;
        return ec;
      }
      resp.data = req.data;
      resp.ack = Ack::OK;
      return ErrorCode::OK;
    }

    resp.ack = Ack::FAILED;
    return ErrorCode::ARG_ERR;
  }

  void IdleClocks(uint32_t cycles) override { swd_.IdleClocks(cycles); }

 private:
  static constexpr uint8_t kOnlineCaprSta = static_cast<uint8_t>(0x7Cu << 1u);
  static constexpr uint8_t kOnlineCfgrMask = static_cast<uint8_t>(0x7Du << 1u);
  static constexpr uint8_t kOnlineCfgrShad = static_cast<uint8_t>(0x7Eu << 1u);
  static constexpr uint32_t kOnlineEnableOutput = 0x5AA50400u;

  static constexpr uint8_t EncodeReadAddr(uint8_t dmi_addr)
  {
    return static_cast<uint8_t>(dmi_addr << 1u);
  }

  static constexpr uint8_t EncodeWriteAddr(uint8_t dmi_addr)
  {
    return static_cast<uint8_t>((dmi_addr << 1u) | 0x01u);
  }

  static void SetBitLsb(uint8_t* buf, uint32_t bit_index, bool bit)
  {
    const uint32_t byte_index = bit_index / 8u;
    const uint32_t bit_off = bit_index & 7u;
    if (bit)
    {
      buf[byte_index] = static_cast<uint8_t>(buf[byte_index] | (1u << bit_off));
    }
  }

  static bool GetBitLsb(const uint8_t* buf, uint32_t bit_index)
  {
    const uint32_t byte_index = bit_index / 8u;
    const uint32_t bit_off = bit_index & 7u;
    return ((buf[byte_index] >> bit_off) & 0x01u) != 0u;
  }

  static ErrorCode EncodeAndWriteMsbFirst(SwdPort& swd, uint64_t payload, uint32_t bits)
  {
    if (bits == 0u || bits > 64u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 8> packed = {};
    for (uint32_t i = 0u; i < bits; ++i)
    {
      const bool bit = ((payload >> (bits - 1u - i)) & 0x01u) != 0u;
      SetBitLsb(packed.data(), i, bit);
    }
    return swd.SeqWriteBits(bits, packed.data());
  }

  static ErrorCode ReadMsbFirstU32(SwdPort& swd, uint32_t bits, uint32_t& out)
  {
    if (bits == 0u || bits > 32u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 4> packed = {};
    const ErrorCode ec = swd.SeqReadBits(bits, packed.data());
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t value = 0u;
    for (uint32_t i = 0u; i < bits; ++i)
    {
      value = (value << 1u) | static_cast<uint32_t>(GetBitLsb(packed.data(), i));
    }
    out = value;
    return ErrorCode::OK;
  }

  ErrorCode SendStopBit()
  {
    // SDI stop gap: one high bit and short idle clocks for line turnaround.
    const uint8_t one_lsb = 0x01u;
    const ErrorCode ec = swd_.SeqWriteBits(1u, &one_lsb);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    swd_.IdleClocks(2u);
    return ErrorCode::OK;
  }

  ErrorCode WriteWordRaw(uint8_t wire_addr_with_rw, uint32_t data)
  {
    const uint64_t frame =
        (static_cast<uint64_t>(1u) << 40u) | (static_cast<uint64_t>(wire_addr_with_rw) << 32u) |
        static_cast<uint64_t>(data);

    ErrorCode ec = EncodeAndWriteMsbFirst(swd_, frame, 41u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    ec = SendStopBit();
    return ec;
  }

  ErrorCode ReadWordRaw(uint8_t wire_addr_with_rw, uint32_t& data)
  {
    const uint16_t frame = static_cast<uint16_t>((1u << 8u) | wire_addr_with_rw);

    ErrorCode ec = EncodeAndWriteMsbFirst(swd_, frame, 9u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = ReadMsbFirstU32(swd_, 32u, data);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = SendStopBit();
    return ec;
  }

 private:
  SwdPort& swd_;
  bool online_ready_ = false;
};

}  // namespace LibXR::Debug

