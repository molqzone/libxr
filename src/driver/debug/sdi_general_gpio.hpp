#pragma once

#include <array>
#include <cstdint>

#include "sdi.hpp"

namespace LibXR::Debug
{
/**
 * @brief SDI protocol over a GPIO bit-serial backend.
 *
 * BitPort contract:
 * - ErrorCode SetClockHz(uint32_t)
 * - void Close()
 * - ErrorCode LineReset()
 * - void IdleClocks(uint32_t)
 * - ErrorCode SeqWriteBits(uint32_t, const uint8_t*)
 * - ErrorCode SeqReadBits(uint32_t, uint8_t*)
 */
template <typename BitPort>
class SdiGeneralGPIO final : public Sdi
{
 public:
  explicit SdiGeneralGPIO(BitPort& bit_port) : port_(bit_port) {}
  ~SdiGeneralGPIO() override = default;

  SdiGeneralGPIO(const SdiGeneralGPIO&) = delete;
  SdiGeneralGPIO& operator=(const SdiGeneralGPIO&) = delete;

  ErrorCode SetClockHz(uint32_t hz) override { return port_.SetClockHz(hz); }

  void Close() override
  {
    online_ready_ = false;
    port_.Close();
  }

  ErrorCode LineReset() override { return port_.LineReset(); }

  ErrorCode EnterSdi() override
  {
    online_ready_ = false;

    ErrorCode ec = port_.LineReset();
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = WriteWordRaw(SdiProtocol::ONLINE_CFGR_SHAD, SdiProtocol::ONLINE_ENABLE_OUTPUT);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = WriteWordRaw(SdiProtocol::ONLINE_CFGR_MASK, SdiProtocol::ONLINE_ENABLE_OUTPUT);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    uint32_t capr = 0u;
    ec = ReadWordRaw(SdiProtocol::ONLINE_CAPR_STA, capr);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

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
      const ErrorCode ec = ReadWordRaw(SdiProtocol::EncodeReadAddr(req.addr), data);
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
      const ErrorCode ec = WriteWordRaw(SdiProtocol::EncodeWriteAddr(req.addr), req.data);
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

  void IdleClocks(uint32_t cycles) override { port_.IdleClocks(cycles); }

 private:
  static void SetBitLsb(uint8_t* buf, uint32_t bit_index, bool bit)
  {
    const uint32_t byte_index = bit_index / 8u;
    const uint32_t bit_offset = bit_index & 7u;
    if (bit)
    {
      buf[byte_index] = static_cast<uint8_t>(buf[byte_index] | (1u << bit_offset));
    }
  }

  static bool GetBitLsb(const uint8_t* buf, uint32_t bit_index)
  {
    const uint32_t byte_index = bit_index / 8u;
    const uint32_t bit_offset = bit_index & 7u;
    return ((buf[byte_index] >> bit_offset) & 0x01u) != 0u;
  }

  static ErrorCode EncodeAndWriteMsbFirst(BitPort& port, uint64_t payload, uint32_t bits)
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
    return port.SeqWriteBits(bits, packed.data());
  }

  static ErrorCode ReadMsbFirstU32(BitPort& port, uint32_t bits, uint32_t& out)
  {
    if (bits == 0u || bits > 32u)
    {
      return ErrorCode::ARG_ERR;
    }

    std::array<uint8_t, 4> packed = {};
    const ErrorCode ec = port.SeqReadBits(bits, packed.data());
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
    const uint8_t one_lsb = 0x01u;
    const ErrorCode ec = port_.SeqWriteBits(1u, &one_lsb);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    port_.IdleClocks(2u);
    return ErrorCode::OK;
  }

  ErrorCode WriteWordRaw(uint8_t wire_addr_with_rw, uint32_t data)
  {
    const uint64_t frame =
        (static_cast<uint64_t>(1u) << 40u) | (static_cast<uint64_t>(wire_addr_with_rw) << 32u) |
        static_cast<uint64_t>(data);

    ErrorCode ec = EncodeAndWriteMsbFirst(port_, frame, 41u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }
    return SendStopBit();
  }

  ErrorCode ReadWordRaw(uint8_t wire_addr_with_rw, uint32_t& data)
  {
    const uint16_t frame = static_cast<uint16_t>((1u << 8u) | wire_addr_with_rw);

    ErrorCode ec = EncodeAndWriteMsbFirst(port_, frame, 9u);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    ec = ReadMsbFirstU32(port_, 32u, data);
    if (ec != ErrorCode::OK)
    {
      return ec;
    }

    return SendStopBit();
  }

 private:
  BitPort& port_;
  bool online_ready_ = false;
};

}  // namespace LibXR::Debug

