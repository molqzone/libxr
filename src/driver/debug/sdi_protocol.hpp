#pragma once

#include <cstdint>

namespace LibXR::Debug::SdiProtocol
{
enum class Op : uint8_t
{
  NOP = 0x00,
  READ = 0x01,
  WRITE = 0x02,
};

enum class Ack : uint8_t
{
  OK = 0x00,
  RESERVED = 0x01,
  FAILED = 0x02,
  BUSY = 0x03,
  PROTOCOL = 0xFF,
};

struct Request
{
  uint8_t addr = 0u;
  uint32_t data = 0u;
  Op op = Op::NOP;
};

struct Response
{
  uint8_t addr = 0u;
  uint32_t data = 0u;
  Ack ack = Ack::PROTOCOL;
};

inline constexpr uint8_t ONLINE_CAPR_STA = static_cast<uint8_t>(0x7Cu << 1u);
inline constexpr uint8_t ONLINE_CFGR_MASK = static_cast<uint8_t>(0x7Du << 1u);
inline constexpr uint8_t ONLINE_CFGR_SHAD = static_cast<uint8_t>(0x7Eu << 1u);
inline constexpr uint32_t ONLINE_ENABLE_OUTPUT = 0x5AA50400u;

constexpr uint8_t EncodeReadAddr(uint8_t dmi_addr)
{
  return static_cast<uint8_t>((dmi_addr & 0x7Fu) << 1u);
}

constexpr uint8_t EncodeWriteAddr(uint8_t dmi_addr)
{
  return static_cast<uint8_t>(((dmi_addr & 0x7Fu) << 1u) | 0x01u);
}

}  // namespace LibXR::Debug::SdiProtocol

