#pragma once

#include <cstdint>

namespace LibXR::EtherCAT
{

// ESC registers and mailbox payloads are little-endian on the wire (ETG.1000-4),
// so every field is decoded byte by byte instead of type-punned. Shared by the
// device core (SyncManager/FMMU register blocks) and the mailbox protocol
// handlers (CoE SDO headers and payloads).

inline uint16_t ReadLe16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
}

inline uint32_t ReadLe32(const uint8_t* data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

inline void WriteLe16(uint8_t* data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8U);
}

inline void WriteLe32(uint8_t* data, uint32_t value)
{
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8U);
  data[2] = static_cast<uint8_t>(value >> 16U);
  data[3] = static_cast<uint8_t>(value >> 24U);
}

}  // namespace LibXR::EtherCAT
