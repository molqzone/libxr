#pragma once

#include <cstddef>
#include <cstdint>

#include "object_dictionary.hpp"

namespace LibXR::EtherCAT
{

enum class PdoDirection : uint8_t
{
  RX,
  TX
};

struct PdoEntry
{
  ObjectEntry* object = nullptr;
  uint16_t bit_offset = 0;
};

struct Pdo
{
  uint16_t index = 0;
  PdoDirection direction = PdoDirection::RX;
  PdoEntry* entries = nullptr;
  size_t entry_count = 0;
  uint16_t bit_length = 0;
  DeviceClass* owner = nullptr;
};

}  // namespace LibXR::EtherCAT
