#pragma once

#include <cstdint>

namespace LibXR::EtherCAT
{

/** EtherCAT application-layer state. */
enum class AlState : uint8_t
{
  INIT = 0x01,
  PRE_OPERATIONAL = 0x02,
  BOOTSTRAP = 0x03,
  SAFE_OPERATIONAL = 0x04,
  OPERATIONAL = 0x08
};

}  // namespace LibXR::EtherCAT
