#pragma once

#include <cstdint>

#include "core/libxr_type.hpp"

namespace LibXR::EtherCAT
{

/**
 * Normalized events delivered by the board ESC driver.
 *
 * The driver translates controller-specific interrupt status into these
 * protocol events before calling DeviceCore::HandleInterrupt().
 */
enum class EscEvent : uint32_t
{
  NONE = 0,
  AL_CONTROL = 1U << 0U,
  SYNC_MANAGER_CHANGE = 1U << 1U,
  SYNC_MANAGER = 1U << 2U,
  MAILBOX = 1U << 3U,
  PROCESS_DATA_OUTPUT = 1U << 4U,
  PROCESS_DATA_INPUT = 1U << 5U,
  WATCHDOG = 1U << 6U,
  EEPROM = 1U << 7U,
  SYNC0 = 1U << 8U,
  SYNC1 = 1U << 9U,
  DISTRIBUTED_CLOCK_LATCH = 1U << 10U
};

constexpr EscEvent operator|(EscEvent left, EscEvent right)
{
  return static_cast<EscEvent>(static_cast<uint32_t>(left) | static_cast<uint32_t>(right));
}

constexpr EscEvent operator&(EscEvent left, EscEvent right)
{
  return static_cast<EscEvent>(static_cast<uint32_t>(left) & static_cast<uint32_t>(right));
}

constexpr bool HasEvent(EscEvent events, EscEvent event)
{
  return (static_cast<uint32_t>(events) & static_cast<uint32_t>(event)) != 0U;
}

/**
 * ESC register access supplied by a board driver.
 *
 * This interface deliberately has no SPI, DMA, GPIO, or IRQ configuration
 * details. Those belong to the concrete board driver.
 */
class EscPort
{
 public:
  virtual ~EscPort() = default;

  virtual ErrorCode Read(uint16_t address, RawData data) = 0;
  virtual ErrorCode Write(uint16_t address, ConstRawData data) = 0;
};

}  // namespace LibXR::EtherCAT
