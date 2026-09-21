#pragma once

#include <cstdint>

#include "core/esc_registers.hpp"
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
  WATCHDOG = 1U << 3U,
  EEPROM = 1U << 4U,
  SYNC0 = 1U << 5U,
  SYNC1 = 1U << 6U,
  DISTRIBUTED_CLOCK_LATCH = 1U << 7U
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

  /**
   * Write the AL Event Mask (0x0204) the way this ESC needs it.
   *
   * Defaults to the ETG layout, a plain 16 bit field. A part whose silicon
   * differs - a wider register, or bits outside the ETG layout that have to
   * survive the write - overrides this, so the protocol core never has to know
   * which device it is driving.
   */
  virtual ErrorCode WriteAleventMask(uint16_t mask)
  {
    const uint8_t bytes[2] = {static_cast<uint8_t>(mask & 0xFFU),
                              static_cast<uint8_t>(mask >> 8U)};
    return Write(EscRegister::AL_EVENT_MASK, ConstRawData(bytes, sizeof(bytes)));
  }
};

}  // namespace LibXR::EtherCAT
