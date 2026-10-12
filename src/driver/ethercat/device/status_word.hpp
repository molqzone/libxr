#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "ethercat/device/coe/coe_layout_device.hpp"

namespace LibXR::EtherCAT
{

/**
 * The compile-time CoE layout of the one object `StatusWord` owns, in the plain
 * LibXR-free table shape `CoeLayoutDevice` consumes.
 *
 * XRECAT's discipline is "no object-dictionary generator dependency": the table
 * is hand-written here and asserted against this stack's wire enums by the
 * template, so it cannot drift from what the master was told in the ESI. The
 * wire form of 0x6041 / 0x1A0F is copied verbatim from the RM profile's
 * generated table (`EcatLayout::kEntries` / `kPdos` of `ecat_layout_gen.hpp`):
 *
 *   - object 0x6041 "statusword", subindex 0, UNSIGNED32, bit length 32,
 *     ACCESS_TX (master reads it);
 *   - TxPDO 0x1A0F "statusword" carrying that single entry.
 *
 * Every field is part of the master's contract, so a change here is a change of
 * the dictionary, not a refactor.
 */
struct StatusWordLayout
{
  /** Data types, as the numeric values of `ObjectDataType`. */
  enum class DataType : uint16_t
  {
    BOOLEAN = 0x0001,
    INTEGER8 = 0x0002,
    INTEGER16 = 0x0003,
    INTEGER32 = 0x0004,
    UNSIGNED8 = 0x0005,
    UNSIGNED16 = 0x0006,
    UNSIGNED32 = 0x0007,
    REAL32 = 0x0008,
    VISIBLE_STRING = 0x0009,
    OCTET_STRING = 0x000A,
    INTEGER64 = 0x0015,
    UNSIGNED64 = 0x001B,
    REAL64 = 0x0011
  };

  /** PDO transfer direction, as the numeric value of `PdoDirection`. */
  enum class Direction : uint8_t
  {
    RX = 0,  ///< master -> slave (outputs)
    TX = 1   ///< slave -> master (inputs)
  };

  /**
   * Access flags, as the numeric value of `ObjectAccess`.
   *
   * RX_PDO = 1u << 6, TX_PDO = 1u << 7, READ = bits 0..2, WRITE = bits 3..5.
   */
  enum Access : uint16_t
  {
    ACCESS_RX = 0x78,  ///< outputs: master writes
    ACCESS_TX = 0x87   ///< inputs:  master reads
  };

  /** One CoE object entry and the PDO it is mapped into. */
  struct Entry
  {
    uint16_t index;
    uint8_t subindex;
    uint16_t bit_length;
    DataType type;
    uint16_t access;
    uint16_t pdo_index;
    Direction pdo_direction;
    const char* object_name;
  };

  /** One PDO, in the order it appears in the process data image. */
  struct Pdo
  {
    uint16_t index;
    Direction direction;
  };

  static constexpr uint16_t kStatusWordIndex = 0x6041U;
  static constexpr uint8_t kStatusWordSubindex = 0U;
  static constexpr uint16_t kStatusWordPdoIndex = 0x1a0fU;

  static constexpr size_t kEntryCount = 1U;
  static constexpr size_t kPdoCount = 1U;
  static constexpr size_t kObjectCount = 1U;

  static constexpr std::array<Entry, kEntryCount> kEntries = {{
      {kStatusWordIndex, kStatusWordSubindex, 32U, DataType::UNSIGNED32, ACCESS_TX,
       kStatusWordPdoIndex, Direction::TX, "statusword"},
  }};

  static constexpr std::array<Pdo, kPdoCount> kPdos = {{
      {kStatusWordPdoIndex, Direction::TX},
  }};
};

/**
 * The single owner of CoE object 0x6041 (statusword), with per-source-region
 * atomic publish.
 *
 * Several device instances cannot each declare 0x6041: `DeviceComposition`
 * REQUIRE-fails on a duplicate object index, and two writers of one status word
 * would overwrite each other. This class is therefore the only writer, and it
 * splits the 32-bit word into per-source regions that are published
 * independently. Bits outside the published region are preserved, so sources
 * never tread on each other.
 *
 * The region table is data declared by this class -- it carries no family
 * knowledge and makes no equal-width-stride assumption. UART consumers (link
 * liveness, error ids) need different widths and different offsets than motor
 * liveness, so the layout is a list of {source, bit_offset, width} triples the
 * static_asserts below hold together, not `source * 8`.
 *
 * The flattened bitmap of a variant is a wire contract of that variant (the RM
 * profile's u32 motor/CAN-IMU bitmap), not of this class: sources claim regions
 * here, the variant decides what each region means.
 *
 * Context: `Publish()` is protocol-thread only. The one-shot IMU bits the
 * reference firmware clears inside its TxPDO override (the master reads them
 * once) need an explicit flush seam that this class does not have yet; that is a
 * later variant's concern and is deliberately left unimplemented here.
 */
class StatusWord final : public CoeLayoutDevice<StatusWordLayout>
{
 public:
  /** One bit region of the status word, owned by exactly one source. */
  struct Region
  {
    uint8_t source;
    uint8_t bit_offset;
    uint8_t width;
  };

  /**
   * The regions this class declares.
   *
   * Four 8-bit regions exactly filling the u32: source 0 owns bits 0..7, source
   * 1 bits 8..15, source 2 bits 16..23, source 3 bits 24..31. The split is
   * chosen per variant from its flattened bitmap (RM: motor liveness on 0..15,
   * CAN-IMU one-shot bits on 16..23), never computed from the source number.
   */
  static constexpr std::array<Region, 4U> kRegions = {{
      {0U, 0U, 8U},
      {1U, 8U, 8U},
      {2U, 16U, 8U},
      {3U, 24U, 8U},
  }};

  static constexpr size_t kRegionCount = kRegions.size();

  /**
   * @return the region @p source owns, or nullptr when it is not declared.
   */
  [[nodiscard]] static constexpr const Region* FindRegion(uint8_t source)
  {
    for (const Region& region : kRegions)
    {
      if (region.source == source)
      {
        return &region;
      }
    }
    return nullptr;
  }

  /**
   * @return how many bits @p source owns, or 0 when it is not declared.
   *
   * A publisher that owns N slots (e.g. `CanDeviceTransport`) uses this to check
   * that its slot count fits the region it publishes into, so a configuration
   * mistake fails at construction instead of silently dropping the high bits.
   */
  [[nodiscard]] static constexpr uint8_t RegionWidth(uint8_t source)
  {
    const Region* region = FindRegion(source);
    return region == nullptr ? 0U : region->width;
  }

  /**
   * The region table must tile the whole 32-bit word exactly: every bit has
   * exactly one owner, so no bit can be left undefined by a publish and no two
   * sources can fight over one. The width ceiling is `Publish()`'s 16-bit mask.
   *
   * Asserted below, outside the class, because the check has to run after the
   * class is complete.
   */
  [[nodiscard]] static constexpr bool RegionsTileWord()
  {
    uint32_t covered = 0;
    for (const Region& region : kRegions)
    {
      if (region.width == 0U || region.width > 16U ||
          (static_cast<uint32_t>(region.bit_offset) + region.width) > 32U)
      {
        return false;
      }

      const uint32_t bits = RegionBits(region);
      if ((covered & bits) != 0U)
      {
        return false;
      }
      covered |= bits;
    }
    return covered == UINT32_C(0xFFFFFFFF);
  }

  StatusWord();

  StatusWord(const StatusWord&) = delete;
  StatusWord& operator=(const StatusWord&) = delete;
  StatusWord(StatusWord&&) = delete;
  StatusWord& operator=(StatusWord&&) = delete;

  /**
   * Publish one source's bit region.
   *
   * Bit @p i of @p mask becomes bit `bit_offset + i` of the status word; the
   * other bits of that region are cleared and every other region is preserved.
   * The read-modify-write is atomic, so a source publishing while another
   * context reads the word never produces a torn or mixed value.
   *
   * @param source the declared region to publish into.
   * @param mask   the source's bits, bit i = slot i of that source.
   *
   * Protocol-thread context only: an ISR must hand its value to the protocol
   * thread instead (the one-shot IMU flush seam is the known future case).
   */
  void Publish(uint8_t source, uint16_t mask);

 private:
  /** The bits of one region, in status word positions. */
  [[nodiscard]] static constexpr uint32_t RegionBits(const Region& region)
  {
    return ((UINT32_C(1) << region.width) - 1U) << region.bit_offset;
  }

  uint32_t* storage_{};
};

static_assert(StatusWord::RegionsTileWord(),
              "StatusWord regions must tile the 32-bit status word exactly once");

}  // namespace LibXR::EtherCAT
