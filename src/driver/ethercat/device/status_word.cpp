#include "status_word.hpp"

#include <atomic>

namespace LibXR::EtherCAT
{

StatusWord::StatusWord()
{
  // Bind the one storage location the layout declares. The template's BindAs()
  // REQUIREs that the entry exists and that its type and bit length match, so a
  // table typo is a boot-time failure rather than a value that never moves.
  storage_ = BindAs<uint32_t>(StatusWordLayout::kStatusWordIndex,
                              StatusWordLayout::kStatusWordSubindex);
}

void StatusWord::Publish(uint8_t source, uint16_t mask)
{
  const Region* region = FindRegion(source);
  REQUIRE(region != nullptr);
  REQUIRE(storage_ != nullptr);

  // Bit i of the mask is slot i of this source, i.e. status word bit
  // (bit_offset + i). Bits of the mask beyond the region width cannot be
  // represented and are dropped here rather than leaking into a neighbour.
  const uint32_t region_bits = RegionBits(*region);
  const uint32_t field =
      (static_cast<uint32_t>(mask) << region->bit_offset) & region_bits;
  const uint32_t preserved = ~region_bits;

  // One atomic read-modify-write of the whole region: a source never observes
  // another source's half-written word, and a concurrent reader (the PDO pack
  // path reads the same bytes) never sees a torn value. Relaxed ordering is
  // enough -- every writer is the protocol thread and the word carries the
  // latest state, not a happens-before edge.
  std::atomic_ref<uint32_t> word(*storage_);
  uint32_t current = word.load(std::memory_order_relaxed);
  while (!word.compare_exchange_weak(current, (current & preserved) | field,
                                     std::memory_order_relaxed,
                                     std::memory_order_relaxed))
  {
  }
}

}  // namespace LibXR::EtherCAT
