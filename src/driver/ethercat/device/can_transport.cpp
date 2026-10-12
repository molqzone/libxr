#include "can_transport.hpp"

namespace LibXR::EtherCAT
{

CanDeviceTransport::CanDeviceTransport(LibXR::CAN& bus, StatusWord& status,
                                       const Config& cfg)
    : bus_(bus),
      status_(status),
      cfg_(cfg),
      rx_binding_{this},
      rx_queue_(ResolveQueueCapacity(cfg))
{
  // The queue capacity was already validated (and resolved) by the member
  // initializer above. What is left is what the slot tables and the status word
  // region need: a source region wide enough for every slot, so a publish can
  // never silently drop the high slots.
  REQUIRE(cfg_.slot_count <= StatusWord::RegionWidth(cfg_.status_source));

  MarkAllSlotsNeverSeen();

  auto callback =
      LibXR::CAN::Callback::Create(&CanDeviceTransport::RxCallback, &rx_binding_);
  bus_.Register(callback, LibXR::CAN::Type::STANDARD, LibXR::CAN::FilterMode::ID_RANGE,
                kFilterFirstId, kFilterLastId);
}

uint16_t CanDeviceTransport::ResolveQueueCapacity(const Config& cfg)
{
  REQUIRE(cfg.slot_count > 0U);
  REQUIRE(cfg.slot_count <= kMaxSlotCount);
  REQUIRE(cfg.response_timeout_cycles > 0U);
  // LibXR's MPMCQueueBase asserts capacity_ > 1 (a capacity-1 queue once hung
  // the boot of this firmware), so 0 is the only way to ask for the default.
  REQUIRE(cfg.rx_queue_capacity == 0U || cfg.rx_queue_capacity > 1U);

  if (cfg.rx_queue_capacity == 0U)
  {
    return static_cast<uint16_t>(2U * cfg.slot_count);
  }
  return cfg.rx_queue_capacity;
}

void CanDeviceTransport::Attach(Wire& wire)
{
  Wire* const attached = &wire;
  REQUIRE(attached != nullptr);
  wire_ = attached;
}

void CanDeviceTransport::OnFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack)
{
  // The context flag is deliberately unused: this path neither blocks nor
  // allocates, so it is the same bounded work in a thread and in an ISR.
  (void)in_isr;

  // Full queue -> drop the oldest frame and retry. Pop() on an empty queue is a
  // no-op, so the loop always terminates, and nothing here waits on a lock.
  while (rx_queue_.Push(pack) != ErrorCode::OK)
  {
    (void)rx_queue_.Pop();
  }
}

void CanDeviceTransport::RefreshOutputs()
{
  // Protocol-thread context. A family hook that forwards OnOutputsUpdated()
  // must drop the call when in_isr is true; this mechanism cannot tell the two
  // apart on its own, because the flag only exists at the hook boundary.
  if (wire_ == nullptr)
  {
    return;
  }
  wire_->SendCommands(false);
}

void CanDeviceTransport::RefreshInputs()
{
  // Protocol-thread context, same discipline as RefreshOutputs().
  LibXR::CAN::ClassicPack pack;
  while (rx_queue_.Pop(pack) == ErrorCode::OK)
  {
    if (wire_ == nullptr)
    {
      // Frames that arrived before Attach() are discarded here.
      continue;
    }

    // DecodeFrame answers kNoSlot for a frame that is not this transport's; the
    // range check also rejects a slot outside this instance's table.
    const uint8_t slot = wire_->DecodeFrame(pack);
    if (slot != kNoSlot && slot < cfg_.slot_count)
    {
      cycles_since_rx_[slot] = 0U;
    }
  }

  // Age every slot once per input cycle, saturating at the never-seen sentinel
  // so the counter can never wrap back into the "alive" range.
  uint16_t liveness = 0;
  for (uint8_t slot = 0U; slot < cfg_.slot_count; ++slot)
  {
    uint16_t& cycles = cycles_since_rx_[slot];
    if (cycles < kNeverSeen)
    {
      ++cycles;
    }

    if (cycles <= cfg_.response_timeout_cycles)
    {
      liveness |= static_cast<uint16_t>(1U << slot);
    }
  }

  // The whole region is republished every cycle, which is what gives the
  // reference firmware's "assume online at cycle start, clear on timeout" its
  // single-writer form: a slot that timed out simply drops out of this cycle's
  // mask instead of being cleared by a second writer.
  PublishLiveness(liveness);
}

void CanDeviceTransport::LeaveOperational()
{
  // Protocol-thread context, same discipline as RefreshOutputs().
  if (wire_ != nullptr)
  {
    wire_->SendCommands(true);
  }

  MarkAllSlotsNeverSeen();
  PublishLiveness(0U);
}

void CanDeviceTransport::RxCallback(bool in_isr, RxBinding* binding,
                                    const LibXR::CAN::ClassicPack& pack)
{
  binding->self->OnFrame(in_isr, pack);
}

void CanDeviceTransport::MarkAllSlotsNeverSeen()
{
  for (uint16_t& cycles : cycles_since_rx_)
  {
    cycles = kNeverSeen;
  }
}

void CanDeviceTransport::PublishLiveness(uint16_t mask)
{
  status_.Publish(cfg_.status_source, mask);
}

}  // namespace LibXR::EtherCAT
