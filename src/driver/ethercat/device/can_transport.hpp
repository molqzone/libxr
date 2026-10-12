#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "can.hpp"
#include "ethercat/device/status_word.hpp"
#include "queue/queue.hpp"

namespace LibXR::EtherCAT
{

/**
 * The CAN transport mechanism of an EtherCAT slave device: bus, RX queue,
 * per-slot liveness and the status word bits that report it.
 *
 * This is a plain class on purpose -- it is NOT a `DeviceClass`. A family device
 * class already inherits `DeviceClass` to contribute its own objects and hooks,
 * so making the mechanism one too would form a diamond; the family class owns
 * an instance of this instead and forwards its hooks here.
 *
 * It owns only the mechanism: the tick, the liveness counters, the timeout
 * decision and the `StatusWord` region publish. Everything family-specific (how
 * a command object becomes CAN bytes, which feedback frame belongs to which
 * slot) is the `Wire` implementation, so the family class carries no liveness
 * code at all.
 *
 * Context discipline follows the rest of XRECAT: the ISR path may only enqueue
 * (`OnFrame`), and everything that decodes, publishes or touches the bus happens
 * on the protocol thread. The family class's `DeviceClass` hooks are what
 * translate `OnOutputsUpdated(bool in_isr)` and friends into the calls below;
 * that wiring belongs to the family, not to this mechanism.
 */
class CanDeviceTransport
{
 public:
  /**
   * @param slot_count              how many slots this instance supervises.
   * @param response_timeout_cycles a slot stays alive for this many input
   *                                cycles after its last feedback frame.
   * @param status_source           the `StatusWord` region this instance
   *                                publishes to (the family's bus index).
   * @param rx_queue_capacity       RX queue capacity; 0 = 2 * slot_count.
   */
  struct Config
  {
    uint8_t slot_count = 0;
    uint16_t response_timeout_cycles = 0;
    uint8_t status_source = 0;
    // 0 selects the default (2 * slot_count). Anything else must be > 1: LibXR's
    // MPMCQueueBase asserts capacity_ > 1, and a capacity-1 queue once hung the
    // boot of this firmware (docs/ecat-bringup.md).
    uint16_t rx_queue_capacity = 0;
  };

  /**
   * The whole family contract: send this cycle's commands, and say which slot a
   * received frame belongs to.
   */
  class Wire
  {
   public:
    virtual ~Wire() = default;

    /**
     * @param zero true = leave operational: send the safe (all-zero) command
     *             image instead of the commanded one.
     */
    virtual void SendCommands(bool zero) = 0;

    /**
     * @return the slot this frame is feedback for, or 0xFF when the frame does
     *         not belong to this transport.
     *         Runs on the protocol thread only, so it may touch device state.
     */
    virtual uint8_t DecodeFrame(const LibXR::CAN::ClassicPack& pack) = 0;
  };

  /**
   * Registers the permissive CAN receive filter here, at construction time: the
   * mechanism owns the bus subscription for its lifetime, and ID filtering is
   * `Wire::DecodeFrame`'s job (it answers 0xFF for foreign frames), so a family
   * never has to know which IDs arrive.
   *
   * @param bus    the CAN bus this transport drives.
   * @param status the status word this transport publishes its liveness to.
   * @param cfg    slot count, timeout, source region and queue capacity.
   */
  CanDeviceTransport(LibXR::CAN& bus, StatusWord& status, const Config& cfg);

  CanDeviceTransport(const CanDeviceTransport&) = delete;
  CanDeviceTransport& operator=(const CanDeviceTransport&) = delete;
  CanDeviceTransport(CanDeviceTransport&&) = delete;
  CanDeviceTransport& operator=(CanDeviceTransport&&) = delete;

  /**
   * Attach the family's wire implementation. Non-owning; may be called after
   * construction. Frames received before it are drained and discarded.
   */
  void Attach(Wire& wire);

  /**
   * ISR-safe receive path, and the only thing an ISR may do here: enqueue.
   *
   * When the queue is full the oldest frame is dropped, so a stalled protocol
   * thread can never wedge the CAN interrupt. No decoding, no publishing, no
   * allocation -- the call is bounded and never blocks.
   */
  void OnFrame(bool in_isr, const LibXR::CAN::ClassicPack& pack);

  /**
   * Send this cycle's commands. Protocol-thread context only (the family hook
   * must drop the call when `in_isr` is true).
   */
  void RefreshOutputs();

  /**
   * Refresh liveness: drain the queue, decode what arrived, age every slot and
   * publish the whole liveness mask of this instance's status word region.
   *
   * Protocol-thread context only (the family hook must drop the call when
   * `in_isr` is true).
   */
  void RefreshInputs();

  /**
   * Leave operational: send the safe command image, forget every slot's
   * liveness and publish an all-offline mask.
   *
   * Protocol-thread context only (the family hook must drop the call when
   * `in_isr` is true).
   */
  void LeaveOperational();

 private:
  /** The binding the CAN callback trampoline recovers `this` from. */
  struct RxBinding
  {
    CanDeviceTransport* self;
  };

  /**
   * Compile-time ceiling on the per-slot tables. The live ceiling is the width
   * of the status word region, checked in the constructor.
   */
  static constexpr uint8_t kMaxSlotCount = 16U;

  /** A slot that has never been seen, matching the RM profile's sentinel. */
  static constexpr uint16_t kNeverSeen = 0xFFFFU;

  /** `DecodeFrame`'s "not my frame" answer. */
  static constexpr uint8_t kNoSlot = 0xFFU;

  /** Accept every standard ID; `Wire::DecodeFrame` does the filtering. */
  static constexpr uint32_t kFilterFirstId = 0U;
  static constexpr uint32_t kFilterLastId = UINT32_MAX;

  /**
   * Validate the config and resolve the RX queue capacity.
   *
   * Runs before the queue member is constructed, so an invalid capacity fails
   * here instead of inside `MPMCQueueBase`'s own assertion.
   */
  [[nodiscard]] static uint16_t ResolveQueueCapacity(const Config& cfg);

  static void RxCallback(bool in_isr, RxBinding* binding,
                         const LibXR::CAN::ClassicPack& pack);

  void MarkAllSlotsNeverSeen();
  void PublishLiveness(uint16_t mask);

  LibXR::CAN& bus_;
  StatusWord& status_;
  Wire* wire_{};
  Config cfg_{};
  RxBinding rx_binding_;
  std::array<uint16_t, kMaxSlotCount> cycles_since_rx_{};
  // Fixed-size, allocated once while the transport is constructed and never
  // freed; the ISR path only pushes to and the protocol thread only pops from
  // it, which is exactly what MPMCQueue is for.
  LibXR::MPMCQueue<LibXR::CAN::ClassicPack> rx_queue_;
};

}  // namespace LibXR::EtherCAT
