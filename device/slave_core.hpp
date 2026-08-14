#pragma once

#include <initializer_list>

#include "core/esc_port.hpp"
#include "slave_composition.hpp"

namespace LibXR::EtherCAT
{

/**
 * EtherCAT slave protocol core.
 *
 * The core owns the completed composition in the same sense that USB
 * DeviceCore owns DeviceComposition. It never owns SlaveClass modules or the
 * concrete ESC driver.
 */
class SlaveCore final
{
 public:
  SlaveCore(EscPort& port, SlavePool& pool, std::initializer_list<SlaveClass*> classes);

  SlaveCore(const SlaveCore&) = delete;
  SlaveCore& operator=(const SlaveCore&) = delete;
  SlaveCore(SlaveCore&&) = delete;
  SlaveCore& operator=(SlaveCore&&) = delete;

  /** Enter the protocol core from the board driver's ESC IRQ path. */
  void HandleInterrupt(EscEvent events);

  [[nodiscard]] SlaveState GetState() const { return state_; }
  [[nodiscard]] const SlaveComposition& GetComposition() const { return composition_; }

 private:
  void TransitionTo(SlaveState next_state);

  EscPort& port_;
  SlaveComposition composition_;
  SlaveState state_ = SlaveState::INIT;
};

}  // namespace LibXR::EtherCAT
