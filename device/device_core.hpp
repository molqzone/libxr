#pragma once

#include <initializer_list>

#include "core/esc_port.hpp"
#include "device_composition.hpp"

namespace LibXR::EtherCAT
{

/**
 * EtherCAT device protocol core.
 *
 * The core owns the completed composition in the same sense that USB
 * DeviceCore owns DeviceComposition. It never owns DeviceClass modules or the
 * concrete ESC driver.
 */
class DeviceCore final
{
 public:
  DeviceCore(EscPort& port, DevicePool& pool, std::initializer_list<DeviceClass*> classes);

  DeviceCore(const DeviceCore&) = delete;
  DeviceCore& operator=(const DeviceCore&) = delete;
  DeviceCore(DeviceCore&&) = delete;
  DeviceCore& operator=(DeviceCore&&) = delete;

  /** Enter the protocol core from the board driver's ESC IRQ path. */
  void HandleInterrupt(EscEvent events);

  [[nodiscard]] AlState GetState() const { return state_; }
  [[nodiscard]] const DeviceComposition& GetComposition() const { return composition_; }

 private:
  void TransitionTo(AlState next_state);

  EscPort& port_;
  DeviceComposition composition_;
  AlState state_ = AlState::INIT;
};

}  // namespace LibXR::EtherCAT
