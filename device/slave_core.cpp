#include "slave_core.hpp"

namespace LibXR::EtherCAT
{

SlaveCore::SlaveCore(EscPort& port, SlavePool& pool,
                     std::initializer_list<SlaveClass*> classes)
    : port_(port), composition_(pool, classes)
{
}

void SlaveCore::HandleInterrupt(EscEvent events)
{
  // The board port has already converted its IRQ status to protocol events.
  // PDO packing and unpacking will be added to the native protocol engine;
  // these notifications define the application-side ordering contract now.
  if (HasEvent(events, EscEvent::PROCESS_DATA_OUTPUT))
  {
    composition_.DispatchOutputsUpdated();
  }

  if (HasEvent(events, EscEvent::PROCESS_DATA_INPUT) || HasEvent(events, EscEvent::SYNC0) ||
      HasEvent(events, EscEvent::SYNC1))
  {
    composition_.DispatchInputsRequested();
  }

  (void)port_;
}

void SlaveCore::TransitionTo(SlaveState next_state)
{
  if (next_state == state_)
  {
    return;
  }

  const SlaveState previous_state = state_;
  state_ = next_state;
  composition_.DispatchStateChanged(previous_state, next_state);
}

}  // namespace LibXR::EtherCAT
