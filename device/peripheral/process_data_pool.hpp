#pragma once

#include <array>
#include <cstddef>

#include "core/libxr_def.hpp"
#include "process_data.hpp"

namespace LibXR::EtherCAT
{

/**
 * Fixed registry for PDO/peripheral bindings.
 *
 * Like USB's EndpointPool, this class registers externally owned objects only. It does
 * not allocate or own a binding; construct bindings during application initialization and
 * keep them alive while the Slave is active.
 */
class ProcessDataPool
{
 public:
  static constexpr size_t MAX_BINDINGS = 32;

  ErrorCode Put(ProcessData* binding)
  {
    if (binding == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    for (ProcessData* registered : bindings_)
    {
      if (registered == binding)
      {
        return ErrorCode::FULL;
      }
    }

    for (ProcessData*& slot : bindings_)
    {
      if (slot == nullptr)
      {
        slot = binding;
        return ErrorCode::OK;
      }
    }
    return ErrorCode::FULL;
  }

  ErrorCode Remove(ProcessData* binding)
  {
    if (binding == nullptr)
    {
      return ErrorCode::PTR_NULL;
    }

    for (ProcessData*& slot : bindings_)
    {
      if (slot == binding)
      {
        slot = nullptr;
        return ErrorCode::OK;
      }
    }
    return ErrorCode::NOT_FOUND;
  }

  void UpdateInputs(bool in_isr)
  {
    for (ProcessData* binding : bindings_)
    {
      if (binding != nullptr && binding->GetDirection() != ProcessData::Direction::OUTPUT)
      {
        binding->UpdateInput(in_isr);
      }
    }
  }

  void UpdateOutputs(bool in_isr)
  {
    for (ProcessData* binding : bindings_)
    {
      if (binding != nullptr && binding->GetDirection() != ProcessData::Direction::INPUT)
      {
        binding->UpdateOutput(in_isr);
      }
    }
  }

  void ApplySafeOutputs(bool in_isr)
  {
    for (ProcessData* binding : bindings_)
    {
      if (binding != nullptr && binding->GetDirection() != ProcessData::Direction::INPUT)
      {
        binding->ApplySafeOutput(in_isr);
      }
    }
  }

 private:
  std::array<ProcessData*, MAX_BINDINGS> bindings_{};
};

}  // namespace LibXR::EtherCAT
