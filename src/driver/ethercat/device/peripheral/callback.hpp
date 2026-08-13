#pragma once

#include "core/libxr_cb.hpp"
#include "process_data.hpp"

namespace LibXR::EtherCAT
{

/** Bind application logic to the EtherCAT input and output process-data phases. */
class CallbackBinding final : public ProcessData
{
 public:
  CallbackBinding(Callback<> input, Callback<> output, Callback<> safe_output = {})
      : ProcessData(Direction::BOTH),
        input_(input),
        output_(output),
        safe_output_(safe_output)
  {
  }

  void UpdateInput(bool in_isr) override { input_.Run(in_isr); }
  void UpdateOutput(bool in_isr) override { output_.Run(in_isr); }
  void ApplySafeOutput(bool in_isr) override { safe_output_.Run(in_isr); }

 private:
  Callback<> input_;
  Callback<> output_;
  Callback<> safe_output_;
};

}  // namespace LibXR::EtherCAT
