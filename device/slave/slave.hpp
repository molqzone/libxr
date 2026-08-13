#pragma once

#include "core/ethercat.h"
#include "device/peripheral/process_data_pool.hpp"
#include "core/libxr_def.hpp"

namespace LibXR::EtherCAT
{

/**
 * C++ owner for one SOES EtherCAT slave instance and its LibXR peripheral bindings.
 *
 * SOES itself uses global callback symbols, so one active Slave is supported per image.
 */
class Slave
{
 public:
  explicit Slave(ProcessDataPool& process_data) : process_data_(process_data) {}

  ~Slave()
  {
    if (active_ == this)
    {
      active_ = nullptr;
    }
  }

  Slave(const Slave&) = delete;
  Slave& operator=(const Slave&) = delete;

  ErrorCode Initialize()
  {
    if (initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    if (active_ != nullptr && active_ != this)
    {
      return ErrorCode::BUSY;
    }
    active_ = this;
    ethercat_slave_init();
    initialized_ = true;
    return ErrorCode::OK;
  }

  ErrorCode Poll()
  {
    if (!initialized_)
    {
      return ErrorCode::STATE_ERR;
    }
    ethercat_slave_poll();
    return ErrorCode::OK;
  }

  static void DispatchInputs(bool in_isr)
  {
    if (active_ != nullptr)
    {
      active_->process_data_.UpdateInputs(in_isr);
    }
  }

  static void DispatchOutputs(bool in_isr)
  {
    if (active_ != nullptr)
    {
      active_->process_data_.UpdateOutputs(in_isr);
    }
  }

  static void DispatchSafeOutputs(bool in_isr)
  {
    if (active_ != nullptr)
    {
      active_->process_data_.ApplySafeOutputs(in_isr);
    }
  }

 private:
  inline static Slave* active_ = nullptr;

  ProcessDataPool& process_data_;
  bool initialized_ = false;
};

}  // namespace LibXR::EtherCAT
