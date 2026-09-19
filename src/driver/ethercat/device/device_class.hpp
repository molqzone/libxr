#pragma once

#include "core/object_dictionary.hpp"
#include "core/al_state.hpp"

namespace LibXR::EtherCAT
{

class DeviceBuilder;
class DeviceComposition;

/**
 * One functional contribution to an EtherCAT slave.
 *
 * Classes describe their own CoE objects and PDO mappings. DeviceComposition
 * combines completed classes into one device before DeviceCore starts handling
 * protocol events.
 */
class DeviceClass
{
 public:
  virtual ~DeviceClass() = default;

 protected:
  virtual void Describe(DeviceBuilder& builder) = 0;

  // Every runtime hook carries the context it is called from, the same way the
  // LibXR USB device classes do. A memory-mapped ESC runs the whole protocol
  // core from the ISR, a bus-attached ESC runs it from a thread, so a class
  // must be able to tell where it is before it touches anything blocking or
  // shared. Deferring work is then the class's own decision, e.g. through
  // LibXR::ASync::AssignJobFromCallback(job, in_isr).

  /** @param in_isr 是否在中断上下文中调用 / Called from an interrupt context. */
  virtual void OnStateChanged(bool in_isr, AlState from, AlState to)
  {
    (void)in_isr;
    (void)from;
    (void)to;
  }
  virtual void OnOutputsUpdated(bool in_isr) { (void)in_isr; }
  virtual void OnInputsRequested(bool in_isr) { (void)in_isr; }
  virtual ErrorCode OnObjectRead(bool in_isr, ObjectEntry& entry)
  {
    (void)in_isr;
    (void)entry;
    return ErrorCode::OK;
  }
  virtual ErrorCode OnObjectWrite(bool in_isr, ObjectEntry& entry)
  {
    (void)in_isr;
    (void)entry;
    return ErrorCode::OK;
  }

 private:
  friend class DeviceComposition;
};

}  // namespace LibXR::EtherCAT
