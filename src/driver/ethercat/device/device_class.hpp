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
  virtual void OnStateChanged(AlState from, AlState to)
  {
    (void)from;
    (void)to;
  }
  virtual void OnOutputsUpdated() {}
  virtual void OnInputsRequested() {}
  virtual ErrorCode OnObjectRead(ObjectEntry& entry)
  {
    (void)entry;
    return ErrorCode::OK;
  }
  virtual ErrorCode OnObjectWrite(ObjectEntry& entry)
  {
    (void)entry;
    return ErrorCode::OK;
  }

 private:
  friend class DeviceComposition;
};

}  // namespace LibXR::EtherCAT
