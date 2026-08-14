#pragma once

#include "core/object_dictionary.hpp"
#include "core/slave_state.hpp"

namespace LibXR::EtherCAT
{

class SlaveBuilder;
class SlaveComposition;

/**
 * One functional contribution to an EtherCAT slave.
 *
 * Classes describe their own CoE objects and PDO mappings. SlaveComposition
 * combines completed classes into one slave before SlaveCore starts handling
 * protocol events.
 */
class SlaveClass
{
 public:
  virtual ~SlaveClass() = default;

 protected:
  virtual void Describe(SlaveBuilder& builder) = 0;
  virtual void OnStateChanged(SlaveState from, SlaveState to)
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
  friend class SlaveComposition;
};

}  // namespace LibXR::EtherCAT
