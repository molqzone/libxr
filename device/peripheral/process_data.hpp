#pragma once

#include <cstdint>

namespace LibXR::EtherCAT
{

/** One application-side process-data binding serviced by the EtherCAT cycle. */
class ProcessData
{
 public:
  enum class Direction : uint8_t
  {
    INPUT,
    OUTPUT,
    BOTH,
  };

  explicit ProcessData(Direction direction) : direction_(direction) {}
  virtual ~ProcessData() = default;

  ProcessData(const ProcessData&) = delete;
  ProcessData& operator=(const ProcessData&) = delete;

  [[nodiscard]] Direction GetDirection() const { return direction_; }

  /** Refresh a TxPDO field from a LibXR peripheral before SOES packs inputs. */
  virtual void UpdateInput(bool) {}

  /** Apply an RxPDO field to a LibXR peripheral after SOES unpacks outputs. */
  virtual void UpdateOutput(bool) {}

  /** Drive an output binding to its deterministic safe state. */
  virtual void ApplySafeOutput(bool) {}

 private:
  Direction direction_;
};

}  // namespace LibXR::EtherCAT
