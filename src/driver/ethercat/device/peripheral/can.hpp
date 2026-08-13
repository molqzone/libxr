#pragma once

#include <cstdint>

#include "driver/can.hpp"
#include "process_data.hpp"

namespace LibXR::EtherCAT
{

/** Publish the latest subscribed CAN frame into a TxPDO field. */
class CanInput final : public ProcessData
{
 public:
  CanInput(CAN& can, CAN::ClassicPack& pdo_frame, uint32_t& pdo_sequence,
           CAN::Type type = CAN::Type::STANDARD)
      : ProcessData(Direction::INPUT),
        can_(can),
        pdo_frame_(pdo_frame),
        pdo_sequence_(pdo_sequence),
        callback_(CAN::Callback::Create(OnFrameStatic, this))
  {
    can_.Register(callback_, type);
  }

  /** This binding must outlive the CAN object because CAN has no unregister API. */
  void UpdateInput(bool) override {}

 private:
  static void OnFrameStatic(bool, CanInput* self, const CAN::ClassicPack& frame)
  {
    self->pdo_frame_ = frame;
    self->pdo_sequence_++;
  }

  CAN& can_;
  CAN::ClassicPack& pdo_frame_;
  uint32_t& pdo_sequence_;
  CAN::Callback callback_;
};

/** Transmit an RxPDO CAN frame when its application-owned generation changes. */
class CanOutput final : public ProcessData
{
 public:
  CanOutput(CAN& can, const CAN::ClassicPack& pdo_frame, const uint32_t& pdo_generation)
      : ProcessData(Direction::OUTPUT),
        can_(can),
        pdo_frame_(pdo_frame),
        pdo_generation_(pdo_generation),
        sent_generation_(pdo_generation)
  {
  }

  void UpdateOutput(bool) override
  {
    if (pdo_generation_ == sent_generation_ || pdo_frame_.dlc > sizeof(pdo_frame_.data))
    {
      return;
    }
    if (can_.AddMessage(pdo_frame_) == ErrorCode::OK)
    {
      sent_generation_ = pdo_generation_;
    }
  }

 private:
  CAN& can_;
  const CAN::ClassicPack& pdo_frame_;
  const uint32_t& pdo_generation_;
  uint32_t sent_generation_;
};

}  // namespace LibXR::EtherCAT
