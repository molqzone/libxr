#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "debug/sdi.hpp"
#include "dev_core.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "usb/core/desc_cfg.hpp"

namespace LibXR::USB
{
/**
 * @brief WCH-Link RV compatible USB class (bring-up skeleton).
 *
 * This class implements the WCH-Link RV bulk endpoint topology:
 * - Command plane: EP1 OUT (0x01), EP1 IN (0x81)
 * - Data plane:    EP2 OUT (0x02), EP2 IN (0x82)
 *
 * Current goal is protocol bring-up for host-side command sequencing with
 * real SDI DMI transactions.
 */
template <typename SdiPort>
class WchLinkRvClass : public DeviceClass
{
 public:
  struct ProbeIdentity
  {
    uint8_t major = 0x02;     ///< Firmware major reported by 0x0d/0x01.
    uint8_t minor = 0x12;     ///< Firmware minor reported by 0x0d/0x01.
    uint8_t variant = 0x12;   ///< Link variant reported by 0x0d/0x01.
    uint8_t chip_family = 0u; ///< Runtime-detected or host-selected family code.
    uint32_t chip_id = 0u;    ///< Runtime-detected chip id.
  };

  explicit WchLinkRvClass(
      SdiPort& sdi_link, LibXR::GPIO* nreset_gpio = nullptr,
      Endpoint::EPNumber command_ep_num = Endpoint::EPNumber::EP1,
      Endpoint::EPNumber data_ep_num = Endpoint::EPNumber::EP2)
      : sdi_(sdi_link),
        nreset_gpio_(nreset_gpio),
        command_ep_num_(command_ep_num),
        data_ep_num_(data_ep_num)
  {
    ResetRuntimeModel();
  }

  ~WchLinkRvClass() override = default;

  WchLinkRvClass(const WchLinkRvClass&) = delete;
  WchLinkRvClass& operator=(const WchLinkRvClass&) = delete;

  void SetProbeIdentity(const ProbeIdentity& id) { probe_id_ = id; }

  /**
   * @brief For future hardware mode, expose direct SDI access.
   */
  SdiPort& SdiLink() { return sdi_; }

 protected:
  void BindEndpoints(EndpointPool& endpoint_pool, uint8_t start_itf_num, bool) override
  {
    inited_ = false;
    interface_num_ = start_itf_num;

    auto ec = endpoint_pool.Get(ep_cmd_out_, Endpoint::Direction::OUT, command_ep_num_);
    ASSERT(ec == ErrorCode::OK);
    ec = endpoint_pool.Get(ep_cmd_in_, Endpoint::Direction::IN, command_ep_num_);
    ASSERT(ec == ErrorCode::OK);
    ec = endpoint_pool.Get(ep_data_out_, Endpoint::Direction::OUT, data_ep_num_);
    ASSERT(ec == ErrorCode::OK);
    ec = endpoint_pool.Get(ep_data_in_, Endpoint::Direction::IN, data_ep_num_);
    ASSERT(ec == ErrorCode::OK);

    // Cold-start cleanup: clear any residual state/packet from previous sessions.
    ep_cmd_out_->Close();
    ep_cmd_in_->Close();
    ep_data_out_->Close();
    ep_data_in_->Close();
    ep_cmd_out_->SetActiveLength(0u);
    ep_cmd_in_->SetActiveLength(0u);
    ep_data_out_->SetActiveLength(0u);
    ep_data_in_->SetActiveLength(0u);

    ep_cmd_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, CMD_PACKET_SIZE, false});
    ep_cmd_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, CMD_PACKET_SIZE, false});
    ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, DATA_PACKET_SIZE, false});
    ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, DATA_PACKET_SIZE, false});

    ep_cmd_out_->SetOnTransferCompleteCallback(on_cmd_out_cb_);
    ep_cmd_in_->SetOnTransferCompleteCallback(on_cmd_in_cb_);
    ep_data_out_->SetOnTransferCompleteCallback(on_data_out_cb_);
    ep_data_in_->SetOnTransferCompleteCallback(on_data_in_cb_);

    desc_block_.intf = {9,
                        static_cast<uint8_t>(DescriptorType::INTERFACE),
                        interface_num_,
                        0,
                        4,
                        0xFF,  // vendor specific
                        0x00,
                        0x00,
                        0};

    desc_block_.ep_cmd_out = {7,
                              static_cast<uint8_t>(DescriptorType::ENDPOINT),
                              static_cast<uint8_t>(ep_cmd_out_->GetAddress()),
                              static_cast<uint8_t>(Endpoint::Type::BULK),
                              ep_cmd_out_->MaxPacketSize(),
                              0};
    desc_block_.ep_cmd_in = {7,
                             static_cast<uint8_t>(DescriptorType::ENDPOINT),
                             static_cast<uint8_t>(ep_cmd_in_->GetAddress()),
                             static_cast<uint8_t>(Endpoint::Type::BULK),
                             ep_cmd_in_->MaxPacketSize(),
                             0};
    desc_block_.ep_data_out = {7,
                               static_cast<uint8_t>(DescriptorType::ENDPOINT),
                               static_cast<uint8_t>(ep_data_out_->GetAddress()),
                               static_cast<uint8_t>(Endpoint::Type::BULK),
                               ep_data_out_->MaxPacketSize(),
                               0};
    desc_block_.ep_data_in = {7,
                              static_cast<uint8_t>(DescriptorType::ENDPOINT),
                              static_cast<uint8_t>(ep_data_in_->GetAddress()),
                              static_cast<uint8_t>(Endpoint::Type::BULK),
                              ep_data_in_->MaxPacketSize(),
                              0};

    SetData(RawData{reinterpret_cast<uint8_t*>(&desc_block_), sizeof(desc_block_)});

    pending_cmd_valid_ = false;
    session_started_ = false;
    ResetRuntimeModel();

    inited_ = true;
    ArmCommandOutIfIdle();
    ArmDataOutIfIdle();
  }

  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    pending_cmd_valid_ = false;
    session_started_ = false;

    ReleaseEndpoint(endpoint_pool, ep_cmd_in_);
    ReleaseEndpoint(endpoint_pool, ep_cmd_out_);
    ReleaseEndpoint(endpoint_pool, ep_data_in_);
    ReleaseEndpoint(endpoint_pool, ep_data_out_);

    sdi_.Close();
    ResetRuntimeModel();
  }

  size_t GetInterfaceCount() override { return 1; }
  bool HasIAD() override { return false; }
  size_t GetMaxConfigSize() override { return sizeof(desc_block_); }

  bool OwnsEndpoint(uint8_t ep_addr) const override
  {
    if (!inited_)
    {
      return false;
    }
    return (ep_cmd_in_ && ep_addr == ep_cmd_in_->GetAddress()) ||
           (ep_cmd_out_ && ep_addr == ep_cmd_out_->GetAddress()) ||
           (ep_data_in_ && ep_addr == ep_data_in_->GetAddress()) ||
           (ep_data_out_ && ep_addr == ep_data_out_->GetAddress());
  }

 private:
  static void OnCommandOutStatic(bool in_isr, WchLinkRvClass* self, LibXR::ConstRawData& data)
  {
    if (self && self->inited_)
    {
      self->OnCommandOut(in_isr, data);
    }
  }

  static void OnCommandInStatic(bool in_isr, WchLinkRvClass* self, LibXR::ConstRawData& data)
  {
    if (self && self->inited_)
    {
      self->OnCommandIn(in_isr, data);
    }
  }

  static void OnDataOutStatic(bool in_isr, WchLinkRvClass* self, LibXR::ConstRawData& data)
  {
    if (self && self->inited_)
    {
      self->OnDataOut(in_isr, data);
    }
  }

  static void OnDataInStatic(bool in_isr, WchLinkRvClass* self, LibXR::ConstRawData& data)
  {
    if (self && self->inited_)
    {
      self->OnDataIn(in_isr, data);
    }
  }

  void OnCommandOut(bool /*in_isr*/, LibXR::ConstRawData& data)
  {
    TryFinalizeDataInRecovery();

    const auto* req_raw = static_cast<const uint8_t*>(data.addr_);
    const uint16_t req_raw_len = static_cast<uint16_t>(data.size_);
    if (!req_raw || req_raw_len == 0u)
    {
      ArmCommandOutIfIdle();
      return;
    }

    const uint16_t req_len =
        (req_raw_len <= CMD_PACKET_SIZE) ? req_raw_len : static_cast<uint16_t>(CMD_PACKET_SIZE);
    std::array<uint8_t, CMD_PACKET_SIZE> req_shadow = {};
    std::memcpy(req_shadow.data(), req_raw, req_len);
    const auto* req = req_shadow.data();

    const bool start_with_probe_info = IsGetProbeInfoRequest(req, req_len);
    const bool start_with_attach = IsAttachChipRequest(req, req_len);

    // Drop stale queued OUT frames from prior sessions until the host starts
    // with a clean GetProbeInfo transaction. Accept AttachChip as an
    // alternative start marker because some host flows reattach directly after
    // OptEnd without re-querying probe info.
    if (start_with_probe_info || start_with_attach)
    {
      // Treat GetProbeInfo/Attach as a fresh command-plane boundary.
      // Keep host-selected chip family only for the normal in-session path:
      // GetProbeInfo -> SetSpeed(0x0C) -> Attach(0x0D/0x02).
      // Otherwise clear stale selection to avoid cross-session carry-over.
      const bool keep_selected_chip_for_attach = start_with_attach &&
                                                 chip_family_selected_by_speed_ &&
                                                 chip_family_selected_for_next_attach_ && !attached_;
      const bool clear_host_selected_chip = !keep_selected_chip_for_attach;
      PrepareFreshSessionStart(clear_host_selected_chip);
      session_started_ = true;
    }
    else if (!session_started_)
    {
      // Keep dropping stale pre-session frames until host starts with
      // GetProbeInfo/Attach. This avoids injecting unexpected error frames into
      // host reconnect sequences.
      ArmCommandOutIfIdle();
      return;
    }

    std::array<uint8_t, CMD_PACKET_SIZE> resp_shadow = {};
    uint16_t out_len = 0u;
    (void)HandleCommand(req, req_len, resp_shadow.data(), static_cast<uint16_t>(resp_shadow.size()),
                        out_len);
    QueueCommandResponse(resp_shadow.data(), out_len);
  }

  void QueueCommandResponse(const uint8_t* resp, uint16_t out_len)
  {
    if (!resp || out_len == 0u)
    {
      // Avoid silent no-response path: return explicit protocol error when the
      // command handler produced an empty payload.
      if (TryStartProtocolErrorCommandIn(0x55u))
      {
        pending_cmd_overflow_count_ = 0u;
        return;
      }
      QueueCommandOverflowError();
      ArmCommandOutIfIdle();
      return;
    }

    if (TryStartCommandInPayloadWithRecovery(resp, out_len))
    {
      // Do not inject delayed historical overflow errors after a successful
      // command response, to keep host request/response pairing stable.
      pending_cmd_overflow_count_ = 0u;
      return;
    }

    if (ep_cmd_in_ && ep_cmd_in_->GetState() == Endpoint::State::BUSY &&
        !pending_cmd_valid_ && out_len <= pending_cmd_buf_.size())
    {
      std::memcpy(pending_cmd_buf_.data(), resp, out_len);
      pending_cmd_len_ = out_len;
      pending_cmd_valid_ = true;
      return;
    }

    // Queue is already occupied or payload does not fit software backup
    // buffer. Defer an explicit protocol error frame to avoid silent drop.
    if (TryStartProtocolErrorCommandIn(0x55u))
    {
      return;
    }
    QueueCommandOverflowError();
    ArmCommandOutIfIdle();
  }

  void ResetBulkInEndpoint(Endpoint* ep_in, uint16_t packet_size,
                           const LibXR::Callback<LibXR::ConstRawData&>& in_cb)
  {
    if (!ep_in)
    {
      return;
    }

    ep_in->Close();
    ep_in->SetActiveLength(0u);
    ep_in->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, packet_size, false});
    ep_in->SetOnTransferCompleteCallback(in_cb);
  }

  void ResetBulkInAndRestoreOut(Endpoint* ep_out, Endpoint* ep_in, uint16_t packet_size,
                                const LibXR::Callback<LibXR::ConstRawData&>& out_cb,
                                const LibXR::Callback<LibXR::ConstRawData&>& in_cb)
  {
    if (!ep_out || !ep_in)
    {
      return;
    }

    // Keep OUT endpoint object alive and reconfigure it after IN close, because
    // some backends disable both directions when IN is closed.
    ResetBulkInEndpoint(ep_in, packet_size, in_cb);
    ep_out->SetActiveLength(0u);
    ep_out->Configure({Endpoint::Direction::OUT, Endpoint::Type::BULK, packet_size, false});
    ep_out->SetOnTransferCompleteCallback(out_cb);
  }

  void ResetCommandPlaneEndpoints(bool arm_out = true)
  {
    ResetBulkInAndRestoreOut(ep_cmd_out_, ep_cmd_in_, CMD_PACKET_SIZE, on_cmd_out_cb_, on_cmd_in_cb_);
    if (arm_out)
    {
      ArmCommandOutIfIdle();
    }
  }

  void ResetDataPlaneEndpoints(bool arm_out = true)
  {
    ResetBulkInAndRestoreOut(ep_data_out_, ep_data_in_, DATA_PACKET_SIZE, on_data_out_cb_, on_data_in_cb_);
    if (arm_out)
    {
      ArmDataOutIfIdle();
    }
  }

  void ResetDataInEndpoint()
  {
    data_ack_in_flight_ = false;
    ResetBulkInEndpoint(ep_data_in_, DATA_PACKET_SIZE, on_data_in_cb_);
    ArmDataOutIfIdle();
  }

  void ResetCommandInEndpoint()
  {
    ResetBulkInEndpoint(ep_cmd_in_, CMD_PACKET_SIZE, on_cmd_in_cb_);
  }

  bool TryStartCommandInPayloadWithRecovery(const uint8_t* payload, uint16_t len)
  {
    if (!payload || len == 0u || !ep_cmd_in_)
    {
      return false;
    }

    auto try_start_once = [&]() -> bool
    {
      if (!ep_cmd_in_ || ep_cmd_in_->GetState() != Endpoint::State::IDLE)
      {
        return false;
      }
      auto tx = ep_cmd_in_->GetBuffer();
      if (!tx.addr_ || tx.size_ < len)
      {
        return false;
      }
      std::memcpy(tx.addr_, payload, len);
      return ep_cmd_in_->Transfer(len) == ErrorCode::OK;
    };

    if (try_start_once())
    {
      return true;
    }

    // Busy is handled by normal completion callback retry path.
    if (!ep_cmd_in_ || ep_cmd_in_->GetState() == Endpoint::State::BUSY)
    {
      return false;
    }

    // Hard failure while not busy: recover command IN only. Avoid touching
    // OUT here because OUT may already have an in-flight host transfer.
    ResetCommandInEndpoint();
    return try_start_once();
  }

  bool TryStartProtocolErrorCommandIn(uint8_t reason)
  {
    uint8_t err[3] = {};
    uint16_t out_len = 0u;
    if (BuildErrorResponse(reason, err, static_cast<uint16_t>(sizeof(err)), out_len) != ErrorCode::OK)
    {
      return false;
    }
    return TryStartCommandInPayloadWithRecovery(err, out_len);
  }

  void ScheduleDataInRecovery()
  {
    data_in_recovery_pending_ = true;
    data_in_recovery_spins_ = 0u;
  }

  void TryFinalizeDataInRecovery()
  {
    if (!data_in_recovery_pending_)
    {
      return;
    }

    if (!IsDataInBusy())
    {
      data_in_recovery_pending_ = false;
      data_in_recovery_spins_ = 0u;
      read_stream_fault_pending_finalize_ = false;
      ResetDataInEndpoint();
      return;
    }

    if (data_in_recovery_spins_ < kDataInRecoveryForceAfterSpins)
    {
      ++data_in_recovery_spins_;
      return;
    }

    // Recovery deadline reached: force-close EP2 IN to break a stuck BUSY
    // state and avoid indefinite ACK/data plane starvation.
    data_in_recovery_pending_ = false;
    data_in_recovery_spins_ = 0u;
    read_stream_fault_pending_finalize_ = false;
    ResetDataInEndpoint();
  }

  void OnCommandIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    TryFinalizeDataInRecovery();

    if (!pending_cmd_valid_)
    {
      if (pending_cmd_overflow_count_ != 0u)
      {
        if (TryStartProtocolErrorCommandIn(0x55u))
        {
          --pending_cmd_overflow_count_;
          return;
        }
        ArmCommandOutIfIdle();
        return;
      }
      PumpReadStreamIfIdle();
      ArmCommandOutIfIdle();
      return;
    }

    if (TryStartCommandInPayloadWithRecovery(pending_cmd_buf_.data(), pending_cmd_len_))
    {
      pending_cmd_valid_ = false;
      pending_cmd_len_ = 0u;
      pending_cmd_overflow_count_ = 0u;
      return;
    }

    // Do bounded in-callback retries before dropping the pending response.
    for (uint8_t i = 0u; i < kPendingCmdSendRetryLimit; ++i)
    {
      if (TryStartCommandInPayloadWithRecovery(pending_cmd_buf_.data(), pending_cmd_len_))
      {
        pending_cmd_valid_ = false;
        pending_cmd_len_ = 0u;
        pending_cmd_overflow_count_ = 0u;
        return;
      }
    }

    pending_cmd_valid_ = false;
    pending_cmd_len_ = 0u;
    if (TryStartProtocolErrorCommandIn(0x55u))
    {
      return;
    }
    QueueCommandOverflowError();
    ArmCommandOutIfIdle();
  }

  void OnDataOut(bool /*in_isr*/, LibXR::ConstRawData& data)
  {
    TryFinalizeDataInRecovery();

    const auto* rx = static_cast<const uint8_t*>(data.addr_);
    const uint32_t rx_bytes = static_cast<uint32_t>(data.size_);
    if (program_mode_ == ProgramMode::WRITE_FLASH_OP)
    {
      flash_op_rx_bytes_ += rx_bytes;
    }
    else if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM)
    {
      ProcessFlashStreamData(rx, rx_bytes);
    }
    FlushPendingDataAck();
    ArmDataOutIfIdle();
  }

  void OnDataIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    TryFinalizeDataInRecovery();
    if (data_ack_in_flight_)
    {
      data_ack_in_flight_ = false;
      if (pending_data_ack_ > 0u)
      {
        --pending_data_ack_;
      }
    }

    if (read_stream_active_)
    {
      PumpReadStreamIfIdle();
      return;
    }
    FlushPendingDataAck();
  }

  void PrepareFreshSessionStart(bool clear_host_selected_chip)
  {
    // Host reconnection may leave unread/stale IN payload in previous session.
    // Reset software queue and model state to guarantee the new session starts
    // from GetProbeInfo/Attach on a clean command/data plane.
    pending_cmd_valid_ = false;
    pending_cmd_len_ = 0u;
    pending_cmd_overflow_count_ = 0u;
    attached_ = false;
    read_stream_fault_latched_ = false;
    read_stream_fault_pending_finalize_ = false;
    if (clear_host_selected_chip)
    {
      ClearHostSelectedChipFamily();
    }
    ExitProgramStream();
    ExitReadStream();
    sdi_.Close();

    // Do not re-arm EP1 OUT here: we are still handling the current command
    // in OnCommandOut. Re-arming too early can allow command overlap on single
    // EP1 IN buffer and corrupt in-flight responses.
    ResetCommandPlaneEndpoints(false);

    // Avoid aggressive EP2 OUT reconfigure in command callback. Recover EP2 IN
    // only and keep OUT side naturally drained/armed.
    if (IsDataInBusy())
    {
      ScheduleDataInRecovery();
    }
    else
    {
      data_in_recovery_pending_ = false;
      data_in_recovery_spins_ = 0u;
      ResetDataInEndpoint();
    }
  }

  void ResetRuntimeModel()
  {
    attached_ = false;
    ClearHostSelectedChipFamily();
    write_region_addr_ = 0u;
    write_region_len_ = 0u;
    read_region_addr_ = 0u;
    read_region_len_ = 0u;
    current_sdi_clock_hz_ = kSdiClockHzHigh;
    program_mode_ = ProgramMode::IDLE;
    flash_op_ready_ = false;
    flash_op_rx_bytes_ = 0u;
    flash_stream_chunk_bytes_ = kDefaultWritePackSize;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
    pending_cmd_overflow_count_ = 0u;
    read_stream_active_ = false;
    read_stream_fault_latched_ = false;
    read_stream_fault_pending_finalize_ = false;
    data_in_recovery_pending_ = false;
    data_in_recovery_spins_ = 0u;
    read_stream_addr_ = 0u;
    read_stream_remaining_ = 0u;
    read_stream_error_ = false;
  }

  static uint32_t LoadBe32(const uint8_t* p)
  {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
  }

  static uint32_t LoadLe32(const uint8_t* p)
  {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  }

  static void StoreBe32(uint8_t* p, uint32_t v)
  {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
  }

  static uint32_t DecodeSdiClockHz(uint8_t speed_code)
  {
    switch (speed_code)
    {
      case 0x03u:
        return kSdiClockHzLow;
      case 0x02u:
        return kSdiClockHzMedium;
      case 0x01u:
      default:
        return kSdiClockHzHigh;
    }
  }

  static uint32_t DecodeWritePackBytes(uint8_t chip_family)
  {
    switch (chip_family)
    {
      case 0x09u:
      case 0x0Au:
        return 1024u;
      default:
        return kDefaultWritePackSize;
    }
  }

  static bool IsFlashOpLengthValid(uint8_t chip_family, uint32_t bytes)
  {
    // Length table is derived from wlink flash_op defaults (and known alt
    // variants) to guard program stage 0x07/0x0B against partial/garbled EP2
    // payloads.
    switch (chip_family)
    {
      case 0x01u:  // CH32V103
        return bytes == 494u;
      case 0x02u:  // CH57X
        return bytes == 1102u;
      case 0x03u:  // CH56X
        return bytes == 1156u;
      case 0x05u:  // CH32V20X
      case 0x06u:  // CH32V30X
        return bytes == 446u;
      case 0x07u:  // CH582/583
      case 0x0Bu:  // CH59X
      case 0x4Bu:  // CH585
        return bytes == 1326u;
      case 0x09u:  // CH32V003
      case 0x49u:  // CH641
        return bytes == 498u || bytes == 466u;
      case 0x0Au:  // CH8571
        return bytes == 1408u || bytes == 1386u;
      case 0x0Cu:  // CH643
      case 0x0Du:  // CH32X035
        return bytes == 488u;
      case 0x0Eu:  // CH32L103
        return bytes == 512u;
      case 0x0Fu:  // CH564
        return bytes == 1532u;
      case 0x4Eu:  // CH32V007
        return bytes == 500u;
      case 0x46u:  // CH645
        return bytes == 440u || bytes == 486u;
      case 0x86u:  // CH32V317
        return bytes == 440u || bytes == 460u;
      default:
        // Unknown family: keep compatibility, but still reject empty flash-op.
        return bytes > 0u;
    }
  }

  static uint32_t MinU32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

  static uint8_t AckToDmiOp(LibXR::Debug::Sdi::Ack ack)
  {
    switch (ack)
    {
      case LibXR::Debug::Sdi::Ack::OK:
        return 0x00u;
      case LibXR::Debug::Sdi::Ack::BUSY:
        return 0x03u;
      case LibXR::Debug::Sdi::Ack::FAILED:
      case LibXR::Debug::Sdi::Ack::RESERVED:
      case LibXR::Debug::Sdi::Ack::PROTOCOL:
      default:
        return 0x02u;
    }
  }

  static ErrorCode BuildDmiResponse(uint8_t addr, uint32_t data, uint8_t op, uint8_t* resp,
                                    uint16_t cap, uint16_t& out_len)
  {
    uint8_t dmi_resp[6] = {};
    dmi_resp[0] = addr;
    StoreBe32(dmi_resp + 1u, data);
    dmi_resp[5] = op;
    return BuildStandardResponse(0x08u, dmi_resp, sizeof(dmi_resp), resp, cap, out_len);
  }

  static ErrorCode BuildDmiNotAttachedResponse(uint8_t* resp, uint16_t cap, uint16_t& out_len)
  {
    // Follow host-side not-attached sentinel used by wlink:
    // addr=0x7d, data=0xffffffff, op=0x03(busy).
    return BuildDmiResponse(0x7Du, 0xFFFFFFFFu, 0x03u, resp, cap, out_len);
  }

  static ErrorCode BuildErrorResponse(uint8_t reason, uint8_t* resp, uint16_t cap,
                                      uint16_t& out_len)
  {
    if (!resp || cap < 3u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = 0x81u;
    resp[1] = reason;
    resp[2] = 0x00u;
    out_len = 3u;
    return ErrorCode::OK;
  }

  static ErrorCode BuildStandardResponse(uint8_t cmd, const uint8_t* payload,
                                         uint16_t payload_len, uint8_t* resp, uint16_t cap,
                                         uint16_t& out_len)
  {
    if (!resp || cap < static_cast<uint16_t>(payload_len + 3u))
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = 0x82u;
    resp[1] = cmd;
    resp[2] = static_cast<uint8_t>(payload_len);
    if (payload_len > 0u && payload)
    {
      std::memcpy(resp + 3u, payload, payload_len);
    }
    out_len = static_cast<uint16_t>(payload_len + 3u);
    return ErrorCode::OK;
  }

  static bool IsDmHalted(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 9u)) != 0u && (dmstatus & (1u << 8u)) != 0u;
  }

  static bool IsDmRunning(uint32_t dmstatus)
  {
    return (dmstatus & (1u << 11u)) != 0u && (dmstatus & (1u << 10u)) != 0u;
  }

  uint8_t ActiveChipFamily() const
  {
    if (requested_chip_family_ != 0u)
    {
      return requested_chip_family_;
    }
    return probe_id_.chip_family;
  }

  void ClearHostSelectedChipFamily()
  {
    requested_chip_family_ = 0u;
    chip_family_selected_by_speed_ = false;
    chip_family_selected_for_next_attach_ = false;
  }

  bool IsDataInBusy() const
  {
    return ep_data_in_ != nullptr && ep_data_in_->GetState() == Endpoint::State::BUSY;
  }

  void HandleReadStreamLinkFault(bool defer_data_in_finalize = false)
  {
    read_stream_active_ = false;
    attached_ = false;
    read_stream_fault_latched_ = true;
    const bool need_defer = defer_data_in_finalize || IsDataInBusy();
    read_stream_fault_pending_finalize_ = need_defer;
    if (need_defer)
    {
      ScheduleDataInRecovery();
    }
    sdi_.Close();
    if (!need_defer)
    {
      data_in_recovery_pending_ = false;
      data_in_recovery_spins_ = 0u;
      ResetDataInEndpoint();
    }
  }

  void QueueDataAck(uint8_t count = 1u)
  {
    if (count == 0u)
    {
      return;
    }
    const uint16_t next = static_cast<uint16_t>(pending_data_ack_) + static_cast<uint16_t>(count);
    pending_data_ack_ = (next > 0xFFu) ? 0xFFu : static_cast<uint8_t>(next);
  }

  void FlushPendingDataAck()
  {
    if (pending_data_ack_ == 0u || data_ack_in_flight_)
    {
      return;
    }
    (void)TrySendDataAck();
  }

  void EnterFlashOpStream()
  {
    ExitReadStream();
    ResetDataInEndpoint();
    program_mode_ = ProgramMode::WRITE_FLASH_OP;
    flash_op_rx_bytes_ = 0u;
    flash_op_ready_ = false;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
  }

  void ExitProgramStream()
  {
    program_mode_ = ProgramMode::IDLE;
    flash_op_ready_ = false;
    flash_op_rx_bytes_ = 0u;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
  }

  bool EnterFlashWriteStream()
  {
    if (!flash_op_ready_)
    {
      return false;
    }

    flash_stream_total_raw_bytes_ = write_region_len_;
    if (flash_stream_total_raw_bytes_ == 0u)
    {
      return false;
    }

    const uint32_t chunk = DecodeWritePackBytes(ActiveChipFamily());
    flash_stream_chunk_bytes_ = (chunk == 0u) ? kDefaultWritePackSize : chunk;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = MinU32(flash_stream_chunk_bytes_, flash_stream_total_raw_bytes_);
    flash_stream_write_addr_ = write_region_addr_;
    flash_stream_error_ = false;
    pending_data_ack_ = 0u;
    program_mode_ = ProgramMode::WRITE_FLASH_STREAM;
    return true;
  }

  uint8_t PendingDataAckBacklog() const
  {
    if (pending_data_ack_ == 0u)
    {
      return 0u;
    }
    if (data_ack_in_flight_ && pending_data_ack_ > 0u)
    {
      return static_cast<uint8_t>(pending_data_ack_ - 1u);
    }
    return pending_data_ack_;
  }

  void ProcessFlashStreamData(const uint8_t* data, uint32_t rx_bytes)
  {
    if (!data || rx_bytes == 0u || flash_stream_next_ack_at_ == 0u || flash_stream_error_)
    {
      return;
    }

    if (flash_stream_rx_bytes_ >= flash_stream_total_raw_bytes_)
    {
      return;
    }
    const uint32_t remain_total = flash_stream_total_raw_bytes_ - flash_stream_rx_bytes_;
    const uint32_t consume = MinU32(rx_bytes, remain_total);
    if (!WriteFlashStreamChunk(data, consume))
    {
      flash_stream_error_ = true;
      return;
    }
    flash_stream_rx_bytes_ += consume;
    QueueFlashWriteAcks();
  }

  bool IsFlashWriteFinished() const
  {
    return flash_stream_total_raw_bytes_ > 0u && flash_stream_rx_bytes_ >= flash_stream_total_raw_bytes_;
  }

  void QueueFlashWriteAcks()
  {
    while (flash_stream_rx_bytes_ >= flash_stream_next_ack_at_)
    {
      QueueDataAck();
      if (flash_stream_next_ack_at_ >= flash_stream_total_raw_bytes_)
      {
        flash_stream_next_ack_at_ = 0u;
        break;
      }

      const uint32_t remain = flash_stream_total_raw_bytes_ - flash_stream_next_ack_at_;
      flash_stream_next_ack_at_ += MinU32(flash_stream_chunk_bytes_, remain);
    }
  }

  bool WriteFlashStreamChunk(const uint8_t* data, uint32_t size)
  {
    if (!data || size == 0u)
    {
      return true;
    }

    uint32_t off = 0u;
    while (off < size)
    {
      if (((flash_stream_write_addr_ & 0x3u) == 0u) && (size - off >= 4u))
      {
        const uint32_t word = LoadLe32(data + off);
        if (!TryWriteTargetWordByAbstract(flash_stream_write_addr_, word))
        {
          return false;
        }
        flash_stream_write_addr_ += 4u;
        off += 4u;
      }
      else
      {
        if (!TryWriteTargetByteByAbstract(flash_stream_write_addr_, data[off]))
        {
          return false;
        }
        ++flash_stream_write_addr_;
        ++off;
      }
    }
    return true;
  }

  void EnterReadStream(uint32_t addr, uint32_t len)
  {
    ExitProgramStream();
    read_stream_active_ = true;
    read_stream_fault_latched_ = false;
    read_stream_fault_pending_finalize_ = false;
    read_stream_addr_ = addr;
    read_stream_remaining_ = len;
    read_stream_error_ = false;
  }

  void ExitReadStream()
  {
    read_stream_active_ = false;
    read_stream_addr_ = 0u;
    read_stream_remaining_ = 0u;
    read_stream_error_ = false;
  }

  void PumpReadStreamIfIdle()
  {
    if (!read_stream_active_)
    {
      return;
    }
    if (read_stream_remaining_ == 0u)
    {
      read_stream_active_ = false;
      return;
    }
    (void)TrySendReadStreamChunk();
  }

  bool TrySendReadStreamChunk()
  {
    if (!read_stream_active_ || read_stream_remaining_ == 0u)
    {
      return false;
    }
    if (!ep_data_in_ || ep_data_in_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }

    auto tx = ep_data_in_->GetBuffer();
    if (!tx.addr_ || tx.size_ == 0u)
    {
      read_stream_error_ = true;
      HandleReadStreamLinkFault();
      return false;
    }

    const uint32_t chunk_bytes =
        MinU32(static_cast<uint32_t>(tx.size_), read_stream_remaining_);
    auto* out = static_cast<uint8_t*>(tx.addr_);

    uint32_t filled = 0u;
    while (filled < chunk_bytes)
    {
      uint32_t word = 0xFFFFFFFFu;
      if (!read_stream_error_)
      {
        uint32_t read_word = 0u;
        if (!TryReadTargetWordByAbstract(read_stream_addr_, read_word))
        {
          // Keep the EP2 stream progressing to avoid host-side blocking on
          // partial reads after a mid-stream SDI failure.
          read_stream_error_ = true;
        }
        else
        {
          word = read_word;
        }
      }

      const uint32_t remain = chunk_bytes - filled;
      const uint32_t emit = MinU32(4u, remain);
      uint8_t be[4] = {};
      StoreBe32(be, word);
      std::memcpy(out + filled, be, emit);

      filled += emit;
      read_stream_addr_ += 4u;
    }

    read_stream_remaining_ -= chunk_bytes;
    if (ep_data_in_->Transfer(static_cast<uint16_t>(chunk_bytes)) != ErrorCode::OK)
    {
      read_stream_error_ = true;
      HandleReadStreamLinkFault();
      return false;
    }
    if (read_stream_remaining_ == 0u)
    {
      read_stream_active_ = false;
      if (read_stream_error_)
      {
        // The final chunk is already queued on EP2 IN. Defer endpoint reopen
        // until its completion callback to avoid aborting the in-flight packet.
        HandleReadStreamLinkFault(true);
      }
    }
    return true;
  }

  bool DmiReadWord(uint8_t addr, uint32_t& data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = sdi_.DmiReadTxn(addr, data, ack);
    return ec == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool DmiWriteWord(uint8_t addr, uint32_t data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = sdi_.DmiWriteTxn(addr, data, ack);
    return ec == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool ClearAbstractCommandError() { return DmiWriteWord(kDmiAbstractcs, 0x00000700u); }

  bool WaitAbstractCommandDone()
  {
    for (uint8_t i = 0u; i < 32u; ++i)
    {
      uint32_t abstractcs = 0u;
      if (!DmiReadWord(kDmiAbstractcs, abstractcs))
      {
        return false;
      }
      if ((abstractcs & (1u << 12u)) != 0u)
      {
        continue;
      }
      if (((abstractcs >> 8u) & 0x7u) != 0u)
      {
        (void)ClearAbstractCommandError();
        return false;
      }
      return true;
    }
    return false;
  }

  bool RunAbstractCommand(uint32_t command)
  {
    if (!DmiWriteWord(kDmiCommand, command))
    {
      return false;
    }
    return WaitAbstractCommandDone();
  }

  bool EnsureHartHaltedForProbe(bool& need_resume)
  {
    need_resume = false;
    uint32_t dmstatus = 0u;
    if (!DmiReadWord(kDmiDmstatus, dmstatus))
    {
      return false;
    }
    if (IsDmHalted(dmstatus))
    {
      return true;
    }

    if (!DmiWriteWord(kDmiDmcontrol, 0x80000001u))
    {
      return false;
    }

    for (uint8_t i = 0u; i < 32u; ++i)
    {
      if (!DmiReadWord(kDmiDmstatus, dmstatus))
      {
        return false;
      }
      if (IsDmHalted(dmstatus))
      {
        if (!DmiWriteWord(kDmiDmcontrol, 0x00000001u))
        {
          return false;
        }
        need_resume = true;
        return true;
      }
    }
    return false;
  }

  void TryResumeHartAfterProbe(bool need_resume)
  {
    if (!need_resume)
    {
      return;
    }

    (void)DmiWriteWord(kDmiDmcontrol, 0x40000001u);
    for (uint8_t i = 0u; i < 16u; ++i)
    {
      uint32_t dmstatus = 0u;
      if (!DmiReadWord(kDmiDmstatus, dmstatus))
      {
        break;
      }
      if (IsDmRunning(dmstatus))
      {
        break;
      }
    }
    (void)DmiWriteWord(kDmiDmcontrol, 0x00000001u);
  }

  bool TryReadTargetWordByAbstract(uint32_t addr, uint32_t& data)
  {
    if (!DmiWriteWord(kDmiProgbuf0, 0x0002A303u))  // lw x6, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(kDmiProgbuf1, 0x00100073u))  // ebreak
    {
      return false;
    }
    if (!DmiWriteWord(kDmiData0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00271005u))  // x5 <- data0 with postexec
    {
      return false;
    }
    if (!RunAbstractCommand(0x00221006u))  // data0 <- x6
    {
      return false;
    }
    return DmiReadWord(kDmiData0, data);
  }

  bool TryWriteTargetWordByAbstract(uint32_t addr, uint32_t data)
  {
    if (!DmiWriteWord(kDmiProgbuf0, 0x0072A023u))  // sw x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(kDmiProgbuf1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00231005u))  // x5 <- data0
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, data))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool TryWriteTargetByteByAbstract(uint32_t addr, uint8_t data)
  {
    if (!DmiWriteWord(kDmiProgbuf0, 0x00728023u))  // sb x7, 0(x5)
    {
      return false;
    }
    if (!DmiWriteWord(kDmiProgbuf1, 0x00100073u))  // ebreak
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, addr))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    if (!RunAbstractCommand(0x00231005u))  // x5 <- data0
    {
      return false;
    }

    if (!DmiWriteWord(kDmiData0, static_cast<uint32_t>(data)))
    {
      return false;
    }
    if (!ClearAbstractCommandError())
    {
      return false;
    }
    return RunAbstractCommand(0x00271007u);  // x7 <- data0 with postexec
  }

  bool TryProbeChipIdentity(uint32_t& chip_id_out, uint8_t& chip_family_out)
  {
    bool need_resume = false;
    if (!EnsureHartHaltedForProbe(need_resume))
    {
      return false;
    }

    bool ok = true;
    uint32_t chip_id = 0u;
    do
    {
      if (!TryReadTargetWordByAbstract(kChipIdAddress, chip_id))
      {
        ok = false;
        break;
      }
      if (chip_id == 0u || chip_id == 0xFFFFFFFFu)
      {
        ok = false;
        break;
      }

      chip_id_out = chip_id;
      if (requested_chip_family_ != 0u)
      {
        chip_family_out = requested_chip_family_;
      }
      else
      {
        chip_family_out = probe_id_.chip_family;
      }

      uint32_t flash_size = 0u;
      if (TryReadTargetWordByAbstract(kFlashSizeAddress, flash_size))
      {
        esig_flash_size_kb_ = static_cast<uint16_t>(flash_size & 0xFFFFu);
      }

      uint32_t uid0 = 0u;
      if (TryReadTargetWordByAbstract(kUidWord0Address, uid0))
      {
        esig_uid_word0_ = uid0;
      }

      uint32_t uid1 = 0u;
      if (TryReadTargetWordByAbstract(kUidWord1Address, uid1))
      {
        esig_uid_word1_ = uid1;
      }
    } while (false);

    TryResumeHartAfterProbe(need_resume);
    return ok;
  }

  ErrorCode BuildEsigV2Response(uint8_t* resp, uint16_t cap, uint16_t& out_len) const
  {
    if (!resp || cap < 20u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    resp[0] = 0xFFu;
    resp[1] = 0xFFu;
    resp[2] = static_cast<uint8_t>(esig_flash_size_kb_ >> 8u);
    resp[3] = static_cast<uint8_t>(esig_flash_size_kb_);
    StoreBe32(resp + 4u, esig_uid_word0_);
    StoreBe32(resp + 8u, esig_uid_word1_);
    StoreBe32(resp + 12u, esig_reserved_word_);
    StoreBe32(resp + 16u, probe_id_.chip_id);
    out_len = 20u;
    return ErrorCode::OK;
  }

  ErrorCode HandleControlCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                 uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t sub = payload[0];
    if (sub == 0x01u)
    {
      const uint8_t probe_info[4] = {probe_id_.major, probe_id_.minor, probe_id_.variant, 0x00u};
      return BuildStandardResponse(0x0Du, probe_info, sizeof(probe_info), resp, cap, out_len);
    }
    if (sub == 0x02u)
    {
      attached_ = false;
      const ErrorCode ec = sdi_.EnterSdi();
      if (ec != ErrorCode::OK)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }

      uint32_t dmstatus = 0u;
      if (!DmiReadWord(kDmiDmstatus, dmstatus))
      {
        sdi_.Close();
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      UNUSED(dmstatus);

      uint32_t chip_id = probe_id_.chip_id;
      uint8_t chip_family = (requested_chip_family_ != 0u) ? requested_chip_family_ : probe_id_.chip_family;
      if (!TryProbeChipIdentity(chip_id, chip_family) &&
          !TryProbeChipIdentity(chip_id, chip_family))
      {
        // Keep attach compatibility with hosts that continue with chip-id
        // fallback when probe-id read fails.
        chip_id = 0u;
      }
      probe_id_.chip_id = chip_id;
      if (chip_family != 0u)
      {
        probe_id_.chip_family = chip_family;
      }
      attached_ = true;
      read_stream_fault_latched_ = false;
      // Consume a host-selected family only after a successful attach.
      chip_family_selected_for_next_attach_ = false;

      const uint8_t attach[5] = {
          probe_id_.chip_family,
          static_cast<uint8_t>(probe_id_.chip_id >> 24),
          static_cast<uint8_t>(probe_id_.chip_id >> 16),
          static_cast<uint8_t>(probe_id_.chip_id >> 8),
          static_cast<uint8_t>(probe_id_.chip_id)};
      return BuildStandardResponse(0x0Du, attach, sizeof(attach), resp, cap, out_len);
    }
    if (sub == 0xFFu)
    {
      attached_ = false;
      ExitProgramStream();
      ExitReadStream();
      sdi_.Close();
      read_stream_fault_latched_ = false;
      ClearHostSelectedChipFamily();
      pending_cmd_overflow_count_ = 0u;
      session_started_ = false;
      const uint8_t done[1] = {0xFFu};
      return BuildStandardResponse(0x0Du, done, sizeof(done), resp, cap, out_len);
    }

    const uint8_t pass[1] = {sub};
    return BuildStandardResponse(0x0Du, pass, sizeof(pass), resp, cap, out_len);
  }

  ErrorCode HandleConfigChipCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                    uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }
    const uint8_t sub = payload[0];
    uint8_t value = 0x00u;
    if (sub == 0x01u)
    {
      value = 0x02u;  // not protected
    }
    return BuildStandardResponse(0x06u, &value, 1u, resp, cap, out_len);
  }

  ErrorCode HandleDmiCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                             uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len != 6u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    if (!attached_)
    {
      return BuildDmiNotAttachedResponse(resp, cap, out_len);
    }

    const uint8_t addr = payload[0];
    const uint32_t data = LoadBe32(payload + 1u);
    const uint8_t op = payload[5];

    uint32_t out_data = 0u;
    uint8_t out_op = 0x00u;
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;

    if (op == 0x01u)
    {
      const ErrorCode ec = sdi_.DmiReadTxn(addr, out_data, ack);
      if (ec != ErrorCode::OK && ec != ErrorCode::FAILED && ec != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_op = AckToDmiOp(ack);
    }
    else if (op == 0x02u)
    {
      const ErrorCode ec = sdi_.DmiWriteTxn(addr, data, ack);
      if (ec != ErrorCode::OK && ec != ErrorCode::FAILED && ec != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_data = data;
      out_op = AckToDmiOp(ack);
    }
    else if (op == 0x00u)
    {
      LibXR::Debug::Sdi::Response nop_resp = {};
      const ErrorCode ec = sdi_.TransferWithRetry({addr, 0u, LibXR::Debug::Sdi::Op::NOP}, nop_resp);
      if (ec != ErrorCode::OK && ec != ErrorCode::FAILED && ec != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_data = nop_resp.data;
      out_op = AckToDmiOp(nop_resp.ack);
    }
    else
    {
      out_op = 0x02u;  // failed
    }

    return BuildDmiResponse(addr, out_data, out_op, resp, cap, out_len);
  }

  ErrorCode HandleProgramCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                 uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t sub = payload[0];
    if (sub == 0x05u)
    {
      EnterFlashOpStream();
    }
    else if (sub == 0x07u || sub == 0x0Bu)
    {
      // Protocol sequence from analysis:
      // 0x05 -> EP2 flash-op bytes -> 0x07/0x0B.
      if (program_mode_ != ProgramMode::WRITE_FLASH_OP ||
          !IsFlashOpLengthValid(ActiveChipFamily(), flash_op_rx_bytes_))
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      flash_op_ready_ = true;
      program_mode_ = ProgramMode::IDLE;
    }
    else if (sub == 0x02u || sub == 0x04u)
    {
      // Protocol sequence from analysis:
      // 0x07/0x0B must complete flash-op stage before entering write stream.
      if (program_mode_ != ProgramMode::IDLE)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      if (!EnterFlashWriteStream())
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
    }
    else if (sub == 0x08u)
    {
      FlushPendingDataAck();
      // Program End must arrive after the full write-region payload is streamed.
      if (program_mode_ != ProgramMode::WRITE_FLASH_STREAM || !IsFlashWriteFinished() ||
          PendingDataAckBacklog() != 0u || flash_stream_error_)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      ExitProgramStream();
    }
    else if (sub == 0x0Cu)
    {
      if (!attached_ || program_mode_ != ProgramMode::IDLE || (read_region_addr_ & 0x3u) != 0u ||
          (read_region_len_ & 0x3u) != 0u || read_region_len_ == 0u)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      EnterReadStream(read_region_addr_, read_region_len_);
    }
    else if (sub == 0x09u || sub == 0x01u)
    {
      ExitProgramStream();
      ExitReadStream();
    }

    return BuildStandardResponse(0x02u, &sub, 1u, resp, cap, out_len);
  }

  ErrorCode HandleCommand(const uint8_t* req, uint16_t req_len, uint8_t* resp, uint16_t cap,
                          uint16_t& out_len)
  {
    if (!req || req_len < 3u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }
    if (req[0] != 0x81u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t cmd = req[1];
    const uint8_t payload_len_u8 = req[2];
    const uint16_t payload_len = payload_len_u8;
    // Some host stacks/drivers may deliver a padded frame with trailing bytes.
    // Keep protocol parsing based on declared payload length and ignore tail.
    if (req_len < static_cast<uint16_t>(payload_len + 3u))
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t* payload = req + 3u;

    if (read_stream_fault_latched_)
    {
      const bool allow_control_recovery =
          cmd == 0x0Du && payload_len >= 1u &&
          (payload[0] == 0x01u || payload[0] == 0x02u || payload[0] == 0xFFu);
      const bool allow_set_speed = (cmd == 0x0Cu);
      // Some host flows issue program-plane cleanup before reattach.
      const bool allow_program_cleanup =
          cmd == 0x02u && payload_len >= 1u && (payload[0] == 0x09u || payload[0] == 0x01u);
      if (!allow_control_recovery && !allow_set_speed && !allow_program_cleanup)
      {
        if (cmd == 0x08u)
        {
          return BuildDmiNotAttachedResponse(resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
    }

    switch (cmd)
    {
      case 0x0Du:
        return HandleControlCommand(payload, payload_len, resp, cap, out_len);
      case 0x0Cu:
      {
        if (!payload || payload_len < 2u)
        {
          return BuildErrorResponse(0x55u, resp, cap, out_len);
        }
        const uint8_t requested_chip_family = payload[0];
        const uint32_t hz = DecodeSdiClockHz(payload[1]);
        const bool success = (sdi_.SetClockHz(hz) == ErrorCode::OK);
        if (success)
        {
          current_sdi_clock_hz_ = hz;
          requested_chip_family_ = requested_chip_family;
          chip_family_selected_by_speed_ = (requested_chip_family_ != 0u);
          chip_family_selected_for_next_attach_ = chip_family_selected_by_speed_;
        }
        const uint8_t ok[1] = {static_cast<uint8_t>(success ? 0x01u : 0x00u)};
        return BuildStandardResponse(0x0Cu, ok, sizeof(ok), resp, cap, out_len);
      }
      case 0x11u:
      {
        if (payload_len == 1u && payload[0] == 0x06u)
        {
          return BuildEsigV2Response(resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      case 0x06u:
        return HandleConfigChipCommand(payload, payload_len, resp, cap, out_len);
      case 0x08u:
        return HandleDmiCommand(payload, payload_len, resp, cap, out_len);
      case 0x01u:
      {
        if (payload_len == 8u)
        {
          write_region_addr_ = LoadBe32(payload);
          write_region_len_ = LoadBe32(payload + 4u);
          return BuildStandardResponse(0x01u, nullptr, 0u, resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      case 0x03u:
      {
        if (payload_len == 8u)
        {
          read_region_addr_ = LoadBe32(payload);
          read_region_len_ = LoadBe32(payload + 4u);
          return BuildStandardResponse(0x03u, nullptr, 0u, resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      case 0x02u:
        return HandleProgramCommand(payload, payload_len, resp, cap, out_len);
      case 0x0Bu:
      case 0x0Eu:
      case 0xFFu:
        return BuildStandardResponse(cmd, nullptr, 0u, resp, cap, out_len);
      default:
        return BuildErrorResponse(0x55u, resp, cap, out_len);
    }
  }

  bool TryStartCommandIn(uint16_t len)
  {
    if (!ep_cmd_in_)
    {
      return false;
    }
    if (ep_cmd_in_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }
    if (ep_cmd_in_->Transfer(len) != ErrorCode::OK)
    {
      return false;
    }
    return true;
  }

  bool TrySendDataAck()
  {
    if (!ep_data_in_)
    {
      return false;
    }
    if (data_ack_in_flight_)
    {
      return false;
    }
    if (ep_data_in_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }

    auto tx = ep_data_in_->GetBuffer();
    if (!tx.addr_ || tx.size_ < DATA_ACK_FRAME.size())
    {
      return false;
    }

    std::memcpy(tx.addr_, DATA_ACK_FRAME.data(), DATA_ACK_FRAME.size());
    if (ep_data_in_->Transfer(DATA_ACK_FRAME.size()) != ErrorCode::OK)
    {
      return false;
    }
    data_ack_in_flight_ = true;
    return true;
  }

  void QueueCommandOverflowError(uint8_t count = 1u)
  {
    if (count == 0u)
    {
      return;
    }
    const uint16_t next =
        static_cast<uint16_t>(pending_cmd_overflow_count_) + static_cast<uint16_t>(count);
    pending_cmd_overflow_count_ = (next > 0xFFu) ? 0xFFu : static_cast<uint8_t>(next);
  }

  void ArmCommandOutIfIdle()
  {
    if (!ep_cmd_out_)
    {
      return;
    }
    if (ep_cmd_out_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }
    const uint16_t rx_len = ep_cmd_out_->MaxPacketSize();
    (void)ep_cmd_out_->Transfer(rx_len == 0u ? CMD_PACKET_SIZE : rx_len);
  }

  void ArmDataOutIfIdle()
  {
    if (!ep_data_out_)
    {
      return;
    }
    if (ep_data_out_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }
    const uint16_t rx_len = ep_data_out_->MaxPacketSize();
    (void)ep_data_out_->Transfer(rx_len == 0u ? DATA_PACKET_SIZE : rx_len);
  }

  static void ReleaseEndpoint(EndpointPool& endpoint_pool, Endpoint*& ep)
  {
    if (!ep)
    {
      return;
    }
    ep->Close();
    ep->SetActiveLength(0u);
    endpoint_pool.Release(ep);
    ep = nullptr;
  }

 private:
  static constexpr uint16_t CMD_PACKET_SIZE = 64u;
  static constexpr uint16_t DATA_PACKET_SIZE = 64u;
  static constexpr uint8_t kDataInRecoveryForceAfterSpins = 16u;
  static constexpr uint8_t kPendingCmdSendRetryLimit = 2u;
  static constexpr std::array<uint8_t, 4> DATA_ACK_FRAME = {0x41u, 0x01u, 0x01u, 0x04u};
  static constexpr uint32_t kDefaultWritePackSize = 4096u;
  static constexpr uint32_t kSdiClockHzLow = 400'000u;
  static constexpr uint32_t kSdiClockHzMedium = 4'000'000u;
  static constexpr uint32_t kSdiClockHzHigh = 6'000'000u;
  static constexpr uint8_t kDmiData0 = 0x04u;
  static constexpr uint8_t kDmiDmcontrol = 0x10u;
  static constexpr uint8_t kDmiDmstatus = 0x11u;
  static constexpr uint8_t kDmiAbstractcs = 0x16u;
  static constexpr uint8_t kDmiCommand = 0x17u;
  static constexpr uint8_t kDmiProgbuf0 = 0x20u;
  static constexpr uint8_t kDmiProgbuf1 = 0x21u;
  static constexpr uint32_t kChipIdAddress = 0x1FFFF704u;
  static constexpr uint32_t kFlashSizeAddress = 0x1FFFF7E0u;
  static constexpr uint32_t kUidWord0Address = 0x1FFFF7E8u;
  static constexpr uint32_t kUidWord1Address = 0x1FFFF7ECu;

  static bool IsGetProbeInfoRequest(const uint8_t* req, uint16_t req_len)
  {
    return req != nullptr && req_len >= 4u && req[0] == 0x81u && req[1] == 0x0Du &&
           req[2] == 0x01u && req[3] == 0x01u;
  }

  static bool IsAttachChipRequest(const uint8_t* req, uint16_t req_len)
  {
    return req != nullptr && req_len >= 4u && req[0] == 0x81u && req[1] == 0x0Du &&
           req[2] == 0x01u && req[3] == 0x02u;
  }

#pragma pack(push, 1)
  struct WchLinkRvDescBlock
  {
    InterfaceDescriptor intf;
    EndpointDescriptor ep_cmd_out;
    EndpointDescriptor ep_cmd_in;
    EndpointDescriptor ep_data_out;
    EndpointDescriptor ep_data_in;
  } desc_block_{};
#pragma pack(pop)

  ProbeIdentity probe_id_{};

  SdiPort& sdi_;
  LibXR::GPIO* nreset_gpio_ = nullptr;

  Endpoint::EPNumber command_ep_num_;
  Endpoint::EPNumber data_ep_num_;

  Endpoint* ep_cmd_in_ = nullptr;
  Endpoint* ep_cmd_out_ = nullptr;
  Endpoint* ep_data_in_ = nullptr;
  Endpoint* ep_data_out_ = nullptr;

  bool inited_ = false;
  uint8_t interface_num_ = 0u;

  bool attached_ = false;

  enum class ProgramMode : uint8_t
  {
    IDLE = 0u,
    WRITE_FLASH_OP,
    WRITE_FLASH_STREAM,
  };

  uint32_t write_region_addr_ = 0u;
  uint32_t write_region_len_ = 0u;
  uint32_t read_region_addr_ = 0u;
  uint32_t read_region_len_ = 0u;
  uint32_t current_sdi_clock_hz_ = kSdiClockHzHigh;
  uint16_t esig_flash_size_kb_ = 0u;
  uint32_t esig_uid_word0_ = 0u;
  uint32_t esig_uid_word1_ = 0u;
  uint32_t esig_reserved_word_ = 0xFFFFFFFFu;
  uint8_t requested_chip_family_ = 0u;
  bool chip_family_selected_by_speed_ = false;
  bool chip_family_selected_for_next_attach_ = false;
  ProgramMode program_mode_ = ProgramMode::IDLE;
  bool flash_op_ready_ = false;
  uint32_t flash_op_rx_bytes_ = 0u;
  uint32_t flash_stream_chunk_bytes_ = kDefaultWritePackSize;
  uint32_t flash_stream_total_raw_bytes_ = 0u;
  uint32_t flash_stream_rx_bytes_ = 0u;
  uint32_t flash_stream_next_ack_at_ = 0u;
  uint32_t flash_stream_write_addr_ = 0u;
  bool flash_stream_error_ = false;
  uint8_t pending_data_ack_ = 0u;
  bool data_ack_in_flight_ = false;
  bool read_stream_active_ = false;
  bool read_stream_fault_latched_ = false;
  bool read_stream_fault_pending_finalize_ = false;
  uint32_t read_stream_addr_ = 0u;
  uint32_t read_stream_remaining_ = 0u;
  bool read_stream_error_ = false;

  std::array<uint8_t, 96> pending_cmd_buf_ = {};
  uint16_t pending_cmd_len_ = 0u;
  bool pending_cmd_valid_ = false;
  uint8_t pending_cmd_overflow_count_ = 0u;
  bool session_started_ = false;
  bool data_in_recovery_pending_ = false;
  uint8_t data_in_recovery_spins_ = 0u;

  LibXR::Callback<LibXR::ConstRawData&> on_cmd_out_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnCommandOutStatic, this);
  LibXR::Callback<LibXR::ConstRawData&> on_cmd_in_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnCommandInStatic, this);
  LibXR::Callback<LibXR::ConstRawData&> on_data_out_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataOutStatic, this);
  LibXR::Callback<LibXR::ConstRawData&> on_data_in_cb_ =
      LibXR::Callback<LibXR::ConstRawData&>::Create(OnDataInStatic, this);
};

}  // namespace LibXR::USB
