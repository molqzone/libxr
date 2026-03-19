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
    const auto* req = static_cast<const uint8_t*>(data.addr_);
    const uint16_t req_len = static_cast<uint16_t>(data.size_);

    // Drop stale queued OUT frames from prior sessions until the host starts
    // with a clean GetProbeInfo transaction. Accept AttachChip as an
    // alternative start marker because some host flows reattach directly after
    // OptEnd without re-querying probe info.
    if (!session_started_)
    {
      const bool start_with_probe_info = IsGetProbeInfoRequest(req, req_len);
      const bool start_with_attach = IsAttachChipRequest(req, req_len);
      if (!start_with_probe_info && !start_with_attach)
      {
        ArmCommandOutIfIdle();
        return;
      }
      PrepareFreshSessionStart(start_with_probe_info);
      session_started_ = true;
    }

    uint16_t out_len = 0u;
    auto tx = ep_cmd_in_ ? ep_cmd_in_->GetBuffer() : RawData{nullptr, 0};
    if (!tx.addr_ || tx.size_ == 0u)
    {
      ArmCommandOutIfIdle();
      return;
    }

    auto* resp = static_cast<uint8_t*>(tx.addr_);

    (void)HandleCommand(req, req_len, resp, static_cast<uint16_t>(tx.size_), out_len);
    if (!TryStartCommandIn(out_len))
    {
      if (out_len <= pending_cmd_buf_.size())
      {
        std::memcpy(pending_cmd_buf_.data(), resp, out_len);
        pending_cmd_len_ = out_len;
        pending_cmd_valid_ = true;
      }
    }
  }

  void OnCommandIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    if (!pending_cmd_valid_)
    {
      ArmCommandOutIfIdle();
      return;
    }

    auto tx = ep_cmd_in_ ? ep_cmd_in_->GetBuffer() : RawData{nullptr, 0};
    if (!tx.addr_ || tx.size_ < pending_cmd_len_)
    {
      pending_cmd_valid_ = false;
      pending_cmd_len_ = 0u;
      ArmCommandOutIfIdle();
      return;
    }

    std::memcpy(tx.addr_, pending_cmd_buf_.data(), pending_cmd_len_);
    if (TryStartCommandIn(pending_cmd_len_))
    {
      pending_cmd_valid_ = false;
      pending_cmd_len_ = 0u;
      return;
    }

    ArmCommandOutIfIdle();
  }

  void OnDataOut(bool /*in_isr*/, LibXR::ConstRawData& data)
  {
    const uint32_t rx_bytes = static_cast<uint32_t>(data.size_);
    if (program_mode_ == ProgramMode::WRITE_FLASH_OP)
    {
      flash_op_rx_bytes_ += rx_bytes;
    }
    else if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM)
    {
      ProcessFlashStreamBytes(rx_bytes);
    }
    FlushPendingDataAck();
    ArmDataOutIfIdle();
  }

  void OnDataIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/) { FlushPendingDataAck(); }

  void ReopenBulkInEndpoint(Endpoint* ep, uint16_t mps,
                            const LibXR::Callback<LibXR::ConstRawData&>& cb)
  {
    if (!ep)
    {
      return;
    }
    ep->Close();
    ep->SetActiveLength(0u);
    ep->Configure({Endpoint::Direction::IN, Endpoint::Type::BULK, mps, false});
    ep->SetOnTransferCompleteCallback(cb);
  }

  void PrepareFreshSessionStart(bool clear_host_selected_chip)
  {
    // Host reconnection may leave unread/stale IN payload in previous session.
    // Reset software queue and reopen IN endpoints to guarantee the new session
    // starts from GetProbeInfo/Attach on a clean command/data plane.
    pending_cmd_valid_ = false;
    pending_cmd_len_ = 0u;
    attached_ = false;
    if (clear_host_selected_chip)
    {
      requested_chip_family_ = 0u;
    }
    ExitProgramStream();
    sdi_.Close();

    ReopenBulkInEndpoint(ep_cmd_in_, CMD_PACKET_SIZE, on_cmd_in_cb_);
    ReopenBulkInEndpoint(ep_data_in_, DATA_PACKET_SIZE, on_data_in_cb_);
  }

  void ResetRuntimeModel()
  {
    attached_ = false;
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
    pending_data_ack_ = 0u;
  }

  static uint32_t LoadBe32(const uint8_t* p)
  {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
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
    if (pending_data_ack_ == 0u)
    {
      return;
    }
    if (TrySendDataAck())
    {
      --pending_data_ack_;
    }
  }

  void EnterFlashOpStream()
  {
    program_mode_ = ProgramMode::WRITE_FLASH_OP;
    flash_op_rx_bytes_ = 0u;
    flash_op_ready_ = false;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    pending_data_ack_ = 0u;
  }

  void ExitProgramStream()
  {
    program_mode_ = ProgramMode::IDLE;
    flash_op_ready_ = false;
    flash_op_rx_bytes_ = 0u;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    pending_data_ack_ = 0u;
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
    pending_data_ack_ = 0u;
    program_mode_ = ProgramMode::WRITE_FLASH_STREAM;
    return true;
  }

  void ProcessFlashStreamBytes(uint32_t rx_bytes)
  {
    if (rx_bytes == 0u || flash_stream_next_ack_at_ == 0u)
    {
      return;
    }

    if (flash_stream_rx_bytes_ >= flash_stream_total_raw_bytes_)
    {
      return;
    }
    const uint32_t remain_total = flash_stream_total_raw_bytes_ - flash_stream_rx_bytes_;
    flash_stream_rx_bytes_ += MinU32(rx_bytes, remain_total);
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
      const ErrorCode ec = sdi_.EnterSdi();
      if (ec != ErrorCode::OK)
      {
        attached_ = false;
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      attached_ = true;

      uint32_t chip_id = probe_id_.chip_id;
      uint8_t chip_family = (requested_chip_family_ != 0u) ? requested_chip_family_ : probe_id_.chip_family;
      if (TryProbeChipIdentity(chip_id, chip_family))
      {
        probe_id_.chip_id = chip_id;
        probe_id_.chip_family = chip_family;
      }

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
      sdi_.Close();
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
          pending_data_ack_ != 0u)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      ExitProgramStream();
    }
    else if (sub == 0x09u || sub == 0x01u)
    {
      ExitProgramStream();
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
        requested_chip_family_ = payload[0];
        const uint32_t hz = DecodeSdiClockHz(payload[1]);
        const bool success = (sdi_.SetClockHz(hz) == ErrorCode::OK);
        if (success)
        {
          current_sdi_clock_hz_ = hz;
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
    return ep_data_in_->Transfer(DATA_ACK_FRAME.size()) == ErrorCode::OK;
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
  ProgramMode program_mode_ = ProgramMode::IDLE;
  bool flash_op_ready_ = false;
  uint32_t flash_op_rx_bytes_ = 0u;
  uint32_t flash_stream_chunk_bytes_ = kDefaultWritePackSize;
  uint32_t flash_stream_total_raw_bytes_ = 0u;
  uint32_t flash_stream_rx_bytes_ = 0u;
  uint32_t flash_stream_next_ack_at_ = 0u;
  uint8_t pending_data_ack_ = 0u;

  std::array<uint8_t, 96> pending_cmd_buf_ = {};
  uint16_t pending_cmd_len_ = 0u;
  bool pending_cmd_valid_ = false;
  bool session_started_ = false;

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
