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
#include "riscv_dmi_target.hpp"
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
        riscv_target_(sdi_),
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

    const uint16_t cmd_out_mps = SelectBulkPacketSize(ep_cmd_out_);
    const uint16_t cmd_in_mps = SelectBulkPacketSize(ep_cmd_in_);
    const uint16_t data_out_mps = SelectBulkPacketSize(ep_data_out_);
    const uint16_t data_in_mps = SelectBulkPacketSize(ep_data_in_);

    ep_cmd_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, cmd_out_mps, false});
    ep_cmd_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, cmd_in_mps, false});
    ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, data_out_mps, false});
    ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, data_in_mps, false});

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
                        0x80,  // WCH-Link vendor subclass
                        0x55,  // WCH-Link vendor protocol
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

    ResetCommandResponseQueue();
    session_state_ = SessionState::ACTIVE;
    ResetRuntimeModel();

    inited_ = true;
    ArmCommandOutIfIdle();
  }

  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    ResetCommandResponseQueue();
    session_state_ = SessionState::DISCONNECTED;

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

    std::array<uint8_t, CMD_PACKET_SIZE> resp_shadow = {};
    uint16_t out_len = 0u;
    (void)HandleCommand(req, req_len, resp_shadow.data(), static_cast<uint16_t>(resp_shadow.size()),
                        out_len);
    QueueCommandResponse(resp_shadow.data(), out_len);
    PumpReadStreamIfIdle();
    ArmCommandOutIfIdle();
  }

  void QueueCommandResponse(const uint8_t* resp, uint16_t out_len)
  {
    std::array<uint8_t, 3> err_resp = {};
    if (!resp || out_len == 0u)
    {
      uint16_t err_len = 0u;
      if (BuildErrorResponse(0x55u, err_resp.data(), static_cast<uint16_t>(err_resp.size()),
                             err_len) != ErrorCode::OK ||
          err_len == 0u)
      {
        return;
      }
      resp = err_resp.data();
      out_len = err_len;
    }

    if (out_len > CMD_PACKET_SIZE)
    {
      out_len = CMD_PACKET_SIZE;
    }

    if (TryStartCommandInPayload(resp, out_len))
    {
      return;
    }

    (void)HoldPendingCommandResponse(resp, out_len);
  }

  void OnCommandIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    if (SubmitPendingCommandResponseIfIdle())
    {
      return;
    }
    PumpReadStreamIfIdle();
    ArmCommandOutIfIdle();
  }

  void OnDataOut(bool /*in_isr*/, LibXR::ConstRawData& data)
  {
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
    ArmDataOutForStreamIfIdle();
  }

  void OnDataIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
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
    flash_page_buffer_addr_ = 0u;
    flash_page_buffer_active_ = false;
    flash_page_buffer_dirty_ = false;
    flash_page_buffer_.fill(0xFFu);
    target_debug_session_active_ = false;
    target_debug_session_needs_resume_ = false;
    target_flash_fast_mode_ready_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
    read_stream_active_ = false;
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
        return MatchFlashOpLengthWithPadding(bytes, 494u);
      case 0x02u:  // CH57X
        return MatchFlashOpLengthWithPadding(bytes, 1102u);
      case 0x03u:  // CH56X
        return MatchFlashOpLengthWithPadding(bytes, 1156u);
      case 0x05u:  // CH32V20X
      case 0x06u:  // CH32V30X
        return MatchFlashOpLengthWithPadding(bytes, 446u);
      case 0x07u:  // CH582/583
      case 0x0Bu:  // CH59X
      case 0x4Bu:  // CH585
        return MatchFlashOpLengthWithPadding(bytes, 1326u);
      case 0x09u:  // CH32V003
      case 0x49u:  // CH641
        return MatchFlashOpLengthWithPadding(bytes, 498u) ||
               MatchFlashOpLengthWithPadding(bytes, 466u);
      case 0x0Au:  // CH8571
        return MatchFlashOpLengthWithPadding(bytes, 1408u) ||
               MatchFlashOpLengthWithPadding(bytes, 1386u);
      case 0x0Cu:  // CH643
      case 0x0Du:  // CH32X035
        return MatchFlashOpLengthWithPadding(bytes, 488u);
      case 0x0Eu:  // CH32L103
        return MatchFlashOpLengthWithPadding(bytes, 512u);
      case 0x0Fu:  // CH564
        return MatchFlashOpLengthWithPadding(bytes, 1532u);
      case 0x4Eu:  // CH32V007
        return MatchFlashOpLengthWithPadding(bytes, 500u);
      case 0x46u:  // CH645
        return MatchFlashOpLengthWithPadding(bytes, 440u) ||
               MatchFlashOpLengthWithPadding(bytes, 486u);
      case 0x86u:  // CH32V317
        return MatchFlashOpLengthWithPadding(bytes, 440u) ||
               MatchFlashOpLengthWithPadding(bytes, 460u);
      default:
        // Unknown family: keep compatibility, but still reject empty flash-op.
        return bytes > 0u;
    }
  }

  static uint32_t RoundUpU32(uint32_t value, uint32_t align)
  {
    if (align == 0u)
    {
      return value;
    }
    const uint32_t rem = value % align;
    return (rem == 0u) ? value : (value + align - rem);
  }

  static bool MatchFlashOpLengthWithPadding(uint32_t actual_bytes, uint32_t logical_bytes)
  {
    if (logical_bytes == 0u)
    {
      return false;
    }
    if (actual_bytes == logical_bytes)
    {
      return true;
    }

    // wlink sends flash-op payloads over EP2 in 256-byte chunks and pads the
    // final chunk with trailing zeroes. Accept the transport-sized length while
    // still rejecting unrelated payload sizes.
    return actual_bytes == RoundUpU32(logical_bytes, 256u);
  }

  static uint32_t MinU32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

  static uint32_t AlignDownU32(uint32_t value, uint32_t align)
  {
    if (align == 0u)
    {
      return value;
    }
    return value & ~(align - 1u);
  }

  static uint32_t AlignUpU32(uint32_t value, uint32_t align)
  {
    if (align == 0u)
    {
      return value;
    }
    return AlignDownU32(value + align - 1u, align);
  }

  static uint8_t DetectChipFamilyFromChipId(uint32_t chip_id)
  {
    // Keep mapping consistent with wlink chip-id family grouping.
    switch (chip_id & 0xFFF00000u)
    {
      case 0x20300000u:  // CH32V203
      case 0x20800000u:  // CH32V208
        return 0x05u;    // CH32V20X
      case 0x30300000u:  // CH32V303
      case 0x30500000u:  // CH32V305
      case 0x30700000u:  // CH32V307
        return 0x06u;    // CH32V30X
      case 0x31700000u:  // CH32V317
        return 0x86u;
      default:
        return 0u;
    }
  }

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

  static void BusyDelayCycles(uint32_t cycles)
  {
    volatile uint32_t n = cycles;
    while (n-- > 0u)
    {
      __asm__ volatile("nop");
    }
  }

  void PulseTargetReset()
  {
    if (!nreset_gpio_)
    {
      return;
    }

    (void)nreset_gpio_->SetConfig(
        {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
    nreset_gpio_->Write(false);
    BusyDelayCycles(120000u);
    nreset_gpio_->Write(true);
    BusyDelayCycles(180000u);
  }

  static bool IsPlausibleDmiRegisterValue(uint32_t value)
  {
    return value != 0x00000000u && value != 0xFFFFFFFFu;
  }

  enum class AttachFailureStage : uint8_t
  {
    ENTER_SDI = 0u,
    DMACTIVE,
    DMSTATUS,
    DMSTATUS_ZERO,
    DMSTATUS_ALL_ONES,
    ABSTRACTCS,
    CHIP_ID,
  };

  enum class DmiWordReadStatus : uint8_t
  {
    TRANSACTION = 0u,
    ZERO,
    ALL_ONES,
    OK,
  };

  bool ReadValidatedDmiWord(uint8_t addr, uint32_t& data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = riscv_target_.DmiRead(addr, data, ack);
    return ec == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK &&
           IsPlausibleDmiRegisterValue(data);
  }

  bool ReadDmiWordForAttach(uint8_t addr, uint32_t& data, DmiWordReadStatus& status)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = riscv_target_.DmiRead(addr, data, ack);
    if (ec != ErrorCode::OK || ack != LibXR::Debug::Sdi::Ack::OK)
    {
      status = DmiWordReadStatus::TRANSACTION;
      return false;
    }
    if (data == 0x00000000u)
    {
      status = DmiWordReadStatus::ZERO;
      return false;
    }
    if (data == 0xFFFFFFFFu)
    {
      status = DmiWordReadStatus::ALL_ONES;
      return false;
    }

    status = DmiWordReadStatus::OK;
    return true;
  }

  bool WriteValidatedDmiWord(uint8_t addr, uint32_t data)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode ec = riscv_target_.DmiWrite(addr, data, ack);
    return ec == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool WarmUpDmStatus(uint32_t& dmstatus, AttachFailureStage& failure_stage)
  {
    DmiWordReadStatus last_status = DmiWordReadStatus::TRANSACTION;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
      if (ReadDmiWordForAttach(kAttachDmstatusAddr, dmstatus, last_status))
      {
        return true;
      }

      (void)WriteValidatedDmiWord(kAttachDmcontrolAddr, 0x00000001u);
      sdi_.IdleClocks(static_cast<uint32_t>(16u + attempt * 8u));
    }

    switch (last_status)
    {
      case DmiWordReadStatus::ZERO:
        failure_stage = AttachFailureStage::DMSTATUS_ZERO;
        break;
      case DmiWordReadStatus::ALL_ONES:
        failure_stage = AttachFailureStage::DMSTATUS_ALL_ONES;
        break;
      case DmiWordReadStatus::TRANSACTION:
      case DmiWordReadStatus::OK:
      default:
        failure_stage = AttachFailureStage::DMSTATUS;
        break;
    }

    return false;
  }

  bool ActivateDebugModule()
  {
    for (uint8_t attempt = 0u; attempt < 4u; ++attempt)
    {
      (void)WriteValidatedDmiWord(kAttachDmcontrolAddr, 0x00000000u);
      sdi_.IdleClocks(8u);

      if (!WriteValidatedDmiWord(kAttachDmcontrolAddr, 0x00000001u))
      {
        sdi_.IdleClocks(16u);
        continue;
      }

      sdi_.IdleClocks(static_cast<uint32_t>(16u + attempt * 8u));

      uint32_t dmcontrol = 0u;
      if (ReadValidatedDmiWord(kAttachDmcontrolAddr, dmcontrol) && (dmcontrol & 0x1u) != 0u)
      {
        sdi_.IdleClocks(16u);
        return true;
      }

      sdi_.IdleClocks(24u);
    }

    return false;
  }

  static uint8_t AttachFailureReasonCode(AttachFailureStage stage)
  {
    switch (stage)
    {
      case AttachFailureStage::DMACTIVE:
        return 0x5Au;
      case AttachFailureStage::DMSTATUS:
        return 0x57u;
      case AttachFailureStage::DMSTATUS_ZERO:
        return 0x5Bu;
      case AttachFailureStage::DMSTATUS_ALL_ONES:
        return 0x5Cu;
      case AttachFailureStage::ABSTRACTCS:
        return 0x58u;
      case AttachFailureStage::CHIP_ID:
        return 0x59u;
      case AttachFailureStage::ENTER_SDI:
      default:
        return 0x56u;
    }
  }

  bool TryAttachTarget(typename RiscvDmiTarget<SdiPort>::ProbeData& probe_data,
                       AttachFailureStage& failure_stage)
  {
    if (!ActivateDebugModule())
    {
      failure_stage = AttachFailureStage::DMACTIVE;
      return false;
    }

    uint32_t dmstatus = 0u;
    if (!WarmUpDmStatus(dmstatus, failure_stage))
    {
      return false;
    }

    uint32_t abstractcs = 0u;
    if (!ReadValidatedDmiWord(kAttachAbstractcsAddr, abstractcs))
    {
      failure_stage = AttachFailureStage::ABSTRACTCS;
      return false;
    }

    const bool probe_ok =
        riscv_target_.ProbeChipIdentity(probe_data) || riscv_target_.ProbeChipIdentity(probe_data);
    if (!probe_ok || probe_data.chip_id == 0u || probe_data.chip_id == 0xFFFFFFFFu)
    {
      failure_stage = AttachFailureStage::CHIP_ID;
      return false;
    }
    return true;
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

  void HandleReadStreamLinkFault(bool /*defer_data_in_finalize*/ = false)
  {
    read_stream_active_ = false;
    attached_ = false;
    session_state_ = SessionState::LINK_FAULT;
    sdi_.Close();
  }

  bool ShouldAcceptDataOut() const
  {
    if (program_mode_ == ProgramMode::WRITE_FLASH_OP)
    {
      return true;
    }

    if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM)
    {
      if (flash_stream_error_)
      {
        return false;
      }
      return !IsFlashWriteFinished();
    }

    return false;
  }

  void ArmDataOutForStreamIfIdle()
  {
    if (!ShouldAcceptDataOut())
    {
      return;
    }
    ArmDataOutIfIdle();
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
    program_mode_ = ProgramMode::WRITE_FLASH_OP;
    flash_op_rx_bytes_ = 0u;
    flash_op_ready_ = false;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    flash_page_buffer_addr_ = 0u;
    flash_page_buffer_active_ = false;
    flash_page_buffer_dirty_ = false;
    flash_page_buffer_.fill(0xFFu);
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
    ArmDataOutForStreamIfIdle();
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
    flash_page_buffer_addr_ = 0u;
    flash_page_buffer_active_ = false;
    flash_page_buffer_dirty_ = false;
    flash_page_buffer_.fill(0xFFu);
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

    // Host-side wlink streams flash payloads over EP2 in 256-byte packets and
    // waits for an EP2-IN ack after each packet, even when the logical write
    // pack size is 4096 bytes. Keep the ack cadence aligned with the transport
    // packet size to avoid host/device deadlock mid-programming.
    flash_stream_chunk_bytes_ = kFlashStreamAckBytes;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_next_ack_at_ = MinU32(flash_stream_chunk_bytes_, flash_stream_total_raw_bytes_);
    flash_stream_write_addr_ = write_region_addr_;
    flash_stream_error_ = false;
    flash_page_buffer_addr_ = 0u;
    flash_page_buffer_active_ = false;
    flash_page_buffer_dirty_ = false;
    flash_page_buffer_.fill(0xFFu);
    pending_data_ack_ = 0u;
    program_mode_ = ProgramMode::WRITE_FLASH_STREAM;
    ArmDataOutForStreamIfIdle();
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

    if (ActiveChipFamily() == 0x05u)
    {
      return WriteFlashStreamChunkV20x(data, size);
    }

    uint32_t off = 0u;
    while (off < size)
    {
      if (((flash_stream_write_addr_ & 0x3u) == 0u) && (size - off >= 4u))
      {
        const uint32_t word = LoadLe32(data + off);
        if (!riscv_target_.WriteWordByAbstract(flash_stream_write_addr_, word))
        {
          return false;
        }
        flash_stream_write_addr_ += 4u;
        off += 4u;
      }
      else
      {
        if (!riscv_target_.WriteByteByAbstract(flash_stream_write_addr_, data[off]))
        {
          return false;
        }
        ++flash_stream_write_addr_;
        ++off;
      }
    }
    return true;
  }

  bool WriteFlashStreamChunkV20x(const uint8_t* data, uint32_t size)
  {
    if (!EnsureTargetFlashWriteSession())
    {
      return false;
    }

    uint32_t off = 0u;
    while (off < size)
    {
      const uint32_t page_addr = AlignDownU32(flash_stream_write_addr_, kTargetFlashPageBytes);
      if (!flash_page_buffer_active_)
      {
        BeginFlashPageBuffer(page_addr);
      }
      else if (flash_page_buffer_addr_ != page_addr)
      {
        if (!FlushFlashPageBuffer())
        {
          return false;
        }
        BeginFlashPageBuffer(page_addr);
      }

      const uint32_t page_off = flash_stream_write_addr_ - flash_page_buffer_addr_;
      const uint32_t chunk = MinU32(size - off, kTargetFlashPageBytes - page_off);
      std::memcpy(flash_page_buffer_.data() + page_off, data + off, chunk);
      flash_page_buffer_dirty_ = true;
      flash_stream_write_addr_ += chunk;
      off += chunk;

      if ((page_off + chunk) >= kTargetFlashPageBytes)
      {
        if (!FlushFlashPageBuffer())
        {
          return false;
        }
      }
    }

    return true;
  }

  void BeginFlashPageBuffer(uint32_t page_addr)
  {
    flash_page_buffer_addr_ = page_addr;
    flash_page_buffer_active_ = true;
    flash_page_buffer_dirty_ = false;
    flash_page_buffer_.fill(0xFFu);
  }

  bool FlushFlashPageBuffer()
  {
    if (!flash_page_buffer_active_)
    {
      return true;
    }

    const bool should_program = flash_page_buffer_dirty_;
    const uint32_t page_addr = flash_page_buffer_addr_;
    flash_page_buffer_active_ = false;
    flash_page_buffer_dirty_ = false;

    if (!should_program)
    {
      return true;
    }

    if (!EraseFlashPage256(page_addr))
    {
      return false;
    }
    if (!ProgramFlashPage256(page_addr, flash_page_buffer_))
    {
      return false;
    }

    flash_page_buffer_.fill(0xFFu);
    return true;
  }

  bool FinalizeFlashWriteStream()
  {
    if (ActiveChipFamily() != 0x05u)
    {
      return true;
    }
    return FlushFlashPageBuffer();
  }

  bool EnsureTargetDebugSession()
  {
    if (target_debug_session_active_)
    {
      return true;
    }

    bool need_resume = false;
    if (!riscv_target_.BeginHartHaltSession(need_resume))
    {
      return false;
    }
    target_debug_session_active_ = true;
    target_debug_session_needs_resume_ = need_resume;
    return true;
  }

  void EndTargetDebugSession()
  {
    if (target_flash_fast_mode_ready_)
    {
      (void)WriteTargetFlashRegisterOr(kTargetFlashCtlr, kTargetFlashCtlLock | kTargetFlashCtlFastLock);
      target_flash_fast_mode_ready_ = false;
    }

    if (target_debug_session_active_)
    {
      riscv_target_.EndHartHaltSession(target_debug_session_needs_resume_);
      target_debug_session_active_ = false;
      target_debug_session_needs_resume_ = false;
    }
  }

  bool EnsureTargetFlashWriteSession()
  {
    if (!EnsureTargetDebugSession())
    {
      return false;
    }

    if (target_flash_fast_mode_ready_)
    {
      return true;
    }

    if (!WriteTargetFlashRegister(kTargetFlashKeyr, kTargetFlashKey1) ||
        !WriteTargetFlashRegister(kTargetFlashKeyr, kTargetFlashKey2) ||
        !WriteTargetFlashRegister(kTargetFlashModekeyr, kTargetFlashKey1) ||
        !WriteTargetFlashRegister(kTargetFlashModekeyr, kTargetFlashKey2))
    {
      return false;
    }

    target_flash_fast_mode_ready_ = true;
    return ClearTargetFlashFlags();
  }

  bool WriteTargetFlashRegister(uint32_t addr, uint32_t value)
  {
    return riscv_target_.WriteWordByAbstract(addr, value);
  }

  bool ReadTargetFlashRegister(uint32_t addr, uint32_t& value)
  {
    return riscv_target_.ReadWordByAbstract(addr, value);
  }

  bool WriteTargetFlashRegisterOr(uint32_t addr, uint32_t bits)
  {
    uint32_t value = 0u;
    if (!ReadTargetFlashRegister(addr, value))
    {
      return false;
    }
    return WriteTargetFlashRegister(addr, value | bits);
  }

  bool WriteTargetFlashRegisterAnd(uint32_t addr, uint32_t bits_to_clear)
  {
    uint32_t value = 0u;
    if (!ReadTargetFlashRegister(addr, value))
    {
      return false;
    }
    return WriteTargetFlashRegister(addr, value & ~bits_to_clear);
  }

  bool ClearTargetFlashFlags()
  {
    return WriteTargetFlashRegister(kTargetFlashStatr,
                                    kTargetFlashStatWriteProtectErr | kTargetFlashStatEop);
  }

  bool WaitTargetFlashIdle(uint32_t mask, uint16_t retries = 512u)
  {
    for (uint16_t i = 0u; i < retries; ++i)
    {
      uint32_t statr = 0u;
      if (!ReadTargetFlashRegister(kTargetFlashStatr, statr))
      {
        return false;
      }
      if ((statr & mask) == 0u)
      {
        return true;
      }
    }
    return false;
  }

  bool CheckTargetFlashError()
  {
    uint32_t statr = 0u;
    if (!ReadTargetFlashRegister(kTargetFlashStatr, statr))
    {
      return false;
    }
    return (statr & kTargetFlashStatWriteProtectErr) == 0u;
  }

  bool EraseFlashPage256(uint32_t page_addr)
  {
    if (!ClearTargetFlashFlags())
    {
      return false;
    }
    if (!WriteTargetFlashRegisterOr(kTargetFlashCtlr, kTargetFlashCtlPageErase))
    {
      return false;
    }
    if (!WriteTargetFlashRegister(kTargetFlashAddr, AlignDownU32(page_addr, kTargetFlashPageBytes)))
    {
      return false;
    }
    if (!WriteTargetFlashRegisterOr(kTargetFlashCtlr, kTargetFlashCtlStart))
    {
      return false;
    }
    if (!WaitTargetFlashIdle(kTargetFlashStatBusy))
    {
      return false;
    }
    if (!WriteTargetFlashRegisterAnd(kTargetFlashCtlr, kTargetFlashCtlPageErase))
    {
      return false;
    }
    if (!CheckTargetFlashError())
    {
      (void)ClearTargetFlashFlags();
      return false;
    }
    return ClearTargetFlashFlags();
  }

  bool ProgramFlashPage256(uint32_t page_addr, const std::array<uint8_t, 256u>& page)
  {
    if (!ClearTargetFlashFlags())
    {
      return false;
    }
    if (!WriteTargetFlashRegisterOr(kTargetFlashCtlr, kTargetFlashCtlPageProgram))
    {
      return false;
    }
    if (!WaitTargetFlashIdle(kTargetFlashStatBusy | kTargetFlashStatWriteBusy))
    {
      return false;
    }

    for (uint32_t off = 0u; off < kTargetFlashPageBytes; off += 4u)
    {
      if (!riscv_target_.WriteWordByAbstract(page_addr + off, LoadLe32(page.data() + off)))
      {
        return false;
      }
      if (!WaitTargetFlashIdle(kTargetFlashStatWriteBusy))
      {
        return false;
      }
    }

    if (!WriteTargetFlashRegisterOr(kTargetFlashCtlr, kTargetFlashCtlPageProgramStart))
    {
      return false;
    }
    if (!WaitTargetFlashIdle(kTargetFlashStatBusy))
    {
      return false;
    }
    if (!WriteTargetFlashRegisterAnd(kTargetFlashCtlr, kTargetFlashCtlPageProgram))
    {
      return false;
    }
    if (!CheckTargetFlashError())
    {
      (void)ClearTargetFlashFlags();
      return false;
    }
    return ClearTargetFlashFlags();
  }

  bool EnterReadStream(uint32_t addr, uint32_t len)
  {
    if (!EnsureTargetDebugSession())
    {
      return false;
    }
    ExitProgramStream();
    session_state_ = SessionState::ACTIVE;
    read_stream_active_ = true;
    read_stream_addr_ = addr;
    read_stream_remaining_ = len;
    read_stream_error_ = false;
    return true;
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
        if (!riscv_target_.ReadWordByAbstract(read_stream_addr_, read_word))
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
      EndTargetDebugSession();
      typename RiscvDmiTarget<SdiPort>::ProbeData probe_data = {};
      bool attach_ok = false;
      AttachFailureStage last_failure_stage = AttachFailureStage::ENTER_SDI;
      for (uint8_t attempt = 0u; attempt < kAttachRetryCount; ++attempt)
      {
        sdi_.Close();
        PulseTargetReset();
        BusyDelayCycles(kAttachRetryDelayCycles);

        const ErrorCode ec = sdi_.EnterSdi();
        if (ec != ErrorCode::OK)
        {
          last_failure_stage = AttachFailureStage::ENTER_SDI;
          continue;
        }

        if (TryAttachTarget(probe_data, last_failure_stage))
        {
          attach_ok = true;
          break;
        }
      }

      if (!attach_ok)
      {
        sdi_.Close();
        session_state_ = SessionState::LINK_FAULT;
        probe_id_.chip_id = 0u;
        esig_flash_size_kb_ = 0u;
        esig_uid_word0_ = 0u;
        esig_uid_word1_ = 0u;
        return BuildErrorResponse(AttachFailureReasonCode(last_failure_stage), resp, cap, out_len);
      }

      probe_id_.chip_id = probe_data.chip_id;
      esig_flash_size_kb_ = probe_data.flash_size_kb;
      esig_uid_word0_ = probe_data.uid_word0;
      esig_uid_word1_ = probe_data.uid_word1;

      uint8_t chip_family = DetectChipFamilyFromChipId(probe_id_.chip_id);
      if (chip_family == 0u && requested_chip_family_ != 0u && requested_chip_family_ != 0x01u)
      {
        chip_family = requested_chip_family_;
      }
      if (chip_family == 0u && probe_id_.chip_family != 0u && probe_id_.chip_family != 0x01u)
      {
        chip_family = probe_id_.chip_family;
      }
      probe_id_.chip_family = chip_family;

      attached_ = true;
      session_state_ = SessionState::ACTIVE;
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
      EndTargetDebugSession();
      sdi_.Close();
      session_state_ = SessionState::ACTIVE;
      ClearHostSelectedChipFamily();
      ResetCommandResponseQueue();
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
      const ErrorCode ec = riscv_target_.DmiRead(addr, out_data, ack);
      if (ec != ErrorCode::OK && ec != ErrorCode::FAILED && ec != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        session_state_ = SessionState::LINK_FAULT;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_op = AckToDmiOp(ack);
    }
    else if (op == 0x02u)
    {
      const ErrorCode ec = riscv_target_.DmiWrite(addr, data, ack);
      if (ec != ErrorCode::OK && ec != ErrorCode::FAILED && ec != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        session_state_ = SessionState::LINK_FAULT;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_data = data;
      out_op = AckToDmiOp(ack);
    }
    else if (op == 0x00u)
    {
      const ErrorCode ec = riscv_target_.DmiNop(addr, out_data, ack);
      if (ec != ErrorCode::OK)
      {
        attached_ = false;
        session_state_ = SessionState::LINK_FAULT;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_op = AckToDmiOp(ack);
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
          !FinalizeFlashWriteStream() ||
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
      if (!EnterReadStream(read_region_addr_, read_region_len_))
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
    }
    else if (sub == 0x09u || sub == 0x01u)
    {
      ExitProgramStream();
      ExitReadStream();
      EndTargetDebugSession();
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

    if (session_state_ == SessionState::LINK_FAULT)
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

  void ResetCommandResponseQueue()
  {
    pending_cmd_resp_valid_ = false;
    pending_cmd_resp_len_ = 0u;
  }

  bool TryStartCommandInPayload(const uint8_t* payload, uint16_t len)
  {
    if (!payload || len == 0u || !ep_cmd_in_)
    {
      return false;
    }
    if (ep_cmd_in_->GetState() != Endpoint::State::IDLE)
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
  }

  bool HoldPendingCommandResponse(const uint8_t* payload, uint16_t len)
  {
    if (!payload || len == 0u || pending_cmd_resp_valid_)
    {
      return false;
    }
    if (len > CMD_PACKET_SIZE)
    {
      len = CMD_PACKET_SIZE;
    }

    std::memcpy(pending_cmd_resp_.data(), payload, len);
    pending_cmd_resp_len_ = len;
    pending_cmd_resp_valid_ = true;
    return true;
  }

  bool SubmitPendingCommandResponseIfIdle()
  {
    if (!pending_cmd_resp_valid_)
    {
      return false;
    }
    if (!TryStartCommandInPayload(pending_cmd_resp_.data(), pending_cmd_resp_len_))
    {
      return false;
    }

    pending_cmd_resp_valid_ = false;
    pending_cmd_resp_len_ = 0u;
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

  void ArmCommandOutIfIdle()
  {
    if (!ep_cmd_out_ || !ep_cmd_in_)
    {
      return;
    }
    if (ep_cmd_out_->GetState() != Endpoint::State::IDLE)
    {
      return;
    }
    // Keep strict command request/response pairing on command plane.
    if (pending_cmd_resp_valid_ || ep_cmd_in_->GetState() != Endpoint::State::IDLE)
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
  static uint16_t SelectBulkPacketSize(Endpoint* ep)
  {
    if (!ep)
    {
      return BULK_MPS_FS;
    }
    const auto ep_buf = ep->GetBuffer();
    if (ep_buf.size_ >= BULK_MPS_HS)
    {
      return BULK_MPS_HS;
    }
    return BULK_MPS_FS;
  }

 private:
  static constexpr uint16_t CMD_PACKET_SIZE = 64u;
  static constexpr uint16_t DATA_PACKET_SIZE = 64u;
  static constexpr uint16_t BULK_MPS_FS = 64u;
  static constexpr uint16_t BULK_MPS_HS = 512u;
  static constexpr std::array<uint8_t, 4> DATA_ACK_FRAME = {0x41u, 0x01u, 0x01u, 0x04u};
  static constexpr uint32_t kDefaultWritePackSize = 4096u;
  static constexpr uint32_t kFlashStreamAckBytes = 256u;
  static constexpr uint32_t kSdiClockHzLow = 5'000u;
  static constexpr uint32_t kSdiClockHzMedium = 10'000u;
  static constexpr uint32_t kSdiClockHzHigh = 20'000u;
  static constexpr uint32_t kTargetFlashPageBytes = 256u;
  static constexpr uint32_t kTargetFlashKey1 = 0x45670123u;
  static constexpr uint32_t kTargetFlashKey2 = 0xCDEF89ABu;
  static constexpr uint32_t kTargetFlashBase = 0x40022000u;
  static constexpr uint32_t kTargetFlashKeyr = kTargetFlashBase + 0x04u;
  static constexpr uint32_t kTargetFlashStatr = kTargetFlashBase + 0x0Cu;
  static constexpr uint32_t kTargetFlashCtlr = kTargetFlashBase + 0x10u;
  static constexpr uint32_t kTargetFlashAddr = kTargetFlashBase + 0x14u;
  static constexpr uint32_t kTargetFlashModekeyr = kTargetFlashBase + 0x24u;
  static constexpr uint32_t kTargetFlashStatBusy = 0x00000001u;
  static constexpr uint32_t kTargetFlashStatWriteBusy = 0x00000002u;
  static constexpr uint32_t kTargetFlashStatWriteProtectErr = 0x00000010u;
  static constexpr uint32_t kTargetFlashStatEop = 0x00000020u;
  static constexpr uint32_t kTargetFlashCtlStart = 0x00000040u;
  static constexpr uint32_t kTargetFlashCtlLock = 0x00000080u;
  static constexpr uint32_t kTargetFlashCtlFastLock = 0x00008000u;
  static constexpr uint32_t kTargetFlashCtlPageProgram = 0x00010000u;
  static constexpr uint32_t kTargetFlashCtlPageErase = 0x00020000u;
  static constexpr uint32_t kTargetFlashCtlPageProgramStart = 0x00200000u;
  static constexpr uint8_t kAttachRetryCount = 10u;
  static constexpr uint32_t kAttachRetryDelayCycles = 60000u;
  static constexpr uint8_t kAttachDmcontrolAddr = 0x10u;
  static constexpr uint8_t kAttachDmstatusAddr = 0x11u;
  static constexpr uint8_t kAttachAbstractcsAddr = 0x16u;

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
  RiscvDmiTarget<SdiPort> riscv_target_;
  LibXR::GPIO* nreset_gpio_ = nullptr;

  Endpoint::EPNumber command_ep_num_;
  Endpoint::EPNumber data_ep_num_;

  Endpoint* ep_cmd_in_ = nullptr;
  Endpoint* ep_cmd_out_ = nullptr;
  Endpoint* ep_data_in_ = nullptr;
  Endpoint* ep_data_out_ = nullptr;

  bool inited_ = false;
  uint8_t interface_num_ = 0u;

  enum class SessionState : uint8_t
  {
    DISCONNECTED = 0u,
    ACTIVE,
    LINK_FAULT,
  };

  SessionState session_state_ = SessionState::DISCONNECTED;

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
  uint32_t flash_page_buffer_addr_ = 0u;
  bool flash_page_buffer_active_ = false;
  bool flash_page_buffer_dirty_ = false;
  std::array<uint8_t, kTargetFlashPageBytes> flash_page_buffer_ = {};
  bool target_debug_session_active_ = false;
  bool target_debug_session_needs_resume_ = false;
  bool target_flash_fast_mode_ready_ = false;
  uint8_t pending_data_ack_ = 0u;
  bool data_ack_in_flight_ = false;
  bool read_stream_active_ = false;
  uint32_t read_stream_addr_ = 0u;
  uint32_t read_stream_remaining_ = 0u;
  bool read_stream_error_ = false;

  std::array<uint8_t, CMD_PACKET_SIZE> pending_cmd_resp_ = {};
  uint16_t pending_cmd_resp_len_ = 0u;
  bool pending_cmd_resp_valid_ = false;

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
