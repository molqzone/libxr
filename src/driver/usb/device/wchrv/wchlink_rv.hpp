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
  struct AttachDebugSnapshot
  {
    uint8_t failure_stage = 0xFFu;
    uint8_t attached = 0u;
    uint8_t reserved0 = 0u;
    uint8_t reserved1 = 0u;
    uint32_t dmstatus = 0u;
    uint32_t abstractcs = 0u;
    uint32_t chip_id = 0u;
  };

  static inline volatile AttachDebugSnapshot debug_attach_snapshot_ = {};

  struct FlashLoaderDebugSnapshot
  {
    uint8_t failure_stage = 0u;
    uint8_t resume_ack_seen = 0u;
    uint8_t hart_halted_seen = 0u;
    uint8_t a0_valid = 0u;
    uint32_t dmstatus_before_resume = 0u;
    uint32_t dmstatus_after_resume = 0u;
    uint32_t dmstatus_after_halt = 0u;
    uint32_t dmcontrol_after_resume = 0u;
    uint32_t a0_result = 0u;
    uint32_t dpc = 0u;
    uint32_t mepc = 0u;
    uint32_t mcause = 0u;
  };

  static inline volatile FlashLoaderDebugSnapshot debug_flash_loader_snapshot_ = {};

  struct ProbeIdentity
  {
    uint8_t major = 0x02;   ///< Firmware major reported by 0x0d/0x01.
    uint8_t minor = 0x12;   ///< Firmware minor reported by 0x0d/0x01.
    uint8_t variant = 0x12; ///< Link variant reported by 0x0d/0x01.
  };

 private:
  struct TargetCompatState
  {
    uint8_t requested_family = 0u; ///< Host-selected family code from SetSpeed.
    uint8_t reported_family = 0u;  ///< Minimal family code echoed back to host on attach.
    uint32_t chip_id = 0u;         ///< Runtime-detected chip-id value.
    uint16_t flash_size_kb = 0u;   ///< Runtime-detected flash capacity for compatibility shims.
    typename RiscvDmiTarget<SdiPort>::WchLinkCompatDescriptor compat = {};
  };

  struct FlashOpLayout
  {
    uint32_t code_addr = 0x20000000u;
    uint32_t page_buffer_addr = 0x20001000u;
    uint32_t verify_sum_addr = 0x20002010u;
    uint32_t stack_addr = 0x20002800u;
  };

 public:

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

  bool Poll()
  {
    if (!inited_)
    {
      return false;
    }

    bool did_work = false;
    did_work = ProcessPendingCommandOut() || did_work;
    did_work = ProcessPendingDataOut() || did_work;
    did_work = SubmitPendingCommandResponseIfIdle() || did_work;
    if (read_stream_active_)
    {
      did_work = PumpReadStreamIfIdle() || did_work;
    }
    else
    {
      did_work = FlushPendingDataAck() || did_work;
    }
    did_work = ArmCommandOutIfIdle() || did_work;
    if (FlashStreamStageHasRoom())
    {
      did_work = ArmDataOutForStreamIfIdle() || did_work;
    }
    return did_work;
  }

 protected:
  ErrorCode WriteDeviceDescriptor(DeviceDescriptor& header) override
  {
    // Present as a plain single-interface vendor device. The default composite/IAD-style
    // descriptor confuses parity with the official WCH-Link layout and is unnecessary here.
    header.data_.bcdUSB = USBSpec::USB_2_0;
    header.data_.bDeviceClass = DeviceDescriptor::ClassID::PER_INTERFACE;
    header.data_.bDeviceSubClass = 0u;
    header.data_.bDeviceProtocol = 0u;
    return ErrorCode::OK;
  }

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

    const uint16_t CMD_OUT_MPS = SelectBulkPacketSize(ep_cmd_out_);
    const uint16_t CMD_IN_MPS = SelectBulkPacketSize(ep_cmd_in_);
    const uint16_t DATA_OUT_MPS = SelectBulkPacketSize(ep_data_out_);
    const uint16_t DATA_IN_MPS = SelectBulkPacketSize(ep_data_in_);

    ep_cmd_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, CMD_OUT_MPS, false});
    ep_cmd_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, CMD_IN_MPS, false});
    ep_data_out_->Configure(
        {Endpoint::Direction::OUT, Endpoint::Type::BULK, DATA_OUT_MPS, false});
    ep_data_in_->Configure(
        {Endpoint::Direction::IN, Endpoint::Type::BULK, DATA_IN_MPS, false});

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

    desc_block_.ep_data_in = {7,
                              static_cast<uint8_t>(DescriptorType::ENDPOINT),
                              static_cast<uint8_t>(ep_data_in_->GetAddress()),
                              static_cast<uint8_t>(Endpoint::Type::BULK),
                              ep_data_in_->MaxPacketSize(),
                              0};
    desc_block_.ep_data_out = {7,
                               static_cast<uint8_t>(DescriptorType::ENDPOINT),
                               static_cast<uint8_t>(ep_data_out_->GetAddress()),
                               static_cast<uint8_t>(Endpoint::Type::BULK),
                               ep_data_out_->MaxPacketSize(),
                               0};
    desc_block_.ep_cmd_in = {7,
                             static_cast<uint8_t>(DescriptorType::ENDPOINT),
                             static_cast<uint8_t>(ep_cmd_in_->GetAddress()),
                             static_cast<uint8_t>(Endpoint::Type::BULK),
                             ep_cmd_in_->MaxPacketSize(),
                             0};
    desc_block_.ep_cmd_out = {7,
                              static_cast<uint8_t>(DescriptorType::ENDPOINT),
                              static_cast<uint8_t>(ep_cmd_out_->GetAddress()),
                              static_cast<uint8_t>(Endpoint::Type::BULK),
                              ep_cmd_out_->MaxPacketSize(),
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
    const uint16_t REQ_RAW_LEN = static_cast<uint16_t>(data.size_);
    if (!req_raw || REQ_RAW_LEN == 0u)
    {
      (void)ArmCommandOutIfIdle();
      return;
    }

    if (pending_cmd_out_valid_)
    {
      session_state_ = SessionState::LINK_FAULT;
      return;
    }

    const uint16_t REQ_LEN =
        (REQ_RAW_LEN <= CMD_PACKET_SIZE) ? REQ_RAW_LEN : static_cast<uint16_t>(CMD_PACKET_SIZE);
    std::memcpy(pending_cmd_out_.data(), req_raw, REQ_LEN);
    pending_cmd_out_len_ = REQ_LEN;
    pending_cmd_out_valid_ = true;
  }

  void QueueCommandResponse(const uint8_t* resp, uint16_t out_len)
  {
    std::array<uint8_t, 4> err_resp = {};
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
  }

  void OnDataOut(bool /*in_isr*/, LibXR::ConstRawData& data)
  {
    const auto* rx = static_cast<const uint8_t*>(data.addr_);
    const uint32_t RX_BYTES = static_cast<uint32_t>(data.size_);
    if (program_mode_ == ProgramMode::WRITE_FLASH_OP)
    {
      CaptureFlashOpData(rx, RX_BYTES);
      (void)ArmDataOutForStreamIfIdle();
    }
    else if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM)
    {
      if (!rx || RX_BYTES == 0u)
      {
        (void)ArmDataOutForStreamIfIdle();
        return;
      }
      const uint32_t REMAIN_TOTAL = flash_stream_total_raw_bytes_ - flash_stream_received_bytes_;
      const uint32_t ACCEPT_BYTES = MinU32(RX_BYTES, REMAIN_TOTAL);
      if (ACCEPT_BYTES == 0u)
      {
        (void)ArmDataOutForStreamIfIdle();
        return;
      }
      if (!QueueFlashStreamData(rx, ACCEPT_BYTES))
      {
        SetFlashStreamError(DATA_ACK_CODE_STREAM_STAGE_OVERFLOW);
        return;
      }
      flash_stream_received_bytes_ += ACCEPT_BYTES;
      QueueFlashWriteAcks();
      (void)ArmDataOutForStreamIfIdle();
      return;
    }
    else
    {
      (void)ArmDataOutForStreamIfIdle();
    }
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
  }

  void ResetRuntimeModel()
  {
    attached_ = false;
    dmi_needs_warmup_ = false;
    debug_attach_snapshot_.failure_stage = 0xFFu;
    debug_attach_snapshot_.attached = 0u;
    debug_attach_snapshot_.dmstatus = 0u;
    debug_attach_snapshot_.abstractcs = 0u;
    debug_attach_snapshot_.chip_id = 0u;
    ClearHostSelectedChipFamily();
    write_region_addr_ = 0u;
    write_region_len_ = 0u;
    read_region_addr_ = 0u;
    read_region_len_ = 0u;
    current_sdi_clock_hz_ = SDI_CLOCK_HZ_HIGH;
    target_state_ = {};
    program_mode_ = ProgramMode::IDLE;
    flash_op_ready_ = false;
    flash_op_rx_bytes_ = 0u;
    flash_op_overflow_ = false;
    flash_op_prepared_ = false;
    flash_op_layout_ = {};
    flash_loader_fill_ = 0u;
    flash_loader_buffer_bytes_ = 0u;
    flash_loader_verify_sum_ = 0u;
    flash_loader_verify_tail_fill_ = 0u;
    flash_program_flags_ = 0u;
    flash_stream_chunk_bytes_ = 4096u;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_received_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_acked_bytes_ = 0u;
    flash_stream_pages_committed_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    flash_stream_writer_active_ = false;
    flash_stream_writer_supported_ = false;
    target_debug_session_active_ = false;
    target_debug_session_needs_resume_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
    read_stream_active_ = false;
    read_stream_addr_ = 0u;
    read_stream_remaining_ = 0u;
    read_stream_error_ = false;
    flash_stream_ack_code_ = DATA_ACK_CODE_STREAM;
    flash_erase_requested_ = false;
    power_3v3_enabled_ = false;
    power_5v_enabled_ = false;
    rstout_state_ = 0u;
    pending_cmd_out_len_ = 0u;
    pending_cmd_out_valid_ = false;
    flash_stream_stage_head_ = 0u;
    flash_stream_stage_size_ = 0u;
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

  static void StoreLe32(uint8_t* p, uint32_t v)
  {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
  }

  static uint32_t DecodeSdiClockHz(uint8_t speed_code)
  {
    switch (speed_code)
    {
      case 0x03u:
        return SDI_CLOCK_HZ_LOW;
      case 0x02u:
        return SDI_CLOCK_HZ_MEDIUM;
      case 0x01u:
      default:
        return SDI_CLOCK_HZ_HIGH;
    }
  }

  static uint32_t RoundUpU32(uint32_t value, uint32_t align)
  {
    if (align == 0u)
    {
      return value;
    }
    const uint32_t REMAINDER = value % align;
    return (REMAINDER == 0u) ? value : (value + align - REMAINDER);
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
    const ErrorCode RESULT = riscv_target_.DmiRead(addr, data, ack);
    return RESULT == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK &&
           IsPlausibleDmiRegisterValue(data);
  }

  bool ReadDmiWordForAttach(uint8_t addr, uint32_t& data, DmiWordReadStatus& status)
  {
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;
    const ErrorCode RESULT = riscv_target_.DmiRead(addr, data, ack);
    if (RESULT != ErrorCode::OK || ack != LibXR::Debug::Sdi::Ack::OK)
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
    const ErrorCode RESULT = riscv_target_.DmiWrite(addr, data, ack);
    return RESULT == ErrorCode::OK && ack == LibXR::Debug::Sdi::Ack::OK;
  }

  bool WarmUpDmStatus(uint32_t& dmstatus, AttachFailureStage& failure_stage)
  {
    DmiWordReadStatus last_status = DmiWordReadStatus::TRANSACTION;
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt)
    {
      if (ReadDmiWordForAttach(ATTACH_DMSTATUS_ADDR, dmstatus, last_status))
      {
        return true;
      }

      (void)WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x00000001u);
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
      (void)WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x00000000u);
      sdi_.IdleClocks(8u);

      if (!WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x00000001u))
      {
        sdi_.IdleClocks(16u);
        continue;
      }

      sdi_.IdleClocks(static_cast<uint32_t>(16u + attempt * 8u));

      uint32_t dmcontrol = 0u;
      if (ReadValidatedDmiWord(ATTACH_DMCONTROL_ADDR, dmcontrol) && (dmcontrol & 0x1u) != 0u)
      {
        sdi_.IdleClocks(16u);
        return true;
      }

      sdi_.IdleClocks(24u);
    }

    return false;
  }

  bool TryAttachTarget(uint32_t& chip_id, AttachFailureStage& failure_stage)
  {
    debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(AttachFailureStage::ENTER_SDI);
    debug_attach_snapshot_.attached = 0u;
    debug_attach_snapshot_.dmstatus = 0u;
    debug_attach_snapshot_.abstractcs = 0u;
    debug_attach_snapshot_.chip_id = 0u;

    if (!ActivateDebugModule())
    {
      failure_stage = AttachFailureStage::DMACTIVE;
      debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(failure_stage);
      return false;
    }

    uint32_t dmstatus = 0u;
    if (!WarmUpDmStatus(dmstatus, failure_stage))
    {
      debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(failure_stage);
      debug_attach_snapshot_.dmstatus = dmstatus;
      return false;
    }
    debug_attach_snapshot_.dmstatus = dmstatus;

    uint32_t abstractcs = 0u;
    if (!ReadValidatedDmiWord(ATTACH_ABSTRACTCS_ADDR, abstractcs))
    {
      failure_stage = AttachFailureStage::ABSTRACTCS;
      debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(failure_stage);
      return false;
    }
    debug_attach_snapshot_.abstractcs = abstractcs;

    if (!WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x80000001u))
    {
      failure_stage = AttachFailureStage::DMSTATUS;
      debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(failure_stage);
      return false;
    }

    bool hart_halted = false;
    for (uint8_t attempt = 0u; attempt < 32u; ++attempt)
    {
      uint32_t halt_dmstatus = 0u;
      DmiWordReadStatus halt_status = DmiWordReadStatus::TRANSACTION;
      if (ReadDmiWordForAttach(ATTACH_DMSTATUS_ADDR, halt_dmstatus, halt_status))
      {
        debug_attach_snapshot_.dmstatus = halt_dmstatus;
        if (RiscvDmiTarget<SdiPort>::IsDmStatusHalted(halt_dmstatus))
        {
          hart_halted = true;
          break;
        }
      }
      sdi_.IdleClocks(static_cast<uint32_t>(16u + attempt * 4u));
    }

    if (!hart_halted)
    {
      if (RiscvDmiTarget<SdiPort>::IsDmStatusRunning(debug_attach_snapshot_.dmstatus) &&
          TryResetAndHaltHart(dmstatus))
      {
        hart_halted = true;
      }
    }

    if (!hart_halted)
    {
      failure_stage = AttachFailureStage::DMSTATUS;
      debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(failure_stage);
      return false;
    }

    chip_id = 0u;
    for (uint8_t attempt = 0u; attempt < 2u; ++attempt)
    {
      if (riscv_target_.ReadWchChipId(chip_id))
      {
        return true;
      }
    }
    if (chip_id == 0u || chip_id == 0xFFFFFFFFu)
    {
      failure_stage = AttachFailureStage::CHIP_ID;
      debug_attach_snapshot_.failure_stage = static_cast<uint8_t>(failure_stage);
      debug_attach_snapshot_.chip_id = chip_id;
      return false;
    }
    debug_attach_snapshot_.chip_id = chip_id;
    debug_attach_snapshot_.failure_stage = 0xFFu;
    debug_attach_snapshot_.attached = 1u;
    return true;
  }

  bool TryResetAndHaltHart(uint32_t& dmstatus)
  {
    if (!WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x80000003u))
    {
      return false;
    }

    for (uint8_t attempt = 0u; attempt < 32u; ++attempt)
    {
      DmiWordReadStatus status = DmiWordReadStatus::TRANSACTION;
      if (ReadDmiWordForAttach(ATTACH_DMSTATUS_ADDR, dmstatus, status))
      {
        debug_attach_snapshot_.dmstatus = dmstatus;
        if (RiscvDmiTarget<SdiPort>::IsDmStatusHaveResetLatched(dmstatus) ||
            RiscvDmiTarget<SdiPort>::IsDmStatusHalted(dmstatus))
        {
          break;
        }
      }
      sdi_.IdleClocks(static_cast<uint32_t>(16u + attempt * 4u));
    }

    for (uint8_t attempt = 0u; attempt < 32u; ++attempt)
    {
      if (!WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x90000001u))
      {
        return false;
      }

      DmiWordReadStatus status = DmiWordReadStatus::TRANSACTION;
      if (ReadDmiWordForAttach(ATTACH_DMSTATUS_ADDR, dmstatus, status))
      {
        debug_attach_snapshot_.dmstatus = dmstatus;
        if (!RiscvDmiTarget<SdiPort>::IsDmStatusHaveResetLatched(dmstatus) &&
            RiscvDmiTarget<SdiPort>::IsDmStatusHalted(dmstatus))
        {
          return WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x00000001u);
        }
      }

      sdi_.IdleClocks(static_cast<uint32_t>(16u + attempt * 4u));
    }

    (void)WriteValidatedDmiWord(ATTACH_DMCONTROL_ADDR, 0x00000001u);
    return false;
  }

  uint8_t EffectiveChipFamily() const
  {
    if (target_state_.reported_family != 0u)
    {
      return target_state_.reported_family;
    }
    return target_state_.requested_family;
  }

  uint8_t ResolveReportedChipFamily(uint32_t chip_id) const
  {
    if (target_state_.requested_family != 0u)
    {
      return target_state_.requested_family;
    }
    return RiscvDmiTarget<SdiPort>::DetectWchChipFamilyFromChipId(chip_id);
  }

  void RefreshCompatDescriptorFromState()
  {
    target_state_.compat = RiscvDmiTarget<SdiPort>::BuildWchLinkCompatDescriptor(
        EffectiveChipFamily(), target_state_.flash_size_kb);
  }

  bool RefreshTargetFlashCompatState()
  {
    uint16_t flash_size_kb = 0u;
    if (!riscv_target_.ReadWchFlashSizeKb(flash_size_kb))
    {
      return false;
    }

    target_state_.flash_size_kb = flash_size_kb;
    RefreshCompatDescriptorFromState();
    return true;
  }

  bool RewarmDebugModuleAfterReset(bool halt_after_reset)
  {
    if (!attached_)
    {
      return false;
    }
    if (!ActivateDebugModule())
    {
      return false;
    }

    uint32_t dmstatus = 0u;
    AttachFailureStage failure_stage = AttachFailureStage::DMSTATUS;
    if (!WarmUpDmStatus(dmstatus, failure_stage))
    {
      return false;
    }

    if (halt_after_reset)
    {
      return riscv_target_.RequestHartHalt();
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
    if (!resp || cap < 4u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }
    resp[0] = 0x81u;
    resp[1] = 0x55u;
    resp[2] = 0x01u;
    resp[3] = reason;
    out_len = 4u;
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

  void ClearHostSelectedChipFamily()
  {
    target_state_.requested_family = 0u;
    RefreshCompatDescriptorFromState();
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
      return !IsFlashReceiveFinished() && FlashStreamStageHasRoom();
    }

    return false;
  }

  bool FlashStreamStageHasRoom() const
  {
    return flash_stream_stage_size_ < static_cast<uint16_t>(flash_stream_stage_.size());
  }

  bool QueueFlashStreamData(const uint8_t* data, uint32_t size)
  {
    if (!data || size == 0u)
    {
      return true;
    }
    if (size > flash_stream_stage_.size())
    {
      return false;
    }

    const uint16_t QUEUED = flash_stream_stage_size_;
    if ((static_cast<uint32_t>(QUEUED) + size) > flash_stream_stage_.size())
    {
      return false;
    }

    const uint16_t HEAD = flash_stream_stage_head_;
    const uint16_t TAIL =
        static_cast<uint16_t>((static_cast<uint32_t>(HEAD) + QUEUED) % flash_stream_stage_.size());
    const uint16_t FIRST_BYTES = static_cast<uint16_t>(
        MinU32(size, static_cast<uint32_t>(flash_stream_stage_.size() - TAIL)));
    std::memcpy(flash_stream_stage_.data() + TAIL, data, FIRST_BYTES);
    if (size > FIRST_BYTES)
    {
      std::memcpy(flash_stream_stage_.data(), data + FIRST_BYTES, size - FIRST_BYTES);
    }

    flash_stream_stage_size_ = static_cast<uint16_t>(QUEUED + size);
    return true;
  }

  bool ArmDataOutForStreamIfIdle()
  {
    if (!ShouldAcceptDataOut())
    {
      return false;
    }
    return ArmDataOutIfIdle();
  }

  void QueueDataAck(uint8_t count = 1u)
  {
    if (count == 0u)
    {
      return;
    }
    const uint16_t NEXT_PENDING_ACK =
        static_cast<uint16_t>(pending_data_ack_) + static_cast<uint16_t>(count);
    pending_data_ack_ =
        (NEXT_PENDING_ACK > 0xFFu) ? 0xFFu : static_cast<uint8_t>(NEXT_PENDING_ACK);
  }

  bool FlushPendingDataAck()
  {
    if (pending_data_ack_ == 0u || data_ack_in_flight_)
    {
      return false;
    }
    if (!CanSendFlashWriteAckNow())
    {
      return false;
    }
    return TrySendDataAck();
  }

  void EnterFlashOpStream()
  {
    ExitReadStream();
    program_mode_ = ProgramMode::WRITE_FLASH_OP;
    flash_op_image_len_ = 0u;
    flash_op_rx_bytes_ = 0u;
    flash_op_overflow_ = false;
    flash_op_ready_ = false;
    flash_op_prepared_ = false;
    flash_op_layout_ = {};
    flash_loader_fill_ = 0u;
    flash_loader_buffer_bytes_ = 0u;
    flash_loader_verify_sum_ = 0u;
    flash_loader_verify_tail_fill_ = 0u;
    flash_program_flags_ = 0u;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_received_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_acked_bytes_ = 0u;
    flash_stream_pages_committed_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    flash_stream_ack_code_ = DATA_ACK_CODE_STREAM;
    flash_stream_writer_active_ = false;
    flash_stream_writer_supported_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
    flash_stream_stage_head_ = 0u;
    flash_stream_stage_size_ = 0u;
    ArmDataOutForStreamIfIdle();
  }

  void ExitProgramStream()
  {
    if (flash_stream_writer_active_)
    {
      riscv_target_.EndMemoryWriteSession();
    }
    program_mode_ = ProgramMode::IDLE;
    flash_op_ready_ = false;
    flash_op_prepared_ = false;
    flash_op_image_len_ = 0u;
    flash_op_rx_bytes_ = 0u;
    flash_op_overflow_ = false;
    flash_op_layout_ = {};
    flash_loader_fill_ = 0u;
    flash_loader_buffer_bytes_ = 0u;
    flash_loader_verify_sum_ = 0u;
    flash_loader_verify_tail_fill_ = 0u;
    flash_program_flags_ = 0u;
    flash_stream_total_raw_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_acked_bytes_ = 0u;
    flash_stream_pages_committed_ = 0u;
    flash_loader_fill_ = 0u;
    flash_loader_buffer_bytes_ = 0u;
    flash_loader_verify_sum_ = 0u;
    flash_loader_verify_tail_fill_ = 0u;
    flash_stream_next_ack_at_ = 0u;
    flash_stream_write_addr_ = 0u;
    flash_stream_error_ = false;
    flash_stream_ack_code_ = DATA_ACK_CODE_STREAM;
    flash_stream_writer_active_ = false;
    flash_stream_writer_supported_ = false;
    pending_data_ack_ = 0u;
    data_ack_in_flight_ = false;
    flash_stream_stage_head_ = 0u;
    flash_stream_stage_size_ = 0u;
    EndTargetDebugSession();
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

    // OpenOCD wlink batches flash payloads by family-specific pack size and
    // only drains one EP2-IN completion frame after each batch.
    RefreshCompatDescriptorFromState();
    flash_stream_chunk_bytes_ = target_state_.compat.write_pack_size;
    flash_stream_received_bytes_ = 0u;
    flash_stream_rx_bytes_ = 0u;
    flash_stream_acked_bytes_ = 0u;
    flash_stream_pages_committed_ = 0u;
    flash_stream_next_ack_at_ = MinU32(flash_stream_chunk_bytes_, flash_stream_total_raw_bytes_);
    flash_stream_write_addr_ = write_region_addr_;
    flash_stream_error_ = false;
    flash_stream_ack_code_ = flash_erase_requested_ ? DATA_ACK_CODE_ERASE : DATA_ACK_CODE_STREAM;
    flash_stream_writer_active_ = false;
    flash_stream_writer_supported_ = true;
    flash_erase_requested_ = false;
    pending_data_ack_ = 0u;
    flash_stream_stage_head_ = 0u;
    flash_stream_stage_size_ = 0u;
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
    if (!data || rx_bytes == 0u || flash_stream_error_)
    {
      return;
    }

    if (flash_stream_rx_bytes_ >= flash_stream_total_raw_bytes_)
    {
      return;
    }
    const uint32_t REMAIN_TOTAL = flash_stream_total_raw_bytes_ - flash_stream_rx_bytes_;
    const uint32_t CONSUME_BYTES = MinU32(rx_bytes, REMAIN_TOTAL);
    if (!WriteFlashStreamChunk(data, CONSUME_BYTES))
    {
      if (!flash_stream_error_)
      {
        SetFlashStreamError(DATA_ACK_CODE_STREAM_WRITE_FAIL);
      }
      return;
    }
    flash_stream_rx_bytes_ += CONSUME_BYTES;
    if (flash_stream_rx_bytes_ >= flash_stream_total_raw_bytes_ && !FinalizeFlashWriteStream())
    {
      if (!flash_stream_error_)
      {
        SetFlashStreamError(DATA_ACK_CODE_STREAM_FINALIZE_FAIL);
      }
      return;
    }
  }

  bool IsFlashWriteFinished() const
  {
    return flash_stream_total_raw_bytes_ > 0u && flash_stream_rx_bytes_ >= flash_stream_total_raw_bytes_ &&
           flash_stream_stage_size_ == 0u;
  }

  bool IsFlashReceiveFinished() const
  {
    return flash_stream_total_raw_bytes_ > 0u &&
           flash_stream_received_bytes_ >= flash_stream_total_raw_bytes_;
  }

  void QueueFlashWriteAcks()
  {
    if (flash_stream_next_ack_at_ == 0u)
    {
      return;
    }
    while (flash_stream_received_bytes_ >= flash_stream_next_ack_at_)
    {
      QueueDataAck();
      if (flash_stream_next_ack_at_ >= flash_stream_total_raw_bytes_)
      {
        flash_stream_next_ack_at_ = 0u;
        break;
      }

      const uint32_t REMAIN_BYTES = flash_stream_total_raw_bytes_ - flash_stream_next_ack_at_;
      flash_stream_next_ack_at_ += MinU32(flash_stream_chunk_bytes_, REMAIN_BYTES);
    }
  }

  void SetFlashStreamError(uint8_t ack_code)
  {
    flash_stream_error_ = true;
    flash_stream_ack_code_ = ack_code;
    QueueDataAck();
  }

  bool IsFlashStreamSuccessAck() const
  {
    return flash_stream_ack_code_ == DATA_ACK_CODE_STREAM || flash_stream_ack_code_ == DATA_ACK_CODE_ERASE;
  }

  uint32_t CurrentFlashAckBytes() const
  {
    if (flash_stream_acked_bytes_ >= flash_stream_total_raw_bytes_)
    {
      return 0u;
    }
    return MinU32(flash_stream_chunk_bytes_, flash_stream_total_raw_bytes_ - flash_stream_acked_bytes_);
  }

  uint32_t NextFlashHostChunkBytes() const
  {
    const uint32_t CURRENT_ACK_BYTES = CurrentFlashAckBytes();
    if (CURRENT_ACK_BYTES == 0u)
    {
      return 0u;
    }

    const uint32_t ACKED_AFTER_SEND = flash_stream_acked_bytes_ + CURRENT_ACK_BYTES;
    if (ACKED_AFTER_SEND >= flash_stream_total_raw_bytes_)
    {
      return 0u;
    }
    return MinU32(flash_stream_chunk_bytes_, flash_stream_total_raw_bytes_ - ACKED_AFTER_SEND);
  }

  bool CanSendFlashWriteAckNow() const
  {
    if (program_mode_ != ProgramMode::WRITE_FLASH_STREAM || !IsFlashStreamSuccessAck())
    {
      return true;
    }

    const uint32_t NEXT_HOST_CHUNK_BYTES = NextFlashHostChunkBytes();
    if (NEXT_HOST_CHUNK_BYTES == 0u)
    {
      return true;
    }

    return static_cast<uint32_t>(flash_stream_stage_size_) + NEXT_HOST_CHUNK_BYTES <=
           static_cast<uint32_t>(flash_stream_stage_.size());
  }

  void CaptureFlashOpData(const uint8_t* data, uint32_t size)
  {
    if (!data || size == 0u)
    {
      return;
    }

    flash_op_rx_bytes_ += size;
    const uint32_t REMAIN = static_cast<uint32_t>(flash_op_image_.size()) - flash_op_image_len_;
    const uint32_t COPY_BYTES = MinU32(size, REMAIN);
    if (COPY_BYTES > 0u)
    {
      std::memcpy(flash_op_image_.data() + flash_op_image_len_, data, COPY_BYTES);
      flash_op_image_len_ += static_cast<uint16_t>(COPY_BYTES);
    }
    if (COPY_BYTES != size)
    {
      flash_op_overflow_ = true;
    }
  }

  bool DetectFlashOpLayout(FlashOpLayout& layout) const
  {
    layout = {};
    if (flash_op_image_len_ < 4u)
    {
      return false;
    }

    const auto SIGN_EXTEND = [](uint32_t value, uint8_t bits) -> int32_t {
      const uint32_t SIGN_BIT = 1u << (bits - 1u);
      const uint32_t MASK = (1u << bits) - 1u;
      value &= MASK;
      return static_cast<int32_t>((value ^ SIGN_BIT) - SIGN_BIT);
    };

    uint32_t min_ram_addr = 0xFFFFFFFFu;
    uint32_t max_ram_addr = 0u;
    for (uint16_t off = 0u; (off + 4u) <= flash_op_image_len_;)
    {
      const uint16_t HALFWORD = static_cast<uint16_t>(flash_op_image_[off]) |
                                (static_cast<uint16_t>(flash_op_image_[off + 1u]) << 8u);
      if ((HALFWORD & 0x3u) != 0x3u)
      {
        off += 2u;
        continue;
      }

      const uint32_t INSN = LoadLe32(flash_op_image_.data() + off);
      if ((INSN & 0x7Fu) == 0x37u)
      {
        const uint8_t RD = static_cast<uint8_t>((INSN >> 7u) & 0x1Fu);
        uint32_t addr = INSN & 0xFFFFF000u;
        if (RD != 0u && (addr & 0xFFF00000u) == 0x20000000u && addr > layout.code_addr)
        {
          int32_t delta = 0;
          const uint16_t NEXT = static_cast<uint16_t>(off + 4u);
          if ((NEXT + 2u) <= flash_op_image_len_)
          {
            const uint16_t NEXT_HALFWORD =
                static_cast<uint16_t>(flash_op_image_[NEXT]) |
                (static_cast<uint16_t>(flash_op_image_[NEXT + 1u]) << 8u);
            if ((NEXT_HALFWORD & 0x3u) != 0x3u)
            {
              const uint8_t FUNCT3 = static_cast<uint8_t>((NEXT_HALFWORD >> 13u) & 0x7u);
              const uint8_t RD_RS1 = static_cast<uint8_t>((NEXT_HALFWORD >> 7u) & 0x1Fu);
              if (FUNCT3 == 0u && RD_RS1 == RD)
              {
                const uint32_t IMM =
                    ((static_cast<uint32_t>(NEXT_HALFWORD) >> 2u) & 0x1Fu) |
                    ((static_cast<uint32_t>(NEXT_HALFWORD) >> 12u) & 0x20u);
                delta = SIGN_EXTEND(IMM, 6u);
              }
            }
            else if ((NEXT + 4u) <= flash_op_image_len_)
            {
              const uint32_t NEXT_INSN = LoadLe32(flash_op_image_.data() + NEXT);
              const uint8_t OPCODE = static_cast<uint8_t>(NEXT_INSN & 0x7Fu);
              const uint8_t FUNCT3 = static_cast<uint8_t>((NEXT_INSN >> 12u) & 0x7u);
              const uint8_t RD2 = static_cast<uint8_t>((NEXT_INSN >> 7u) & 0x1Fu);
              const uint8_t RS1 = static_cast<uint8_t>((NEXT_INSN >> 15u) & 0x1Fu);
              if (OPCODE == 0x13u && RD2 == RD && RS1 == RD && (FUNCT3 == 0u || FUNCT3 == 0x6u))
              {
                delta = SIGN_EXTEND(NEXT_INSN >> 20u, 12u);
              }
            }
          }

          addr = static_cast<uint32_t>(static_cast<int32_t>(addr) + delta);
          if (addr < min_ram_addr)
          {
            min_ram_addr = addr;
          }
          if (addr > max_ram_addr)
          {
            max_ram_addr = addr;
          }
        }
      }
      off += 4u;
    }

    if (min_ram_addr != 0xFFFFFFFFu)
    {
      layout.page_buffer_addr = AlignDownU32(min_ram_addr, 0x100u);
    }
    if (max_ram_addr != 0u)
    {
      layout.verify_sum_addr = ((max_ram_addr & 0xFFu) == 0u) ? (max_ram_addr + 0x10u) : max_ram_addr;
      layout.stack_addr = AlignUpU32(layout.verify_sum_addr + 0x400u, 0x100u);
    }
    return true;
  }

  bool RunFlashLoader(uint8_t flags, uint32_t addr, uint32_t len, uint8_t& error_code)
  {
    error_code = DATA_ACK_CODE_STREAM_LOADER_RUN_FAIL_BASE;
    if (!EnsureTargetDebugSession())
    {
      return false;
    }

    ClearFlashLoaderDebugSnapshot();

    if (!riscv_target_.WriteCpuRegister(kDebugRegSp, flash_op_layout_.stack_addr) ||
        !riscv_target_.WriteCpuRegister(kDebugRegA0, static_cast<uint32_t>(flags)) ||
        !riscv_target_.WriteCpuRegister(kDebugRegA1, addr) ||
        !riscv_target_.WriteCpuRegister(kDebugRegA2, len))
    {
      error_code = DATA_ACK_CODE_STREAM_LOADER_REG_FAIL_BASE;
      return false;
    }

    const uint32_t PAGE_COUNT =
        (flash_program_page_size_ == 0u) ? 1u : AlignUpU32(len, flash_program_page_size_) / flash_program_page_size_;
    const uint16_t HALT_POLLS =
        static_cast<uint16_t>(MinU32(0xFFFFu, 2048u + (PAGE_COUNT * 4096u)));
    uint32_t result = 0u;
    typename RiscvDmiTarget<SdiPort>::RunProgramDebugSnapshot run_debug = {};
    if (!riscv_target_.RunProgramAndWaitForHalt(
            flash_op_layout_.code_addr, result, HALT_POLLS, &run_debug))
    {
      SaveFlashLoaderDebugSnapshot(run_debug);
      error_code = DATA_ACK_CODE_STREAM_LOADER_RUN_FAIL_BASE;
      return false;
    }
    SaveFlashLoaderDebugSnapshot(run_debug);
    debug_flash_loader_snapshot_.a0_valid = 1u;
    debug_flash_loader_snapshot_.a0_result = result;
    if (result != 0u)
    {
      error_code = DATA_ACK_CODE_STREAM_LOADER_RESULT_FAIL_BASE;
      return false;
    }
    return true;
  }

  bool PrepareFlashLoader()
  {
    if (flash_op_prepared_)
    {
      return true;
    }
    if (flash_op_overflow_ || flash_op_image_len_ == 0u)
    {
      return false;
    }

    (void)DetectFlashOpLayout(flash_op_layout_);
    flash_program_page_size_ = target_state_.compat.data_packet_size;
    if (flash_program_page_size_ == 0u || flash_program_page_size_ > flash_page_buffer_.size())
    {
      return false;
    }

    if (!EnsureTargetDebugSession() ||
        !riscv_target_.WriteMemoryBlock(flash_op_layout_.code_addr, flash_op_image_.data(),
                                        flash_op_image_len_))
    {
      return false;
    }

    const uint32_t BUFFER_CAPACITY =
        AlignDownU32(FlashLoaderBufferCapacityBytes(), flash_program_page_size_);
    flash_loader_buffer_bytes_ =
        AlignDownU32(MinU32(BUFFER_CAPACITY, flash_stream_chunk_bytes_), flash_program_page_size_);
    if (flash_loader_buffer_bytes_ == 0u)
    {
      return false;
    }

    flash_program_flags_ = static_cast<uint8_t>(kFlashOpFlagUnlock | kFlashOpFlagPageErase |
                                                kFlashOpFlagProgram | kFlashOpFlagVerify);
    flash_loader_fill_ = 0u;
    flash_loader_verify_sum_ = 0u;
    flash_loader_verify_tail_fill_ = 0u;
    flash_op_prepared_ = true;
    return true;
  }

  uint32_t FlashLoaderBufferCapacityBytes() const
  {
    if (flash_program_page_size_ == 0u || flash_op_layout_.verify_sum_addr <= flash_op_layout_.page_buffer_addr)
    {
      return 0u;
    }

    return flash_op_layout_.verify_sum_addr - flash_op_layout_.page_buffer_addr;
  }

  void AccumulateFlashVerifySum(const uint8_t* data, uint32_t len)
  {
    if (!data || len == 0u)
    {
      return;
    }

    uint32_t off = 0u;
    if (flash_loader_verify_tail_fill_ != 0u)
    {
      while (off < len && flash_loader_verify_tail_fill_ < 4u)
      {
        flash_loader_verify_tail_[flash_loader_verify_tail_fill_] = data[off];
        ++flash_loader_verify_tail_fill_;
        ++off;
      }
      if (flash_loader_verify_tail_fill_ == 4u)
      {
        flash_loader_verify_sum_ += LoadLe32(flash_loader_verify_tail_.data());
        flash_loader_verify_tail_fill_ = 0u;
      }
    }

    while ((off + 4u) <= len)
    {
      flash_loader_verify_sum_ += LoadLe32(data + off);
      off += 4u;
    }

    if (off < len)
    {
      std::memset(flash_loader_verify_tail_.data(), 0xFFu, flash_loader_verify_tail_.size());
      flash_loader_verify_tail_fill_ = static_cast<uint8_t>(len - off);
      std::memcpy(flash_loader_verify_tail_.data(), data + off, flash_loader_verify_tail_fill_);
    }
  }

  uint32_t FinalizeFlashVerifySum()
  {
    uint32_t sum = flash_loader_verify_sum_;
    if (flash_loader_verify_tail_fill_ != 0u)
    {
      std::memset(flash_loader_verify_tail_.data() + flash_loader_verify_tail_fill_, 0xFFu,
                  flash_loader_verify_tail_.size() - flash_loader_verify_tail_fill_);
      sum += LoadLe32(flash_loader_verify_tail_.data());
    }
    return sum;
  }

  bool FlushFlashLoaderBuffer(uint32_t valid_bytes)
  {
    if (valid_bytes == 0u)
    {
      return true;
    }
    if (!flash_op_prepared_ || flash_program_page_size_ == 0u || flash_loader_buffer_bytes_ == 0u ||
        valid_bytes > flash_loader_buffer_bytes_)
    {
      return false;
    }

    const uint32_t PADDED_BYTES = AlignUpU32(valid_bytes, flash_program_page_size_);
    const uint8_t PAGE_SLOT = static_cast<uint8_t>(flash_stream_pages_committed_ & 0x0Fu);
    if (PADDED_BYTES > valid_bytes)
    {
      std::memset(flash_page_buffer_.data(), 0xFFu, flash_page_buffer_.size());
      const uint32_t PAD_BYTES = PADDED_BYTES - valid_bytes;
      if (!riscv_target_.WriteMemoryBlock(flash_op_layout_.page_buffer_addr + valid_bytes,
                                          flash_page_buffer_.data(), PAD_BYTES))
      {
        SetFlashStreamError(static_cast<uint8_t>(DATA_ACK_CODE_STREAM_PAGE_BUFFER_FAIL_BASE | PAGE_SLOT));
        return false;
      }
    }

    const uint32_t VERIFY_SUM = FinalizeFlashVerifySum();
    uint8_t verify_sum_bytes[4] = {};
    StoreLe32(verify_sum_bytes, VERIFY_SUM);
    if (!riscv_target_.WriteMemoryBlock(
            flash_op_layout_.verify_sum_addr, verify_sum_bytes, sizeof(verify_sum_bytes)))
    {
      SetFlashStreamError(static_cast<uint8_t>(DATA_ACK_CODE_STREAM_VERIFY_SUM_FAIL_BASE | PAGE_SLOT));
      return false;
    }
    uint8_t loader_error_code = DATA_ACK_CODE_STREAM_LOADER_RUN_FAIL_BASE;
    if (!RunFlashLoader(flash_program_flags_, flash_stream_write_addr_, valid_bytes, loader_error_code))
    {
      SetFlashStreamError(static_cast<uint8_t>(loader_error_code | PAGE_SLOT));
      return false;
    }

    flash_stream_write_addr_ += valid_bytes;
    flash_loader_fill_ = 0u;
    flash_loader_verify_sum_ = 0u;
    flash_loader_verify_tail_fill_ = 0u;
    flash_stream_pages_committed_ += PADDED_BYTES / flash_program_page_size_;
    return true;
  }

  bool FinalizeFlashWriteStream()
  {
    if (!flash_op_prepared_ || flash_loader_fill_ == 0u)
    {
      return true;
    }
    return FlushFlashLoaderBuffer(flash_loader_fill_);
  }

  void ClearFlashLoaderDebugSnapshot()
  {
    debug_flash_loader_snapshot_.failure_stage = 0u;
    debug_flash_loader_snapshot_.resume_ack_seen = 0u;
    debug_flash_loader_snapshot_.hart_halted_seen = 0u;
    debug_flash_loader_snapshot_.a0_valid = 0u;
    debug_flash_loader_snapshot_.dmstatus_before_resume = 0u;
    debug_flash_loader_snapshot_.dmstatus_after_resume = 0u;
    debug_flash_loader_snapshot_.dmstatus_after_halt = 0u;
    debug_flash_loader_snapshot_.dmcontrol_after_resume = 0u;
    debug_flash_loader_snapshot_.a0_result = 0u;
    debug_flash_loader_snapshot_.dpc = 0u;
    debug_flash_loader_snapshot_.mepc = 0u;
    debug_flash_loader_snapshot_.mcause = 0u;
  }

  void SaveFlashLoaderDebugSnapshot(
      const typename RiscvDmiTarget<SdiPort>::RunProgramDebugSnapshot& run_debug)
  {
    debug_flash_loader_snapshot_.failure_stage = run_debug.failure_stage;
    debug_flash_loader_snapshot_.resume_ack_seen = run_debug.resume_ack_seen;
    debug_flash_loader_snapshot_.hart_halted_seen = run_debug.hart_halted_seen;
    debug_flash_loader_snapshot_.a0_valid = run_debug.a0_valid;
    debug_flash_loader_snapshot_.dmstatus_before_resume = run_debug.dmstatus_before_resume;
    debug_flash_loader_snapshot_.dmstatus_after_resume = run_debug.dmstatus_after_resume;
    debug_flash_loader_snapshot_.dmstatus_after_halt = run_debug.dmstatus_after_halt;
    debug_flash_loader_snapshot_.dmcontrol_after_resume = run_debug.dmcontrol_after_resume;
    debug_flash_loader_snapshot_.a0_result = run_debug.a0_result;
    debug_flash_loader_snapshot_.dpc = run_debug.dpc;
    debug_flash_loader_snapshot_.mepc = run_debug.mepc;
    debug_flash_loader_snapshot_.mcause = run_debug.mcause;
  }

  bool WriteFlashLoaderChunk(const uint8_t* data, uint32_t size)
  {
    if (!data || size == 0u || !flash_op_prepared_ || flash_program_page_size_ == 0u)
    {
      return size == 0u;
    }

    uint32_t off = 0u;
    while (off < size)
    {
      const uint32_t BUFFER_REMAIN = flash_loader_buffer_bytes_ - flash_loader_fill_;
      const uint32_t COPY_BYTES = MinU32(size - off, BUFFER_REMAIN);
      const uint8_t PAGE_SLOT = static_cast<uint8_t>(flash_stream_pages_committed_ & 0x0Fu);
      if (!riscv_target_.WriteMemoryBlock(flash_op_layout_.page_buffer_addr + flash_loader_fill_,
                                          data + off, COPY_BYTES))
      {
        SetFlashStreamError(static_cast<uint8_t>(DATA_ACK_CODE_STREAM_PAGE_BUFFER_FAIL_BASE | PAGE_SLOT));
        return false;
      }
      AccumulateFlashVerifySum(data + off, COPY_BYTES);
      flash_loader_fill_ += COPY_BYTES;
      off += COPY_BYTES;
      if (flash_loader_fill_ == flash_loader_buffer_bytes_ &&
          !FlushFlashLoaderBuffer(flash_loader_fill_))
      {
        return false;
      }
    }
    return true;
  }

  bool WriteFlashStreamChunk(const uint8_t* data, uint32_t size)
  {
    if (!data || size == 0u)
    {
      return true;
    }

    if (flash_op_ready_ && !flash_op_prepared_)
    {
      if (!PrepareFlashLoader())
      {
        return false;
      }
    }

    if (flash_op_prepared_)
    {
      return WriteFlashLoaderChunk(data, size);
    }

    uint32_t off = 0u;
    while (off < size)
    {
      if (((flash_stream_write_addr_ & 0x3u) == 0u) && (size - off >= 4u))
      {
        if (!flash_stream_writer_active_ && flash_stream_writer_supported_)
        {
          flash_stream_writer_active_ =
              riscv_target_.BeginMemoryWriteSession(flash_stream_write_addr_);
          if (!flash_stream_writer_active_)
          {
            flash_stream_writer_supported_ = false;
          }
        }

        if (flash_stream_writer_active_)
        {
          while (((flash_stream_write_addr_ & 0x3u) == 0u) && (size - off >= 4u))
          {
            const uint32_t WORD = LoadLe32(data + off);
            if (!riscv_target_.WriteMemoryWordStreaming(WORD))
            {
              return false;
            }
            flash_stream_write_addr_ += 4u;
            off += 4u;
          }
          continue;
        }

        const uint32_t WORD = LoadLe32(data + off);
        if (!riscv_target_.WriteWordByAbstract(flash_stream_write_addr_, WORD))
        {
          return false;
        }
        flash_stream_write_addr_ += 4u;
        off += 4u;
      }
      else
      {
        if (flash_stream_writer_active_)
        {
          riscv_target_.EndMemoryWriteSession();
          flash_stream_writer_active_ = false;
        }
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
    if (target_debug_session_active_)
    {
      riscv_target_.EndHartHaltSession(target_debug_session_needs_resume_);
      target_debug_session_active_ = false;
      target_debug_session_needs_resume_ = false;
    }
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

  bool PumpReadStreamIfIdle()
  {
    if (!read_stream_active_)
    {
      return false;
    }
    if (read_stream_remaining_ == 0u)
    {
      read_stream_active_ = false;
      return false;
    }
    return TrySendReadStreamChunk();
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

    const uint32_t CHUNK_BYTES =
        MinU32(static_cast<uint32_t>(tx.size_), read_stream_remaining_);
    auto* out = static_cast<uint8_t*>(tx.addr_);

    uint32_t filled = 0u;
    while (filled < CHUNK_BYTES)
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

      const uint32_t REMAIN_BYTES = CHUNK_BYTES - filled;
      const uint32_t EMIT_BYTES = MinU32(4u, REMAIN_BYTES);
      uint8_t be[4] = {};
      StoreBe32(be, word);
      std::memcpy(out + filled, be, EMIT_BYTES);

      filled += EMIT_BYTES;
      read_stream_addr_ += 4u;
    }

    read_stream_remaining_ -= CHUNK_BYTES;
    if (ep_data_in_->Transfer(static_cast<uint16_t>(CHUNK_BYTES)) != ErrorCode::OK)
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

  ErrorCode BuildEsigV2Response(uint8_t* resp, uint16_t cap, uint16_t& out_len)
  {
    if (!resp || cap < 20u)
    {
      out_len = 0u;
      return ErrorCode::NOT_FOUND;
    }

    uint32_t uid_word0 = 0u;
    uint32_t uid_word1 = 0u;
    if (EnsureTargetDebugSession())
    {
      (void)RefreshTargetFlashCompatState();
      (void)riscv_target_.ReadWchUidWords(uid_word0, uid_word1);
    }

    resp[0] = 0xFFu;
    resp[1] = 0xFFu;
    resp[2] = static_cast<uint8_t>(target_state_.flash_size_kb >> 8u);
    resp[3] = static_cast<uint8_t>(target_state_.flash_size_kb & 0xFFu);
    StoreBe32(resp + 4u, uid_word0);
    StoreBe32(resp + 8u, uid_word1);
    StoreBe32(resp + 12u, 0xFFFFFFFFu);
    StoreBe32(resp + 16u, target_state_.chip_id);
    out_len = 20u;
    return ErrorCode::OK;
  }

  ErrorCode HandleResetCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                               uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t SUB_CMD = payload[0];
    EndTargetDebugSession();

    switch (SUB_CMD)
    {
      case 0x01u:
        BusyDelayCycles(ATTACH_RETRY_DELAY_CYCLES);
        (void)riscv_target_.RequestHartResume();
        break;
      case 0x02u:
        PulseTargetReset();
        (void)RewarmDebugModuleAfterReset(false);
        break;
      case 0x03u:
        PulseTargetReset();
        (void)RewarmDebugModuleAfterReset(true);
        break;
      default:
        break;
    }

    session_state_ = SessionState::ACTIVE;
    return BuildStandardResponse(0x0Bu, &SUB_CMD, 1u, resp, cap, out_len);
  }

  ErrorCode HandleDisableDebugCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                      uint16_t cap, uint16_t& out_len)
  {
    const uint8_t SUB_CMD = (payload && payload_len >= 1u) ? payload[0] : 0x00u;
    return BuildStandardResponse(0x0Eu, &SUB_CMD, 1u, resp, cap, out_len);
  }

  ErrorCode HandleControlCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                 uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t SUB_CMD = payload[0];
    if (SUB_CMD == 0x01u)
    {
      const uint8_t PROBE_INFO[4] = {probe_id_.major, probe_id_.minor, probe_id_.variant, 0x00u};
      return BuildStandardResponse(0x0Du, PROBE_INFO, sizeof(PROBE_INFO), resp, cap, out_len);
    }
    if (SUB_CMD == 0x02u)
    {
      attached_ = false;
      flash_erase_requested_ = false;
      target_state_.reported_family = 0u;
      target_state_.chip_id = 0u;
      target_state_.flash_size_kb = 0u;
      RefreshCompatDescriptorFromState();
      ExitProgramStream();
      ExitReadStream();
      EndTargetDebugSession();
      uint32_t chip_id = 0u;
      bool attach_ok = false;
      bool provisional_attach = false;
      AttachFailureStage last_failure_stage = AttachFailureStage::ENTER_SDI;
      for (uint8_t attempt = 0u; attempt < ATTACH_RETRY_COUNT; ++attempt)
      {
        sdi_.Close();
        PulseTargetReset();
        BusyDelayCycles(ATTACH_RETRY_DELAY_CYCLES);

        const ErrorCode RESULT = sdi_.EnterSdi();
        if (RESULT != ErrorCode::OK)
        {
          last_failure_stage = AttachFailureStage::ENTER_SDI;
          continue;
        }

        if (TryAttachTarget(chip_id, last_failure_stage))
        {
          attach_ok = true;
          break;
        }
      }

      if (!attach_ok)
      {
        if (last_failure_stage == AttachFailureStage::CHIP_ID && target_state_.requested_family != 0u)
        {
          // Keep attach transport-oriented: once DMCONTROL/DMSTATUS are alive,
          // let the host continue with DMI even if chip-id probing is not yet reliable.
          attach_ok = true;
          provisional_attach = true;
          chip_id = 0u;
        }
        else
        {
          sdi_.Close();
          session_state_ = SessionState::LINK_FAULT;
          target_state_.reported_family = 0u;
          target_state_.chip_id = 0u;
          (void)last_failure_stage;
          return BuildErrorResponse(0x01u, resp, cap, out_len);
        }
      }

      target_state_.chip_id = chip_id;
      target_state_.reported_family = ResolveReportedChipFamily(chip_id);
      RefreshCompatDescriptorFromState();

      attached_ = true;
      dmi_needs_warmup_ = provisional_attach;
      session_state_ = SessionState::ACTIVE;
      if (!provisional_attach)
      {
        if (EnsureTargetDebugSession())
        {
          (void)RefreshTargetFlashCompatState();
        }
      }

      const uint8_t ATTACH_INFO[5] = {
          target_state_.reported_family,
          static_cast<uint8_t>(target_state_.chip_id >> 24),
          static_cast<uint8_t>(target_state_.chip_id >> 16),
          static_cast<uint8_t>(target_state_.chip_id >> 8),
          static_cast<uint8_t>(target_state_.chip_id)};
      return BuildStandardResponse(0x0Du, ATTACH_INFO, sizeof(ATTACH_INFO), resp, cap, out_len);
    }
    if (SUB_CMD == 0x03u || SUB_CMD == 0x10u)
    {
      if (attached_ && target_state_.flash_size_kb == 0u)
      {
        (void)RefreshTargetFlashCompatState();
      }
      return BuildStandardResponse(0x0Du, &SUB_CMD, 1u, resp, cap, out_len);
    }
    if (SUB_CMD == 0x04u)
    {
      if (attached_ && target_state_.flash_size_kb == 0u)
      {
        (void)RefreshTargetFlashCompatState();
      }
      const uint8_t LAYOUT_CLASS[1] = {target_state_.compat.rom_ram_class};
      return BuildStandardResponse(0x0Du, LAYOUT_CLASS, sizeof(LAYOUT_CLASS), resp, cap, out_len);
    }
    if (SUB_CMD == 0x17u)
    {
      if (attached_ && target_state_.flash_size_kb == 0u)
      {
        (void)RefreshTargetFlashCompatState();
      }
      const uint8_t LAYOUT_CLASS[1] = {target_state_.compat.extended_rom_ram_class};
      return BuildStandardResponse(0x0Du, LAYOUT_CLASS, sizeof(LAYOUT_CLASS), resp, cap, out_len);
    }
    if (SUB_CMD == 0x09u || SUB_CMD == 0x0Au)
    {
      power_3v3_enabled_ = SUB_CMD == 0x09u;
      return BuildStandardResponse(0x0Du, &SUB_CMD, 1u, resp, cap, out_len);
    }
    if (SUB_CMD == 0x0Bu || SUB_CMD == 0x0Cu)
    {
      power_5v_enabled_ = SUB_CMD == 0x0Bu;
      return BuildStandardResponse(0x0Du, &SUB_CMD, 1u, resp, cap, out_len);
    }
    if (SUB_CMD == 0x13u || SUB_CMD == 0x14u || SUB_CMD == 0x15u)
    {
      rstout_state_ = SUB_CMD;
      const uint8_t STATUS[1] = {0x00u};
      return BuildStandardResponse(0x0Du, STATUS, sizeof(STATUS), resp, cap, out_len);
    }
    if (SUB_CMD == 0xFFu)
    {
      attached_ = false;
      flash_erase_requested_ = false;
      target_state_.reported_family = 0u;
      target_state_.chip_id = 0u;
      target_state_.flash_size_kb = 0u;
      RefreshCompatDescriptorFromState();
      ExitProgramStream();
      ExitReadStream();
      EndTargetDebugSession();
      sdi_.Close();
      session_state_ = SessionState::ACTIVE;
      ClearHostSelectedChipFamily();
      ResetCommandResponseQueue();
      const uint8_t DONE_STATUS[1] = {0xFFu};
      return BuildStandardResponse(0x0Du, DONE_STATUS, sizeof(DONE_STATUS), resp, cap, out_len);
    }

    const uint8_t PASS_THROUGH[1] = {SUB_CMD};
    return BuildStandardResponse(0x0Du, PASS_THROUGH, sizeof(PASS_THROUGH), resp, cap, out_len);
  }

  ErrorCode HandleConfigChipCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                    uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }
    const uint8_t SUB_CMD = payload[0];
    uint8_t value = 0x00u;
    if (SUB_CMD == 0x01u)
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

    const uint8_t DMI_ADDR = payload[0];
    const uint32_t DMI_DATA = LoadBe32(payload + 1u);
    const uint8_t DMI_OP = payload[5];

    uint32_t out_data = 0u;
    uint8_t out_op = 0x00u;
    LibXR::Debug::Sdi::Ack ack = LibXR::Debug::Sdi::Ack::PROTOCOL;

    if (DMI_OP == 0x01u)
    {
      if (dmi_needs_warmup_ && DMI_ADDR == ATTACH_DMSTATUS_ADDR)
      {
        AttachFailureStage failure_stage = AttachFailureStage::DMSTATUS;
        if (ActivateDebugModule() && WarmUpDmStatus(out_data, failure_stage))
        {
          ack = LibXR::Debug::Sdi::Ack::OK;
          out_op = AckToDmiOp(ack);
          dmi_needs_warmup_ = false;
          return BuildDmiResponse(DMI_ADDR, out_data, out_op, resp, cap, out_len);
        }
      }

      const ErrorCode RESULT = riscv_target_.DmiRead(DMI_ADDR, out_data, ack);
      if (RESULT != ErrorCode::OK && RESULT != ErrorCode::FAILED &&
          RESULT != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        session_state_ = SessionState::LINK_FAULT;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_op = AckToDmiOp(ack);
      if (ack == LibXR::Debug::Sdi::Ack::OK && out_data != 0x00000000u &&
          out_data != 0xFFFFFFFFu)
      {
        dmi_needs_warmup_ = false;
      }
    }
    else if (DMI_OP == 0x02u)
    {
      const ErrorCode RESULT = riscv_target_.DmiWrite(DMI_ADDR, DMI_DATA, ack);
      if (RESULT != ErrorCode::OK && RESULT != ErrorCode::FAILED &&
          RESULT != ErrorCode::TIMEOUT)
      {
        attached_ = false;
        session_state_ = SessionState::LINK_FAULT;
        return BuildDmiNotAttachedResponse(resp, cap, out_len);
      }
      out_data = DMI_DATA;
      out_op = AckToDmiOp(ack);
    }
    else if (DMI_OP == 0x00u)
    {
      const ErrorCode RESULT = riscv_target_.DmiNop(DMI_ADDR, out_data, ack);
      if (RESULT != ErrorCode::OK)
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

    return BuildDmiResponse(DMI_ADDR, out_data, out_op, resp, cap, out_len);
  }

  ErrorCode HandleProgramCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                 uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t SUB_CMD = payload[0];
    if (SUB_CMD == 0x01u)
    {
      ExitProgramStream();
      ExitReadStream();
      EndTargetDebugSession();
      flash_erase_requested_ = true;
    }
    else if (SUB_CMD == 0x06u)
    {
      ExitProgramStream();
      ExitReadStream();
      EndTargetDebugSession();
    }
    else if (SUB_CMD == 0x05u)
    {
      EnterFlashOpStream();
    }
    else if (SUB_CMD == 0x07u || SUB_CMD == 0x0Bu)
    {
      // Protocol sequence from analysis:
      // 0x05 -> EP2 flash-op bytes -> 0x07/0x0B.
      if (program_mode_ != ProgramMode::WRITE_FLASH_OP || flash_op_rx_bytes_ == 0u)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      flash_op_ready_ = true;
      program_mode_ = ProgramMode::IDLE;
      if (!EnterFlashWriteStream())
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
    }
    else if (SUB_CMD == 0x02u || SUB_CMD == 0x04u)
    {
      if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM)
      {
        return BuildStandardResponse(0x02u, &SUB_CMD, 1u, resp, cap, out_len);
      }
      if (program_mode_ != ProgramMode::IDLE)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      if (!EnterFlashWriteStream())
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
    }
    else if (SUB_CMD == 0x08u)
    {
      FlushPendingDataAck();
      DrainFlashWriteStage();
      // Program End must arrive after the full write-region payload is streamed.
      if (program_mode_ != ProgramMode::WRITE_FLASH_STREAM || !IsFlashWriteFinished() ||
          flash_stream_error_)
      {
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      pending_data_ack_ = 0u;
      data_ack_in_flight_ = false;
      ExitProgramStream();
      flash_erase_requested_ = false;
    }
    else if (SUB_CMD == 0x0Cu)
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
    else if (SUB_CMD == 0x09u || SUB_CMD == 0x01u)
    {
      flash_erase_requested_ = false;
      ExitProgramStream();
      ExitReadStream();
      EndTargetDebugSession();
    }

    return BuildStandardResponse(0x02u, &SUB_CMD, 1u, resp, cap, out_len);
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

    const uint8_t COMMAND = req[1];
    const uint8_t PAYLOAD_LEN_U8 = req[2];
    const uint16_t PAYLOAD_LEN = PAYLOAD_LEN_U8;
    // Some host stacks/drivers may deliver a padded frame with trailing bytes.
    // Keep protocol parsing based on declared payload length and ignore tail.
    if (req_len < static_cast<uint16_t>(PAYLOAD_LEN + 3u))
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t* payload = req + 3u;

    if (session_state_ == SessionState::LINK_FAULT)
    {
      const bool ALLOW_CONTROL_RECOVERY =
          COMMAND == 0x0Du && PAYLOAD_LEN >= 1u &&
          (payload[0] == 0x01u || payload[0] == 0x02u || payload[0] == 0xFFu);
      const bool ALLOW_SET_SPEED = (COMMAND == 0x0Cu);
      // Some host flows issue program-plane cleanup before reattach.
      const bool ALLOW_PROGRAM_CLEANUP =
          COMMAND == 0x02u && PAYLOAD_LEN >= 1u &&
          (payload[0] == 0x09u || payload[0] == 0x01u);
      const bool ALLOW_BRINGUP_DIAG = (COMMAND == 0x11u);
      if (!ALLOW_CONTROL_RECOVERY && !ALLOW_SET_SPEED && !ALLOW_PROGRAM_CLEANUP &&
          !ALLOW_BRINGUP_DIAG)
      {
        if (COMMAND == 0x08u)
        {
          return BuildDmiNotAttachedResponse(resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
    }

    switch (COMMAND)
    {
      case 0x0Du:
        return HandleControlCommand(payload, PAYLOAD_LEN, resp, cap, out_len);
      case 0x0Cu:
      {
        if (!payload || PAYLOAD_LEN < 2u)
        {
          return BuildErrorResponse(0x55u, resp, cap, out_len);
        }
        const uint8_t REQUESTED_CHIP_FAMILY = payload[0];
        const uint32_t CLOCK_HZ = DecodeSdiClockHz(payload[1]);
        const bool CLOCK_SET_OK = (sdi_.SetClockHz(CLOCK_HZ) == ErrorCode::OK);
        if (CLOCK_SET_OK)
        {
          current_sdi_clock_hz_ = CLOCK_HZ;
          target_state_.requested_family = REQUESTED_CHIP_FAMILY;
          RefreshCompatDescriptorFromState();
        }
        const uint8_t STATUS_PAYLOAD[1] = {
            static_cast<uint8_t>(CLOCK_SET_OK ? 0x01u : 0x00u)};
        return BuildStandardResponse(
            0x0Cu, STATUS_PAYLOAD, sizeof(STATUS_PAYLOAD), resp, cap, out_len);
      }
      case 0x11u:
      {
        if (PAYLOAD_LEN == 1u && payload[0] == 0x06u)
        {
          return BuildEsigV2Response(resp, cap, out_len);
        }
        if (PAYLOAD_LEN == 1u && payload[0] == 0x84u)
        {
          std::array<uint8_t, 17u> debug_payload = {};
          debug_payload[0] = 0x84u;
          debug_payload[1] = debug_attach_snapshot_.failure_stage;
          debug_payload[2] = debug_attach_snapshot_.attached;
          debug_payload[3] = debug_attach_snapshot_.reserved0;
          debug_payload[4] = debug_attach_snapshot_.reserved1;
          StoreBe32(debug_payload.data() + 5u, debug_attach_snapshot_.dmstatus);
          StoreBe32(debug_payload.data() + 9u, debug_attach_snapshot_.abstractcs);
          StoreBe32(debug_payload.data() + 13u, debug_attach_snapshot_.chip_id);
          return BuildStandardResponse(
              0x11u, debug_payload.data(), static_cast<uint16_t>(debug_payload.size()), resp, cap,
              out_len);
        }
        if (PAYLOAD_LEN == 1u && payload[0] == 0x85u)
        {
          const auto& transfer_snapshot = SdiPort::debug_snapshot_;
          std::array<uint8_t, 33u> debug_payload = {};
          debug_payload[0] = 0x85u;
          debug_payload[1] = transfer_snapshot.stage;
          debug_payload[2] = static_cast<uint8_t>(transfer_snapshot.last_ec);
          debug_payload[3] = transfer_snapshot.last_req_addr;
          debug_payload[4] = transfer_snapshot.last_req_tail;
          debug_payload[5] = transfer_snapshot.last_resp_addr;
          debug_payload[6] = transfer_snapshot.last_resp_tail;
          debug_payload[7] = transfer_snapshot.last_parity_rx;
          debug_payload[8] = transfer_snapshot.last_parity_calc;
          StoreBe32(debug_payload.data() + 9u, transfer_snapshot.last_req_data);
          StoreBe32(debug_payload.data() + 13u, transfer_snapshot.last_resp_data);
          StoreBe32(debug_payload.data() + 17u, transfer_snapshot.last_tx_lo);
          StoreBe32(debug_payload.data() + 21u, transfer_snapshot.last_tx_hi);
          StoreBe32(debug_payload.data() + 25u, transfer_snapshot.last_rx_lo);
          StoreBe32(debug_payload.data() + 29u, transfer_snapshot.last_rx_hi);
          return BuildStandardResponse(
              0x11u, debug_payload.data(), static_cast<uint16_t>(debug_payload.size()), resp, cap,
              out_len);
        }
        if (PAYLOAD_LEN == 1u && payload[0] == 0x86u)
        {
          std::array<uint8_t, 37u> debug_payload = {};
          debug_payload[0] = 0x86u;
          debug_payload[1] = debug_flash_loader_snapshot_.failure_stage;
          debug_payload[2] = debug_flash_loader_snapshot_.resume_ack_seen;
          debug_payload[3] = debug_flash_loader_snapshot_.hart_halted_seen;
          debug_payload[4] = debug_flash_loader_snapshot_.a0_valid;
          StoreBe32(debug_payload.data() + 5u, debug_flash_loader_snapshot_.dmstatus_before_resume);
          StoreBe32(debug_payload.data() + 9u, debug_flash_loader_snapshot_.dmstatus_after_resume);
          StoreBe32(debug_payload.data() + 13u, debug_flash_loader_snapshot_.dmstatus_after_halt);
          StoreBe32(debug_payload.data() + 17u,
                    debug_flash_loader_snapshot_.dmcontrol_after_resume);
          StoreBe32(debug_payload.data() + 21u, debug_flash_loader_snapshot_.a0_result);
          StoreBe32(debug_payload.data() + 25u, debug_flash_loader_snapshot_.dpc);
          StoreBe32(debug_payload.data() + 29u, debug_flash_loader_snapshot_.mepc);
          StoreBe32(debug_payload.data() + 33u, debug_flash_loader_snapshot_.mcause);
          return BuildStandardResponse(
              0x11u, debug_payload.data(), static_cast<uint16_t>(debug_payload.size()), resp, cap,
              out_len);
        }
        if (PAYLOAD_LEN == 3u && payload[0] == 0x80u)
        {
          typename SdiPort::RawReadCapture capture = {};
          const ErrorCode RESULT = sdi_.DebugCaptureReadBits(payload[1], payload[2], capture);
          const uint8_t RAW_BYTES = static_cast<uint8_t>((capture.bit_count + 7u) / 8u);
          std::array<uint8_t, 16u> debug_payload = {};
          debug_payload[0] = 0x80u;
          debug_payload[1] = capture.dmi_addr;
          debug_payload[2] = capture.bit_count;
          debug_payload[3] = static_cast<uint8_t>(capture.last_ec);
          if (RAW_BYTES > 0u)
          {
            std::memcpy(debug_payload.data() + 4u, capture.raw_bits.data(), RAW_BYTES);
          }
          (void)RESULT;
          return BuildStandardResponse(
              0x11u, debug_payload.data(), static_cast<uint16_t>(RAW_BYTES + 4u), resp, cap,
              out_len);
        }
        if (PAYLOAD_LEN == 7u && payload[0] == 0x81u)
        {
          typename SdiPort::SwdTransferCapture capture = {};
          const uint32_t WDATA = LoadBe32(payload + 3u);
          const ErrorCode RESULT = sdi_.DebugSwdTransfer(payload[1], payload[2], WDATA, capture);
          std::array<uint8_t, 12u> debug_payload = {};
          debug_payload[0] = 0x81u;
          debug_payload[1] = capture.flags;
          debug_payload[2] = capture.addr2b;
          debug_payload[3] = static_cast<uint8_t>(capture.last_ec);
          debug_payload[4] = capture.ack;
          debug_payload[5] = capture.parity_ok;
          StoreBe32(debug_payload.data() + 6u, capture.rdata);
          (void)RESULT;
          return BuildStandardResponse(
              0x11u, debug_payload.data(), 10u, resp, cap, out_len);
        }
        if (PAYLOAD_LEN == 4u && payload[0] == 0x82u)
        {
          typename SdiPort::SwdRawCapture capture = {};
          const ErrorCode RESULT = sdi_.DebugCaptureSwdBits(payload[1], payload[2], payload[3],
                                                            capture);
          const uint8_t RAW_BYTES = static_cast<uint8_t>((capture.bit_count + 7u) / 8u);
          std::array<uint8_t, 20u> debug_payload = {};
          debug_payload[0] = 0x82u;
          debug_payload[1] = capture.flags;
          debug_payload[2] = capture.addr2b;
          debug_payload[3] = capture.bit_count;
          debug_payload[4] = static_cast<uint8_t>(capture.last_ec);
          if (RAW_BYTES > 0u)
          {
            std::memcpy(debug_payload.data() + 5u, capture.raw_bits.data(), RAW_BYTES);
          }
          (void)RESULT;
          return BuildStandardResponse(
              0x11u, debug_payload.data(), static_cast<uint16_t>(RAW_BYTES + 5u), resp, cap,
              out_len);
        }
        if (PAYLOAD_LEN >= 3u && payload[0] == 0x83u)
        {
          const uint8_t BATCH_FLAGS = payload[1];
          const uint8_t COUNT = payload[2];
          if (COUNT == 0u || COUNT > 8u ||
              PAYLOAD_LEN != static_cast<uint16_t>(3u + static_cast<uint16_t>(COUNT) * 6u))
          {
            return BuildErrorResponse(0x55u, resp, cap, out_len);
          }

          std::array<typename SdiPort::SwdBatchOp, 8u> ops = {};
          const uint8_t* cur = payload + 3u;
          for (uint8_t i = 0u; i < COUNT; ++i)
          {
            ops[i].flags = cur[0];
            ops[i].addr2b = cur[1];
            ops[i].wdata = LoadBe32(cur + 2u);
            cur += 6u;
          }

          typename SdiPort::SwdBatchCapture capture = {};
          const ErrorCode RESULT = sdi_.DebugSwdBatch(BATCH_FLAGS, ops.data(), COUNT, capture);
          std::array<uint8_t, 75u> debug_payload = {};
          debug_payload[0] = 0x83u;
          debug_payload[1] = capture.batch_flags;
          debug_payload[2] = capture.count;
          uint8_t* out = debug_payload.data() + 3u;
          for (uint8_t i = 0u; i < capture.count; ++i)
          {
            out[0] = capture.results[i].flags;
            out[1] = capture.results[i].addr2b;
            out[2] = static_cast<uint8_t>(capture.results[i].last_ec);
            out[3] = capture.results[i].ack;
            out[4] = capture.results[i].parity_ok;
            StoreBe32(out + 5u, capture.results[i].rdata);
            out += 9u;
          }
          (void)RESULT;
          return BuildStandardResponse(
              0x11u, debug_payload.data(),
              static_cast<uint16_t>(3u + static_cast<uint16_t>(capture.count) * 9u), resp, cap,
              out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      case 0x06u:
        return HandleConfigChipCommand(payload, PAYLOAD_LEN, resp, cap, out_len);
      case 0x08u:
        return HandleDmiCommand(payload, PAYLOAD_LEN, resp, cap, out_len);
      case 0x01u:
      {
        if (PAYLOAD_LEN == 8u)
        {
          write_region_addr_ = LoadBe32(payload);
          write_region_len_ = LoadBe32(payload + 4u);
          return BuildStandardResponse(0x01u, nullptr, 0u, resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      case 0x03u:
      {
        if (PAYLOAD_LEN == 8u)
        {
          read_region_addr_ = LoadBe32(payload);
          read_region_len_ = LoadBe32(payload + 4u);
          return BuildStandardResponse(0x03u, nullptr, 0u, resp, cap, out_len);
        }
        return BuildErrorResponse(0x55u, resp, cap, out_len);
      }
      case 0x02u:
        return HandleProgramCommand(payload, PAYLOAD_LEN, resp, cap, out_len);
      case 0x0Bu:
        return HandleResetCommand(payload, PAYLOAD_LEN, resp, cap, out_len);
      case 0x0Eu:
        return HandleDisableDebugCommand(payload, PAYLOAD_LEN, resp, cap, out_len);
      case 0xFFu:
        return BuildStandardResponse(COMMAND, nullptr, 0u, resp, cap, out_len);
      default:
        return BuildErrorResponse(0x55u, resp, cap, out_len);
    }
  }

  void ResetCommandResponseQueue()
  {
    pending_cmd_resp_valid_ = false;
    pending_cmd_resp_len_ = 0u;
  }

  bool ProcessPendingCommandOut()
  {
    if (!pending_cmd_out_valid_)
    {
      return false;
    }

    const uint16_t REQ_LEN = pending_cmd_out_len_;
    pending_cmd_out_len_ = 0u;
    pending_cmd_out_valid_ = false;

    std::array<uint8_t, CMD_PACKET_SIZE> resp_shadow = {};
    uint16_t out_len = 0u;
    (void)HandleCommand(pending_cmd_out_.data(), REQ_LEN, resp_shadow.data(),
                        static_cast<uint16_t>(resp_shadow.size()), out_len);
    QueueCommandResponse(resp_shadow.data(), out_len);
    return true;
  }

  bool ProcessPendingDataOut()
  {
    if (flash_stream_stage_size_ == 0u)
    {
      return false;
    }

    if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM)
    {
      const uint16_t PROCESS_SLICE = flash_program_page_size_ != 0u ? flash_program_page_size_ : 256u;
      const uint16_t HEAD = flash_stream_stage_head_;
      const uint16_t QUEUED = flash_stream_stage_size_;
      const uint16_t CHUNK_BYTES = static_cast<uint16_t>(
          MinU32(MinU32(QUEUED, static_cast<uint32_t>(PROCESS_SLICE)),
                 static_cast<uint32_t>(flash_stream_stage_.size() - HEAD)));
      ProcessFlashStreamData(flash_stream_stage_.data() + HEAD, CHUNK_BYTES);
      flash_stream_stage_head_ = static_cast<uint16_t>(
          (static_cast<uint32_t>(HEAD) + CHUNK_BYTES) % flash_stream_stage_.size());
      const uint16_t UPDATED_QUEUED = flash_stream_stage_size_;
      flash_stream_stage_size_ =
          (UPDATED_QUEUED > CHUNK_BYTES) ? static_cast<uint16_t>(UPDATED_QUEUED - CHUNK_BYTES) : 0u;
      return true;
    }
    return false;
  }

  bool DrainFlashWriteStage()
  {
    if (program_mode_ != ProgramMode::WRITE_FLASH_STREAM)
    {
      return false;
    }

    bool drained = false;
    while (flash_stream_stage_size_ != 0u && !flash_stream_error_)
    {
      const uint16_t QUEUED_BEFORE = flash_stream_stage_size_;
      if (!ProcessPendingDataOut())
      {
        break;
      }
      drained = true;
      if (flash_stream_stage_size_ >= QUEUED_BEFORE)
      {
        break;
      }
    }
    return drained;
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
    if (!tx.addr_ || tx.size_ < DATA_ACK_PREFIX.size() + 1u)
    {
      return false;
    }

    std::memcpy(tx.addr_, DATA_ACK_PREFIX.data(), DATA_ACK_PREFIX.size());
    static_cast<uint8_t*>(tx.addr_)[DATA_ACK_PREFIX.size()] = flash_stream_ack_code_;
    if (ep_data_in_->Transfer(static_cast<uint16_t>(DATA_ACK_PREFIX.size() + 1u)) != ErrorCode::OK)
    {
      return false;
    }
    if (program_mode_ == ProgramMode::WRITE_FLASH_STREAM && IsFlashStreamSuccessAck())
    {
      flash_stream_acked_bytes_ += CurrentFlashAckBytes();
    }
    data_ack_in_flight_ = true;
    return true;
  }

  bool ArmCommandOutIfIdle()
  {
    if (!ep_cmd_out_ || !ep_cmd_in_)
    {
      return false;
    }
    if (ep_cmd_out_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }
    // Keep strict command request/response pairing on command plane.
    if (pending_cmd_resp_valid_ || ep_cmd_in_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }
    const uint16_t RX_LEN = ep_cmd_out_->MaxPacketSize();
    return ep_cmd_out_->Transfer(RX_LEN == 0u ? CMD_PACKET_SIZE : RX_LEN) == ErrorCode::OK;
  }

  bool ArmDataOutIfIdle()
  {
    if (!ep_data_out_)
    {
      return false;
    }
    if (ep_data_out_->GetState() != Endpoint::State::IDLE)
    {
      return false;
    }
    const uint16_t RX_LEN = ep_data_out_->MaxPacketSize();
    return ep_data_out_->Transfer(RX_LEN == 0u ? DATA_PACKET_SIZE : RX_LEN) == ErrorCode::OK;
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

    const auto EP_BUFFER = ep->GetBuffer();
    if (EP_BUFFER.size_ >= BULK_MPS_HS)
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
  static constexpr std::array<uint8_t, 3> DATA_ACK_PREFIX = {0x41u, 0x01u, 0x01u};
  static constexpr uint8_t DATA_ACK_CODE_ERASE = 0x02u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM = 0x04u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_LOADER_REG_FAIL_BASE = 0xA0u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_LOADER_RUN_FAIL_BASE = 0xB0u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_LOADER_RESULT_FAIL_BASE = 0xC0u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_PAGE_BUFFER_FAIL_BASE = 0xD0u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_VERIFY_SUM_FAIL_BASE = 0xE0u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_WRITE_FAIL = 0xA4u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_FINALIZE_FAIL = 0xA5u;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_STAGE_OVERFLOW = 0xAEu;
  static constexpr uint8_t DATA_ACK_CODE_STREAM_RX_OVERFLOW = 0xAFu;
  static constexpr uint32_t SDI_CLOCK_HZ_LOW = 125'000u;
  static constexpr uint32_t SDI_CLOCK_HZ_MEDIUM = 484'375u;
  static constexpr uint32_t SDI_CLOCK_HZ_HIGH = 500'000u;
  static constexpr uint8_t ATTACH_RETRY_COUNT = 10u;
  static constexpr uint32_t ATTACH_RETRY_DELAY_CYCLES = 60000u;
  static constexpr uint8_t ATTACH_DMCONTROL_ADDR = 0x10u;
  static constexpr uint8_t ATTACH_DMSTATUS_ADDR = 0x11u;
  static constexpr uint8_t ATTACH_ABSTRACTCS_ADDR = 0x16u;
  static constexpr uint16_t FLASH_OP_IMAGE_MAX_BYTES = 2048u;
  static constexpr uint16_t FLASH_PAGE_BUFFER_MAX_BYTES = 256u;
  static constexpr uint16_t kDebugRegSp = 0x1002u;
  static constexpr uint16_t kDebugRegA0 = 0x100Au;
  static constexpr uint16_t kDebugRegA1 = 0x100Bu;
  static constexpr uint16_t kDebugRegA2 = 0x100Cu;
  static constexpr uint8_t kFlashOpFlagUnlock = 0x01u;
  static constexpr uint8_t kFlashOpFlagMassErase = 0x02u;
  static constexpr uint8_t kFlashOpFlagPageErase = 0x04u;
  static constexpr uint8_t kFlashOpFlagProgram = 0x08u;
  static constexpr uint8_t kFlashOpFlagVerify = 0x10u;

#pragma pack(push, 1)
  struct WchLinkRvDescBlock
  {
    InterfaceDescriptor intf;
    EndpointDescriptor ep_data_in;
    EndpointDescriptor ep_data_out;
    EndpointDescriptor ep_cmd_in;
    EndpointDescriptor ep_cmd_out;
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
  bool dmi_needs_warmup_ = false;

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
  uint32_t current_sdi_clock_hz_ = SDI_CLOCK_HZ_HIGH;
  TargetCompatState target_state_ = {};
  ProgramMode program_mode_ = ProgramMode::IDLE;
  bool flash_op_ready_ = false;
  uint16_t flash_op_image_len_ = 0u;
  uint32_t flash_op_rx_bytes_ = 0u;
  bool flash_op_overflow_ = false;
  bool flash_op_prepared_ = false;
  FlashOpLayout flash_op_layout_ = {};
  uint16_t flash_program_page_size_ = FLASH_PAGE_BUFFER_MAX_BYTES;
  uint32_t flash_loader_fill_ = 0u;
  uint32_t flash_loader_buffer_bytes_ = 0u;
  uint32_t flash_loader_verify_sum_ = 0u;
  uint8_t flash_loader_verify_tail_fill_ = 0u;
  uint8_t flash_program_flags_ = 0u;
  uint32_t flash_stream_chunk_bytes_ = 4096u;
  uint32_t flash_stream_total_raw_bytes_ = 0u;
  uint32_t flash_stream_received_bytes_ = 0u;
  uint32_t flash_stream_rx_bytes_ = 0u;
  uint32_t flash_stream_acked_bytes_ = 0u;
  uint32_t flash_stream_pages_committed_ = 0u;
  uint32_t flash_stream_next_ack_at_ = 0u;
  uint32_t flash_stream_write_addr_ = 0u;
  bool flash_stream_error_ = false;
  uint8_t flash_stream_ack_code_ = DATA_ACK_CODE_STREAM;
  bool flash_erase_requested_ = false;
  bool flash_stream_writer_active_ = false;
  bool flash_stream_writer_supported_ = false;
  bool target_debug_session_active_ = false;
  bool target_debug_session_needs_resume_ = false;
  uint8_t pending_data_ack_ = 0u;
  bool data_ack_in_flight_ = false;
  bool read_stream_active_ = false;
  uint32_t read_stream_addr_ = 0u;
  uint32_t read_stream_remaining_ = 0u;
  bool read_stream_error_ = false;
  bool power_3v3_enabled_ = false;
  bool power_5v_enabled_ = false;
  uint8_t rstout_state_ = 0u;

  std::array<uint8_t, CMD_PACKET_SIZE> pending_cmd_out_ = {};
  volatile uint16_t pending_cmd_out_len_ = 0u;
  volatile bool pending_cmd_out_valid_ = false;
  std::array<uint8_t, FLASH_OP_IMAGE_MAX_BYTES> flash_op_image_ = {};
  std::array<uint8_t, FLASH_PAGE_BUFFER_MAX_BYTES> flash_page_buffer_ = {};
  std::array<uint8_t, 4u> flash_loader_verify_tail_ = {};
  std::array<uint8_t, 6144u> flash_stream_stage_ = {};
  volatile uint16_t flash_stream_stage_head_ = 0u;
  volatile uint16_t flash_stream_stage_size_ = 0u;
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
