#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "debug/swd.hpp"
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
 * Current goal is protocol bring-up for host-side command sequencing.
 * SWD execution paths are intentionally simplified and mapped to a small
 * emulated DM register model so the link layer can be validated first.
 */
template <typename SwdPort>
class WchLinkRvClass : public DeviceClass
{
 public:
  struct ProbeIdentity
  {
    uint8_t major = 0x02;           ///< Firmware major reported by 0x0d/0x01.
    uint8_t minor = 0x12;           ///< Firmware minor reported by 0x0d/0x01.
    uint8_t variant = 0x12;         ///< Link variant reported by 0x0d/0x01.
    uint8_t chip_family = 0x0D;     ///< CH32X035 family tag in 0x0d/0x02.
    uint32_t chip_id = 0x03510611;  ///< CH32X035C8T6 example chip-id.
  };

  explicit WchLinkRvClass(
      SwdPort& swd_link, LibXR::GPIO* nreset_gpio = nullptr,
      Endpoint::EPNumber command_ep_num = Endpoint::EPNumber::EP1,
      Endpoint::EPNumber data_ep_num = Endpoint::EPNumber::EP2)
      : swd_(swd_link),
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
   * @brief For future hardware mode, expose direct SWD access.
   */
  SwdPort& SwdLink() { return swd_; }

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
    ResetRuntimeModel();

    inited_ = true;
    ArmCommandOutIfIdle();
    ArmDataOutIfIdle();
  }

  void UnbindEndpoints(EndpointPool& endpoint_pool, bool) override
  {
    inited_ = false;
    pending_cmd_valid_ = false;

    ReleaseEndpoint(endpoint_pool, ep_cmd_in_);
    ReleaseEndpoint(endpoint_pool, ep_cmd_out_);
    ReleaseEndpoint(endpoint_pool, ep_data_in_);
    ReleaseEndpoint(endpoint_pool, ep_data_out_);

    swd_.Close();
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
    uint16_t out_len = 0u;
    auto tx = ep_cmd_in_ ? ep_cmd_in_->GetBuffer() : RawData{nullptr, 0};
    if (!tx.addr_ || tx.size_ == 0u)
    {
      ArmCommandOutIfIdle();
      return;
    }

    auto* resp = static_cast<uint8_t*>(tx.addr_);
    const auto* req = static_cast<const uint8_t*>(data.addr_);
    const uint16_t req_len = static_cast<uint16_t>(data.size_);

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

    ArmCommandOutIfIdle();
  }

  void OnCommandIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    if (!pending_cmd_valid_)
    {
      return;
    }

    auto tx = ep_cmd_in_ ? ep_cmd_in_->GetBuffer() : RawData{nullptr, 0};
    if (!tx.addr_ || tx.size_ < pending_cmd_len_)
    {
      pending_cmd_valid_ = false;
      pending_cmd_len_ = 0u;
      return;
    }

    std::memcpy(tx.addr_, pending_cmd_buf_.data(), pending_cmd_len_);
    if (TryStartCommandIn(pending_cmd_len_))
    {
      pending_cmd_valid_ = false;
      pending_cmd_len_ = 0u;
    }
  }

  void OnDataOut(bool /*in_isr*/, LibXR::ConstRawData& /*data*/)
  {
    if (flash_write_stream_enabled_)
    {
      (void)TrySendDataAck();
    }
    ArmDataOutIfIdle();
  }

  void OnDataIn(bool /*in_isr*/, LibXR::ConstRawData& /*data*/) {}

  void ResetRuntimeModel()
  {
    attached_ = false;
    flash_write_stream_enabled_ = false;
    write_region_addr_ = 0u;
    write_region_len_ = 0u;
    read_region_addr_ = 0u;
    read_region_len_ = 0u;
    reg_dpc_ = 0u;
    gpr_.fill(0u);
    dmi_regs_.fill(0u);

    dmi_regs_[0x10] = 0x80000001u;  // dmcontrol
    dmi_regs_[0x11] = 0x00000382u;  // dmstatus (halted/auth/version=2)
    dmi_regs_[0x12] = 0x00312380u;  // hartinfo
    dmi_regs_[0x16] = 0x08000002u;  // abstractcs
    dmi_regs_[0x40] = 0x00000001u;  // haltsum0
    dmi_regs_[0x04] = 0x00000000u;  // data0
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
      attached_ = true;
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
      flash_write_stream_enabled_ = false;
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

  void HandleAbstractCommandWrite(uint32_t cmd)
  {
    const uint32_t op = cmd & 0xFFFF0000u;
    const uint16_t regno = static_cast<uint16_t>(cmd & 0x0000FFFFu);

    if (op == 0x00220000u)
    {
      dmi_regs_[0x04] = ReadPseudoRegister(regno);
      return;
    }

    if (op == 0x00230000u)
    {
      WritePseudoRegister(regno, dmi_regs_[0x04]);
    }
  }

  uint32_t ReadPseudoRegister(uint16_t regno) const
  {
    if (regno == 0x0301u)
    {
      return 0x40901105u;  // misa
    }
    if (regno == 0x0F12u)
    {
      return 0xDC68D883u;  // marchid
    }
    if (regno == 0x07B1u)
    {
      return reg_dpc_;
    }
    if (regno >= 0x1000u && regno < 0x1020u)
    {
      return gpr_[regno - 0x1000u];
    }
    return 0u;
  }

  void WritePseudoRegister(uint16_t regno, uint32_t value)
  {
    if (regno == 0x07B1u)
    {
      reg_dpc_ = value;
      return;
    }
    if (regno >= 0x1000u && regno < 0x1020u)
    {
      gpr_[regno - 0x1000u] = value;
    }
  }

  ErrorCode HandleDmiCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                             uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len != 6u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t addr = payload[0];
    const uint32_t data = LoadBe32(payload + 1u);
    const uint8_t op = payload[5];

    uint32_t out_data = dmi_regs_[addr];
    uint8_t out_op = 0x00u;

    if (op == 0x01u)
    {
      out_data = dmi_regs_[addr];
    }
    else if (op == 0x02u)
    {
      dmi_regs_[addr] = data;
      out_data = data;

      if (addr == 0x17u)
      {
        HandleAbstractCommandWrite(data);
      }
      else if (addr == 0x10u)
      {
        // Emulate halt/resume bits transition for host polling.
        if ((data & 0x80000000u) != 0u)
        {
          dmi_regs_[0x11] |= (1u << 9u) | (1u << 8u);
          dmi_regs_[0x11] &= ~((1u << 11u) | (1u << 10u));
        }
        if ((data & 0x40000000u) != 0u)
        {
          dmi_regs_[0x11] |= (1u << 11u) | (1u << 10u);
          dmi_regs_[0x11] &= ~((1u << 9u) | (1u << 8u));
        }
      }
    }
    else if (op == 0x00u)
    {
      out_data = dmi_regs_[addr];
    }
    else
    {
      out_op = 0x02u;  // failed
    }

    uint8_t dmi_resp[6] = {};
    dmi_resp[0] = addr;
    StoreBe32(dmi_resp + 1u, out_data);
    dmi_resp[5] = out_op;
    return BuildStandardResponse(0x08u, dmi_resp, sizeof(dmi_resp), resp, cap, out_len);
  }

  ErrorCode HandleProgramCommand(const uint8_t* payload, uint16_t payload_len, uint8_t* resp,
                                 uint16_t cap, uint16_t& out_len)
  {
    if (!payload || payload_len < 1u)
    {
      return BuildErrorResponse(0x55u, resp, cap, out_len);
    }

    const uint8_t sub = payload[0];
    if (sub == 0x02u)
    {
      flash_write_stream_enabled_ = true;
    }
    else if (sub == 0x08u || sub == 0x09u)
    {
      flash_write_stream_enabled_ = false;
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
    if (req_len != static_cast<uint16_t>(payload_len + 3u))
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
        const uint8_t ok[1] = {0x01u};
        return BuildStandardResponse(0x0Cu, ok, sizeof(ok), resp, cap, out_len);
      }
      case 0x11u:
      {
        // WCH V2 ESIG raw response (non 0x82-framed format).
        static constexpr uint8_t kEsigRaw[] = {
            0xFF, 0xFF, 0x00, 0x3E, 0x2E, 0x86, 0xAB, 0xCD, 0x96, 0x76,
            0xBC, 0x23, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x51, 0x06, 0x11};
        if (payload_len == 1u && payload[0] == 0x06u && cap >= sizeof(kEsigRaw))
        {
          std::memcpy(resp, kEsigRaw, sizeof(kEsigRaw));
          out_len = sizeof(kEsigRaw);
          return ErrorCode::OK;
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

  SwdPort& swd_;
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
  bool flash_write_stream_enabled_ = false;

  uint32_t write_region_addr_ = 0u;
  uint32_t write_region_len_ = 0u;
  uint32_t read_region_addr_ = 0u;
  uint32_t read_region_len_ = 0u;

  uint32_t reg_dpc_ = 0u;
  std::array<uint32_t, 32> gpr_ = {};
  std::array<uint32_t, 256> dmi_regs_ = {};

  std::array<uint8_t, 96> pending_cmd_buf_ = {};
  uint16_t pending_cmd_len_ = 0u;
  bool pending_cmd_valid_ = false;

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

