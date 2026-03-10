#include <cstdint>

#include "ch32_usb_endpoint.hpp"
#include "ep.hpp"

using namespace LibXR;
using namespace LibXR::USB;

#if defined(USBFSD)

#if defined(__CH32X035_H)
namespace
{

static volatile uint16_t* get_ctrl_addr(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP0:
      return &USBFSD->UEP0_CTRL_H;
    case Endpoint::EPNumber::EP1:
      return &USBFSD->UEP1_CTRL_H;
    case Endpoint::EPNumber::EP2:
      return &USBFSD->UEP2_CTRL_H;
    case Endpoint::EPNumber::EP3:
      return &USBFSD->UEP3_CTRL_H;
    case Endpoint::EPNumber::EP4:
      return &USBFSD->UEP4_CTRL_H;
    case Endpoint::EPNumber::EP5:
      return &USBFSD->UEP5_CTRL_H;
    case Endpoint::EPNumber::EP6:
      return &USBFSD->UEP6_CTRL_H;
    case Endpoint::EPNumber::EP7:
      return &USBFSD->UEP7_CTRL_H;
    default:
      return &USBFSD->UEP0_CTRL_H;
  }
}

static volatile uint16_t* get_tx_len_addr(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP0:
      return &USBFSD->UEP0_TX_LEN;
    case Endpoint::EPNumber::EP1:
      return &USBFSD->UEP1_TX_LEN;
    case Endpoint::EPNumber::EP2:
      return &USBFSD->UEP2_TX_LEN;
    case Endpoint::EPNumber::EP3:
      return &USBFSD->UEP3_TX_LEN;
    case Endpoint::EPNumber::EP4:
      return &USBFSD->UEP4_TX_LEN;
    case Endpoint::EPNumber::EP5:
      return &USBFSD->UEP5_TX_LEN;
    case Endpoint::EPNumber::EP6:
      return &USBFSD->UEP6_TX_LEN;
    case Endpoint::EPNumber::EP7:
      return &USBFSD->UEP7_TX_LEN;
    default:
      return &USBFSD->UEP0_TX_LEN;
  }
}

static volatile uint32_t* get_dma_addr(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP0:
      return &USBFSD->UEP0_DMA;
    case Endpoint::EPNumber::EP1:
      return &USBFSD->UEP1_DMA;
    case Endpoint::EPNumber::EP2:
      return &USBFSD->UEP2_DMA;
    case Endpoint::EPNumber::EP3:
      return &USBFSD->UEP3_DMA;
    case Endpoint::EPNumber::EP4:
      return &USBFSD->UEP0_DMA;
    case Endpoint::EPNumber::EP5:
      return &USBFSD->UEP5_DMA;
    case Endpoint::EPNumber::EP6:
      return &USBFSD->UEP6_DMA;
    case Endpoint::EPNumber::EP7:
      return &USBFSD->UEP7_DMA;
    default:
      return &USBFSD->UEP0_DMA;
  }
}

static volatile uint8_t* get_mod_addr(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP1:
    case Endpoint::EPNumber::EP4:
      return &USBFSD->UEP4_1_MOD;
    case Endpoint::EPNumber::EP2:
    case Endpoint::EPNumber::EP3:
      return &USBFSD->UEP2_3_MOD;
    case Endpoint::EPNumber::EP5:
    case Endpoint::EPNumber::EP6:
    case Endpoint::EPNumber::EP7:
      return &USBFSD->UEP567_MOD;
    default:
      return nullptr;
  }
}

static uint8_t get_tx_enable_mask(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP1:
      return USBFS_UEP1_TX_EN;
    case Endpoint::EPNumber::EP2:
      return USBFS_UEP2_TX_EN;
    case Endpoint::EPNumber::EP3:
      return USBFS_UEP3_TX_EN;
    case Endpoint::EPNumber::EP4:
      return USBFS_UEP4_TX_EN;
    case Endpoint::EPNumber::EP5:
      return USBFS_UEP5_TX_EN;
    case Endpoint::EPNumber::EP6:
      return USBFS_UEP6_TX_EN;
    case Endpoint::EPNumber::EP7:
      return USBFS_UEP7_TX_EN;
    default:
      return 0;
  }
}

static uint8_t get_rx_enable_mask(Endpoint::EPNumber ep)
{
  switch (ep)
  {
    case Endpoint::EPNumber::EP1:
      return USBFS_UEP1_RX_EN;
    case Endpoint::EPNumber::EP2:
      return USBFS_UEP2_RX_EN;
    case Endpoint::EPNumber::EP3:
      return USBFS_UEP3_RX_EN;
    case Endpoint::EPNumber::EP4:
      return USBFS_UEP4_RX_EN;
    case Endpoint::EPNumber::EP5:
      return USBFS_UEP5_RX_EN;
    case Endpoint::EPNumber::EP6:
      return USBFS_UEP6_RX_EN;
    case Endpoint::EPNumber::EP7:
      return USBFS_UEP7_RX_EN;
    default:
      return 0;
  }
}

static uint8_t get_buf_mod_mask(Endpoint::EPNumber ep)
{
  switch (ep)
  {
#if defined(USBFS_UEP1_BUF_MOD)
    case Endpoint::EPNumber::EP1:
      return USBFS_UEP1_BUF_MOD;
#endif
#if defined(USBFS_UEP2_BUF_MOD)
    case Endpoint::EPNumber::EP2:
      return USBFS_UEP2_BUF_MOD;
#endif
#if defined(USBFS_UEP3_BUF_MOD)
    case Endpoint::EPNumber::EP3:
      return USBFS_UEP3_BUF_MOD;
#endif
#if defined(USBFS_UEP4_BUF_MOD)
    case Endpoint::EPNumber::EP4:
      return USBFS_UEP4_BUF_MOD;
#endif
#if defined(USBFS_UEP5_BUF_MOD)
    case Endpoint::EPNumber::EP5:
      return USBFS_UEP5_BUF_MOD;
#endif
#if defined(USBFS_UEP6_BUF_MOD)
    case Endpoint::EPNumber::EP6:
      return USBFS_UEP6_BUF_MOD;
#endif
#if defined(USBFS_UEP7_BUF_MOD)
    case Endpoint::EPNumber::EP7:
      return USBFS_UEP7_BUF_MOD;
#endif
    default:
      return 0;
  }
}

static void set_ctrl_bits(Endpoint::EPNumber ep, uint16_t mask, uint16_t value)
{
  auto* ctrl = get_ctrl_addr(ep);
  *ctrl = static_cast<uint16_t>((*ctrl & ~mask) | value);
}

static void set_tx_response(Endpoint::EPNumber ep, uint16_t response)
{
  set_ctrl_bits(ep, USBFS_UEP_T_RES_MASK, response);
}

static void set_rx_response(Endpoint::EPNumber ep, uint16_t response)
{
  set_ctrl_bits(ep, USBFS_UEP_R_RES_MASK, response);
}

static void set_tx_toggle(Endpoint::EPNumber ep, bool data1)
{
  set_ctrl_bits(ep, USBFS_UEP_T_TOG, data1 ? USBFS_UEP_T_TOG : 0);
}

static void set_rx_toggle(Endpoint::EPNumber ep, bool data1)
{
  set_ctrl_bits(ep, USBFS_UEP_R_TOG, data1 ? USBFS_UEP_R_TOG : 0);
}

static void set_tx_len(Endpoint::EPNumber ep, uint16_t value)
{
  *get_tx_len_addr(ep) = value;
}

static void set_dma_buffer(Endpoint::EPNumber ep, void* value, bool double_buffer)
{
  *get_dma_addr(ep) = reinterpret_cast<uint32_t>(value);

  auto* mod = get_mod_addr(ep);
  if (mod == nullptr)
  {
    return;
  }

  const uint8_t BUF_MASK = get_buf_mod_mask(ep);
  if (BUF_MASK == 0)
  {
    return;
  }

  if (double_buffer)
  {
    *mod |= BUF_MASK;
  }
  else
  {
    *mod &= static_cast<uint8_t>(~BUF_MASK);
  }
}

static void enable_tx(Endpoint::EPNumber ep)
{
  auto* mod = get_mod_addr(ep);
  if (mod != nullptr)
  {
    *mod |= get_tx_enable_mask(ep);
  }
}

static void disable_tx(Endpoint::EPNumber ep)
{
  auto* mod = get_mod_addr(ep);
  if (mod != nullptr)
  {
    *mod &= static_cast<uint8_t>(~get_tx_enable_mask(ep));
  }
}

static void enable_rx(Endpoint::EPNumber ep)
{
  auto* mod = get_mod_addr(ep);
  if (mod != nullptr)
  {
    *mod |= get_rx_enable_mask(ep);
  }
}

static void disable_rx(Endpoint::EPNumber ep)
{
  auto* mod = get_mod_addr(ep);
  if (mod != nullptr)
  {
    *mod &= static_cast<uint8_t>(~get_rx_enable_mask(ep));
  }
}

static RawData select_buffer(Endpoint::EPNumber ep_num, Endpoint::Direction dir,
                             const RawData& buffer)
{
  if (ep_num == Endpoint::EPNumber::EP0 || buffer.size_ < 2)
  {
    return buffer;
  }

  const size_t HALF = buffer.size_ / 2;
  if (HALF == 0)
  {
    return buffer;
  }

  if (dir == Endpoint::Direction::OUT)
  {
    return RawData(buffer.addr_, HALF);
  }

  return RawData(reinterpret_cast<uint8_t*>(buffer.addr_) + HALF, HALF);
}

}  // namespace

CH32EndpointOtgFs::CH32EndpointOtgFs(EPNumber ep_num, Direction dir, RawData buffer,
                                     bool is_isochronous)
    : Endpoint(ep_num, dir, is_isochronous ? buffer : select_buffer(ep_num, dir, buffer)),
      is_isochronous_(is_isochronous),
      dma_buffer_(buffer)
{
  map_otg_fs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;
  set_dma_buffer(GetNumber(), dma_buffer_.addr_, false);

  if (dir == Direction::IN)
  {
    set_tx_len(GetNumber(), 0);
    set_tx_response(GetNumber(), USBFS_UEP_T_RES_NAK);
  }
  else
  {
    set_rx_response(GetNumber(), GetNumber() == EPNumber::EP0 ? USBFS_UEP_R_RES_ACK
                                                              : USBFS_UEP_R_RES_NAK);
  }
}

void CH32EndpointOtgFs::Configure(const Config& cfg)
{
  auto& ep_cfg = GetConfig();
  ep_cfg = cfg;

  const bool SINGLE_DIRECTION = is_isochronous_;
  ep_cfg.double_buffer = (GetNumber() != EPNumber::EP0) && !SINGLE_DIRECTION;

  const auto buffer = GetBuffer();
  size_t packet_size_limit = (cfg.type == Type::ISOCHRONOUS) ? 1023U : 64U;
  if (packet_size_limit > buffer.size_)
  {
    packet_size_limit = buffer.size_;
  }

  size_t max_packet_size = cfg.max_packet_size;
  if (max_packet_size > packet_size_limit)
  {
    max_packet_size = packet_size_limit;
  }
  if (max_packet_size < 8 && packet_size_limit >= 8)
  {
    max_packet_size = 8;
  }
  if (max_packet_size == 0)
  {
    max_packet_size = packet_size_limit;
  }
  ep_cfg.max_packet_size = static_cast<uint16_t>(max_packet_size);

  tog_ = false;
  set_tx_len(GetNumber(), 0);
  set_tx_toggle(GetNumber(), false);
  set_rx_toggle(GetNumber(), false);

  if (GetNumber() != EPNumber::EP0)
  {
    disable_tx(GetNumber());
    disable_rx(GetNumber());
  }

  if (!SINGLE_DIRECTION)
  {
    enable_tx(GetNumber());
    enable_rx(GetNumber());
    set_tx_response(GetNumber(), USBFS_UEP_T_RES_NAK);
    set_rx_response(GetNumber(), GetNumber() == EPNumber::EP0 ? USBFS_UEP_R_RES_ACK
                                                              : USBFS_UEP_R_RES_NAK);
  }
  else if (GetDirection() == Direction::IN)
  {
    enable_tx(GetNumber());
    set_tx_response(GetNumber(), USBFS_UEP_T_RES_NAK);
    if (GetNumber() == EPNumber::EP0)
    {
      set_rx_response(GetNumber(), USBFS_UEP_R_RES_ACK);
    }
  }
  else
  {
    enable_rx(GetNumber());
    set_rx_response(GetNumber(), GetNumber() == EPNumber::EP0 ? USBFS_UEP_R_RES_ACK
                                                              : USBFS_UEP_R_RES_NAK);
  }

  set_dma_buffer(GetNumber(), dma_buffer_.addr_, ep_cfg.double_buffer);
  SetState(State::IDLE);
}

void CH32EndpointOtgFs::Close()
{
  disable_tx(GetNumber());
  disable_rx(GetNumber());
  set_tx_response(GetNumber(), USBFS_UEP_T_RES_NAK);
  set_rx_response(GetNumber(), USBFS_UEP_R_RES_NAK);
  SetState(State::DISABLED);
}

ErrorCode CH32EndpointOtgFs::Transfer(size_t size)
{
  if (GetState() == State::BUSY)
  {
    return ErrorCode::BUSY;
  }

  auto buffer = GetBuffer();
  if (buffer.size_ < size)
  {
    return ErrorCode::NO_BUFF;
  }

  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  if (IS_IN && UseDoubleBuffer() && size > 0)
  {
    SwitchBuffer();
  }

  if (IS_IN)
  {
    set_tx_len(GetNumber(), static_cast<uint16_t>(size));
    set_tx_toggle(GetNumber(), tog_);
    set_tx_response(GetNumber(), IS_ISO ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK);
  }
  else
  {
    set_rx_toggle(GetNumber(), tog_);
    set_rx_response(GetNumber(), IS_ISO ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK);
  }

  if (GetNumber() == EPNumber::EP0)
  {
    tog_ = !tog_;
  }

  last_transfer_size_ = size;
  SetState(State::BUSY);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::Stall()
{
  if (GetState() != State::IDLE)
  {
    return ErrorCode::BUSY;
  }

  if (GetDirection() == Direction::IN)
  {
    set_tx_response(GetNumber(), USBFS_UEP_T_RES_STALL);
  }
  else
  {
    set_rx_response(GetNumber(), USBFS_UEP_R_RES_STALL);
  }

  SetState(State::STALLED);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::ClearStall()
{
  if (GetState() != State::STALLED)
  {
    return ErrorCode::FAILED;
  }

  if (GetDirection() == Direction::IN)
  {
    set_tx_response(GetNumber(), USBFS_UEP_T_RES_NAK);
  }
  else
  {
    set_rx_response(GetNumber(), GetNumber() == EPNumber::EP0 ? USBFS_UEP_R_RES_ACK
                                                              : USBFS_UEP_R_RES_NAK);
  }

  SetState(State::IDLE);
  return ErrorCode::OK;
}

void CH32EndpointOtgFs::TransferComplete(size_t size)
{
  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_EP0 = (GetNumber() == Endpoint::EPNumber::EP0);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  if (IS_IN)
  {
    set_tx_response(GetNumber(), USBFS_UEP_T_RES_NAK);
    size = last_transfer_size_;
  }
  else if (!IS_EP0)
  {
    set_rx_response(GetNumber(), USBFS_UEP_R_RES_NAK);
  }

  if (!IS_IN)
  {
    const bool TOG_OK = (USBFSD->INT_ST & USBFS_UIS_TOG_OK) == USBFS_UIS_TOG_OK;
    if (!TOG_OK)
    {
      SetState(State::IDLE);
      (void)Transfer(last_transfer_size_);
      return;
    }
  }

  if (GetState() == State::BUSY && !IS_EP0 && !IS_ISO)
  {
    tog_ = !tog_;
  }

  if (IS_EP0 && !IS_IN)
  {
    tog_ = true;
    set_rx_response(GetNumber(), USBFS_UEP_R_RES_ACK);
  }

  OnTransferCompleteCallback(true, size);
}

void CH32EndpointOtgFs::SwitchBuffer()
{
  if (GetDirection() == Direction::IN)
  {
    SetActiveBlock(!tog_);
  }
  else
  {
    SetActiveBlock(tog_);
  }
}
#else
// NOLINTBEGIN

static inline volatile uint8_t* get_tx_ctrl_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_TX_CTRL + 4 * (USB::Endpoint::EPNumberToInt8(ep));
}
static inline volatile uint8_t* get_rx_ctrl_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_RX_CTRL + 4 * (USB::Endpoint::EPNumberToInt8(ep));
}
static inline volatile uint16_t* get_tx_len_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_TX_LEN + 2 * (USB::Endpoint::EPNumberToInt8(ep));
}
static inline volatile uint32_t* get_dma_addr(USB::Endpoint::EPNumber ep)
{
  return &USBFSD->UEP0_DMA + USB::Endpoint::EPNumberToInt8(ep);
}

static void set_dma_buffer(USB::Endpoint::EPNumber ep_num, void* value,
                           bool double_buffer)
{
  *get_dma_addr(ep_num) = (uint32_t)value;

  if (!double_buffer)
  {
    return;
  }

  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD |= USBFS_UEP1_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD |= USBFS_UEP2_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD |= USBFS_UEP3_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD |= USBFS_UEP4_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD |= USBFS_UEP5_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD |= USBFS_UEP6_BUF_MOD;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD |= USBFS_UEP7_BUF_MOD;
      break;
    default:
      break;
  }
}

static void set_tx_len(USB::Endpoint::EPNumber ep_num, uint32_t value)
{
  *get_tx_len_addr(ep_num) = value;
}

static void enable_tx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD |= USBFS_UEP1_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD |= USBFS_UEP2_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD |= USBFS_UEP3_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD |= USBFS_UEP4_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD |= USBFS_UEP5_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD |= USBFS_UEP6_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD |= USBFS_UEP7_TX_EN;
      break;
    default:
      break;
  }
}
static void disable_tx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP1_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP2_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP3_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP4_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP5_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP6_TX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD &= ~USBFS_UEP7_TX_EN;
      break;
    default:
      break;
  }
}
static void enable_rx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD |= USBFS_UEP1_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD |= USBFS_UEP2_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD |= USBFS_UEP3_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD |= USBFS_UEP4_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD |= USBFS_UEP5_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD |= USBFS_UEP6_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD |= USBFS_UEP7_RX_EN;
      break;
    default:
      break;
  }
}
static void disable_rx(USB::Endpoint::EPNumber ep_num)
{
  switch (ep_num)
  {
    case USB::Endpoint::EPNumber::EP1:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP1_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP2:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP2_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP3:
      USBFSD->UEP2_3_MOD &= ~USBFS_UEP3_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP4:
      USBFSD->UEP4_1_MOD &= ~USBFS_UEP4_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP5:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP5_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP6:
      USBFSD->UEP5_6_MOD &= ~USBFS_UEP6_RX_EN;
      break;
    case USB::Endpoint::EPNumber::EP7:
      USBFSD->UEP7_MOD &= ~USBFS_UEP7_RX_EN;
      break;
    default:
      break;
  }
}
// NOLINTEND

// NOLINTNEXTLINE
static LibXR::RawData select_buffer(USB::Endpoint::EPNumber ep_num,
                                    USB::Endpoint::Direction dir,
                                    const LibXR::RawData& buffer)
{
  if (ep_num == USB::Endpoint::EPNumber::EP0)
  {
    return buffer;
  }
  else
  {
    if (dir == USB::Endpoint::Direction::OUT)
    {
      return LibXR::RawData(buffer.addr_, 128);
    }
    else
    {
      return LibXR::RawData(reinterpret_cast<uint8_t*>(buffer.addr_) + 128, 128);
    }
  }
}

CH32EndpointOtgFs::CH32EndpointOtgFs(EPNumber ep_num, Direction dir,
                                     LibXR::RawData buffer, bool is_isochronous)
    : Endpoint(ep_num, dir, is_isochronous ? buffer : select_buffer(ep_num, dir, buffer)),
      is_isochronous_(is_isochronous),
      dma_buffer_(buffer)
{
  map_otg_fs_[EPNumberToInt8(GetNumber())][static_cast<uint8_t>(dir)] = this;

  set_dma_buffer(GetNumber(), dma_buffer_.addr_, is_isochronous ? false : true);

  if (dir == Direction::IN)
  {
    set_tx_len(GetNumber(), 0);
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;
  }
}

void CH32EndpointOtgFs::Configure(const Config& cfg)
{
  auto& ep_cfg = GetConfig();
  ep_cfg = cfg;

  if (GetNumber() != EPNumber::EP0 && !is_isochronous_)
  {
    ep_cfg.double_buffer = true;
  }
  else
  {
    ep_cfg.double_buffer = false;
  }

  ep_cfg.max_packet_size = GetBuffer().size_;

  set_tx_len(GetNumber(), 0);

  if (!is_isochronous_)
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK | USBFS_UEP_R_AUTO_TOG;
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK | USBFS_UEP_T_AUTO_TOG;
    enable_tx(GetNumber());
    enable_rx(GetNumber());
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;
    *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
    if (GetDirection() == Direction::IN)
    {
      enable_tx(GetNumber());
    }
    else
    {
      enable_rx(GetNumber());
    }
  }

  set_dma_buffer(GetNumber(), dma_buffer_.addr_, is_isochronous_ ? false : true);

  SetState(State::IDLE);
}

void CH32EndpointOtgFs::Close()
{
  disable_tx(GetNumber());
  disable_rx(GetNumber());

  *get_tx_ctrl_addr(GetNumber()) = USBFS_UEP_T_RES_NAK;
  *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_NAK;

  SetState(State::DISABLED);
}

ErrorCode CH32EndpointOtgFs::Transfer(size_t size)
{
  if (GetState() == State::BUSY)
  {
    return ErrorCode::BUSY;
  }

  auto buffer = GetBuffer();
  if (buffer.size_ < size)
  {
    return ErrorCode::NO_BUFF;
  }

  bool is_in = (GetDirection() == Direction::IN);

  if (is_in && UseDoubleBuffer())
  {
    SwitchBuffer();
  }

  if (is_in)
  {
    set_tx_len(GetNumber(), size);
    auto addr = get_tx_ctrl_addr(GetNumber());

    if (GetNumber() != EPNumber::EP0)
    {
      *addr = (is_isochronous_ ? USBFS_UEP_T_RES_NONE : USBFS_UEP_T_RES_ACK) |
              (*addr & (~USBFS_UEP_T_RES_MASK));
    }
    else
    {
      *addr = USBFS_UEP_T_RES_ACK | (tog_ ? USBFS_UEP_T_TOG : 0);
    }
  }
  else
  {
    auto addr = get_rx_ctrl_addr(GetNumber());

    if (GetNumber() != EPNumber::EP0)
    {
      *addr = (is_isochronous_ ? USBFS_UEP_R_RES_NONE : USBFS_UEP_R_RES_ACK) |
              (*addr & (~USBFS_UEP_R_RES_MASK));
    }
    else
    {
      *addr = USBFS_UEP_R_RES_ACK | (tog_ ? USBFS_UEP_R_TOG : 0);
    }
  }

  if (GetNumber() == EPNumber::EP0)
  {
    tog_ = !tog_;
  }

  last_transfer_size_ = size;
  SetState(State::BUSY);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::Stall()
{
  if (GetState() != State::IDLE)
  {
    return ErrorCode::BUSY;
  }

  bool is_in = (GetDirection() == Direction::IN);
  if (is_in)
  {
    *get_tx_ctrl_addr(GetNumber()) |= USBFS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) |= USBFS_UEP_R_RES_STALL;
  }
  SetState(State::STALLED);
  return ErrorCode::OK;
}

ErrorCode CH32EndpointOtgFs::ClearStall()
{
  if (GetState() != State::STALLED)
  {
    return ErrorCode::FAILED;
  }

  bool is_in = (GetDirection() == Direction::IN);
  if (is_in)
  {
    *get_tx_ctrl_addr(GetNumber()) &= ~USBFS_UEP_T_RES_STALL;
  }
  else
  {
    *get_rx_ctrl_addr(GetNumber()) &= ~USBFS_UEP_R_RES_STALL;
  }
  SetState(State::IDLE);
  return ErrorCode::OK;
}

void CH32EndpointOtgFs::TransferComplete(size_t size)
{
  const bool IS_IN = (GetDirection() == Direction::IN);
  const bool IS_OUT = !IS_IN;
  const bool IS_EP0 = (GetNumber() == EPNumber::EP0);
  const bool IS_ISO = (GetType() == Type::ISOCHRONOUS);

  // UIF_TRANSFER/INT_FG are cleared by the IRQ handler after dispatch.

  if (IS_IN)
  {
    // Restore NAK on completion.
    *get_tx_ctrl_addr(GetNumber()) =
        (*get_tx_ctrl_addr(GetNumber()) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;

    size = last_transfer_size_;
  }
  else
  {
    // For non-EP0 OUT endpoints, restore NAK on completion.
    if (!IS_EP0)
    {
      *get_rx_ctrl_addr(GetNumber()) =
          (*get_rx_ctrl_addr(GetNumber()) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
    }
  }

  // TOG mismatch indicates data synchronization failure.
  if (IS_OUT)
  {
    const bool TOG_OK = ((USBFSD->INT_ST & USBFS_U_TOG_OK) == USBFS_U_TOG_OK);  // NOLINT
    if (!TOG_OK)
    {
      SetState(State::IDLE);
      (void)Transfer(last_transfer_size_);
      return;
    }
  }

  // Update software data toggle for non-EP0 non-ISO endpoints.
  if (GetState() == State::BUSY && !IS_EP0 && !IS_ISO)
  {
    tog_ = !tog_;
  }

  if (IS_EP0 && IS_OUT)
  {
    tog_ = true;
    *get_rx_ctrl_addr(GetNumber()) = USBFS_UEP_R_RES_ACK;
  }

  OnTransferCompleteCallback(true, size);
}

void CH32EndpointOtgFs::SwitchBuffer()
{
  if (GetDirection() == Direction::IN)
  {
    tog_ = (*get_tx_ctrl_addr(GetNumber()) & USBFS_UEP_T_TOG) == USBFS_UEP_T_TOG;
    SetActiveBlock(!tog_);
  }
  else
  {
    tog_ = (*get_rx_ctrl_addr(GetNumber()) & USBFS_UEP_R_TOG) == USBFS_UEP_R_TOG;
    SetActiveBlock(tog_);
  }
}
#endif  // defined(__CH32X035_H)

#endif
