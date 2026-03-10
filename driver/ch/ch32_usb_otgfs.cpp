// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_usb_otgfs.cpp  (OTG FS)
#include "ch32_usb_dev.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ep.hpp"

using namespace LibXR;
using namespace LibXR::USB;

#if defined(USBFSD)

#if defined(__CH32X035_H)
namespace
{

static void ch32_usb_clock48_m_config()
{
  RCC_ClocksTypeDef clk{};
  RCC_GetClocksFreq(&clk);

  const uint32_t SYSCLK_HZ = clk.SYSCLK_Frequency;

#if defined(RCC_USBCLK48MCLKSource_PLLCLK) && \
    defined(RCC_USBFSCLKSource_PLLCLK_Div1) && \
    defined(RCC_USBFSCLKSource_PLLCLK_Div2) && defined(RCC_USBFSCLKSource_PLLCLK_Div3)
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_PLLCLK);

  if (SYSCLK_HZ == 144000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div3);
  }
  else if (SYSCLK_HZ == 96000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div2);
  }
  else if (SYSCLK_HZ == 48000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div1);
  }
  else
  {
    ASSERT(false);
  }
#else
  (void)SYSCLK_HZ;
#endif
}

static void ch32_usbfs_rcc_enable()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  ch32_usb_clock48_m_config();
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);
}

static void ch32_usbfs_gpio_init()
{
  GPIO_InitTypeDef gpio_init{};

  gpio_init.GPIO_Pin = GPIO_Pin_16;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &gpio_init);

  gpio_init.GPIO_Pin = GPIO_Pin_17;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &gpio_init);
}

static void ch32_usbfs_phy_enable()
{
  constexpr uint32_t UDP_PUE_1K5 = AFIO_CTLR_UDP_PUE_0 | AFIO_CTLR_UDP_PUE_1;

  AFIO->CTLR = (AFIO->CTLR & ~(AFIO_CTLR_UDP_PUE | AFIO_CTLR_UDM_PUE)) |
               AFIO_CTLR_USB_PHY_V33 | UDP_PUE_1K5 | AFIO_CTLR_USB_IOEN;
}

static void ch32_usbfs_phy_disable()
{
  AFIO->CTLR &= ~(AFIO_CTLR_UDP_PUE | AFIO_CTLR_UDM_PUE | AFIO_CTLR_USB_IOEN);
}

static void set_ep0_ctrl_bits(uint16_t mask, uint16_t value)
{
  USBFSD->UEP0_CTRL_H = static_cast<uint16_t>((USBFSD->UEP0_CTRL_H & ~mask) | value);
}

static void set_ep0_tx_response(uint16_t response)
{
  set_ep0_ctrl_bits(USBFS_UEP_T_RES_MASK, response);
}

static void set_ep0_rx_response(uint16_t response)
{
  set_ep0_ctrl_bits(USBFS_UEP_R_RES_MASK, response);
}

static void set_ep0_tx_toggle(bool data1)
{
  set_ep0_ctrl_bits(USBFS_UEP_T_TOG, data1 ? USBFS_UEP_T_TOG : 0);
}

static void set_ep0_rx_toggle(bool data1)
{
  set_ep0_ctrl_bits(USBFS_UEP_R_TOG, data1 ? USBFS_UEP_R_TOG : 0);
}

}  // namespace

extern "C" __attribute__((interrupt)) void USBFS_IRQHandler(void)
{
  auto& map = LibXR::CH32EndpointOtgFs::map_otg_fs_;

  constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
  constexpr uint8_t IN_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::IN);

  const uint8_t intflag = USBFSD->INT_FG;
  const uint8_t intst = USBFSD->INT_ST;

  if ((intflag & USBFS_UIF_TRANSFER) != 0)
  {
    const uint8_t token = intst & USBFS_UIS_TOKEN_MASK;
    const uint8_t epnum = intst & USBFS_UIS_ENDP_MASK;

    switch (token)
    {
      case USBFS_UIS_TOKEN_SETUP:
        set_ep0_tx_toggle(true);
        set_ep0_rx_toggle(true);
        set_ep0_tx_response(USBFS_UEP_T_RES_NAK);
        set_ep0_rx_response(USBFS_UEP_R_RES_NAK);

        if (map[0][OUT_IDX] != nullptr)
        {
          map[0][OUT_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
          map[0][OUT_IDX]->tog_ = true;
        }
        if (map[0][IN_IDX] != nullptr)
        {
          map[0][IN_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
          map[0][IN_IDX]->tog_ = true;
        }

        LibXR::CH32USBOtgFS::self_->OnSetupPacket(
            true,
            reinterpret_cast<const SetupPacket*>(map[0][OUT_IDX]->GetBuffer().addr_));
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
        return;

      case USBFS_UIS_TOKEN_OUT:
        if (epnum < LibXR::CH32EndpointOtgFs::EP_OTG_FS_MAX_SIZE && map[epnum][OUT_IDX] != nullptr)
        {
          map[epnum][OUT_IDX]->TransferComplete(USBFSD->RX_LEN);
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
        return;

      case USBFS_UIS_TOKEN_IN:
        if (epnum < LibXR::CH32EndpointOtgFs::EP_OTG_FS_MAX_SIZE && map[epnum][IN_IDX] != nullptr)
        {
          map[epnum][IN_IDX]->TransferComplete(0);
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
        return;

      default:
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
        return;
    }
  }

  if ((intflag & USBFS_UIF_BUS_RST) != 0)
  {
    USBFSD->DEV_ADDR = 0;
    LibXR::CH32USBOtgFS::self_->Deinit(true);
    LibXR::CH32USBOtgFS::self_->Init(true);
    USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    return;
  }

  if ((intflag & USBFS_UIF_SUSPEND) != 0)
  {
    USBFSD->INT_FG = USBFS_UIF_SUSPEND;
    return;
  }

  USBFSD->INT_FG = intflag;
}

CH32USBOtgFS::CH32USBOtgFS(
    const std::initializer_list<EPConfig> EP_CFGS,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : USB::EndpointPool(EP_CFGS.size() * 2),
      USB::DeviceCore(*this, USB::USBSpec::USB_2_1, USB::Speed::FULL, packet_size, vid,
                      pid, bcd, LANG_LIST, CONFIGS, uid)
{
  self_ = this;
  ASSERT(EP_CFGS.size() > 0 && EP_CFGS.size() <= CH32EndpointOtgFs::EP_OTG_FS_MAX_SIZE);

  auto cfgs_itr = EP_CFGS.begin();

  auto ep0_out =
      new CH32EndpointOtgFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::OUT,
                            cfgs_itr->buffer, false);
  auto ep0_in =
      new CH32EndpointOtgFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::IN,
                            cfgs_itr->buffer, false);

  USB::EndpointPool::SetEndpoint0(ep0_in, ep0_out);

  USB::Endpoint::EPNumber ep_index = USB::Endpoint::EPNumber::EP1;

  for (++cfgs_itr, ep_index = USB::Endpoint::EPNumber::EP1; cfgs_itr != EP_CFGS.end();
       ++cfgs_itr, ep_index = USB::Endpoint::NextEPNumber(ep_index))
  {
    if (cfgs_itr->is_in == -1)
    {
      auto ep_out = new CH32EndpointOtgFs(ep_index, USB::Endpoint::Direction::OUT,
                                          cfgs_itr->buffer, false);
      USB::EndpointPool::Put(ep_out);

      auto ep_in = new CH32EndpointOtgFs(ep_index, USB::Endpoint::Direction::IN,
                                         cfgs_itr->buffer, false);
      USB::EndpointPool::Put(ep_in);
    }
    else
    {
      auto ep = new CH32EndpointOtgFs(
          ep_index,
          cfgs_itr->is_in ? USB::Endpoint::Direction::IN : USB::Endpoint::Direction::OUT,
          cfgs_itr->buffer, true);
      USB::EndpointPool::Put(ep);
    }
  }
}

ErrorCode CH32USBOtgFS::SetAddress(uint8_t address, USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::STATUS_IN)
  {
    USBFSD->DEV_ADDR = static_cast<uint8_t>((USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | address);
    set_ep0_tx_response(USBFS_UEP_T_RES_NAK);
    set_ep0_rx_response(USBFS_UEP_R_RES_ACK);
  }
  return ErrorCode::OK;
}

void CH32USBOtgFS::Start(bool)
{
  ch32_usbfs_rcc_enable();
  ch32_usbfs_gpio_init();
  ch32_usbfs_phy_enable();

  USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSD->BASE_CTRL = 0x00;
  USBFSD->DEV_ADDR = 0x00;
  USBFSD->INT_FG = 0xFF;
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
  NVIC_EnableIRQ(USBFS_IRQn);
}

void CH32USBOtgFS::Stop(bool)
{
  NVIC_DisableIRQ(USBFS_IRQn);
  ch32_usbfs_phy_disable();
  USBFSD->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSD->BASE_CTRL = 0x00;
}
#else
namespace
{

static void ch32_usb_clock48_m_config()
{
  RCC_ClocksTypeDef clk{};
  RCC_GetClocksFreq(&clk);

  const uint32_t SYSCLK_HZ = clk.SYSCLK_Frequency;

#if defined(RCC_USBCLKSource_PLLCLK_Div1) && defined(RCC_USBCLKSource_PLLCLK_Div2) && \
    defined(RCC_USBCLKSource_PLLCLK_Div3)
  if (SYSCLK_HZ == 144000000u)
  {
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div3);
  }
  else if (SYSCLK_HZ == 96000000u)
  {
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div2);
  }
  else if (SYSCLK_HZ == 48000000u)
  {
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);
  }
#if defined(RCC_USB5PRE_JUDGE) && defined(RCC_USBCLKSource_PLLCLK_Div5)
  else if (SYSCLK_HZ == 240000000u)
  {
    ASSERT(RCC_USB5PRE_JUDGE() == SET);
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div5);
  }
#endif
  else
  {
    ASSERT(false);
  }

#elif defined(RCC_USBCLK48MCLKSource_PLLCLK) && \
    defined(RCC_USBFSCLKSource_PLLCLK_Div1) &&  \
    defined(RCC_USBFSCLKSource_PLLCLK_Div2) && defined(RCC_USBFSCLKSource_PLLCLK_Div3)
  RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_PLLCLK);

  if (SYSCLK_HZ == 144000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div3);
  }
  else if (SYSCLK_HZ == 96000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div2);
  }
  else if (SYSCLK_HZ == 48000000u)
  {
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_PLLCLK_Div1);
  }
  else
  {
    ASSERT(false);
  }

#else
  (void)SYSCLK_HZ;
#endif
}

static void ch32_usbfs_rcc_enable()
{
  ch32_usb_clock48_m_config();

#if defined(RCC_AHBPeriph_USBFS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBFS, ENABLE);
#elif defined(RCC_AHBPeriph_USBOTGFS)
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBOTGFS, ENABLE);
#endif
}

}  // namespace

// NOLINTNEXTLINE(readability-identifier-naming)
extern "C" __attribute__((interrupt)) void USBFS_IRQHandler(void)
{
  auto& map = LibXR::CH32EndpointOtgFs::map_otg_fs_;

  constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
  constexpr uint8_t IN_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::IN);

  constexpr uint8_t CLEARABLE_MASK = USBFS_UIF_FIFO_OV | USBFS_UIF_HST_SOF |
                                     USBFS_UIF_SUSPEND | USBFS_UIF_TRANSFER |
                                     USBFS_UIF_DETECT | USBFS_UIF_BUS_RST;

  while (true)
  {
    const uint16_t INTFGST = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBFSD->INT_FG));

    const uint8_t INTFLAG = static_cast<uint8_t>(INTFGST & 0x00FFu);
    const uint8_t INTST = static_cast<uint8_t>((INTFGST >> 8) & 0x00FFu);

    const uint8_t PENDING = static_cast<uint8_t>(INTFLAG & CLEARABLE_MASK);
    if (PENDING == 0)
    {
      break;
    }

    uint8_t clear_mask = 0;

    if (PENDING & USBFS_UIF_BUS_RST)
    {
      USBFSD->DEV_ADDR = 0;

      LibXR::CH32USBOtgFS::self_->Deinit(true);
      LibXR::CH32USBOtgFS::self_->Init(true);

      if (map[0][OUT_IDX])
      {
        map[0][OUT_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
      }
      if (map[0][IN_IDX])
      {
        map[0][IN_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
      }

      if (map[0][OUT_IDX])
      {
        map[0][OUT_IDX]->tog_ = true;
      }
      if (map[0][IN_IDX])
      {
        map[0][IN_IDX]->tog_ = true;
      }

      USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
      USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;

      clear_mask |= USBFS_UIF_BUS_RST;
    }

    if (PENDING & USBFS_UIF_SUSPEND)
    {
      LibXR::CH32USBOtgFS::self_->Deinit(true);
      LibXR::CH32USBOtgFS::self_->Init(true);

      if (map[0][OUT_IDX])
      {
        map[0][OUT_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
      }
      if (map[0][IN_IDX])
      {
        map[0][IN_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
      }

      if (map[0][OUT_IDX])
      {
        map[0][OUT_IDX]->tog_ = true;
      }
      if (map[0][IN_IDX])
      {
        map[0][IN_IDX]->tog_ = true;
      }

      USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
      USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;

      clear_mask |= USBFS_UIF_SUSPEND;
    }

    if (PENDING & USBFS_UIF_TRANSFER)
    {
      const uint8_t TOKEN = INTST & USBFS_UIS_TOKEN_MASK;
      const uint8_t EPNUM = INTST & USBFS_UIS_ENDP_MASK;

      auto& ep = map[EPNUM];

      switch (TOKEN)
      {
        case USBFS_UIS_TOKEN_SETUP:
        {
          USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
          USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_NAK;

          if (map[0][OUT_IDX])
          {
            map[0][OUT_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
          }
          if (map[0][IN_IDX])
          {
            map[0][IN_IDX]->SetState(LibXR::USB::Endpoint::State::IDLE);
          }

          if (map[0][OUT_IDX])
          {
            map[0][OUT_IDX]->tog_ = true;
          }
          if (map[0][IN_IDX])
          {
            map[0][IN_IDX]->tog_ = true;
          }

          LibXR::CH32USBOtgFS::self_->OnSetupPacket(
              true,
              reinterpret_cast<const SetupPacket*>(map[0][OUT_IDX]->GetBuffer().addr_));
          break;
        }

        case USBFS_UIS_TOKEN_OUT:
        {
          const uint16_t LEN = USBFSD->RX_LEN;
          if (ep[OUT_IDX])
          {
            ep[OUT_IDX]->TransferComplete(LEN);
          }
          break;
        }

        case USBFS_UIS_TOKEN_IN:
        {
          if (ep[IN_IDX])
          {
            ep[IN_IDX]->TransferComplete(0);
          }
          break;
        }

        default:
          break;
      }

      clear_mask |= USBFS_UIF_TRANSFER;
    }

    clear_mask |= static_cast<uint8_t>(PENDING & ~clear_mask);
    USBFSD->INT_FG = clear_mask;
  }
}

CH32USBOtgFS::CH32USBOtgFS(
    const std::initializer_list<EPConfig> EP_CFGS,
    USB::DeviceDescriptor::PacketSize0 packet_size, uint16_t vid, uint16_t pid,
    uint16_t bcd,
    const std::initializer_list<const USB::DescriptorStrings::LanguagePack*> LANG_LIST,
    const std::initializer_list<const std::initializer_list<USB::ConfigDescriptorItem*>>
        CONFIGS,
    ConstRawData uid)
    : USB::EndpointPool(EP_CFGS.size() * 2),
      USB::DeviceCore(*this, USB::USBSpec::USB_2_1, USB::Speed::FULL, packet_size, vid,
                      pid, bcd, LANG_LIST, CONFIGS, uid)
{
  self_ = this;
  ASSERT(EP_CFGS.size() > 0 && EP_CFGS.size() <= CH32EndpointOtgFs::EP_OTG_FS_MAX_SIZE);

  auto cfgs_itr = EP_CFGS.begin();

  auto ep0_out =
      new CH32EndpointOtgFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::OUT,
                            cfgs_itr->buffer, false);
  auto ep0_in =
      new CH32EndpointOtgFs(USB::Endpoint::EPNumber::EP0, USB::Endpoint::Direction::IN,
                            cfgs_itr->buffer, false);

  USB::EndpointPool::SetEndpoint0(ep0_in, ep0_out);

  USB::Endpoint::EPNumber ep_index = USB::Endpoint::EPNumber::EP1;

  for (++cfgs_itr, ep_index = USB::Endpoint::EPNumber::EP1; cfgs_itr != EP_CFGS.end();
       ++cfgs_itr, ep_index = USB::Endpoint::NextEPNumber(ep_index))
  {
    if (cfgs_itr->is_in == -1)
    {
      auto ep_out = new CH32EndpointOtgFs(ep_index, USB::Endpoint::Direction::OUT,
                                          cfgs_itr->buffer, false);
      USB::EndpointPool::Put(ep_out);

      auto ep_in = new CH32EndpointOtgFs(ep_index, USB::Endpoint::Direction::IN,
                                         cfgs_itr->buffer, false);
      USB::EndpointPool::Put(ep_in);
    }
    else
    {
      auto ep = new CH32EndpointOtgFs(
          ep_index,
          cfgs_itr->is_in ? USB::Endpoint::Direction::IN : USB::Endpoint::Direction::OUT,
          cfgs_itr->buffer, true);
      USB::EndpointPool::Put(ep);
    }
  }
}

ErrorCode CH32USBOtgFS::SetAddress(uint8_t address, USB::DeviceCore::Context context)
{
  if (context == USB::DeviceCore::Context::STATUS_IN)
  {
    USBFSD->DEV_ADDR = (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | address;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
  }
  return ErrorCode::OK;
}

void CH32USBOtgFS::Start(bool)
{
  ch32_usbfs_rcc_enable();
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSH->BASE_CTRL = 0x00;
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
  NVIC_EnableIRQ(USBFS_IRQn);
}

void CH32USBOtgFS::Stop(bool)
{
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSD->BASE_CTRL = 0x00;
  NVIC_DisableIRQ(USBFS_IRQn);
}
#endif  // defined(__CH32X035_H)

#endif  // defined(USBFSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
