// NOLINTBEGIN(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
// ch32_usb_otgfs.cpp  (OTG FS)
#include "ch32_usb_dev.hpp"
#include "ch32_usb_endpoint.hpp"
#include "ep.hpp"

using namespace LibXR;
using namespace LibXR::USB;

#if defined(USBFSD)

extern "C" volatile uint32_t g_usbfs_irq_count = 0;
extern "C" volatile uint32_t g_usbfs_setup_count = 0;
extern "C" volatile uint32_t g_usbfs_bus_reset_count = 0;
extern "C" volatile uint32_t g_usbfs_suspend_count = 0;
extern "C" volatile uint32_t g_usbfs_transfer_count = 0;
extern "C" volatile uint32_t g_usbfs_sof_count = 0;
extern "C" volatile uint32_t g_usbfs_last_pending = 0;
extern "C" volatile uint32_t g_usbfs_last_intflag = 0;
extern "C" volatile uint32_t g_usbfs_last_intst = 0;
extern "C" volatile uint32_t g_usbfs_last_token = 0;
extern "C" volatile uint32_t g_usbfs_last_epnum = 0;
extern "C" volatile uint32_t g_usbfs_out_count[8] = {0};
extern "C" volatile uint32_t g_usbfs_in_count[8] = {0};

namespace
{

static inline void ch32_clock_ahb_enable(uint32_t periph)
{
#if defined(__CH32H417_H)
  RCC_HBPeriphClockCmd(periph, ENABLE);
#else
  RCC_AHBPeriphClockCmd(periph, ENABLE);
#endif
}

static void ch32_usb_clock48_m_config()
{
#if defined(__CH32H417_H)
  if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS)
  {
    RCC_USBHS_PLLCmd(DISABLE);
    RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE
                                                   : RCC_USBHSPLLSource_HSI);
    RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
    RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
    RCC_USBHS_PLLCmd(ENABLE);
    while ((RCC->CTLR & RCC_USBHS_PLLRDY) == 0)
    {
    }
  }

  RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
  RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);
#else
  RCC_ClocksTypeDef clk{};
  RCC_GetClocksFreq(&clk);

  const uint32_t SYSCLK_HZ = clk.SYSCLK_Frequency;

#if defined(RCC_USBCLKSource_PLLCLK_Div1) && defined(RCC_USBCLKSource_PLLCLK_Div2) && defined(RCC_USBCLKSource_PLLCLK_Div3)
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

#elif defined(RCC_USBCLK48MCLKSource_PLLCLK) && defined(RCC_USBFSCLKSource_PLLCLK_Div1) && defined(RCC_USBFSCLKSource_PLLCLK_Div2) && defined(RCC_USBFSCLKSource_PLLCLK_Div3)
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
#endif
}

static void ch32_usbfs_rcc_enable()
{
  ch32_usb_clock48_m_config();

#if defined(RCC_AHBPeriph_USBFS)
  ch32_clock_ahb_enable(RCC_AHBPeriph_USBFS);
#elif defined(RCC_AHBPeriph_USBOTGFS)
  ch32_clock_ahb_enable(RCC_AHBPeriph_USBOTGFS);
#elif defined(RCC_HBPeriph_OTG_FS)
  ch32_clock_ahb_enable(RCC_HBPeriph_OTG_FS);
#endif

#if defined(__CH32H417_H) && defined(RCC_HB2Periph_GPIOA)
  RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
#endif
}


static void ch32_usbfs_force_disconnect()
{
#if defined(__CH32H417_H)
  GPIO_InitTypeDef gpio{};
  gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  gpio.GPIO_Speed = GPIO_Speed_High;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &gpio);
  GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);

  for (volatile uint32_t i = 0; i < 5000000u; ++i)
  {
    asm volatile("nop");
  }

  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  for (volatile uint32_t i = 0; i < 1000000u; ++i)
  {
    asm volatile("nop");
  }
#endif
}


static void ch32_usbfs_restore_endpoint_state()
{
  auto& ep_map = LibXR::CH32EndpointOtgFs::map_otg_fs_;
  constexpr uint8_t OUT_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::OUT);
  constexpr uint8_t IN_IDX = static_cast<uint8_t>(LibXR::USB::Endpoint::Direction::IN);
  constexpr uint8_t N_EP = static_cast<uint8_t>(LibXR::CH32EndpointOtgFs::EP_OTG_FS_MAX_SIZE);

  bool rearm_out[N_EP] = {};

  for (uint8_t ep = 0; ep < N_EP; ++ep)
  {
    auto* out = ep_map[ep][OUT_IDX];
    auto* in = ep_map[ep][IN_IDX];

    if (out != nullptr)
    {
      rearm_out[ep] = out->GetState() == LibXR::USB::Endpoint::State::BUSY;
      out->Configure({out->GetDirection(), out->GetType(), out->MaxPacketSize(),
                      out->UseDoubleBuffer()});
    }

    if (in != nullptr)
    {
      in->Configure({in->GetDirection(), in->GetType(), in->MaxPacketSize(),
                     in->UseDoubleBuffer()});
    }
  }

  USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
  USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;

  for (uint8_t ep = 1; ep < N_EP; ++ep)
  {
    if (!rearm_out[ep])
    {
      continue;
    }

    auto* out = ep_map[ep][OUT_IDX];
    if (out == nullptr)
    {
      continue;
    }

    (void)out->Transfer(out->MaxTransferSize());
  }
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
    ++g_usbfs_irq_count;
    const uint16_t INTFGST = *reinterpret_cast<volatile uint16_t*>(
        reinterpret_cast<uintptr_t>(&USBFSD->INT_FG));

    const uint8_t INTFLAG = static_cast<uint8_t>(INTFGST & 0x00FFu);
    const uint8_t INTST = static_cast<uint8_t>((INTFGST >> 8) & 0x00FFu);

    const uint8_t PENDING = static_cast<uint8_t>(INTFLAG & CLEARABLE_MASK);
    g_usbfs_last_intflag = INTFLAG;
    g_usbfs_last_intst = INTST;
    g_usbfs_last_pending = PENDING;
    if (PENDING == 0)
    {
      break;
    }

    uint8_t clear_mask = 0;

    if (PENDING & USBFS_UIF_BUS_RST)
    {
      ++g_usbfs_bus_reset_count;
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
      USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;

      clear_mask |= USBFS_UIF_BUS_RST;
    }

    if (PENDING & USBFS_UIF_SUSPEND)
    {
      ++g_usbfs_suspend_count;
      clear_mask |= USBFS_UIF_SUSPEND;
    }

    if (PENDING & USBFS_UIF_TRANSFER)
    {
      ++g_usbfs_transfer_count;
      const uint8_t TOKEN = INTST & USBFS_UIS_TOKEN_MASK;
      const uint8_t EPNUM = INTST & USBFS_UIS_ENDP_MASK;
      g_usbfs_last_token = TOKEN;
      g_usbfs_last_epnum = EPNUM;

      auto& ep = map[EPNUM];

      switch (TOKEN)
      {
        case USBFS_UIS_TOKEN_SETUP:
        {
          ++g_usbfs_setup_count;
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
          ++g_usbfs_out_count[EPNUM];
          const uint16_t LEN = USBFSD->RX_LEN;
          if (ep[OUT_IDX])
          {
            ep[OUT_IDX]->TransferComplete(LEN);
          }
          break;
        }

        case USBFS_UIS_TOKEN_IN:
        {
          ++g_usbfs_in_count[EPNUM];
          if (ep[IN_IDX])
          {
            ep[IN_IDX]->TransferComplete(0);
          }
          break;
        }

        case USBFS_UIS_TOKEN_SOF:
        {
          ++g_usbfs_sof_count;
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

  ch32_usbfs_force_disconnect();
  ch32_usbfs_restore_endpoint_state();

  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
  NVIC_EnableIRQ(USBFS_IRQn);
}

void CH32USBOtgFS::Stop(bool)
{
  USBFSD->UDEV_CTRL = 0x00;
  USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
  USBFSD->BASE_CTRL = 0x00;
  ch32_usbfs_force_disconnect();
  NVIC_DisableIRQ(USBFS_IRQn);
}

#endif  // defined(USBFSD)

// NOLINTEND(cppcoreguidelines-pro-type-cstyle-cast,performance-no-int-to-ptr)
