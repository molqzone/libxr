#include "hpm_ecat.hpp"

#if LIBXR_ETHERCAT_ENABLE

namespace LibXR
{

HPMEtherCATDevice::HPMEtherCATDevice(ESC_Type& esc, EtherCAT::DevicePool& pool,
                         std::initializer_list<EtherCAT::DeviceClass*> classes)
    : HPMEtherCATDevice(esc, pool, classes,
                  {IRQn_ESC, IRQn_ESC_SYNC0, IRQn_ESC_SYNC1, 4, 3})
{
}

HPMEtherCATDevice::HPMEtherCATDevice(ESC_Type& esc, EtherCAT::DevicePool& pool,
                         std::initializer_list<EtherCAT::DeviceClass*> classes,
                         Interrupts interrupts)
    : esc_(esc), interrupts_(interrupts), core_(*this, pool, classes)
{
  ASSERT(instance_ == nullptr);
  if (instance_ != nullptr)
  {
    return;
  }

  instance_ = this;
  EnableInterrupts();
}

HPMEtherCATDevice::~HPMEtherCATDevice()
{
  if (instance_ == this)
  {
    DisableInterrupts();
    instance_ = nullptr;
  }
}

ErrorCode HPMEtherCATDevice::Read(uint16_t address, RawData data)
{
  if (data.addr_ == nullptr && data.size_ != 0U)
  {
    return ErrorCode::PTR_NULL;
  }
  if (data.size_ > ESC_ADDRESS_SPACE_SIZE - address)
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  const auto* source = reinterpret_cast<volatile const uint8_t*>(&esc_) + address;
  auto* destination = static_cast<uint8_t*>(data.addr_);
  for (size_t index = 0; index < data.size_; ++index)
  {
    destination[index] = source[index];
  }
  return ErrorCode::OK;
}

ErrorCode HPMEtherCATDevice::Write(uint16_t address, ConstRawData data)
{
  if (data.addr_ == nullptr && data.size_ != 0U)
  {
    return ErrorCode::PTR_NULL;
  }
  if (data.size_ > ESC_ADDRESS_SPACE_SIZE - address)
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  auto* destination = reinterpret_cast<volatile uint8_t*>(&esc_) + address;
  const auto* source = static_cast<const uint8_t*>(data.addr_);
  for (size_t index = 0; index < data.size_; ++index)
  {
    destination[index] = source[index];
  }
  return ErrorCode::OK;
}

void HPMEtherCATDevice::OnPdiInterrupt()
{
  if (instance_ != nullptr)
  {
    instance_->core_.HandleInterrupt(instance_->ReadPdiEvents());
  }
}

void HPMEtherCATDevice::OnSync0Interrupt()
{
  if (instance_ == nullptr)
  {
    return;
  }

  [[maybe_unused]] volatile uint8_t acknowledgement = instance_->esc_.SYNC0_STAT;
  instance_->core_.HandleInterrupt(EtherCAT::EscEvent::SYNC0);
}

void HPMEtherCATDevice::OnSync1Interrupt()
{
  if (instance_ == nullptr)
  {
    return;
  }

  [[maybe_unused]] volatile uint8_t acknowledgement = instance_->esc_.SYNC1_STAT;
  instance_->core_.HandleInterrupt(EtherCAT::EscEvent::SYNC1);
}

EtherCAT::EscEvent HPMEtherCATDevice::ReadPdiEvents() const
{
  const uint32_t raw_events = esc_.AL_EVT_REQ;
  EtherCAT::EscEvent events = EtherCAT::EscEvent::NONE;

  if ((raw_events & ESC_AL_EVT_REQ_ALC_EVT_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::AL_CONTROL;
  }
  if ((raw_events & ESC_AL_EVT_REQ_SM_ACT_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::SYNC_MANAGER_CHANGE;
  }
  if ((raw_events & ESC_AL_EVT_REQ_SM_INT_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::SYNC_MANAGER;
  }
  if ((raw_events & ESC_AL_EVT_REQ_WDG_PD_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::WATCHDOG;
  }
  if ((raw_events & ESC_AL_EVT_REQ_EE_EMU_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::EEPROM;
  }
  if ((raw_events & ESC_AL_EVT_REQ_ST_DC_SYNC0_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::SYNC0;
  }
  if ((raw_events & ESC_AL_EVT_REQ_ST_DC_SYNC1_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::SYNC1;
  }
  if ((raw_events & ESC_AL_EVT_REQ_DCL_EVT_MASK) != 0U)
  {
    events = events | EtherCAT::EscEvent::DISTRIBUTED_CLOCK_LATCH;
  }
  return events;
}

void HPMEtherCATDevice::EnableInterrupts()
{
#if defined(HPM_IP_FEATURE_ESC_SYNC_IRQ_MASK) && HPM_IP_FEATURE_ESC_SYNC_IRQ_MASK
  esc_enable_sync_irq_to_pdi_irq(&esc_, false, false);
#endif
  esc_enable_irq(&esc_,
                 static_cast<esc_irq_mask_t>(esc_sync0_irq_mask | esc_sync1_irq_mask));
  intc_m_enable_irq_with_priority(interrupts_.sync0, interrupts_.sync_priority);
  intc_m_enable_irq_with_priority(interrupts_.sync1, interrupts_.sync_priority);
  intc_m_enable_irq_with_priority(interrupts_.pdi, interrupts_.pdi_priority);
}

void HPMEtherCATDevice::DisableInterrupts()
{
  intc_m_disable_irq(interrupts_.pdi);
  intc_m_disable_irq(interrupts_.sync0);
  intc_m_disable_irq(interrupts_.sync1);
  esc_disable_irq(&esc_,
                  static_cast<esc_irq_mask_t>(esc_sync0_irq_mask | esc_sync1_irq_mask));
}

}  // namespace LibXR

#endif  // LIBXR_ETHERCAT_ENABLE
