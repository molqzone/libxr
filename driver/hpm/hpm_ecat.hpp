#pragma once

#ifndef LIBXR_ETHERCAT_ENABLE
#define LIBXR_ETHERCAT_ENABLE 0
#endif

#if LIBXR_ETHERCAT_ENABLE

#include <cstddef>
#include <cstdint>
#include <initializer_list>

extern "C"
{
#include "hpm_soc.h"
#include "hpm_esc_drv.h"
#include "hpm_interrupt.h"
}

#include "ethercat/core/esc_port.hpp"
#include "ethercat/device/device_core.hpp"

namespace LibXR
{

/**
 * HPM on-chip EtherCAT slave controller adapter.
 *
 * Board code configures ESC pins, EEPROM, and PHY before constructing this
 * object. HPMECATDevice then owns the LibXR protocol core, supplies memory-mapped
 * ESC access, and routes the ESC PDI/SYNC interrupts without any polling path.
 */
class HPMECATDevice final : public EtherCAT::EscPort
{
 public:
  struct Interrupts
  {
    uint32_t pdi = IRQn_ESC;
    uint32_t sync0 = IRQn_ESC_SYNC0;
    uint32_t sync1 = IRQn_ESC_SYNC1;
    uint8_t pdi_priority = 4;
    uint8_t sync_priority = 3;
  };

  HPMECATDevice(ESC_Type& esc, EtherCAT::DevicePool& pool,
              std::initializer_list<EtherCAT::DeviceClass*> classes);
  HPMECATDevice(ESC_Type& esc, EtherCAT::DevicePool& pool,
              std::initializer_list<EtherCAT::DeviceClass*> classes,
              Interrupts interrupts);
  ~HPMECATDevice() override;

  HPMECATDevice(const HPMECATDevice&) = delete;
  HPMECATDevice& operator=(const HPMECATDevice&) = delete;
  HPMECATDevice(HPMECATDevice&&) = delete;
  HPMECATDevice& operator=(HPMECATDevice&&) = delete;

  ErrorCode Read(uint16_t address, RawData data) override;
  ErrorCode Write(uint16_t address, ConstRawData data) override;

  [[nodiscard]] EtherCAT::AlState GetState() const { return core_.GetState(); }
  [[nodiscard]] const EtherCAT::DeviceComposition& GetComposition() const
  {
    return core_.GetComposition();
  }

  /** Route the HPM ESC PDI IRQ from the application's ISR vector. */
  static void OnPdiInterrupt();

  /** Route the HPM ESC SYNC0 IRQ from the application's ISR vector. */
  static void OnSync0Interrupt();

  /** Route the HPM ESC SYNC1 IRQ from the application's ISR vector. */
  static void OnSync1Interrupt();

 private:
  static constexpr size_t ESC_ADDRESS_SPACE_SIZE = 0x10000U;

  [[nodiscard]] EtherCAT::EscEvent ReadPdiEvents() const;
  void EnableInterrupts();
  void DisableInterrupts();

  ESC_Type& esc_;
  Interrupts interrupts_;
  EtherCAT::DeviceCore core_;

  static inline HPMECATDevice* instance_ = nullptr;
};

}  // namespace LibXR

#endif  // LIBXR_ETHERCAT_ENABLE
