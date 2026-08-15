#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

#include "core/esc_port.hpp"
#include "core/esc_registers.hpp"
#include "device_composition.hpp"

namespace LibXR::EtherCAT
{

/**
 * Native EtherCAT device protocol core.
 *
 * DeviceCore owns the completed DeviceComposition in the same sense that USB
 * DeviceCore owns DeviceComposition. It never owns DeviceClass modules or the
 * concrete ESC driver. All protocol work is driven from ESC IRQ events; no
 * background polling path is required.
 */
class DeviceCore final
{
 public:
  DeviceCore(EscPort& port, DevicePool& pool, std::initializer_list<DeviceClass*> classes);

  DeviceCore(const DeviceCore&) = delete;
  DeviceCore& operator=(const DeviceCore&) = delete;
  DeviceCore(DeviceCore&&) = delete;
  DeviceCore& operator=(DeviceCore&&) = delete;

  /** Enter the protocol core from the board driver's ESC IRQ path. */
  void HandleInterrupt(EscEvent events);

  [[nodiscard]] AlState GetState() const { return state_; }
  [[nodiscard]] AlError GetAlError() const { return al_error_; }
  [[nodiscard]] bool HasAlError() const { return al_error_ != AlError::NONE; }
  [[nodiscard]] const DeviceComposition& GetComposition() const { return composition_; }

 private:
  struct SyncManager
  {
    uint16_t physical_start = 0;
    uint16_t length = 0;
    uint8_t control = 0;
    uint8_t status = 0;
    uint8_t activate = 0;
    uint8_t pdi_control = 0;
  };

  struct Fmmu
  {
    uint32_t logical_start = 0;
    uint16_t logical_length = 0;
    uint8_t logical_start_bit = 0;
    uint8_t logical_stop_bit = 0;
    uint16_t physical_start = 0;
    uint8_t physical_start_bit = 0;
    uint8_t type = 0;
    uint8_t activate = 0;
  };

  struct ProcessDataConfiguration
  {
    uint16_t output_address = 0;
    size_t output_size = 0;
    uint16_t input_address = 0;
    size_t input_size = 0;
    bool valid = false;
  };

  struct MailboxConfiguration
  {
    SyncManager input{};
    SyncManager output{};
    bool enabled = false;
    uint8_t counter = 0;
  };

  enum class SdoTransferDirection : uint8_t
  {
    NONE,
    UPLOAD,
    DOWNLOAD
  };

  struct SdoTransfer
  {
    SdoTransferDirection direction = SdoTransferDirection::NONE;
    ObjectEntry* entry = nullptr;
    size_t size = 0;
    size_t offset = 0;
    bool toggle = false;
  };

  static uint16_t ReadLe16(const uint8_t* data);
  static uint32_t ReadLe32(const uint8_t* data);
  static void WriteLe16(uint8_t* data, uint16_t value);
  static void WriteLe32(uint8_t* data, uint32_t value);

  [[nodiscard]] ErrorCode ReadEsc(uint16_t address, void* destination, size_t size);
  [[nodiscard]] ErrorCode WriteEsc(uint16_t address, const void* source, size_t size);
  [[nodiscard]] ErrorCode ReadEscConfiguration();
  [[nodiscard]] ErrorCode SetSyncManagerEnabled(uint8_t index, bool enabled);

  [[nodiscard]] bool ValidateMailboxConfiguration(AlError& error);
  [[nodiscard]] bool ValidateProcessDataConfiguration(AlError& error);
  [[nodiscard]] const Fmmu* FindFmmu(uint16_t physical_start, size_t length,
                                      uint8_t required_type) const;
  [[nodiscard]] bool StartMailbox();
  void StopMailbox();
  [[nodiscard]] bool StartProcessData();
  void StopProcessData();
  void StopOutputs();

  void ProcessAlControl();
  void RequestState(AlState requested);
  void CommitState(AlState next_state);
  void Fail(AlState fallback_state, AlError error);
  void PublishAlStatus();

  [[nodiscard]] uint32_t ProcessSyncManagerEvents();
  void TransferOutputs();
  void TransferInputs();

  void ProcessMailbox();
  [[nodiscard]] bool SendMailbox(uint8_t protocol, size_t payload_size);
  void SendMailboxError(uint16_t error);
  void ProcessCoe(const uint8_t* payload, size_t payload_size);
  void ProcessSdoUpload(const uint8_t* payload, size_t payload_size);
  void ProcessSdoDownload(const uint8_t* payload, size_t payload_size);
  void ProcessSdoUploadSegment(const uint8_t* payload, size_t payload_size);
  void ProcessSdoDownloadSegment(const uint8_t* payload, size_t payload_size);
  void SendSdoAbort(uint16_t index, uint8_t subindex, uint32_t abort_code);
  [[nodiscard]] bool SendSdoDownloadResponse(uint16_t index, uint8_t subindex);
  [[nodiscard]] bool IsObjectReadable(const ObjectEntry& entry) const;
  [[nodiscard]] bool IsObjectWritable(const ObjectEntry& entry) const;
  [[nodiscard]] size_t ObjectSize(const ObjectEntry& entry) const;
  void ResetSdoTransfer();

  EscPort& port_;
  DevicePool& pool_;
  DeviceComposition composition_;
  std::array<SyncManager, EscRegister::SYNC_MANAGER_COUNT> sync_managers_{};
  std::array<Fmmu, EscRegister::FMMU_COUNT> fmmus_{};
  ProcessDataConfiguration process_data_{};
  MailboxConfiguration mailbox_{};
  SdoTransfer sdo_transfer_{};
  AlState state_ = AlState::INIT;
  AlError al_error_ = AlError::NONE;
};

}  // namespace LibXR::EtherCAT
