#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>

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
  DeviceCore(EscPort& port, DevicePool& pool, std::span<DeviceClass* const> classes);
  DeviceCore(EscPort& port, DevicePool& pool, std::initializer_list<DeviceClass*> classes);

  DeviceCore(const DeviceCore&) = delete;
  DeviceCore& operator=(const DeviceCore&) = delete;
  DeviceCore(DeviceCore&&) = delete;
  DeviceCore& operator=(DeviceCore&&) = delete;

  /**
   * Enter the protocol core from the board driver's ESC IRQ path.
   *
   * The AL Event Request register (0x0220) is read-clear and only the driver
   * may read it, so its value is handed in here: bit 8..15 name the sync
   * manager that fired, which the core cannot recover by reading again.
   *
   * @param raw_alevent value read from 0x0220 (not an EscEvent).
   * @param in_isr      whether the driver is calling from an interrupt.
   */
  void HandleAlevent(uint32_t raw_alevent, bool in_isr);

  /**
   * Enter the core for an event that has no register behind it.
   *
   * Used for the edge events the driver detects itself, e.g. SYNC0/SYNC1 on a
   * dedicated interrupt line.
   */
  void HandleEvent(EscEvent event, bool in_isr);

  /** AL Event Request bits to normalized events. */
  [[nodiscard]] static EscEvent TranslateAlevent(uint32_t raw_alevent);

  /**
   * Let the PDI interrupt report only the events this device handles, by
   * writing the AL Event Mask (0x0204). A driver that configures the mask
   * through its own ESC interface can skip this.
   */
  ErrorCode SetAleventMask(uint16_t mask);

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
    uint8_t output_sync_manager = 0;
    uint16_t output_address = 0;
    size_t output_size = 0;
    uint8_t input_sync_manager = 0;
    uint16_t input_address = 0;
    size_t input_size = 0;
    bool valid = false;
  };

  struct MailboxConfiguration
  {
    uint8_t request_sync_manager = 0;
    SyncManager request{};
    uint8_t response_sync_manager = 0;
    SyncManager response{};
    bool enabled = false;
    uint8_t response_counter = 0;
    uint8_t last_request_counter = 0;
    uint8_t last_request_protocol = 0;
    size_t response_size = 0;
    bool has_cached_response = false;
    bool response_pending = false;
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

  [[nodiscard]] const SyncManager* FindSyncManager(uint8_t operation_mode, uint8_t direction, uint8_t* index,
                                                   bool require_nonzero_length = false) const;
  [[nodiscard]] bool ValidateMailboxConfiguration(AlError& error);
  [[nodiscard]] bool ValidateProcessDataConfiguration(AlError& error);
  [[nodiscard]] const Fmmu* FindFmmu(uint16_t physical_start, size_t length, uint8_t required_type) const;
  [[nodiscard]] bool StartMailbox();
  void StopMailbox();
  [[nodiscard]] bool StartProcessData();
  void StopProcessData();
  void StopOutputs();

  void ProcessAlControl();
  /** Shared body of the two public entry points. */
  void Dispatch(EscEvent events, uint32_t raw_alevent);
  void RequestState(AlState requested);
  void CommitState(AlState next_state);
  void Fail(AlState fallback_state, AlError error);
  void PublishAlStatus();

  void ProcessSyncManagerEvents(uint32_t sync_manager_events);
  void TransferOutputs();
  void TransferInputs();

  void ProcessMailbox(uint32_t sync_manager_events);
  [[nodiscard]] bool FlushMailboxResponse();
  [[nodiscard]] bool QueueMailboxResponse(uint8_t protocol, size_t payload_size);
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

  // Context of the entry being processed. DeviceCore is entered from one
  // context at a time, so the class dispatch reads this instead of threading
  // the flag through every private helper. The DeviceClass hooks themselves
  // still receive it explicitly.
  bool in_isr_ = false;

  EscPort& port_;
  DevicePool& pool_;
  DeviceComposition composition_;
  std::array<SyncManager, EscRegister::MAX_SYNC_MANAGER_COUNT> sync_managers_{};
  std::array<Fmmu, EscRegister::MAX_FMMU_COUNT> fmmus_{};
  uint8_t sync_manager_count_ = 0;
  uint8_t fmmu_count_ = 0;
  ProcessDataConfiguration process_data_{};
  MailboxConfiguration mailbox_{};
  SdoTransfer sdo_transfer_{};
  AlState state_ = AlState::INIT;
  AlError al_error_ = AlError::NONE;
};

}  // namespace LibXR::EtherCAT
