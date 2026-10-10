#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>

#include "core/esc_port.hpp"
#include "core/esc_registers.hpp"
#include "core/mailbox.hpp"
#include "device_composition.hpp"
#include "ethercat/device/coe/coe_protocol.hpp"

namespace LibXR::EtherCAT
{

/**
 * Native EtherCAT device protocol core.
 *
 * DeviceCore owns the completed DeviceComposition in the same sense that USB
 * DeviceCore owns DeviceComposition. It never owns DeviceClass modules or the
 * concrete ESC driver. Protocol work is entered from two directions: the ESC
 * event path (HandleAlevent/HandleEvent, what an interrupt-driven board driver
 * uses) and the Poll* entry points, which exist for drivers of hardware that
 * does not raise every event reliably. Which of the two a board driver uses is
 * its own choice; see the Poll* methods for when each is needed.
 *
 * The mailbox is protocol-agnostic here: requests are routed to registered
 * MailboxProtocols by protocol number, and the core answers the ones nobody
 * claims. CoE is built in (a dictionary nobody can read is not a dictionary),
 * further protocols (FoE, ...) register through RegisterMailboxProtocol(). The
 * core implements MailboxExchange for them: reply buffer, retained response,
 * state and object access notifications.
 *
 * DeviceCore is not thread-safe. Serialize Handle* and Poll* calls through one
 * execution context; the `in_isr` argument describes callback context and does
 * not provide synchronization. State queries also require external
 * synchronization when another context can advance the core.
 */
class DeviceCore final : private MailboxExchange
{
 public:
  DeviceCore(EscPort& port, DevicePool& pool, std::span<DeviceClass* const> classes);
  DeviceCore(EscPort& port, DevicePool& pool,
             std::initializer_list<DeviceClass*> classes);

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

  /**
   * Service the mailbox without a new AL event.
   *
   * A response that could not be published because the master had not read the
   * previous one, and a request that was skipped for the same reason, are only
   * picked up when the core is entered again. The master is waiting for that
   * very response, so no further request will arrive to trigger it: a driver has
   * to call this periodically for the mailbox to make progress.
   */
  void PollMailbox();

  /**
   * Service the process data without an AL event.
   *
   * The SM event bits are not a reliable "the master wrote new outputs" signal on
   * this hardware: reading AL Event Request clears it, and the ESC re-raises SM
   * events for the PDI's own buffer accesses. A driver polls the process data
   * once per frame instead of relying on them.
   */
  void PollProcessData();

  /**
   * Re-read AL Control without waiting for its event.
   *
   * The AL Control event can be consumed by one of the driver's own register
   * accesses racing the master's write, and then the request is simply gone -
   * that is what made the SAFEOP -> OP transition intermittent. AL Control always
   * holds the master's last requested state, so acting on it periodically
   * converges the state machine no matter how the events landed.
   */
  void PollAlControl();

  /** AL Event Request bits to normalized events. */
  [[nodiscard]] static EscEvent TranslateAlevent(uint32_t raw_alevent);

  /**
   * Let the PDI interrupt report only the events this device handles, by
   * writing the AL Event Mask (0x0204). A driver that configures the mask
   * through its own ESC interface can skip this.
   */
  ErrorCode SetAleventMask(uint16_t mask);

  /**
   * Join the mailbox protocol routing during setup, before the master configures
   * the mailbox. The handler is non-owning and must outlive this DeviceCore.
   *
   * The registry is indexed by protocol number and has exactly
   * MAILBOX_PROTOCOL_COUNT slots (what the type byte's nibble can address), so
   * there is no capacity to run out of.
   *
   * @return false when the protocol is reserved for mailbox errors or the slot
   *         is already taken (the same handler twice, or two handlers claiming
   *         one protocol).
   * @note a protocol number outside the nibble cannot be routed at all and is a
   *       contract violation (`REQUIRE`).
   */
  bool RegisterMailboxProtocol(MailboxProtocol& handler);

  [[nodiscard]] AlState GetState() const override { return state_; }
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

  // MailboxExchange: the reply side and the device context handed to mailbox
  // handlers. Private: handlers see this through MailboxExchange& only.
  [[nodiscard]] RawData ResponsePayload() override;
  [[nodiscard]] bool Respond(uint8_t protocol, size_t payload_size) override;
  void SendError(uint16_t error) override;
  [[nodiscard]] ErrorCode NotifyObjectRead(ObjectAddress address) override;
  [[nodiscard]] ErrorCode NotifyObjectWrite(ObjectAddress address) override;

  [[nodiscard]] ErrorCode ReadEsc(uint16_t address, void* destination, size_t size);
  [[nodiscard]] ErrorCode WriteEsc(uint16_t address, const void* source, size_t size);
  [[nodiscard]] ErrorCode ReadEscConfiguration();
  [[nodiscard]] ErrorCode SetSyncManagerEnabled(uint8_t index, bool enabled);

  /**
   * Mailbox handshake bit (SM status register, bit 3): the writer of a mailbox
   * sets it, the reader clears it. The master polls SM1's bit to know a response
   * is available and SM0's bit to know the request was consumed.
   */
  [[nodiscard]] ErrorCode SetMailboxBufferStatus(uint8_t index, bool full);

  [[nodiscard]] const SyncManager* FindSyncManager(
      uint8_t operation_mode, uint8_t direction, uint8_t* index,
      bool require_nonzero_length = false) const;
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
  [[nodiscard]] MailboxProtocol* FindMailboxProtocol(uint8_t protocol);
  void ResetMailboxProtocols();

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
  CoeProtocol coe_protocol_;
  // Indexed by protocol number (the type byte's nibble): empty slots are
  // protocols nobody registered.
  std::array<MailboxProtocol*, MAILBOX_PROTOCOL_COUNT> mailbox_protocols_{};
  AlState state_ = AlState::INIT;
  AlError al_error_ = AlError::NONE;
};

}  // namespace LibXR::EtherCAT
