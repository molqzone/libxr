#include "device_core.hpp"

#include <cstring>
#include <limits>

#include "core/byte_order.hpp"

namespace LibXR::EtherCAT
{

namespace
{

bool IsState(AlState value, AlState expected) { return value == expected; }

}  // namespace

DeviceCore::DeviceCore(EscPort& port, DevicePool& pool,
                       std::span<DeviceClass* const> classes)
    : port_(port),
      pool_(pool),
      composition_(pool, classes),
      coe_protocol_(composition_.GetObjectDictionary())
{
  composition_.BindEscPort(port_);
  // CoE is the built-in mailbox protocol: the dictionary this composition just
  // built would be unreachable without it.
  (void)RegisterMailboxProtocol(coe_protocol_);
  PublishAlStatus();
}

DeviceCore::DeviceCore(EscPort& port, DevicePool& pool,
                       std::initializer_list<DeviceClass*> classes)
    : DeviceCore(port, pool,
                 std::span<DeviceClass* const>(classes.begin(), classes.size()))
{
}

ErrorCode DeviceCore::SetAleventMask(uint16_t mask)
{
  // How the register has to be touched is a property of the attached ESC, so the
  // port owns it (see EscPort::WriteAleventMask).
  return port_.WriteAleventMask(mask);
}

void DeviceCore::HandleAlevent(uint32_t raw_alevent, bool in_isr)
{
  in_isr_ = in_isr;
  Dispatch(TranslateAlevent(raw_alevent), raw_alevent);
}

void DeviceCore::HandleEvent(EscEvent event, bool in_isr)
{
  in_isr_ = in_isr;
  // No register value behind an edge event: nothing to carry down for the SM
  // dispatch.
  Dispatch(event, 0U);
}

EscEvent DeviceCore::TranslateAlevent(uint32_t raw_alevent)
{
  EscEvent events = EscEvent::NONE;

  if ((raw_alevent & EscRegister::EVENT_AL_CONTROL) != 0U)
  {
    events = events | EscEvent::AL_CONTROL;
  }
  if ((raw_alevent & EscRegister::EVENT_SYNC_MANAGER_CHANGE) != 0U)
  {
    events = events | EscEvent::SYNC_MANAGER_CHANGE;
  }
  if ((raw_alevent & EscRegister::EVENT_EEPROM) != 0U)
  {
    events = events | EscEvent::EEPROM;
  }
  if ((raw_alevent & EscRegister::EVENT_WATCHDOG) != 0U)
  {
    events = events | EscEvent::WATCHDOG;
  }
  if ((raw_alevent & EscRegister::EVENT_SYNC_MANAGER_MASK) != 0U)
  {
    events = events | EscEvent::SYNC_MANAGER;
  }
  if ((raw_alevent & (EscRegister::EVENT_DC_LATCH | EscRegister::EVENT_DC_SYNC0 |
                      EscRegister::EVENT_DC_SYNC1)) != 0U)
  {
    events = events | EscEvent::SYNC0;
  }

  return events;
}

void DeviceCore::Dispatch(EscEvent events, uint32_t raw_alevent)
{
  if (HasEvent(events, EscEvent::AL_CONTROL))
  {
    ProcessAlControl();
  }

  if (HasEvent(events, EscEvent::SYNC_MANAGER_CHANGE))
  {
    AlError error = AlError::NONE;
    if (IsState(state_, AlState::SAFE_OPERATIONAL) ||
        IsState(state_, AlState::OPERATIONAL))
    {
      if (!ValidateProcessDataConfiguration(error))
      {
        Fail(AlState::PRE_OPERATIONAL, error);
      }
    }
    else if (IsState(state_, AlState::PRE_OPERATIONAL) && mailbox_.enabled &&
             !ValidateMailboxConfiguration(error))
    {
      Fail(AlState::INIT, error);
    }
  }

  if (HasEvent(events, EscEvent::SYNC_MANAGER))
  {
    ProcessSyncManagerEvents(raw_alevent);
  }

  if (HasEvent(events, EscEvent::SYNC0) || HasEvent(events, EscEvent::SYNC1))
  {
    TransferInputs();
  }

  if (HasEvent(events, EscEvent::WATCHDOG) && IsState(state_, AlState::OPERATIONAL))
  {
    Fail(AlState::SAFE_OPERATIONAL, AlError::WATCHDOG);
  }
}

ErrorCode DeviceCore::ReadEsc(uint16_t address, void* destination, size_t size)
{
  return port_.Read(address, RawData(destination, size));
}

ErrorCode DeviceCore::WriteEsc(uint16_t address, const void* source, size_t size)
{
  return port_.Write(address, ConstRawData(source, size));
}

ErrorCode DeviceCore::ReadEscConfiguration()
{
  // The channel counts are a property of the device; the pool is a fixed capacity
  // view of them, sized by the board module's Config. A device that exposes more
  // FMMUs or sync managers than this composition reserved cannot be driven - using
  // only the first N would leave a master's channel unserved and silently
  // misbehave - so it is reported as a capacity error. The choice of capacity
  // itself belongs to the module, not here.
  uint8_t device_channels[2]{};
  if (ReadEsc(EscRegister::FMMU_COUNT, device_channels, sizeof(device_channels)) !=
      ErrorCode::OK)
  {
    return ErrorCode::FAILED;
  }
  if (device_channels[0] > fmmus_.size() || device_channels[1] > sync_managers_.size())
  {
    return ErrorCode::OUT_OF_RANGE;
  }
  fmmu_count_ = device_channels[0];
  sync_manager_count_ = device_channels[1];

  for (uint8_t index = 0; index < sync_manager_count_; ++index)
  {
    uint8_t bytes[EscRegister::SYNC_MANAGER_SIZE]{};
    const uint16_t address = static_cast<uint16_t>(
        EscRegister::SYNC_MANAGER_BASE + index * EscRegister::SYNC_MANAGER_SIZE);
    const ErrorCode result = ReadEsc(address, bytes, sizeof(bytes));
    if (result != ErrorCode::OK)
    {
      return result;
    }

    sync_managers_[index] = {
        ReadLe16(bytes), ReadLe16(bytes + 2U), bytes[4], bytes[5], bytes[6], bytes[7]};
  }

  for (uint8_t index = sync_manager_count_; index < sync_managers_.size(); ++index)
  {
    sync_managers_[index] = {};
  }

  for (uint8_t index = 0; index < fmmu_count_; ++index)
  {
    uint8_t bytes[EscRegister::FMMU_SIZE]{};
    const uint16_t address =
        static_cast<uint16_t>(EscRegister::FMMU_BASE + index * EscRegister::FMMU_SIZE);
    const ErrorCode result = ReadEsc(address, bytes, sizeof(bytes));
    if (result != ErrorCode::OK)
    {
      return result;
    }

    // FMMU n block, 16 bytes (ETG.1000-4): logical start address (u32) and length
    // (u16), the logical start and stop bit inside the first and last byte,
    // physical start address (u16) and start bit, the type (1 = read/write, 2 =
    // read, 3 = write) and the activate flag. Field names are assigned one by one
    // so the byte offsets stay readable.
    Fmmu fmmu{};
    fmmu.logical_start = ReadLe32(bytes);
    fmmu.logical_length = ReadLe16(bytes + 4U);
    fmmu.logical_start_bit = bytes[6];
    fmmu.logical_stop_bit = bytes[7];
    fmmu.physical_start = ReadLe16(bytes + 8U);
    fmmu.physical_start_bit = bytes[10];
    fmmu.type = bytes[11];
    fmmu.activate = bytes[12];
    fmmus_[index] = fmmu;
  }
  for (uint8_t index = fmmu_count_; index < fmmus_.size(); ++index)
  {
    fmmus_[index] = {};
  }
  return ErrorCode::OK;
}

ErrorCode DeviceCore::SetSyncManagerEnabled(uint8_t index, bool enabled)
{
  if (index >= sync_manager_count_)
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  uint8_t activate = sync_managers_[index].activate;
  activate = enabled ? static_cast<uint8_t>(activate | EscRegister::SYNC_MANAGER_ENABLE)
                     : static_cast<uint8_t>(activate & ~EscRegister::SYNC_MANAGER_ENABLE);
  const uint16_t address = static_cast<uint16_t>(
      EscRegister::SYNC_MANAGER_BASE + index * EscRegister::SYNC_MANAGER_SIZE + 6U);
  const ErrorCode result = WriteEsc(address, &activate, sizeof(activate));
  if (result == ErrorCode::OK)
  {
    sync_managers_[index].activate = activate;
  }
  return result;
}

ErrorCode DeviceCore::SetMailboxBufferStatus(uint8_t index, bool full)
{
  if (index >= sync_manager_count_)
  {
    return ErrorCode::OUT_OF_RANGE;
  }

  const uint16_t address = static_cast<uint16_t>(
      EscRegister::SYNC_MANAGER_BASE + index * EscRegister::SYNC_MANAGER_SIZE + 5U);
  const uint8_t status = full ? EscRegister::SYNC_MANAGER_STATUS_MAILBOX : 0U;
  return WriteEsc(address, &status, sizeof(status));
}

const DeviceCore::SyncManager* DeviceCore::FindSyncManager(
    uint8_t operation_mode, uint8_t direction, uint8_t* index,
    bool require_nonzero_length) const
{
  for (uint8_t candidate = 0; candidate < sync_manager_count_; ++candidate)
  {
    const SyncManager& sync_manager = sync_managers_[candidate];
    if ((sync_manager.control & EscRegister::SYNC_MANAGER_OPERATION_MODE_MASK) ==
            operation_mode &&
        (sync_manager.control & EscRegister::SYNC_MANAGER_DIRECTION_MASK) == direction &&
        (!require_nonzero_length || sync_manager.length != 0U))
    {
      if (index != nullptr)
      {
        *index = candidate;
      }
      return &sync_manager;
    }
  }
  return nullptr;
}

bool DeviceCore::ValidateMailboxConfiguration(AlError& error)
{
  if (ReadEscConfiguration() != ErrorCode::OK)
  {
    error = AlError::INVALID_MAILBOX_CONFIGURATION;
    return false;
  }

  uint8_t request_index = 0;
  uint8_t response_index = 0;
  const SyncManager* request =
      FindSyncManager(EscRegister::SYNC_MANAGER_MAILBOX_MODE,
                      EscRegister::SYNC_MANAGER_ECAT_WRITE, &request_index);
  const SyncManager* response =
      FindSyncManager(EscRegister::SYNC_MANAGER_MAILBOX_MODE,
                      EscRegister::SYNC_MANAGER_ECAT_READ, &response_index);
  if (request == nullptr && response == nullptr)
  {
    mailbox_ = {};
    return true;
  }
  if (request == nullptr || response == nullptr || request->length == 0U ||
      response->length == 0U)
  {
    error = AlError::INVALID_MAILBOX_CONFIGURATION;
    return false;
  }

  const uint32_t request_end =
      static_cast<uint32_t>(request->physical_start) + request->length;
  const uint32_t response_end =
      static_cast<uint32_t>(response->physical_start) + response->length;

  // The mailbox must carry at least one message every registered mailbox
  // protocol acts on (for CoE: one SDO initiate request/response).
  size_t min_payload = 0;
  for (const MailboxProtocol* handler : mailbox_protocols_)
  {
    if (handler == nullptr)
    {
      continue;
    }
    const size_t required = handler->MinPayloadSize();
    min_payload = required > min_payload ? required : min_payload;
  }

  if (request->length < MAILBOX_HEADER_SIZE + min_payload ||
      response->length < MAILBOX_HEADER_SIZE + min_payload ||
      request_end > 0x10000U || response_end > 0x10000U ||
      pool_.storage_.mailbox_request == nullptr ||
      pool_.storage_.mailbox_request_capacity < request->length ||
      pool_.storage_.mailbox_response == nullptr ||
      pool_.storage_.mailbox_response_capacity < response->length)
  {
    error = AlError::INVALID_MAILBOX_CONFIGURATION;
    return false;
  }

  mailbox_ = {};
  mailbox_.request_sync_manager = request_index;
  mailbox_.request = *request;
  mailbox_.response_sync_manager = response_index;
  mailbox_.response = *response;
  mailbox_.enabled = true;
  return true;
}

const DeviceCore::Fmmu* DeviceCore::FindFmmu(uint16_t physical_start, size_t length,
                                             uint8_t required_type) const
{
  const uint32_t physical_end = static_cast<uint32_t>(physical_start) + length;
  for (uint8_t index = 0; index < fmmu_count_; ++index)
  {
    const Fmmu& fmmu = fmmus_[index];
    const uint32_t fmmu_end =
        static_cast<uint32_t>(fmmu.physical_start) + fmmu.logical_length;
    if ((fmmu.activate & EscRegister::FMMU_ENABLE) != 0U &&
        (fmmu.type & required_type) != 0U && fmmu.physical_start_bit == 0U &&
        fmmu.physical_start <= physical_start && physical_end <= fmmu_end)
    {
      return &fmmu;
    }
  }
  return nullptr;
}

bool DeviceCore::ValidateProcessDataConfiguration(AlError& error)
{
  if (ReadEscConfiguration() != ErrorCode::OK)
  {
    error = AlError::INVALID_SYNC_MANAGER_CONFIGURATION;
    return false;
  }

  const size_t output_size = composition_.GetPdoByteSize(PdoDirection::RX);
  const size_t input_size = composition_.GetPdoByteSize(PdoDirection::TX);
  const size_t process_data_capacity =
      output_size > input_size ? output_size : input_size;
  if (output_size > std::numeric_limits<uint16_t>::max() ||
      input_size > std::numeric_limits<uint16_t>::max() ||
      (process_data_capacity != 0U &&
       (pool_.storage_.process_data == nullptr ||
        pool_.storage_.process_data_capacity < process_data_capacity)))
  {
    error = AlError::INVALID_OUTPUT_MAPPING;
    return false;
  }

  uint8_t output_index = 0;
  const SyncManager* output =
      output_size == 0U
          ? nullptr
          : FindSyncManager(EscRegister::SYNC_MANAGER_BUFFERED_MODE,
                            EscRegister::SYNC_MANAGER_ECAT_WRITE, &output_index, true);
  if (output_size != 0U && (output == nullptr || output->length != output_size ||
                            FindFmmu(output->physical_start, output_size,
                                     EscRegister::FMMU_WRITE_ENABLE) == nullptr))
  {
    error = output == nullptr || output->length != output_size
                ? AlError::INVALID_OUTPUT_SYNC_MANAGER
                : AlError::INVALID_OUTPUT_MAPPING;
    return false;
  }

  uint8_t input_index = 0;
  const SyncManager* input =
      input_size == 0U
          ? nullptr
          : FindSyncManager(EscRegister::SYNC_MANAGER_BUFFERED_MODE,
                            EscRegister::SYNC_MANAGER_ECAT_READ, &input_index, true);
  if (input_size != 0U && (input == nullptr || input->length != input_size ||
                           FindFmmu(input->physical_start, input_size,
                                    EscRegister::FMMU_READ_ENABLE) == nullptr))
  {
    error = input == nullptr || input->length != input_size
                ? AlError::INVALID_INPUT_SYNC_MANAGER
                : AlError::INVALID_INPUT_FMMU_CONFIGURATION;
    return false;
  }

  process_data_ = {};
  process_data_.output_sync_manager = output_index;
  process_data_.output_address = output == nullptr ? 0U : output->physical_start;
  process_data_.output_size = output_size;
  process_data_.input_sync_manager = input_index;
  process_data_.input_address = input == nullptr ? 0U : input->physical_start;
  process_data_.input_size = input_size;
  process_data_.valid = true;
  return true;
}

bool DeviceCore::StartMailbox()
{
  AlError error = AlError::NONE;
  if (!ValidateMailboxConfiguration(error))
  {
    return false;
  }
  if (!mailbox_.enabled)
  {
    return true;
  }
  if (SetSyncManagerEnabled(mailbox_.request_sync_manager, true) != ErrorCode::OK ||
      SetSyncManagerEnabled(mailbox_.response_sync_manager, true) != ErrorCode::OK)
  {
    return false;
  }
  // Start from an empty handshake: a stale "buffer full" would make the master
  // wait for a response that is already gone.
  (void)SetMailboxBufferStatus(mailbox_.request_sync_manager, false);
  (void)SetMailboxBufferStatus(mailbox_.response_sync_manager, false);
  return true;
}

void DeviceCore::StopMailbox()
{
  if (mailbox_.enabled)
  {
    (void)SetSyncManagerEnabled(mailbox_.request_sync_manager, false);
    (void)SetSyncManagerEnabled(mailbox_.response_sync_manager, false);
  }
  mailbox_ = {};
  ResetMailboxProtocols();
}

bool DeviceCore::StartProcessData()
{
  if (!process_data_.valid)
  {
    return false;
  }
  if (process_data_.input_size != 0U &&
      SetSyncManagerEnabled(process_data_.input_sync_manager, true) != ErrorCode::OK)
  {
    return false;
  }
  if (process_data_.output_size != 0U &&
      SetSyncManagerEnabled(process_data_.output_sync_manager, true) != ErrorCode::OK)
  {
    if (process_data_.input_size != 0U)
    {
      (void)SetSyncManagerEnabled(process_data_.input_sync_manager, false);
    }
    return false;
  }
  return true;
}

void DeviceCore::StopProcessData()
{
  if (process_data_.output_size != 0U)
  {
    (void)SetSyncManagerEnabled(process_data_.output_sync_manager, false);
  }
  if (process_data_.input_size != 0U)
  {
    (void)SetSyncManagerEnabled(process_data_.input_sync_manager, false);
  }
  process_data_ = {};
}

void DeviceCore::StopOutputs()
{
  if (process_data_.output_size != 0U)
  {
    (void)SetSyncManagerEnabled(process_data_.output_sync_manager, false);
  }
}

void DeviceCore::ProcessAlControl()
{
  uint8_t bytes[2]{};
  if (ReadEsc(EscRegister::AL_CONTROL, bytes, sizeof(bytes)) != ErrorCode::OK)
  {
    Fail(state_, AlError::UNSPECIFIED);
    return;
  }

  const uint16_t control = ReadLe16(bytes);
  const bool acknowledge_error = (control & AL_CONTROL_ERROR_ACKNOWLEDGE) != 0U;
  if (HasAlError() && !acknowledge_error)
  {
    PublishAlStatus();
    return;
  }
  if (acknowledge_error)
  {
    al_error_ = AlError::NONE;
  }

  switch (control & 0x000FU)
  {
    case static_cast<uint16_t>(AlState::INIT):
      RequestState(AlState::INIT);
      break;
    case static_cast<uint16_t>(AlState::PRE_OPERATIONAL):
      RequestState(AlState::PRE_OPERATIONAL);
      break;
    case static_cast<uint16_t>(AlState::BOOTSTRAP):
      RequestState(AlState::BOOTSTRAP);
      break;
    case static_cast<uint16_t>(AlState::SAFE_OPERATIONAL):
      RequestState(AlState::SAFE_OPERATIONAL);
      break;
    case static_cast<uint16_t>(AlState::OPERATIONAL):
      RequestState(AlState::OPERATIONAL);
      break;
    default:
      Fail(state_, AlError::UNKNOWN_STATE);
      break;
  }
}

void DeviceCore::RequestState(AlState requested)
{
  // The ESC raises one AL Control event flag, so a master that writes SAFEOP and
  // OP back to back can be observed here as a direct PREOP -> OP request (and
  // INIT -> SAFEOP).  EtherCAT expects the slave to pass through the
  // intermediate states instead of refusing the jump, so walk up first and keep
  // the intermediate step's error if it fails.
  if (requested == AlState::OPERATIONAL &&
      (IsState(state_, AlState::INIT) || IsState(state_, AlState::PRE_OPERATIONAL)))
  {
    RequestState(AlState::SAFE_OPERATIONAL);
    if (!IsState(state_, AlState::SAFE_OPERATIONAL))
    {
      return;
    }
  }
  else if (requested == AlState::SAFE_OPERATIONAL && IsState(state_, AlState::INIT))
  {
    RequestState(AlState::PRE_OPERATIONAL);
    if (!IsState(state_, AlState::PRE_OPERATIONAL))
    {
      return;
    }
  }

  const AlState current = state_;
  switch (requested)
  {
    case AlState::INIT:
      StopProcessData();
      StopMailbox();
      al_error_ = AlError::NONE;
      CommitState(AlState::INIT);
      return;

    case AlState::PRE_OPERATIONAL:
      if (!IsState(current, AlState::INIT) &&
          !IsState(current, AlState::PRE_OPERATIONAL) &&
          !IsState(current, AlState::SAFE_OPERATIONAL) &&
          !IsState(current, AlState::OPERATIONAL))
      {
        Fail(current, AlError::INVALID_STATE_CHANGE);
        return;
      }
      StopProcessData();
      if (IsState(current, AlState::INIT) && !StartMailbox())
      {
        Fail(AlState::INIT, AlError::INVALID_MAILBOX_CONFIGURATION);
        return;
      }
      al_error_ = AlError::NONE;
      CommitState(AlState::PRE_OPERATIONAL);
      return;

    case AlState::SAFE_OPERATIONAL:
      if (IsState(current, AlState::OPERATIONAL))
      {
        StopOutputs();
        al_error_ = AlError::NONE;
        CommitState(AlState::SAFE_OPERATIONAL);
        return;
      }
      if (!IsState(current, AlState::PRE_OPERATIONAL) &&
          !IsState(current, AlState::SAFE_OPERATIONAL))
      {
        Fail(current, AlError::INVALID_STATE_CHANGE);
        return;
      }
      {
        AlError error = AlError::NONE;
        if (!ValidateProcessDataConfiguration(error) || !StartProcessData())
        {
          Fail(AlState::PRE_OPERATIONAL, error == AlError::NONE
                                             ? AlError::INVALID_SYNC_MANAGER_CONFIGURATION
                                             : error);
          return;
        }
      }
      al_error_ = AlError::NONE;
      CommitState(AlState::SAFE_OPERATIONAL);
      TransferInputs();
      return;

    case AlState::OPERATIONAL:
      if (!IsState(current, AlState::SAFE_OPERATIONAL) &&
          !IsState(current, AlState::OPERATIONAL))
      {
        Fail(current, AlError::INVALID_STATE_CHANGE);
        return;
      }
      {
        AlError error = AlError::NONE;
        if (!ValidateProcessDataConfiguration(error) || !StartProcessData())
        {
          Fail(AlState::PRE_OPERATIONAL, error == AlError::NONE
                                             ? AlError::INVALID_SYNC_MANAGER_CONFIGURATION
                                             : error);
          return;
        }
      }
      al_error_ = AlError::NONE;
      CommitState(AlState::OPERATIONAL);
      return;

    case AlState::BOOTSTRAP:
      Fail(current, AlError::BOOT_NOT_SUPPORTED);
      return;
  }
}

void DeviceCore::CommitState(AlState next_state)
{
  if (next_state != state_)
  {
    const AlState previous_state = state_;
    state_ = next_state;
    composition_.DispatchStateChanged(in_isr_, previous_state, next_state);
  }
  PublishAlStatus();
}

void DeviceCore::Fail(AlState fallback_state, AlError error)
{
  if (fallback_state == AlState::INIT)
  {
    StopProcessData();
    StopMailbox();
  }
  else if (fallback_state == AlState::PRE_OPERATIONAL)
  {
    StopProcessData();
  }
  else if (fallback_state == AlState::SAFE_OPERATIONAL &&
           IsState(state_, AlState::OPERATIONAL))
  {
    StopOutputs();
  }

  if (fallback_state != state_)
  {
    const AlState previous_state = state_;
    state_ = fallback_state;
    composition_.DispatchStateChanged(in_isr_, previous_state, fallback_state);
  }
  al_error_ = error;
  ResetMailboxProtocols();
  PublishAlStatus();
}

void DeviceCore::PublishAlStatus()
{
  uint8_t status_bytes[2]{};
  uint8_t error_bytes[2]{};
  uint16_t status = static_cast<uint16_t>(state_);
  if (HasAlError())
  {
    status |= AL_STATUS_ERROR;
  }
  WriteLe16(status_bytes, status);
  WriteLe16(error_bytes, static_cast<uint16_t>(al_error_));
  (void)WriteEsc(EscRegister::AL_STATUS_CODE, error_bytes, sizeof(error_bytes));
  (void)WriteEsc(EscRegister::AL_STATUS, status_bytes, sizeof(status_bytes));
}

void DeviceCore::ProcessSyncManagerEvents(uint32_t sync_manager_events)
{
  if (mailbox_.enabled &&
      (sync_manager_events &
       (EscRegister::SyncManagerEvent(mailbox_.request_sync_manager) |
        EscRegister::SyncManagerEvent(mailbox_.response_sync_manager))) != 0U)
  {
    ProcessMailbox(sync_manager_events);
  }
  // Process data is deliberately not driven from here. The SM event bits are not
  // a trustworthy "the master wrote outputs" signal on this hardware (reading AL
  // Event Request clears it, and the PDI's own buffer accesses raise SM events
  // again), so a driver polls it with PollProcessData() once per frame, the same
  // way the reference stack does it from its main loop.
}

void DeviceCore::TransferOutputs()
{
  if (!IsState(state_, AlState::OPERATIONAL) || !process_data_.valid ||
      process_data_.output_size == 0U)
  {
    return;
  }

  uint8_t* buffer = pool_.storage_.process_data;
  if (ReadEsc(process_data_.output_address, buffer, process_data_.output_size) !=
          ErrorCode::OK ||
      composition_.UnpackPdos(in_isr_, ConstRawData(buffer, process_data_.output_size)) !=
          ErrorCode::OK)
  {
    Fail(AlState::SAFE_OPERATIONAL, AlError::NO_VALID_OUTPUTS);
    return;
  }
  composition_.DispatchOutputsUpdated(in_isr_);
}

void DeviceCore::TransferInputs()
{
  if ((!IsState(state_, AlState::SAFE_OPERATIONAL) &&
       !IsState(state_, AlState::OPERATIONAL)) ||
      !process_data_.valid || process_data_.input_size == 0U)
  {
    return;
  }

  composition_.DispatchInputsRequested(in_isr_);
  uint8_t* buffer = pool_.storage_.process_data;
  if (composition_.PackPdos(in_isr_, RawData(buffer, process_data_.input_size)) !=
          ErrorCode::OK ||
      WriteEsc(process_data_.input_address, buffer, process_data_.input_size) !=
          ErrorCode::OK)
  {
    Fail(AlState::PRE_OPERATIONAL, AlError::NO_VALID_INPUTS);
  }
}

void DeviceCore::PollMailbox()
{
  if (!mailbox_.enabled || IsState(state_, AlState::INIT))
  {
    return;
  }
  // Only retry a response that is waiting for the master. Reading the request
  // mailbox here as well would re-answer the request that is still sitting in it
  // every time this is called, which floods the master with unsolicited
  // responses and keeps it from reaching OP.
  (void)FlushMailboxResponse();
  // A request whose SM event was lost (reading AL Event Request clears it) would
  // then sit forever - the master's retries cannot even land, because the ESC
  // refuses a write into a still-full buffer. Feed the request path a synthetic
  // event every poll; the status check inside decides whether one is pending.
  ProcessMailbox(EscRegister::SyncManagerEvent(mailbox_.request_sync_manager));
}

void DeviceCore::PollProcessData()
{
  if (!process_data_.valid || IsState(state_, AlState::INIT) ||
      IsState(state_, AlState::PRE_OPERATIONAL))
  {
    return;
  }
  // TransferOutputs/TransferInputs carry their own state and validity guards.
  // Outputs are read before the inputs are built so the inputs reflect the
  // outputs of this frame, which is the "echo lags one cycle" the master sees.
  if (process_data_.output_size != 0U)
  {
    TransferOutputs();
  }
  if (process_data_.input_size != 0U)
  {
    TransferInputs();
  }
}

void DeviceCore::PollAlControl()
{
  if (IsState(state_, AlState::INIT) && !mailbox_.enabled)
  {
    // Nothing is running yet: the mailbox is only started on the way to PREOP,
    // and ProcessAlControl would still read the register.
  }
  ProcessAlControl();
}

void DeviceCore::ProcessMailbox(uint32_t sync_manager_events)
{
  if (!mailbox_.enabled || IsState(state_, AlState::INIT))
  {
    return;
  }

  if (!FlushMailboxResponse())
  {
    return;
  }
  if ((sync_manager_events &
       EscRegister::SyncManagerEvent(mailbox_.request_sync_manager)) == 0U)
  {
    return;
  }

  // The SM event is advisory; the buffer status is authoritative. The event can
  // arrive before the master's bytes have landed (measured: the request buffer
  // read back as 512 zeros and the request was then silently dropped), and our
  // own buffer accesses raise SM events again. Only a *full* request buffer is
  // read - full means the master wrote the whole message.
  {
    uint8_t request_status = 0;
    const uint16_t status_address = static_cast<uint16_t>(
        EscRegister::SYNC_MANAGER_BASE +
        mailbox_.request_sync_manager * EscRegister::SYNC_MANAGER_SIZE + 5U);
    if (ReadEsc(status_address, &request_status, sizeof(request_status)) !=
            ErrorCode::OK ||
        (request_status & EscRegister::SYNC_MANAGER_STATUS_MAILBOX) == 0U)
    {
      return;
    }
  }

  uint8_t* request = pool_.storage_.mailbox_request;
  // The whole SyncManager buffer has to be read. This ESC does not accept a
  // partial mailbox read: reading only the header (to size the transfer) leaves
  // the buffer status set - measured SM0 status 0x48 and the request never
  // consumed, SDO dead - which is also why a master reads its mailboxes at full
  // SM length. The read cost is therefore inherent, not a choice.
  if (ReadEsc(mailbox_.request.physical_start, request, mailbox_.request.length) !=
      ErrorCode::OK)
  {
    return;
  }

  // The buffer is ours now: release it so the master may write the next request
  // (the ESC refuses a write into a mailbox whose buffer status is still set,
  // and ecx_mbxempty() polls exactly this bit). Released after the read, so the
  // master cannot overwrite the message mid-read.
  (void)SetMailboxBufferStatus(mailbox_.request_sync_manager, false);

  const size_t payload_size = ReadLe16(request);
  if (payload_size > mailbox_.request.length - MAILBOX_HEADER_SIZE)
  {
    SendError(MAILBOX_ERROR_INVALID_SIZE);
    return;
  }
  if (payload_size == 0U)
  {
    return;
  }

  // No duplicate detection here. The mailbox counter alone cannot identify a
  // retransmission on this master: SOEM repeats it for the requests of one SDO
  // transfer, so an initiate and its segments all arrive with the same counter.
  // Treating the segments as duplicates replayed the cached initiate response
  // and the transfer silently transferred nothing. Re-processing a genuinely
  // repeated request is harmless (SDO reads and writes are idempotent) and it is
  // what keeps the state machine honest.
  const uint8_t protocol = static_cast<uint8_t>(request[5] & MAILBOX_PROTOCOL_MASK);
  const uint8_t counter = static_cast<uint8_t>((request[5] >> 4U) & 0x07U);

  // A request we have not seen before proves the master consumed the previous
  // response - the mailbox is a strict request/response pair - so neither the
  // cached response nor a response still waiting to be published may gate this
  // one. Waiting for the master to read (the old check) left the request unread
  // and replayed the stale response on every retry, which is a deadlock.
  mailbox_.response_pending = false;
  mailbox_.last_request_counter = counter;
  mailbox_.last_request_protocol = protocol;
  mailbox_.has_cached_response = false;
  MailboxProtocol* handler = FindMailboxProtocol(protocol);
  if (handler != nullptr)
  {
    handler->Handle(*this, request + MAILBOX_HEADER_SIZE, payload_size);
    return;
  }
  SendError(MAILBOX_ERROR_UNSUPPORTED_PROTOCOL);
}

bool DeviceCore::FlushMailboxResponse()
{
  if (!mailbox_.enabled || !mailbox_.response_pending)
  {
    return true;
  }

  uint8_t status = 0;
  const uint16_t status_address = static_cast<uint16_t>(
      EscRegister::SYNC_MANAGER_BASE +
      mailbox_.response_sync_manager * EscRegister::SYNC_MANAGER_SIZE + 5U);
  if (ReadEsc(status_address, &status, sizeof(status)) != ErrorCode::OK ||
      (status & EscRegister::SYNC_MANAGER_STATUS_MAILBOX) != 0U)
  {
    return false;
  }

  if (WriteEsc(mailbox_.response.physical_start, pool_.storage_.mailbox_response,
               mailbox_.response_size) != ErrorCode::OK)
  {
    return false;
  }

  // Publish the response. The ESC raises the read mailbox's "full" status bit -
  // the one the master polls - when the *last byte* of the SyncManager buffer is
  // written; writing the status register by hand has no effect on this part
  // (measured: it reads back 0x80 either way). The reference AX58400 slave stack
  // does exactly this: write the message, then a single terminating byte at the
  // mailbox end address.
  if (mailbox_.response_size < mailbox_.response.length)
  {
    const uint8_t terminator = 0;
    const uint16_t end_address = static_cast<uint16_t>(mailbox_.response.physical_start +
                                                       mailbox_.response.length - 1U);
    if (WriteEsc(end_address, &terminator, sizeof(terminator)) != ErrorCode::OK)
    {
      return false;
    }
  }
  mailbox_.response_pending = false;
  return true;
}

bool DeviceCore::Respond(uint8_t protocol, size_t payload_size)
{
  if (!mailbox_.enabled || mailbox_.response_pending ||
      payload_size > mailbox_.response.length - MAILBOX_HEADER_SIZE ||
      payload_size + MAILBOX_HEADER_SIZE > pool_.storage_.mailbox_response_capacity)
  {
    return false;
  }

  // Mailbox header, 6 bytes (ETG.1000-4): length of the payload that follows, the
  // address (0 means the master), the channel, the priority and the type byte,
  // whose low nibble is the protocol and whose high nibble is the mailbox counter
  // (1..7, incremented per response so the master can tell them apart).
  uint8_t* buffer = pool_.storage_.mailbox_response;
  mailbox_.response_counter = static_cast<uint8_t>((mailbox_.response_counter % 7U) + 1U);
  WriteLe16(buffer, static_cast<uint16_t>(payload_size));
  buffer[2] = 0;  // address: 0 = master
  buffer[3] = 0;  // channel
  buffer[4] = 0;  // priority: 0 = lowest
  buffer[5] = static_cast<uint8_t>(protocol | (mailbox_.response_counter << 4U));
  mailbox_.response_size = payload_size + MAILBOX_HEADER_SIZE;
  mailbox_.has_cached_response = true;
  mailbox_.response_pending = true;
  (void)FlushMailboxResponse();
  return true;
}

void DeviceCore::SendError(uint16_t error)
{
  uint8_t* payload = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(payload, 0U);
  WriteLe16(payload + 2U, error);
  (void)Respond(MAILBOX_PROTOCOL_ERROR, 4U);
}

bool DeviceCore::RegisterMailboxProtocol(MailboxProtocol& handler)
{
  const uint8_t protocol = handler.Protocol();
  if (protocol == MAILBOX_PROTOCOL_ERROR)
  {
    return false;
  }
  // The wire addresses protocols by the type byte's nibble, so a handler beyond
  // it could never be routed: a contract violation, not a capacity shortage.
  REQUIRE(protocol < MAILBOX_PROTOCOL_COUNT);
  if (protocol >= MAILBOX_PROTOCOL_COUNT || mailbox_protocols_[protocol] != nullptr)
  {
    return false;
  }
  mailbox_protocols_[protocol] = &handler;
  return true;
}

MailboxProtocol* DeviceCore::FindMailboxProtocol(uint8_t protocol)
{
  return protocol < MAILBOX_PROTOCOL_COUNT ? mailbox_protocols_[protocol] : nullptr;
}

void DeviceCore::ResetMailboxProtocols()
{
  for (MailboxProtocol* handler : mailbox_protocols_)
  {
    if (handler != nullptr)
    {
      handler->Reset();
    }
  }
}

RawData DeviceCore::ResponsePayload()
{
  if (!mailbox_.enabled || mailbox_.response.length < MAILBOX_HEADER_SIZE)
  {
    return {};
  }
  return RawData(pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE,
                 mailbox_.response.length - MAILBOX_HEADER_SIZE);
}

ErrorCode DeviceCore::NotifyObjectRead(ObjectAddress address)
{
  return composition_.DispatchObjectRead(in_isr_, address);
}

ErrorCode DeviceCore::NotifyObjectWrite(ObjectAddress address)
{
  return composition_.DispatchObjectWrite(in_isr_, address);
}

}  // namespace LibXR::EtherCAT
