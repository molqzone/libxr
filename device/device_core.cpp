#include "device_core.hpp"

#include <cstring>
#include <limits>

namespace LibXR::EtherCAT
{

namespace
{

constexpr uint8_t MAILBOX_COE = 0x03U;
constexpr uint8_t MAILBOX_ERROR = 0x00U;

constexpr uint16_t MAILBOX_ERROR_UNSUPPORTED_PROTOCOL = 0x0002U;
constexpr uint16_t MAILBOX_ERROR_SERVICE_NOT_SUPPORTED = 0x0004U;
constexpr uint16_t MAILBOX_ERROR_INVALID_HEADER = 0x0005U;
constexpr uint16_t MAILBOX_ERROR_INVALID_SIZE = 0x0008U;

constexpr uint16_t COE_SDO_REQUEST = 0x02U;
constexpr uint16_t COE_SDO_RESPONSE = 0x03U;

constexpr uint8_t SDO_ABORT = 0x80U;
constexpr uint8_t SDO_UPLOAD_REQUEST = 0x40U;
constexpr uint8_t SDO_UPLOAD_RESPONSE = 0x40U;
constexpr uint8_t SDO_UPLOAD_SEGMENT_REQUEST = 0x60U;
constexpr uint8_t SDO_DOWNLOAD_REQUEST = 0x20U;
constexpr uint8_t SDO_DOWNLOAD_RESPONSE = 0x60U;
constexpr uint8_t SDO_DOWNLOAD_SEGMENT_RESPONSE = 0x20U;
constexpr uint8_t SDO_EXPEDITED = 0x02U;
constexpr uint8_t SDO_SIZE_INDICATED = 0x01U;
constexpr uint8_t SDO_TOGGLE = 0x10U;
constexpr uint8_t SDO_LAST_SEGMENT = 0x01U;
constexpr uint8_t SDO_COMPLETE_ACCESS = 0x10U;

constexpr uint32_t SDO_ABORT_TOGGLE = 0x05030000U;
constexpr uint32_t SDO_ABORT_TIMEOUT = 0x05040000U;
constexpr uint32_t SDO_ABORT_UNSUPPORTED = 0x06010000U;
constexpr uint32_t SDO_ABORT_WRITE_ONLY = 0x06010001U;
constexpr uint32_t SDO_ABORT_READ_ONLY = 0x06010002U;
constexpr uint32_t SDO_ABORT_TYPE_MISMATCH = 0x06070010U;
constexpr uint32_t SDO_ABORT_NO_OBJECT = 0x06020000U;
constexpr uint32_t SDO_ABORT_GENERAL = 0x08000000U;

constexpr size_t MAILBOX_HEADER_SIZE = 6U;
constexpr size_t COE_HEADER_SIZE = 2U;
constexpr size_t SDO_INITIATE_SIZE = 8U;
constexpr size_t SDO_INITIATE_PAYLOAD_SIZE = COE_HEADER_SIZE + SDO_INITIATE_SIZE;
constexpr size_t SDO_SEGMENT_HEADER_SIZE = COE_HEADER_SIZE + 1U;

constexpr size_t BytesForBits(size_t bit_count) { return (bit_count + 7U) / 8U; }

bool IsState(AlState value, AlState expected) { return value == expected; }

}  // namespace

DeviceCore::DeviceCore(EscPort& port, DevicePool& pool, std::initializer_list<DeviceClass*> classes)
    : port_(port), pool_(pool), composition_(pool, classes)
{
  PublishAlStatus();
}

void DeviceCore::HandleInterrupt(EscEvent events)
{
  if (HasEvent(events, EscEvent::AL_CONTROL))
  {
    ProcessAlControl();
  }

  if (HasEvent(events, EscEvent::SYNC_MANAGER_CHANGE))
  {
    AlError error = AlError::NONE;
    if (IsState(state_, AlState::SAFE_OPERATIONAL) || IsState(state_, AlState::OPERATIONAL))
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
    (void)ProcessSyncManagerEvents();
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

uint16_t DeviceCore::ReadLe16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t DeviceCore::ReadLe32(const uint8_t* data)
{
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

void DeviceCore::WriteLe16(uint8_t* data, uint16_t value)
{
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8U);
}

void DeviceCore::WriteLe32(uint8_t* data, uint32_t value)
{
  data[0] = static_cast<uint8_t>(value);
  data[1] = static_cast<uint8_t>(value >> 8U);
  data[2] = static_cast<uint8_t>(value >> 16U);
  data[3] = static_cast<uint8_t>(value >> 24U);
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
  uint8_t supported_channels[2]{};
  if (ReadEsc(EscRegister::FMMU_COUNT, supported_channels, sizeof(supported_channels)) != ErrorCode::OK)
  {
    return ErrorCode::FAILED;
  }
  if (supported_channels[0] > fmmus_.size() || supported_channels[1] > sync_managers_.size())
  {
    return ErrorCode::OUT_OF_RANGE;
  }
  fmmu_count_ = supported_channels[0];
  sync_manager_count_ = supported_channels[1];

  for (uint8_t index = 0; index < sync_manager_count_; ++index)
  {
    uint8_t bytes[EscRegister::SYNC_MANAGER_SIZE]{};
    const uint16_t address =
        static_cast<uint16_t>(EscRegister::SYNC_MANAGER_BASE + index * EscRegister::SYNC_MANAGER_SIZE);
    const ErrorCode result = ReadEsc(address, bytes, sizeof(bytes));
    if (result != ErrorCode::OK)
    {
      return result;
    }

    sync_managers_[index] = {ReadLe16(bytes), ReadLe16(bytes + 2U), bytes[4], bytes[5], bytes[6], bytes[7]};
  }

  for (uint8_t index = sync_manager_count_; index < sync_managers_.size(); ++index)
  {
    sync_managers_[index] = {};
  }

  for (uint8_t index = 0; index < fmmu_count_; ++index)
  {
    uint8_t bytes[EscRegister::FMMU_SIZE]{};
    const uint16_t address = static_cast<uint16_t>(EscRegister::FMMU_BASE + index * EscRegister::FMMU_SIZE);
    const ErrorCode result = ReadEsc(address, bytes, sizeof(bytes));
    if (result != ErrorCode::OK)
    {
      return result;
    }

    fmmus_[index] = {
        ReadLe32(bytes), ReadLe16(bytes + 4U), bytes[6], bytes[7], ReadLe16(bytes + 8U), bytes[10], bytes[11],
        bytes[12]};
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
  const uint16_t address =
      static_cast<uint16_t>(EscRegister::SYNC_MANAGER_BASE + index * EscRegister::SYNC_MANAGER_SIZE + 6U);
  const ErrorCode result = WriteEsc(address, &activate, sizeof(activate));
  if (result == ErrorCode::OK)
  {
    sync_managers_[index].activate = activate;
  }
  return result;
}

const DeviceCore::SyncManager* DeviceCore::FindSyncManager(uint8_t operation_mode, uint8_t direction,
                                                           uint8_t* index, bool require_nonzero_length) const
{
  for (uint8_t candidate = 0; candidate < sync_manager_count_; ++candidate)
  {
    const SyncManager& sync_manager = sync_managers_[candidate];
    if ((sync_manager.control & EscRegister::SYNC_MANAGER_OPERATION_MODE_MASK) == operation_mode &&
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
  const SyncManager* request = FindSyncManager(EscRegister::SYNC_MANAGER_MAILBOX_MODE,
                                               EscRegister::SYNC_MANAGER_ECAT_WRITE, &request_index);
  const SyncManager* response = FindSyncManager(EscRegister::SYNC_MANAGER_MAILBOX_MODE,
                                                EscRegister::SYNC_MANAGER_ECAT_READ, &response_index);
  if (request == nullptr && response == nullptr)
  {
    mailbox_ = {};
    return true;
  }
  if (request == nullptr || response == nullptr || request->length == 0U || response->length == 0U)
  {
    error = AlError::INVALID_MAILBOX_CONFIGURATION;
    return false;
  }

  const uint32_t request_end = static_cast<uint32_t>(request->physical_start) + request->length;
  const uint32_t response_end = static_cast<uint32_t>(response->physical_start) + response->length;
  if (request->length < MAILBOX_HEADER_SIZE + SDO_INITIATE_PAYLOAD_SIZE ||
      response->length < MAILBOX_HEADER_SIZE + SDO_INITIATE_PAYLOAD_SIZE || request_end > 0x10000U ||
      response_end > 0x10000U || pool_.storage_.mailbox_request == nullptr ||
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
    const uint32_t fmmu_end = static_cast<uint32_t>(fmmu.physical_start) + fmmu.logical_length;
    if ((fmmu.activate & EscRegister::FMMU_ENABLE) != 0U && (fmmu.type & required_type) != 0U &&
        fmmu.physical_start_bit == 0U && fmmu.physical_start <= physical_start && physical_end <= fmmu_end)
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
  const size_t process_data_capacity = output_size > input_size ? output_size : input_size;
  if (output_size > std::numeric_limits<uint16_t>::max() ||
      input_size > std::numeric_limits<uint16_t>::max() ||
      (process_data_capacity != 0U && (pool_.storage_.process_data == nullptr ||
                                       pool_.storage_.process_data_capacity < process_data_capacity)))
  {
    error = AlError::INVALID_OUTPUT_MAPPING;
    return false;
  }

  uint8_t output_index = 0;
  const SyncManager* output =
      output_size == 0U ? nullptr
                        : FindSyncManager(EscRegister::SYNC_MANAGER_BUFFERED_MODE,
                                          EscRegister::SYNC_MANAGER_ECAT_WRITE, &output_index, true);
  if (output_size != 0U &&
      (output == nullptr || output->length != output_size ||
       FindFmmu(output->physical_start, output_size, EscRegister::FMMU_WRITE_ENABLE) == nullptr))
  {
    error = output == nullptr || output->length != output_size ? AlError::INVALID_OUTPUT_SYNC_MANAGER
                                                               : AlError::INVALID_OUTPUT_MAPPING;
    return false;
  }

  uint8_t input_index = 0;
  const SyncManager* input = input_size == 0U
                                 ? nullptr
                                 : FindSyncManager(EscRegister::SYNC_MANAGER_BUFFERED_MODE,
                                                   EscRegister::SYNC_MANAGER_ECAT_READ, &input_index, true);
  if (input_size != 0U &&
      (input == nullptr || input->length != input_size ||
       FindFmmu(input->physical_start, input_size, EscRegister::FMMU_READ_ENABLE) == nullptr))
  {
    error = input == nullptr || input->length != input_size ? AlError::INVALID_INPUT_SYNC_MANAGER
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
  return SetSyncManagerEnabled(mailbox_.request_sync_manager, true) == ErrorCode::OK &&
         SetSyncManagerEnabled(mailbox_.response_sync_manager, true) == ErrorCode::OK;
}

void DeviceCore::StopMailbox()
{
  if (mailbox_.enabled)
  {
    (void)SetSyncManagerEnabled(mailbox_.request_sync_manager, false);
    (void)SetSyncManagerEnabled(mailbox_.response_sync_manager, false);
  }
  mailbox_ = {};
  ResetSdoTransfer();
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
    if (!IsState(current, AlState::INIT) && !IsState(current, AlState::PRE_OPERATIONAL) &&
        !IsState(current, AlState::SAFE_OPERATIONAL) && !IsState(current, AlState::OPERATIONAL))
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
    if (!IsState(current, AlState::PRE_OPERATIONAL) && !IsState(current, AlState::SAFE_OPERATIONAL))
    {
      Fail(current, AlError::INVALID_STATE_CHANGE);
      return;
    }
    {
      AlError error = AlError::NONE;
      if (!ValidateProcessDataConfiguration(error) || !StartProcessData())
      {
        Fail(AlState::PRE_OPERATIONAL,
             error == AlError::NONE ? AlError::INVALID_SYNC_MANAGER_CONFIGURATION : error);
        return;
      }
    }
    al_error_ = AlError::NONE;
    CommitState(AlState::SAFE_OPERATIONAL);
    TransferInputs();
    return;

  case AlState::OPERATIONAL:
    if (!IsState(current, AlState::SAFE_OPERATIONAL) && !IsState(current, AlState::OPERATIONAL))
    {
      Fail(current, AlError::INVALID_STATE_CHANGE);
      return;
    }
    {
      AlError error = AlError::NONE;
      if (!ValidateProcessDataConfiguration(error) || !StartProcessData())
      {
        Fail(AlState::PRE_OPERATIONAL,
             error == AlError::NONE ? AlError::INVALID_SYNC_MANAGER_CONFIGURATION : error);
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
    composition_.DispatchStateChanged(previous_state, next_state);
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
  else if (fallback_state == AlState::SAFE_OPERATIONAL && IsState(state_, AlState::OPERATIONAL))
  {
    StopOutputs();
  }

  if (fallback_state != state_)
  {
    const AlState previous_state = state_;
    state_ = fallback_state;
    composition_.DispatchStateChanged(previous_state, fallback_state);
  }
  al_error_ = error;
  ResetSdoTransfer();
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

uint32_t DeviceCore::ProcessSyncManagerEvents()
{
  uint8_t bytes[4]{};
  if (ReadEsc(EscRegister::AL_EVENT_REQUEST, bytes, sizeof(bytes)) != ErrorCode::OK)
  {
    return 0U;
  }

  const uint32_t request = ReadLe32(bytes);
  if (mailbox_.enabled && (request & (EscRegister::SyncManagerEvent(mailbox_.request_sync_manager) |
                                      EscRegister::SyncManagerEvent(mailbox_.response_sync_manager))) != 0U)
  {
    ProcessMailbox(request);
  }
  if (process_data_.valid && process_data_.output_size != 0U &&
      (request & EscRegister::SyncManagerEvent(process_data_.output_sync_manager)) != 0U)
  {
    TransferOutputs();
  }
  if (process_data_.valid && process_data_.input_size != 0U &&
      (request & EscRegister::SyncManagerEvent(process_data_.input_sync_manager)) != 0U)
  {
    TransferInputs();
  }
  return request;
}

void DeviceCore::TransferOutputs()
{
  if (!IsState(state_, AlState::OPERATIONAL) || !process_data_.valid || process_data_.output_size == 0U)
  {
    return;
  }

  uint8_t* buffer = pool_.storage_.process_data;
  if (ReadEsc(process_data_.output_address, buffer, process_data_.output_size) != ErrorCode::OK ||
      composition_.UnpackPdos(ConstRawData(buffer, process_data_.output_size)) != ErrorCode::OK)
  {
    Fail(AlState::SAFE_OPERATIONAL, AlError::NO_VALID_OUTPUTS);
    return;
  }
  composition_.DispatchOutputsUpdated();
}

void DeviceCore::TransferInputs()
{
  if ((!IsState(state_, AlState::SAFE_OPERATIONAL) && !IsState(state_, AlState::OPERATIONAL)) ||
      !process_data_.valid || process_data_.input_size == 0U)
  {
    return;
  }

  composition_.DispatchInputsRequested();
  uint8_t* buffer = pool_.storage_.process_data;
  if (composition_.PackPdos(RawData(buffer, process_data_.input_size)) != ErrorCode::OK ||
      WriteEsc(process_data_.input_address, buffer, process_data_.input_size) != ErrorCode::OK)
  {
    Fail(AlState::PRE_OPERATIONAL, AlError::NO_VALID_INPUTS);
  }
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
  if (mailbox_.has_cached_response)
  {
    uint8_t status = 0;
    const uint16_t status_address =
        static_cast<uint16_t>(EscRegister::SYNC_MANAGER_BASE +
                              mailbox_.response_sync_manager * EscRegister::SYNC_MANAGER_SIZE + 5U);
    if (ReadEsc(status_address, &status, sizeof(status)) != ErrorCode::OK ||
        (status & EscRegister::SYNC_MANAGER_STATUS_MAILBOX) != 0U)
    {
      return;
    }
  }
  if ((sync_manager_events & EscRegister::SyncManagerEvent(mailbox_.request_sync_manager)) == 0U)
  {
    return;
  }

  uint8_t* request = pool_.storage_.mailbox_request;
  if (ReadEsc(mailbox_.request.physical_start, request, mailbox_.request.length) != ErrorCode::OK)
  {
    return;
  }

  const size_t payload_size = ReadLe16(request);
  if (payload_size > mailbox_.request.length - MAILBOX_HEADER_SIZE)
  {
    SendMailboxError(MAILBOX_ERROR_INVALID_SIZE);
    return;
  }
  if (payload_size == 0U)
  {
    return;
  }

  const uint8_t protocol = static_cast<uint8_t>(request[5] & 0x0FU);
  const uint8_t counter = static_cast<uint8_t>((request[5] >> 4U) & 0x07U);
  if (counter != 0U && mailbox_.has_cached_response && counter == mailbox_.last_request_counter &&
      protocol == mailbox_.last_request_protocol)
  {
    mailbox_.response_pending = true;
    (void)FlushMailboxResponse();
    return;
  }

  mailbox_.last_request_counter = counter;
  mailbox_.last_request_protocol = protocol;
  mailbox_.has_cached_response = false;
  if (protocol == MAILBOX_COE)
  {
    ProcessCoe(request + MAILBOX_HEADER_SIZE, payload_size);
    return;
  }
  SendMailboxError(MAILBOX_ERROR_UNSUPPORTED_PROTOCOL);
}

bool DeviceCore::FlushMailboxResponse()
{
  if (!mailbox_.enabled || !mailbox_.response_pending)
  {
    return true;
  }

  uint8_t status = 0;
  const uint16_t status_address = static_cast<uint16_t>(
      EscRegister::SYNC_MANAGER_BASE + mailbox_.response_sync_manager * EscRegister::SYNC_MANAGER_SIZE + 5U);
  if (ReadEsc(status_address, &status, sizeof(status)) != ErrorCode::OK ||
      (status & EscRegister::SYNC_MANAGER_STATUS_MAILBOX) != 0U)
  {
    return false;
  }

  if (WriteEsc(mailbox_.response.physical_start, pool_.storage_.mailbox_response, mailbox_.response_size) !=
      ErrorCode::OK)
  {
    return false;
  }
  mailbox_.response_pending = false;
  return true;
}

bool DeviceCore::QueueMailboxResponse(uint8_t protocol, size_t payload_size)
{
  if (!mailbox_.enabled || mailbox_.response_pending ||
      payload_size > mailbox_.response.length - MAILBOX_HEADER_SIZE ||
      payload_size + MAILBOX_HEADER_SIZE > pool_.storage_.mailbox_response_capacity)
  {
    return false;
  }

  uint8_t* buffer = pool_.storage_.mailbox_response;
  mailbox_.response_counter = static_cast<uint8_t>((mailbox_.response_counter % 7U) + 1U);
  WriteLe16(buffer, static_cast<uint16_t>(payload_size));
  buffer[2] = 0;
  buffer[3] = 0;
  buffer[4] = 0;
  buffer[5] = static_cast<uint8_t>(protocol | (mailbox_.response_counter << 4U));
  mailbox_.response_size = payload_size + MAILBOX_HEADER_SIZE;
  mailbox_.has_cached_response = true;
  mailbox_.response_pending = true;
  (void)FlushMailboxResponse();
  return true;
}

void DeviceCore::SendMailboxError(uint16_t error)
{
  uint8_t* payload = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(payload, 0U);
  WriteLe16(payload + 2U, error);
  (void)QueueMailboxResponse(MAILBOX_ERROR, 4U);
}

void DeviceCore::ProcessCoe(const uint8_t* payload, size_t payload_size)
{
  if (payload_size < COE_HEADER_SIZE)
  {
    SendMailboxError(MAILBOX_ERROR_INVALID_HEADER);
    return;
  }

  const uint16_t service = static_cast<uint16_t>(ReadLe16(payload) >> 12U);
  if (service != COE_SDO_REQUEST)
  {
    SendMailboxError(MAILBOX_ERROR_SERVICE_NOT_SUPPORTED);
    return;
  }
  if (payload_size < COE_HEADER_SIZE + 1U)
  {
    SendMailboxError(MAILBOX_ERROR_INVALID_SIZE);
    return;
  }

  const uint8_t command = payload[COE_HEADER_SIZE];
  if ((command & 0xE0U) == SDO_UPLOAD_SEGMENT_REQUEST)
  {
    ProcessSdoUploadSegment(payload, payload_size);
  }
  else if ((command & 0xE0U) == SDO_UPLOAD_REQUEST)
  {
    ProcessSdoUpload(payload, payload_size);
  }
  else if ((command & 0xE0U) == SDO_DOWNLOAD_REQUEST)
  {
    ProcessSdoDownload(payload, payload_size);
  }
  else if ((command & 0xE0U) == 0U)
  {
    ProcessSdoDownloadSegment(payload, payload_size);
  }
  else if (command == SDO_ABORT)
  {
    ResetSdoTransfer();
  }
  else
  {
    SendMailboxError(MAILBOX_ERROR_SERVICE_NOT_SUPPORTED);
  }
}

void DeviceCore::ProcessSdoUpload(const uint8_t* payload, size_t payload_size)
{
  if (payload_size < 6U)
  {
    SendSdoAbort(0U, 0U, SDO_ABORT_GENERAL);
    return;
  }

  const uint8_t command = payload[2];
  const uint16_t index = ReadLe16(payload + 3U);
  const uint8_t subindex = payload[5];
  if ((command & SDO_COMPLETE_ACCESS) != 0U)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_UNSUPPORTED);
    return;
  }
  if (sdo_transfer_.direction != SdoTransferDirection::NONE)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_TIMEOUT);
    return;
  }

  ObjectEntry* entry = composition_.GetObjectDictionary().FindEntry({index, subindex});
  if (entry == nullptr)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_NO_OBJECT);
    return;
  }
  if (!IsObjectReadable(*entry))
  {
    SendSdoAbort(index, subindex, SDO_ABORT_WRITE_ONLY);
    return;
  }
  const size_t size = ObjectSize(*entry);
  if (entry->storage.addr_ == nullptr || entry->storage.size_ < size ||
      composition_.DispatchObjectRead(entry->address) != ErrorCode::OK)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_GENERAL);
    return;
  }

  uint8_t* response = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = static_cast<uint8_t>(SDO_UPLOAD_RESPONSE | SDO_SIZE_INDICATED);
  WriteLe16(response + 3U, index);
  response[5] = subindex;

  const size_t inline_capacity = mailbox_.response.length - MAILBOX_HEADER_SIZE - SDO_INITIATE_PAYLOAD_SIZE;
  if (size <= 4U)
  {
    response[2] = static_cast<uint8_t>(response[2] | SDO_EXPEDITED | ((4U - size) << 2U));
    std::memset(response + 6U, 0, 4U);
    std::memcpy(response + 6U, entry->storage.addr_, size);
    (void)QueueMailboxResponse(MAILBOX_COE, SDO_INITIATE_PAYLOAD_SIZE);
    return;
  }

  WriteLe32(response + 6U, static_cast<uint32_t>(size));
  if (size <= inline_capacity)
  {
    std::memcpy(response + SDO_INITIATE_PAYLOAD_SIZE, entry->storage.addr_, size);
    (void)QueueMailboxResponse(MAILBOX_COE, SDO_INITIATE_PAYLOAD_SIZE + size);
    return;
  }

  if (QueueMailboxResponse(MAILBOX_COE, SDO_INITIATE_PAYLOAD_SIZE))
  {
    sdo_transfer_ = {SdoTransferDirection::UPLOAD, entry, size, 0U, false};
  }
}

void DeviceCore::ProcessSdoDownload(const uint8_t* payload, size_t payload_size)
{
  if (payload_size < 6U)
  {
    SendSdoAbort(0U, 0U, SDO_ABORT_GENERAL);
    return;
  }

  const uint8_t command = payload[2];
  const uint16_t index = ReadLe16(payload + 3U);
  const uint8_t subindex = payload[5];
  if ((command & SDO_COMPLETE_ACCESS) != 0U)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_UNSUPPORTED);
    return;
  }
  if (sdo_transfer_.direction != SdoTransferDirection::NONE)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_TIMEOUT);
    return;
  }

  ObjectEntry* entry = composition_.GetObjectDictionary().FindEntry({index, subindex});
  if (entry == nullptr)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_NO_OBJECT);
    return;
  }
  if (!IsObjectWritable(*entry))
  {
    SendSdoAbort(index, subindex, SDO_ABORT_READ_ONLY);
    return;
  }

  const size_t size = ObjectSize(*entry);
  if (entry->storage.addr_ == nullptr || entry->storage.size_ < size)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_GENERAL);
    return;
  }

  if ((command & SDO_EXPEDITED) != 0U)
  {
    if (payload_size < SDO_INITIATE_PAYLOAD_SIZE)
    {
      SendSdoAbort(index, subindex, SDO_ABORT_TYPE_MISMATCH);
      return;
    }
    const size_t transfer_size = (command & SDO_SIZE_INDICATED) != 0U ? 4U - ((command >> 2U) & 0x03U) : 4U;
    if (transfer_size != size)
    {
      SendSdoAbort(index, subindex, SDO_ABORT_TYPE_MISMATCH);
      return;
    }
    std::memcpy(entry->storage.addr_, payload + 6U, size);
    if (composition_.DispatchObjectWrite(entry->address) != ErrorCode::OK)
    {
      SendSdoAbort(index, subindex, SDO_ABORT_GENERAL);
      return;
    }
    (void)SendSdoDownloadResponse(index, subindex);
    return;
  }

  if ((command & SDO_SIZE_INDICATED) == 0U || payload_size < SDO_INITIATE_PAYLOAD_SIZE)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_TYPE_MISMATCH);
    return;
  }
  if (ReadLe32(payload + 6U) != size)
  {
    SendSdoAbort(index, subindex, SDO_ABORT_TYPE_MISMATCH);
    return;
  }
  if (SendSdoDownloadResponse(index, subindex))
  {
    sdo_transfer_ = {SdoTransferDirection::DOWNLOAD, entry, size, 0U, false};
  }
}

void DeviceCore::ProcessSdoUploadSegment(const uint8_t* payload, size_t payload_size)
{
  if (sdo_transfer_.direction != SdoTransferDirection::UPLOAD || payload_size < 3U)
  {
    SendSdoAbort(0U, 0U, SDO_ABORT_TIMEOUT);
    return;
  }

  const bool toggle = (payload[2] & SDO_TOGGLE) != 0U;
  if (toggle != sdo_transfer_.toggle)
  {
    SendSdoAbort(sdo_transfer_.entry->address.index, sdo_transfer_.entry->address.subindex, SDO_ABORT_TOGGLE);
    ResetSdoTransfer();
    return;
  }

  const size_t available = mailbox_.response.length - MAILBOX_HEADER_SIZE - SDO_SEGMENT_HEADER_SIZE;
  const size_t remaining = sdo_transfer_.size - sdo_transfer_.offset;
  const size_t transfer_size = remaining < available ? remaining : available;
  const bool last = transfer_size == remaining;
  uint8_t* response = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = static_cast<uint8_t>((toggle ? SDO_TOGGLE : 0U) | (last ? SDO_LAST_SEGMENT : 0U));
  if (last && transfer_size < 7U)
  {
    response[2] = static_cast<uint8_t>(response[2] | ((7U - transfer_size) << 1U));
  }
  std::memcpy(response + SDO_SEGMENT_HEADER_SIZE,
              static_cast<const uint8_t*>(sdo_transfer_.entry->storage.addr_) + sdo_transfer_.offset,
              transfer_size);
  if (QueueMailboxResponse(MAILBOX_COE, SDO_SEGMENT_HEADER_SIZE + transfer_size))
  {
    sdo_transfer_.offset += transfer_size;
    if (last)
    {
      ResetSdoTransfer();
    }
    else
    {
      sdo_transfer_.toggle = !sdo_transfer_.toggle;
    }
  }
}

void DeviceCore::ProcessSdoDownloadSegment(const uint8_t* payload, size_t payload_size)
{
  if (sdo_transfer_.direction != SdoTransferDirection::DOWNLOAD || payload_size < 3U)
  {
    SendSdoAbort(0U, 0U, SDO_ABORT_TIMEOUT);
    return;
  }

  const uint8_t command = payload[2];
  const bool toggle = (command & SDO_TOGGLE) != 0U;
  if (toggle != sdo_transfer_.toggle)
  {
    SendSdoAbort(sdo_transfer_.entry->address.index, sdo_transfer_.entry->address.subindex, SDO_ABORT_TOGGLE);
    ResetSdoTransfer();
    return;
  }

  size_t transfer_size = payload_size - SDO_SEGMENT_HEADER_SIZE;
  const bool last = (command & SDO_LAST_SEGMENT) != 0U;
  const size_t unused = last ? ((command >> 1U) & 0x07U) : 0U;
  if (unused > transfer_size || transfer_size - unused > sdo_transfer_.size - sdo_transfer_.offset)
  {
    SendSdoAbort(sdo_transfer_.entry->address.index, sdo_transfer_.entry->address.subindex,
                 SDO_ABORT_TYPE_MISMATCH);
    ResetSdoTransfer();
    return;
  }
  transfer_size -= unused;
  std::memcpy(static_cast<uint8_t*>(sdo_transfer_.entry->storage.addr_) + sdo_transfer_.offset,
              payload + SDO_SEGMENT_HEADER_SIZE, transfer_size);
  sdo_transfer_.offset += transfer_size;

  if (last && sdo_transfer_.offset != sdo_transfer_.size)
  {
    SendSdoAbort(sdo_transfer_.entry->address.index, sdo_transfer_.entry->address.subindex,
                 SDO_ABORT_TYPE_MISMATCH);
    ResetSdoTransfer();
    return;
  }

  uint8_t* response = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = static_cast<uint8_t>(SDO_DOWNLOAD_SEGMENT_RESPONSE | (toggle ? SDO_TOGGLE : 0U));

  if (last && composition_.DispatchObjectWrite(sdo_transfer_.entry->address) != ErrorCode::OK)
  {
    SendSdoAbort(sdo_transfer_.entry->address.index, sdo_transfer_.entry->address.subindex,
                 SDO_ABORT_GENERAL);
    ResetSdoTransfer();
    return;
  }

  if (!QueueMailboxResponse(MAILBOX_COE, SDO_SEGMENT_HEADER_SIZE))
  {
    return;
  }
  if (last)
  {
    ResetSdoTransfer();
  }
  else
  {
    sdo_transfer_.toggle = !sdo_transfer_.toggle;
  }
}

void DeviceCore::SendSdoAbort(uint16_t index, uint8_t subindex, uint32_t abort_code)
{
  uint8_t* response = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = SDO_ABORT;
  WriteLe16(response + 3U, index);
  response[5] = subindex;
  WriteLe32(response + 6U, abort_code);
  (void)QueueMailboxResponse(MAILBOX_COE, SDO_INITIATE_PAYLOAD_SIZE);
}

bool DeviceCore::SendSdoDownloadResponse(uint16_t index, uint8_t subindex)
{
  uint8_t* response = pool_.storage_.mailbox_response + MAILBOX_HEADER_SIZE;
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = SDO_DOWNLOAD_RESPONSE;
  WriteLe16(response + 3U, index);
  response[5] = subindex;
  return QueueMailboxResponse(MAILBOX_COE, 6U);
}

bool DeviceCore::IsObjectReadable(const ObjectEntry& entry) const
{
  switch (state_)
  {
  case AlState::PRE_OPERATIONAL:
    return HasAccess(entry.access, ObjectAccess::READ_PRE_OPERATIONAL);
  case AlState::SAFE_OPERATIONAL:
    return HasAccess(entry.access, ObjectAccess::READ_SAFE_OPERATIONAL);
  case AlState::OPERATIONAL:
    return HasAccess(entry.access, ObjectAccess::READ_OPERATIONAL);
  default:
    return false;
  }
}

bool DeviceCore::IsObjectWritable(const ObjectEntry& entry) const
{
  switch (state_)
  {
  case AlState::PRE_OPERATIONAL:
    return HasAccess(entry.access, ObjectAccess::WRITE_PRE_OPERATIONAL);
  case AlState::SAFE_OPERATIONAL:
    return HasAccess(entry.access, ObjectAccess::WRITE_SAFE_OPERATIONAL);
  case AlState::OPERATIONAL:
    return HasAccess(entry.access, ObjectAccess::WRITE_OPERATIONAL);
  default:
    return false;
  }
}

size_t DeviceCore::ObjectSize(const ObjectEntry& entry) const { return BytesForBits(entry.bit_length); }

void DeviceCore::ResetSdoTransfer() { sdo_transfer_ = {}; }

}  // namespace LibXR::EtherCAT
