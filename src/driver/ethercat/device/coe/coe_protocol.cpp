#include "coe_protocol.hpp"

#include <cstring>

#include "core/byte_order.hpp"

namespace LibXR::EtherCAT
{

namespace
{

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

constexpr size_t COE_HEADER_SIZE = 2U;
constexpr size_t SDO_INITIATE_SIZE = 8U;
constexpr size_t SDO_INITIATE_PAYLOAD_SIZE = COE_HEADER_SIZE + SDO_INITIATE_SIZE;
constexpr size_t SDO_SEGMENT_HEADER_SIZE = COE_HEADER_SIZE + 1U;

constexpr size_t BytesForBits(size_t bit_count) { return (bit_count + 7U) / 8U; }

bool IsValidObjectValue(const ObjectEntry& entry, const uint8_t* value, size_t size)
{
  return entry.type != ObjectDataType::BOOLEAN ||
         (size == 0U || (size == 1U && value[0] <= 1U));
}

}  // namespace

size_t CoeProtocol::MinPayloadSize() const { return SDO_INITIATE_PAYLOAD_SIZE; }

void CoeProtocol::Reset() { transfer_ = {}; }

void CoeProtocol::Handle(MailboxExchange& channel, const uint8_t* payload,
                        size_t payload_size)
{
  if (payload_size < COE_HEADER_SIZE)
  {
    channel.SendError(MAILBOX_ERROR_INVALID_HEADER);
    return;
  }

  // The CoE header's upper nibble: the CoE service number (ETG.1000.6), not to
  // be confused with the mailbox protocol nibble this handler is routed by.
  const uint16_t coe_service = static_cast<uint16_t>(ReadLe16(payload) >> 12U);
  if (coe_service != COE_SDO_REQUEST)
  {
    channel.SendError(MAILBOX_ERROR_SERVICE_NOT_SUPPORTED);
    return;
  }
  if (payload_size < COE_HEADER_SIZE + 1U)
  {
    channel.SendError(MAILBOX_ERROR_INVALID_SIZE);
    return;
  }

  const uint8_t command = payload[COE_HEADER_SIZE];
  if ((command & 0xE0U) == SDO_UPLOAD_SEGMENT_REQUEST)
  {
    ProcessSdoUploadSegment(channel, payload, payload_size);
  }
  else if ((command & 0xE0U) == SDO_UPLOAD_REQUEST)
  {
    ProcessSdoUpload(channel, payload, payload_size);
  }
  else if ((command & 0xE0U) == SDO_DOWNLOAD_REQUEST)
  {
    ProcessSdoDownload(channel, payload, payload_size);
  }
  else if ((command & 0xE0U) == 0U)
  {
    ProcessSdoDownloadSegment(channel, payload, payload_size);
  }
  else if (command == SDO_ABORT)
  {
    Reset();
  }
  else
  {
    channel.SendError(MAILBOX_ERROR_SERVICE_NOT_SUPPORTED);
  }
}

void CoeProtocol::ProcessSdoUpload(MailboxExchange& channel, const uint8_t* payload,
                                  size_t payload_size)
{
  if (payload_size < 6U)
  {
    SendSdoAbort(channel, 0U, 0U, SDO_ABORT_GENERAL);
    return;
  }

  const uint8_t command = payload[2];
  const uint16_t index = ReadLe16(payload + 3U);
  const uint8_t subindex = payload[5];
  if ((command & SDO_COMPLETE_ACCESS) != 0U)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_UNSUPPORTED);
    return;
  }
  if (transfer_.direction != TransferDirection::NONE)
  {
    // The mailbox is a strict request/response pair, so an initiate that arrives
    // while a transfer is still marked as running means that transfer is over:
    // its segments would have been dispatched to the segment handlers instead.
    // Aborting here (0x05040000) made the slave refuse every later SDO once one
    // transfer leaked its state, which is exactly what the master saw.
    Reset();
  }

  const ObjectEntry* entry = dictionary_.FindEntry({index, subindex});
  if (entry == nullptr)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_NO_OBJECT);
    return;
  }
  if (!IsObjectReadable(*entry, channel.GetState()))
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_WRITE_ONLY);
    return;
  }
  const size_t size = ObjectSize(*entry);
  if (entry->storage.addr_ == nullptr || entry->storage.size_ < size ||
      channel.NotifyObjectRead(entry->address) != ErrorCode::OK)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_GENERAL);
    return;
  }

  uint8_t* response = static_cast<uint8_t*>(channel.ResponsePayload().addr_);
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = static_cast<uint8_t>(SDO_UPLOAD_RESPONSE | SDO_SIZE_INDICATED);
  WriteLe16(response + 3U, index);
  response[5] = subindex;

  const size_t inline_capacity =
      channel.ResponsePayload().size_ - SDO_INITIATE_PAYLOAD_SIZE;
  if (size <= 4U)
  {
    response[2] = static_cast<uint8_t>(response[2] | SDO_EXPEDITED | ((4U - size) << 2U));
    std::memset(response + 6U, 0, 4U);
    std::memcpy(response + 6U, entry->storage.addr_, size);
    (void)channel.Respond(MAILBOX_PROTOCOL_COE, SDO_INITIATE_PAYLOAD_SIZE);
    return;
  }

  WriteLe32(response + 6U, static_cast<uint32_t>(size));
  if (size <= inline_capacity)
  {
    std::memcpy(response + SDO_INITIATE_PAYLOAD_SIZE, entry->storage.addr_, size);
    (void)channel.Respond(MAILBOX_PROTOCOL_COE, SDO_INITIATE_PAYLOAD_SIZE + size);
    return;
  }

  if (channel.Respond(MAILBOX_PROTOCOL_COE, SDO_INITIATE_PAYLOAD_SIZE))
  {
    transfer_ = {TransferDirection::UPLOAD, entry, size, 0U, false};
  }
}

void CoeProtocol::ProcessSdoDownload(MailboxExchange& channel, const uint8_t* payload,
                                    size_t payload_size)
{
  if (payload_size < 6U)
  {
    SendSdoAbort(channel, 0U, 0U, SDO_ABORT_GENERAL);
    return;
  }

  const uint8_t command = payload[2];
  const uint16_t index = ReadLe16(payload + 3U);
  const uint8_t subindex = payload[5];
  if ((command & SDO_COMPLETE_ACCESS) != 0U)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_UNSUPPORTED);
    return;
  }
  if (transfer_.direction != TransferDirection::NONE)
  {
    // The mailbox is a strict request/response pair, so an initiate that arrives
    // while a transfer is still marked as running means that transfer is over:
    // its segments would have been dispatched to the segment handlers instead.
    // Aborting here (0x05040000) made the slave refuse every later SDO once one
    // transfer leaked its state, which is exactly what the master saw.
    Reset();
  }

  const ObjectEntry* entry = dictionary_.FindEntry({index, subindex});
  if (entry == nullptr)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_NO_OBJECT);
    return;
  }
  if (!IsObjectWritable(*entry, channel.GetState()))
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_READ_ONLY);
    return;
  }

  const size_t size = ObjectSize(*entry);
  if (entry->storage.addr_ == nullptr || entry->storage.size_ < size)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_GENERAL);
    return;
  }

  if ((command & SDO_EXPEDITED) != 0U)
  {
    if (payload_size < SDO_INITIATE_PAYLOAD_SIZE)
    {
      SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
      return;
    }
    const size_t transfer_size =
        (command & SDO_SIZE_INDICATED) != 0U ? 4U - ((command >> 2U) & 0x03U) : 4U;
    if (transfer_size != size)
    {
      SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
      return;
    }
    if (!IsValidObjectValue(*entry, payload + 6U, size))
    {
      SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
      return;
    }
    std::memcpy(entry->storage.addr_, payload + 6U, size);
    if (channel.NotifyObjectWrite(entry->address) != ErrorCode::OK)
    {
      SendSdoAbort(channel, index, subindex, SDO_ABORT_GENERAL);
      return;
    }
    (void)SendSdoDownloadResponse(channel, index, subindex);
    return;
  }

  if ((command & SDO_SIZE_INDICATED) == 0U || payload_size < SDO_INITIATE_PAYLOAD_SIZE)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
    return;
  }
  if (ReadLe32(payload + 6U) != size)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
    return;
  }

  // A normal (non expedited) download carries data in the initiate frame itself
  // when the object fits the mailbox: the size is at payload[6..9] and the bytes
  // follow right after it. Masters use that for everything up to the mailbox
  // size, so replying without copying these bytes loses the whole transfer.
  const size_t carried = payload_size - SDO_INITIATE_PAYLOAD_SIZE;
  if (carried > size)
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
    return;
  }
  if (!IsValidObjectValue(*entry, payload + SDO_INITIATE_PAYLOAD_SIZE, carried))
  {
    SendSdoAbort(channel, index, subindex, SDO_ABORT_TYPE_MISMATCH);
    return;
  }
  if (carried > 0U)
  {
    std::memcpy(entry->storage.addr_, payload + SDO_INITIATE_PAYLOAD_SIZE, carried);
  }

  if (carried == size)
  {
    if (channel.NotifyObjectWrite(entry->address) != ErrorCode::OK)
    {
      SendSdoAbort(channel, index, subindex, SDO_ABORT_GENERAL);
      return;
    }
    (void)SendSdoDownloadResponse(channel, index, subindex);
    return;
  }

  // Only part of the object arrived: the rest follows as segments.
  if (SendSdoDownloadResponse(channel, index, subindex))
  {
    transfer_ = {TransferDirection::DOWNLOAD, entry, size, carried, false};
  }
}

void CoeProtocol::ProcessSdoUploadSegment(MailboxExchange& channel, const uint8_t* payload,
                                         size_t payload_size)
{
  if (transfer_.direction != TransferDirection::UPLOAD || payload_size < 3U)
  {
    SendSdoAbort(channel, 0U, 0U, SDO_ABORT_TIMEOUT);
    return;
  }

  const bool toggle = (payload[2] & SDO_TOGGLE) != 0U;
  if (toggle != transfer_.toggle)
  {
    SendSdoAbort(channel, transfer_.entry->address.index,
                 transfer_.entry->address.subindex, SDO_ABORT_TOGGLE);
    Reset();
    return;
  }

  const size_t available =
      channel.ResponsePayload().size_ - SDO_SEGMENT_HEADER_SIZE;
  const size_t remaining = transfer_.size - transfer_.offset;
  const size_t transfer_size = remaining < available ? remaining : available;
  const bool last = transfer_size == remaining;
  uint8_t* response = static_cast<uint8_t*>(channel.ResponsePayload().addr_);
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] =
      static_cast<uint8_t>((toggle ? SDO_TOGGLE : 0U) | (last ? SDO_LAST_SEGMENT : 0U));
  if (last && transfer_size < 7U)
  {
    response[2] = static_cast<uint8_t>(response[2] | ((7U - transfer_size) << 1U));
  }
  std::memcpy(response + SDO_SEGMENT_HEADER_SIZE,
              static_cast<const uint8_t*>(transfer_.entry->storage.addr_) +
                  transfer_.offset,
              transfer_size);
  if (channel.Respond(MAILBOX_PROTOCOL_COE, SDO_SEGMENT_HEADER_SIZE + transfer_size))
  {
    transfer_.offset += transfer_size;
    if (last)
    {
      Reset();
    }
    else
    {
      transfer_.toggle = !transfer_.toggle;
    }
  }
  else if (last)
  {
    // Same as for a download: a response that is still waiting for the master
    // must not leave the transfer marked as running.
    Reset();
  }
}

void CoeProtocol::ProcessSdoDownloadSegment(MailboxExchange& channel,
                                           const uint8_t* payload, size_t payload_size)
{
  if (transfer_.direction != TransferDirection::DOWNLOAD || payload_size < 3U)
  {
    SendSdoAbort(channel, 0U, 0U, SDO_ABORT_TIMEOUT);
    return;
  }

  const uint8_t command = payload[2];
  const bool toggle = (command & SDO_TOGGLE) != 0U;
  if (toggle != transfer_.toggle)
  {
    SendSdoAbort(channel, transfer_.entry->address.index,
                 transfer_.entry->address.subindex, SDO_ABORT_TOGGLE);
    Reset();
    return;
  }

  size_t transfer_size = payload_size - SDO_SEGMENT_HEADER_SIZE;
  const bool last = (command & SDO_LAST_SEGMENT) != 0U;
  const size_t unused = last ? ((command >> 1U) & 0x07U) : 0U;
  if (unused > transfer_size ||
      transfer_size - unused > transfer_.size - transfer_.offset)
  {
    SendSdoAbort(channel, transfer_.entry->address.index,
                 transfer_.entry->address.subindex, SDO_ABORT_TYPE_MISMATCH);
    Reset();
    return;
  }
  transfer_size -= unused;
  if (!IsValidObjectValue(*transfer_.entry, payload + SDO_SEGMENT_HEADER_SIZE,
                          transfer_size))
  {
    SendSdoAbort(channel, transfer_.entry->address.index,
                 transfer_.entry->address.subindex, SDO_ABORT_TYPE_MISMATCH);
    Reset();
    return;
  }
  std::memcpy(static_cast<uint8_t*>(transfer_.entry->storage.addr_) + transfer_.offset,
              payload + SDO_SEGMENT_HEADER_SIZE, transfer_size);
  transfer_.offset += transfer_size;

  if (last && transfer_.offset != transfer_.size)
  {
    SendSdoAbort(channel, transfer_.entry->address.index,
                 transfer_.entry->address.subindex, SDO_ABORT_TYPE_MISMATCH);
    Reset();
    return;
  }

  uint8_t* response = static_cast<uint8_t*>(channel.ResponsePayload().addr_);
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] =
      static_cast<uint8_t>(SDO_DOWNLOAD_SEGMENT_RESPONSE | (toggle ? SDO_TOGGLE : 0U));

  if (last && channel.NotifyObjectWrite(transfer_.entry->address) != ErrorCode::OK)
  {
    SendSdoAbort(channel, transfer_.entry->address.index,
                 transfer_.entry->address.subindex, SDO_ABORT_GENERAL);
    Reset();
    return;
  }

  if (!channel.Respond(MAILBOX_PROTOCOL_COE, SDO_SEGMENT_HEADER_SIZE))
  {
    // The response could not be queued because the previous one is still waiting
    // for the master to read it. The transfer itself is finished either way, so
    // the state must not stay in DOWNLOAD: that would abort every following
    // request with a timeout.
    if (last)
    {
      Reset();
    }
    return;
  }
  if (last)
  {
    Reset();
  }
  else
  {
    transfer_.toggle = !transfer_.toggle;
  }
}

void CoeProtocol::SendSdoAbort(MailboxExchange& channel, uint16_t index, uint8_t subindex,
                              uint32_t abort_code)
{
  uint8_t* response = static_cast<uint8_t*>(channel.ResponsePayload().addr_);
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = SDO_ABORT;
  WriteLe16(response + 3U, index);
  response[5] = subindex;
  WriteLe32(response + 6U, abort_code);
  (void)channel.Respond(MAILBOX_PROTOCOL_COE, SDO_INITIATE_PAYLOAD_SIZE);
}

bool CoeProtocol::SendSdoDownloadResponse(MailboxExchange& channel, uint16_t index,
                                         uint8_t subindex)
{
  uint8_t* response = static_cast<uint8_t*>(channel.ResponsePayload().addr_);
  WriteLe16(response, static_cast<uint16_t>(COE_SDO_RESPONSE << 12U));
  response[2] = SDO_DOWNLOAD_RESPONSE;
  WriteLe16(response + 3U, index);
  response[5] = subindex;
  return channel.Respond(MAILBOX_PROTOCOL_COE, 6U);
}

bool CoeProtocol::IsObjectReadable(const ObjectEntry& entry, AlState state) const
{
  switch (state)
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

bool CoeProtocol::IsObjectWritable(const ObjectEntry& entry, AlState state) const
{
  switch (state)
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

size_t CoeProtocol::ObjectSize(const ObjectEntry& entry) const
{
  return BytesForBits(entry.bit_length);
}

}  // namespace LibXR::EtherCAT
