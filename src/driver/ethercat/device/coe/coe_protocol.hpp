#pragma once

#include <cstddef>
#include <cstdint>

#include "core/mailbox.hpp"
#include "core/object_dictionary.hpp"

namespace LibXR::EtherCAT
{

/**
 * The CoE mailbox protocol: one SDO server over the composed object dictionary.
 *
 * It answers SDO expedited and segmented upload/download for every entry the
 * DeviceClasses declared, with the per-transfer state (segment toggle, offset,
 * remaining size) owned here and dropped through Reset() when the mailbox or
 * the device state restarts under it. SDO Information and complete access are
 * answered with the protocol's "not supported" codes.
 *
 * Responses are built in MailboxExchange::ResponsePayload() and retained by the
 * exchange for a mailbox retry; a response that could not be published must not
 * leave a transfer marked as running (see the segmented handlers).
 */
class CoeProtocol final : public MailboxProtocol
{
 public:
  explicit CoeProtocol(const ObjectDictionary& dictionary) : dictionary_(dictionary) {}

  CoeProtocol(const CoeProtocol&) = delete;
  CoeProtocol& operator=(const CoeProtocol&) = delete;
  CoeProtocol(CoeProtocol&&) = delete;
  CoeProtocol& operator=(CoeProtocol&&) = delete;

  [[nodiscard]] uint8_t Protocol() const override { return MAILBOX_PROTOCOL_COE; }

  /** One SDO initiate request/response (CoE header + 8 bytes). */
  [[nodiscard]] size_t MinPayloadSize() const override;

  void Handle(MailboxExchange& channel, const uint8_t* payload,
              size_t payload_size) override;
  void Reset() override;

 private:
  enum class TransferDirection : uint8_t
  {
    NONE,
    UPLOAD,
    DOWNLOAD
  };

  struct Transfer
  {
    TransferDirection direction = TransferDirection::NONE;
    const ObjectEntry* entry = nullptr;
    size_t size = 0;
    size_t offset = 0;
    bool toggle = false;
  };

  void ProcessSdoUpload(MailboxExchange& channel, const uint8_t* payload,
                        size_t payload_size);
  void ProcessSdoDownload(MailboxExchange& channel, const uint8_t* payload,
                          size_t payload_size);
  void ProcessSdoUploadSegment(MailboxExchange& channel, const uint8_t* payload,
                               size_t payload_size);
  void ProcessSdoDownloadSegment(MailboxExchange& channel, const uint8_t* payload,
                                 size_t payload_size);
  void SendSdoAbort(MailboxExchange& channel, uint16_t index, uint8_t subindex,
                    uint32_t abort_code);
  [[nodiscard]] bool SendSdoDownloadResponse(MailboxExchange& channel, uint16_t index,
                                             uint8_t subindex);
  [[nodiscard]] bool IsObjectReadable(const ObjectEntry& entry, AlState state) const;
  [[nodiscard]] bool IsObjectWritable(const ObjectEntry& entry, AlState state) const;
  [[nodiscard]] size_t ObjectSize(const ObjectEntry& entry) const;

  const ObjectDictionary& dictionary_;
  Transfer transfer_{};
};

}  // namespace LibXR::EtherCAT
