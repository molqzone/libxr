#pragma once

#include <cstddef>
#include <cstdint>

#include "core/al_state.hpp"
#include "core/object_dictionary.hpp"

namespace LibXR::EtherCAT
{

// Mailbox framing, ETG.1000-4: a 6-byte header (payload length u16, address,
// channel, priority, type) followed by the protocol payload. The type byte's low
// nibble is the protocol and its high nibble the mailbox counter (1..7), which
// the master uses to tell responses apart.
constexpr size_t MAILBOX_HEADER_SIZE = 6U;

constexpr uint8_t MAILBOX_PROTOCOL_ERROR = 0x00U;
constexpr uint8_t MAILBOX_PROTOCOL_COE = 0x03U;

// The type byte's low nibble carries the protocol number, so the wire format
// itself bounds the protocol space: at most MAILBOX_PROTOCOL_COUNT protocols can
// ever be addressed. A registry indexed by protocol number therefore needs no
// capacity of its own -- an unaddressable protocol cannot be routed anyway.
constexpr uint8_t MAILBOX_PROTOCOL_MASK = 0x0FU;
constexpr size_t MAILBOX_PROTOCOL_COUNT =
    static_cast<size_t>(MAILBOX_PROTOCOL_MASK) + 1U;

// Mailbox-level errors (ETG.1000.6), sent as a protocol-0 reply: two zero bytes
// plus the error detail.
constexpr uint16_t MAILBOX_ERROR_UNSUPPORTED_PROTOCOL = 0x0002U;
constexpr uint16_t MAILBOX_ERROR_SERVICE_NOT_SUPPORTED = 0x0004U;
constexpr uint16_t MAILBOX_ERROR_INVALID_HEADER = 0x0005U;
constexpr uint16_t MAILBOX_ERROR_INVALID_SIZE = 0x0008U;

/**
 * One mailbox request/response exchange, as a mailbox protocol handler sees it.
 *
 * The device core implements this (it owns the response buffer and the object
 * dictionary dispatch), so a handler never touches the ESC directly. Everything
 * a protocol needs beyond its own state is here: the reply side of the mailbox,
 * the state the request is served in (SDO access rights depend on it) and the
 * object access notifications that drive a DeviceClass's OnObjectRead/OnObjectWrite.
 */
class MailboxExchange
{
 public:
  virtual ~MailboxExchange() = default;

  /**
   * The payload area of the next response (mailbox header excluded) and its
   * capacity, which is the send mailbox's payload size: the master cannot
   * receive more than that in one message.
   */
  [[nodiscard]] virtual RawData ResponsePayload() = 0;

  /**
   * Publish a response whose first `payload_size` bytes of ResponsePayload() are
   * valid. The response is retained so a mailbox retry sees it again.
   *
   * @return false when the previous response has not been read yet; the caller
   *         must not assume the payload reached the master.
   */
  [[nodiscard]] virtual bool Respond(uint8_t protocol, size_t payload_size) = 0;

  /** Publish a mailbox-level error reply (protocol 0). */
  virtual void SendError(uint16_t error) = 0;

  /** The AL state the request is being served in. */
  [[nodiscard]] virtual AlState GetState() const = 0;

  /** Notify the owning DeviceClass that an object was read / is being written. */
  [[nodiscard]] virtual ErrorCode NotifyObjectRead(ObjectAddress address) = 0;
  [[nodiscard]] virtual ErrorCode NotifyObjectWrite(ObjectAddress address) = 0;
};

/**
 * One mailbox protocol (CoE today; FoE and friends later), routed by the
 * protocol nibble of the mailbox type byte.
 *
 * A handler owns its transfer state (segmented SDO, a multi-packet file
 * transfer) and is asked to drop it through Reset() whenever the mailbox or the
 * device state restarts under it. The device core answers unknown protocols
 * with MAILBOX_ERROR_UNSUPPORTED_PROTOCOL, so a handler only ever sees its own.
 */
class MailboxProtocol
{
 public:
  virtual ~MailboxProtocol() = default;

  /**
   * The mailbox protocol number this handler answers (0x03 = CoE): the value
   * the type byte's nibble carries, within MAILBOX_PROTOCOL_COUNT.
   */
  [[nodiscard]] virtual uint8_t Protocol() const = 0;

  /**
   * Smallest request payload this protocol can act on. The device core refuses a
   * mailbox configuration that cannot carry one such message, so a transfer
   * cannot fail later for want of space.
   */
  [[nodiscard]] virtual size_t MinPayloadSize() const { return 0U; }

  /** Handle one request payload (mailbox header stripped). */
  virtual void Handle(MailboxExchange& channel, const uint8_t* payload,
                      size_t payload_size) = 0;

  /** Drop any transfer state: the mailbox or the device state restarted. */
  virtual void Reset() {}
};

}  // namespace LibXR::EtherCAT
