# XRECAT

XRECAT is the LibXR-native EtherCAT device protocol foundation. It has no SOES,
SPI, HPM, object-dictionary generator, or device profile dependency.

A device is composed from `DeviceClass` contributors. Each contributor declares its CoE objects
and PDO mappings through `DeviceBuilder`; `DeviceComposition` merges them using caller-
owned fixed storage from `StaticDevicePool`. `DeviceCore` owns the completed composition
and accepts protocol events from a non-owning `EscPort` supplied by a board driver.

Public headers are grouped by responsibility:

- `core/object_dictionary.hpp`, `core/pdo.hpp`, and `core/al_state.hpp` define
  protocol data.
- `core/esc_port.hpp` defines the hardware-independent ESC register interface.
- `core/byte_order.hpp` holds the little-endian wire helpers shared by the core
  and the mailbox protocol handlers.
- `core/mailbox.hpp` defines the mailbox framing (ETG.1000-4), the
  `MailboxProtocol` interface and the `MailboxExchange` a handler replies
  through.
- `device/device_class.hpp`, `device/device_pool.hpp`, `device/device_composition.hpp`,
  and `device/device_core.hpp` define the device composition and runtime core.
- `device/coe/` contains the CoE SDO server and `CoeLayoutDevice`, a helper that
  builds a `DeviceClass` from a compile-time layout table.

`DeviceCore` now implements the IRQ-driven baseline needed by a fixed-PDO device:

- the AL state machine and AL status/error registers;
- ESC capability discovery plus Sync Manager/FMMU validation for the configured
  buffered process-data channels, without assigning protocol meaning to SM numbers;
- bit-accurate PDO transfer between SM PDRAM and declared object storage;
- a protocol-agnostic mailbox: framing, request/response pairing with retained
  responses for a mailbox retry, and routing by protocol number to registered
  `MailboxProtocol`s. CoE is built in (the object dictionary would otherwise be
  unreachable), further mailbox protocols register through
  `DeviceCore::RegisterMailboxProtocol()`.

`CoeProtocol` answers SDO expedited and segmented upload/download for every entry
the composition declared; SDO Information and complete access get the protocol's
"not supported" codes. Per-transfer state lives in the handler and is dropped
through `Reset()` when the mailbox or the device state restarts under it.

`CoeLayoutDevice<Layout>` helps firmware that describes its dictionary as a
compile-time table (e.g. the `EcatLayout` table a layout compiler emits): it
materialises objects, entries and PDOs from the table and owns the per-entry
storage the process image packs and unpacks. The concrete profile implements the
normal `DeviceClass` hooks and binds its own application behavior to that storage.
The table stays a plain LibXR-free constexpr table; the template asserts its
duplicated wire encodings against this stack's enums at compile time. See the
header for the exact table contract.

`StaticDevicePool` owns fixed PDRAM and separate mailbox request/response scratch
buffers in addition to composition storage. Its final two template parameters control
the PDRAM and per-direction mailbox capacities (both default to 128 bytes).

The remaining protocol work is dynamic PDO assignment, CoE SDO information
services, FoE (as another `MailboxProtocol`, whose transfer state model the
protocol interface already provides for), distributed clocks and EEPROM/SII
handling.
