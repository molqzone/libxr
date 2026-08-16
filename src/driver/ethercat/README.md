# XRECAT

XRECAT is the LibXR-native EtherCAT device protocol foundation. It has no SOES,
SPI, HPM, object-dictionary generator, or device profile dependency.

A device is composed from `DeviceClass` modules. Each module declares its CoE objects
and PDO mappings through `DeviceBuilder`; `DeviceComposition` merges them using caller-
owned fixed storage from `StaticDevicePool`. `DeviceCore` owns the completed composition
and accepts protocol events from a non-owning `EscPort` supplied by a board driver.

Public headers are grouped by responsibility:

- `core/object_dictionary.hpp`, `core/pdo.hpp`, and `core/al_state.hpp` define
  protocol data.
- `core/esc_port.hpp` defines the hardware-independent ESC register interface.
- `device/device_class.hpp`, `device/device_pool.hpp`, `device/device_composition.hpp`,
  and `device/device_core.hpp` define the device composition and runtime core.

`DeviceCore` now implements the IRQ-driven baseline needed by a fixed-PDO device:

- the AL state machine and AL status/error registers;
- ESC capability discovery plus Sync Manager/FMMU validation for the configured
  buffered process-data channels, without assigning protocol meaning to SM numbers;
- bit-accurate PDO transfer between SM PDRAM and declared object storage;
- optional mailbox-mode request/response pair with CoE SDO expedited and segmented
  upload/download. Responses are retained for a mailbox retry, not modeled as USB endpoints.

`StaticDevicePool` owns fixed PDRAM and separate mailbox request/response scratch
buffers in addition to composition storage. Its final two template parameters control
the PDRAM and per-direction mailbox capacities (both default to 128 bytes).

The remaining protocol work is dynamic PDO assignment, CoE SDO information
services, distributed clocks, EEPROM/SII handling, and additional mailbox protocols.
