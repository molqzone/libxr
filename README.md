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
- Sync Manager and FMMU validation for SM2/SM3 process data;
- bit-accurate PDO transfer between SM PDRAM and declared object storage;
- optional SM0/SM1 mailbox transport with CoE SDO expedited and segmented upload/download.

`StaticDevicePool` owns fixed PDRAM and mailbox scratch buffers in addition to
composition storage. Its final two template parameters control their capacities
(both default to 128 bytes).

The remaining protocol work is dynamic PDO assignment, CoE SDO information
services, distributed clocks, EEPROM/SII handling, and additional mailbox protocols.
