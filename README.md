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

The current core establishes the fixed-capacity composition and IRQ event boundary.
The native AL, mailbox, CoE, PDO, and distributed-clock engines are added on top of
these types without changing the public device model.
