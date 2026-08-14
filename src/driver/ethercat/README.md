# XRECAT

XRECAT is the LibXR-native EtherCAT slave protocol foundation. It has no SOES,
SPI, HPM, object-dictionary generator, or device profile dependency.

A device is composed from `SlaveClass` modules. Each module declares its CoE objects
and PDO mappings through `SlaveBuilder`; `SlaveComposition` merges them using caller-
owned fixed storage from `StaticSlavePool`. `SlaveCore` owns the completed composition
and accepts protocol events from a non-owning `EscPort` supplied by a board driver.

Public headers are grouped by responsibility:

- `core/object_dictionary.hpp`, `core/pdo.hpp`, and `core/slave_state.hpp` define
  protocol data.
- `core/esc_port.hpp` defines the hardware-independent ESC register interface.
- `device/slave_class.hpp`, `device/slave_pool.hpp`, `device/slave_composition.hpp`,
  and `device/slave_core.hpp` define the device composition and runtime core.

The current core establishes the fixed-capacity composition and IRQ event boundary.
The native AL, mailbox, CoE, PDO, and distributed-clock engines are added on top of
these types without changing the public device model.
