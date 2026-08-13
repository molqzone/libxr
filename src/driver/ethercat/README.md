# XRECAT

XRECAT is an EtherCAT slave driver subtree for LibXR. It follows the same standalone-repository
and LibXR-subtree layout as XRUSB. SOES owns the EtherCAT slave protocol state machine; XRECAT
adds the LibXR C++ integration layer used to bind PDO fields to LibXR peripherals.

## Layout

- `core/`: C bridge shared by SOES and the C++ facade.
- `device/slave/`: SOES slave lifecycle, profile selection, and the `LibXR::EtherCAT::Slave`
  bridge.
- `device/peripheral/`: LibXR-compatible PDO bindings and fixed `ProcessDataPool` registry.
- `device/slave/profile/`: object dictionaries and application-specific PDO profiles.
- `port/hpm/`: HPM ESC/PDI implementation.

## LibXR Peripheral Bindings

Include `ethercat.hpp`, create bindings for fields in the selected SOES profile, and register
them during application initialization. `ProcessDataPool` mirrors XRUSB's fixed object-pool
approach: it records pointers to application-owned bindings and performs no allocation.

Available bindings are `DigitalInput`, `DigitalOutput`, `AnalogInput`, `PwmOutput`, `CanInput`,
`CanOutput`, and `CallbackBinding`. Input bindings run before SOES packs TxPDO data; output
bindings run after SOES unpacks RxPDO data. SOES safe-output transitions also call each binding's
safe-output action.

The binding layer intentionally does not define a second PDO representation. The selected SOES
profile owns its object dictionary, and a binding receives a reference to one of its PDO fields.
For example, the `rm` profile's digital fields can be connected to LibXR GPIOs as follows:

```cpp
#include "ethercat.hpp"
#include "utypes.h"  // Selected by LIBXR_ETHERCAT_PROFILE.

LibXR::EtherCAT::ProcessDataPool process_data;
LibXR::EtherCAT::DigitalInput input(button, Obj.digital_inputs, 0x01U);
LibXR::EtherCAT::DigitalOutput output(led, Obj.digital_outputs, 0x01U);
LibXR::EtherCAT::Slave slave(process_data);

process_data.Put(&input);
process_data.Put(&output);
slave.Initialize();

// Call from the application scheduler when the HPM ESC port is not servicing an IRQ.
slave.Poll();
```

## CMake

Enable the subtree through LibXR and supply a SOES checkout:

```cmake
set(LIBXR_ETHERCAT_ENABLE ON)
set(LIBXR_ETHERCAT_SOES_DIR "/path/to/SOES/soes")
set(LIBXR_ETHERCAT_PROFILE rm) # rm or foc
```

`rm` is the default profile. Each build selects exactly one profile and its object dictionary.
XRECAT compiles SOES's `ecat_slv`, ESC, CoE, and EEPROM sources directly; the `foc` profile also
enables SOES FoE support.
