# XRECAT

XRECAT is the LibXR C++ facade for the SOES EtherCAT slave stack. It provides a
header-only `SlaveClass` application interface and a compiled `SlaveCore` runtime;
it does not select a profile, define an object dictionary, compile SOES, or contain an
ESC/PDI port.

Include `slave_class.hpp` and `slave_core.hpp`, derive an application/board-driver
class from `LibXR::EtherCAT::SlaveClass`, then construct `SlaveCore` with that completed
device object. The `SlaveCore` constructor starts SOES immediately; no later lifecycle
call or state flag exists. `Options` only controls the watchdog. The board driver
supplies the SOES sources and ESC/PDI port, then calls `SlaveCore::HandleInterrupt()`
from its ESC IRQ path. It owns any SOES process-data scheduling required by that port.

The standalone CMake file exports this include directory and builds `slave_core.cpp`
into an existing LibXR target; it never adds SOES protocol or hardware sources.
