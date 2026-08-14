# XRECAT

XRECAT is the LibXR C++ facade for the SOES EtherCAT slave stack. It is deliberately
header-only: it does not select a profile, define an object dictionary, compile SOES,
or contain an ESC/PDI port.

Include `slave_class.hpp`, derive a class from `LibXR::EtherCAT::SlaveClass`, and
override the protected `On...` hooks used by the application or board driver. `Options`
only controls stack defaults; XRECAT installs the SOES callback bridge internally. The
board driver supplies the SOES sources and ESC/PDI port, then calls `HandleInterrupt()`
from its ESC IRQ path. It owns any SOES process-data scheduling required by that port.

The standalone CMake file only exports this include directory to an existing LibXR
target; it never adds protocol or hardware sources.
