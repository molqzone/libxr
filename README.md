# XRECAT

XRECAT is the LibXR C++ facade for the SOES EtherCAT slave stack. It is deliberately
header-only: it does not select a profile, define an object dictionary, compile SOES,
or contain an ESC/PDI port.

Include `slave_class.hpp` (or the `ethercat.hpp` umbrella), construct
`LibXR::EtherCAT::SlaveClass` with an `esc_cfg_t`, and call `Initialize`, `Poll`,
`Worker`, `Process`, or `Run` from the application/driver. The application or board
driver supplies the SOES headers and sources, ESC callbacks, and the global
`cb_get_inputs`/`cb_set_outputs` process-data callbacks.

The standalone CMake file only exports this include directory to an existing LibXR
target; it never adds protocol or hardware sources.

