#pragma once

#include "device/peripheral/peripheral.hpp"
#include "device/slave/slave.hpp"

/**
 * Public LibXR EtherCAT slave API.
 *
 * Create application-owned peripheral bindings, register them in ProcessDataPool, then
 * construct one Slave with that pool. SOES invokes the pool through its process-data
 * callbacks; board-specific ESC/PDI code remains below this interface.
 */
