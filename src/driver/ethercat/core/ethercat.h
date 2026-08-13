#ifndef LIBXR_ETHERCAT_H
#define LIBXR_ETHERCAT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the configured SOES EtherCAT slave profile and ESC port. */
void ethercat_slave_init(void);

/** Run the EtherCAT slave state machine from the application scheduler. */
void ethercat_slave_poll(void);

/** SOES profile hooks called by the C++ LibXR peripheral bridge. */
void ethercat_profile_get_inputs(void);
void ethercat_profile_set_outputs(void);
void ethercat_profile_safe_outputs(void);
void ethercat_safe_outputs(void);

/** Mark the current call chain as originating in the ESC interrupt handler. */
void ethercat_slave_enter_isr(void);

/** Leave an ESC interrupt-originated call chain. */
void ethercat_slave_leave_isr(void);

/** Return whether a SOES callback currently runs in an interrupt context. */
bool ethercat_slave_in_isr(void);

/* Compatibility entry points used by the existing HPM application. */
void rmgo_ecat_init(void);
void rmgo_ecat_poll_1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* LIBXR_ETHERCAT_H */
