#ifndef LIBXR_ETHERCAT_H
#define LIBXR_ETHERCAT_H

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the configured EtherCAT slave profile and ESC port. */
void ethercat_slave_init(void);

/** Run the EtherCAT slave state machine from the application scheduler. */
void ethercat_slave_poll(void);

/* Compatibility entry points used by the existing HPM application. */
void rmgo_ecat_init(void);
void rmgo_ecat_poll_1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* LIBXR_ETHERCAT_H */
