#ifndef RMGO_ECAT_ESC_HW_H
#define RMGO_ECAT_ESC_HW_H

#include <stdint.h>

#include "esc.h"

#define ESC_updateALevent() \
    do { \
        ESCvar.ALevent = ESC_ALeventread(); \
    } while (0)

#ifdef __cplusplus
extern "C" {
#endif

void ESC_interrupt_enable(uint32_t mask);
void ESC_interrupt_disable(uint32_t mask);
void ESC_eeprom_emulation_handler(void);
uint32_t ESC_enable_DC(void);
int ESC_dc_watchdog_init(void);
uint16_t ESC_check_dc(void);
void rmgo_ecat_debug_mark_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* RMGO_ECAT_ESC_HW_H */
