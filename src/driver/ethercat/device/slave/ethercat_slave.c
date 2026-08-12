#include "ethercat.h"

#include "ecat_slv.h"
#include "esc_hw.h"
#include "profile.h"

static void ethercat_application_hook(void)
{
    ethercat_profile_on_cycle();
}

static esc_cfg_t config = {
    .user_arg = "hpm5e31",
    .use_interrupt = 1,
    .watchdog_cnt = 200,
    .application_hook = ethercat_application_hook,
    .esc_hw_interrupt_enable = ESC_interrupt_enable,
    .esc_hw_interrupt_disable = ESC_interrupt_disable,
    .esc_hw_eep_handler = ESC_eeprom_emulation_handler,
    .esc_check_dc_handler = ESC_check_dc,
    .safeoutput_override = ethercat_profile_safe_outputs,
};

void ethercat_slave_init(void)
{
    ethercat_profile_init();
    ecat_slv_init(&config);
    rmgo_ecat_debug_mark_poll();
}

void ethercat_slave_poll(void)
{
    DIG_process(DIG_PROCESS_WD_FLAG);
    ecat_slv_poll();
    rmgo_ecat_debug_mark_poll();
}

void rmgo_ecat_init(void)
{
    ethercat_slave_init();
}

void rmgo_ecat_poll_1ms(void)
{
    ethercat_slave_poll();
}
