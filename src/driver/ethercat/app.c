#include "app.h"

#include <string.h>

#include "RMMotorMockBridge.h"
#include "ecat_slv.h"
#include "esc_hw.h"
#include "utypes.h"

_Objects Obj = {
    .serial = 1,
    .statusword = 0,
    .max_temperature = 100,
};

volatile uint32_t g_rmgo_ecat_debug_app_hook_count;
volatile uint32_t g_rmgo_ecat_debug_cb_get_inputs_count;
volatile uint32_t g_rmgo_ecat_debug_cb_set_outputs_count;
volatile int16_t g_rmgo_ecat_debug_can0_command0;
volatile int16_t g_rmgo_ecat_debug_can0_command1;
volatile int16_t g_rmgo_ecat_debug_can0_command2;
volatile int16_t g_rmgo_ecat_debug_can0_command3;
volatile uint8_t g_rmgo_ecat_debug_can0_temperature0;
volatile uint8_t g_rmgo_ecat_debug_max_temperature;

static void ecat_app_hook(void)
{
    g_rmgo_ecat_debug_app_hook_count++;
    rmgo_rmmotor_mock_tick();
    (void)Obj.controlword;
}

void cb_get_inputs(void)
{
    g_rmgo_ecat_debug_cb_get_inputs_count++;
}

void cb_set_outputs(void)
{
    g_rmgo_ecat_debug_cb_set_outputs_count++;
    const int16_t max_temperature = Obj.max_temperature;
    for (size_t i = 0; i < 8U; i++) {
        if (max_temperature > 0 && Obj.can0_motor_temperatures[i] > max_temperature) {
            Obj.can0_motor_commnads[i] = 0;
        }
        if (max_temperature > 0 && Obj.can1_motor_temperatures[i] > max_temperature) {
            Obj.can1_motor_commnads[i] = 0;
        }
    }
    g_rmgo_ecat_debug_can0_command0 = Obj.can0_motor_commnads[0];
    g_rmgo_ecat_debug_can0_command1 = Obj.can0_motor_commnads[1];
    g_rmgo_ecat_debug_can0_command2 = Obj.can0_motor_commnads[2];
    g_rmgo_ecat_debug_can0_command3 = Obj.can0_motor_commnads[3];
    g_rmgo_ecat_debug_can0_temperature0 = Obj.can0_motor_temperatures[0];
    g_rmgo_ecat_debug_max_temperature = Obj.max_temperature;
}

static void safeoutput_override(void)
{
    memset(Obj.can0_motor_commnads, 0, sizeof(Obj.can0_motor_commnads));
    memset(Obj.can1_motor_commnads, 0, sizeof(Obj.can1_motor_commnads));
    Obj.digital_outputs = 0;
}

static esc_cfg_t config = {
    .user_arg = "hpm5e31",
    .use_interrupt = 1,
    .watchdog_cnt = 200,
    .application_hook = ecat_app_hook,
    .esc_hw_interrupt_enable = ESC_interrupt_enable,
    .esc_hw_interrupt_disable = ESC_interrupt_disable,
    .esc_hw_eep_handler = ESC_eeprom_emulation_handler,
    .esc_check_dc_handler = ESC_check_dc,
    .safeoutput_override = safeoutput_override,
};

void rmgo_ecat_init(void)
{
    ecat_slv_init(&config);
    rmgo_ecat_debug_mark_poll();
}

void rmgo_ecat_poll_1ms(void)
{
    DIG_process(DIG_PROCESS_WD_FLAG);
    ecat_slv_poll();
    rmgo_ecat_debug_mark_poll();
}
