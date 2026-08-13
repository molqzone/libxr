#include "profile.h"

#include <string.h>

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

void ethercat_profile_init(void) {}

void ethercat_profile_on_cycle(void)
{
    g_rmgo_ecat_debug_app_hook_count++;
    (void)Obj.controlword;
}

void ethercat_profile_get_inputs(void)
{
    g_rmgo_ecat_debug_cb_get_inputs_count++;
}

void ethercat_profile_set_outputs(void)
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

void ethercat_profile_safe_outputs(void)
{
    memset(Obj.can0_motor_commnads, 0, sizeof(Obj.can0_motor_commnads));
    memset(Obj.can1_motor_commnads, 0, sizeof(Obj.can1_motor_commnads));
    Obj.digital_outputs = 0;
}
