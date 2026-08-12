#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   uint16_t can0_motor_positions[8];
   uint16_t can1_motor_positions[8];
   int16_t can0_motor_velocities[8];
   int16_t can1_motor_velocities[8];
   int16_t can0_motor_currents[8];
   int16_t can1_motor_currents[8];
   uint8_t can0_motor_temperatures[8];
   uint8_t can1_motor_temperatures[8];
   int16_t can0_imu_linear_acceleration[3];
   int16_t can1_imu_linear_acceleration[3];
   int16_t can0_imu_angular_velocity[3];
   int16_t can1_imu_angular_velocity[3];
   uint8_t digital_inputs;
   int16_t dbus_data1[8];
   int16_t dbus_data2[8];
   uint32_t statusword;

   /* Outputs */

   uint32_t controlword;
   int16_t can0_motor_commnads[8];
   int16_t can1_motor_commnads[8];
   uint8_t digital_outputs;

   /* Parameters */

   int16_t max_temperature;
   uint8_t can0_imu_trigger;
   uint8_t can1_imu_trigger;
   uint8_t gpio_modes;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
