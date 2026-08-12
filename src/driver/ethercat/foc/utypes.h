#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct {
  /* Identity */

  uint32_t serial;

  /* Inputs */

  uint16_t measured_temperature;
  uint16_t measured_motor_voltage;
  float measured_joint_position;
  float measured_joint_velocity;
  uint32_t statusword;
  float measured_joint_torque;

  /* Outputs */

  uint8_t controlword;
  uint8_t mode_of_operation;
  float desired_joint_torque;
  float desired_joint_position;
  float desired_joint_velocity;

  /* Parameters */

  float motor_torque_constant;
  float motor_current_peak;
  float motor_current_nominal;
  float motor_time_peak;
  float motor_temperature_error;
  float motor_temperature_fatal;
  float gear_ratio;
  struct {
    float kp;
    float ki;
    float kd;
  } gains_current;
  struct {
    float kp;
    float ki;
    float kd;
  } gains_velocity;
  struct {
    float kp;
    float ki;
    float kd;
  } gains_position_velocity_torque;
  uint8_t calib_mode;
  uint8_t calib_state;
  struct {
    uint16_t pole_pairs;
    uint8_t direction_phases;
    int32_t offset_phi_e;
    int8_t direction_joint;
    float offset_motor;
  } calib_result;
  uint8_t error_behavior;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
