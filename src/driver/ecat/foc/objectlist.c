#include <stddef.h>

#include "esc_coe.h"
#include "utypes.h"

static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "controlword";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "controlword";
static const char acName1601[] = "mode_of_operation";
static const char acName1601_00[] = "Max SubIndex";
static const char acName1601_01[] = "mode_of_operation";
static const char acName1602[] = "desired_joint_torque";
static const char acName1602_00[] = "Max SubIndex";
static const char acName1602_01[] = "desired_joint_torque";
static const char acName1603[] = "desired_joint_position";
static const char acName1603_00[] = "Max SubIndex";
static const char acName1603_01[] = "desired_joint_position";
static const char acName1604[] = "desired_joint_velocity";
static const char acName1604_00[] = "Max SubIndex";
static const char acName1604_01[] = "desired_joint_velocity";
static const char acName1A00[] = "measured_temperature";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "measured_temperature";
static const char acName1A01[] = "measured_motor_voltage";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "measured_motor_voltage";
static const char acName1A02[] = "measured_joint_position";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "measured_joint_position";
static const char acName1A03[] = "measured_joint_velocity";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "measured_joint_velocity";
static const char acName1A04[] = "statusword";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "statusword";
static const char acName1A05[] = "measured_joint_torque";
static const char acName1A05_00[] = "Max SubIndex";
static const char acName1A05_01[] = "measured_joint_torque";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C12_02[] = "PDO Mapping";
static const char acName1C12_03[] = "PDO Mapping";
static const char acName1C12_04[] = "PDO Mapping";
static const char acName1C12_05[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "PDO Mapping";
static const char acName1C13_03[] = "PDO Mapping";
static const char acName1C13_04[] = "PDO Mapping";
static const char acName1C13_05[] = "PDO Mapping";
static const char acName1C13_06[] = "PDO Mapping";
static const char acName2002[] = "measured_temperature";
static const char acName2003[] = "measured_motor_voltage";
static const char acName2006[] = "measured_joint_position";
static const char acName200A[] = "measured_joint_velocity";
static const char acName6040[] = "controlword";
static const char acName6041[] = "statusword";
static const char acName6060[] = "mode_of_operation";
static const char acName6070[] = "motor_torque_constant";
static const char acName6071[] = "desired_joint_torque";
static const char acName6072[] = "motor_current_peak";
static const char acName6073[] = "motor_current_nominal";
static const char acName6074[] = "motor_time_peak";
static const char acName6075[] = "motor_temperature_error";
static const char acName6076[] = "motor_temperature_fatal";
static const char acName6077[] = "measured_joint_torque";
static const char acName607A[] = "desired_joint_position";
static const char acName6091[] = "gear_ratio";
static const char acName60FF[] = "desired_joint_velocity";
static const char acName7000[] = "gains_current";
static const char acName7000_00[] = "Max SubIndex";
static const char acName7000_01[] = "kp";
static const char acName7000_02[] = "ki";
static const char acName7000_03[] = "kd";
static const char acName7001[] = "gains_velocity";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "kp";
static const char acName7001_02[] = "ki";
static const char acName7001_03[] = "kd";
static const char acName7002[] = "gains_position_velocity_torque";
static const char acName7002_00[] = "Max SubIndex";
static const char acName7002_01[] = "kp";
static const char acName7002_02[] = "ki";
static const char acName7002_03[] = "kd";
static const char acName7050[] = "calib_mode";
static const char acName7051[] = "calib_state";
static const char acName7052[] = "calib_result";
static const char acName7052_00[] = "Max SubIndex";
static const char acName7052_01[] = "pole_pairs";
static const char acName7052_02[] = "direction_phases";
static const char acName7052_03[] = "offset_phi_e";
static const char acName7052_04[] = "direction_joint";
static const char acName7052_05[] = "offset_motor";
static const char acName70C0[] = "error_behavior";

const _objd SDO1000[] = {
    {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 5001, NULL},
};
const _objd SDO1008[] = {
    {0x0, DTYPE_VISIBLE_STRING, 72, ATYPE_RO, acName1008, 0, "CLEAR FOC"},
};
const _objd SDO1009[] = {
    {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName1009, 0, "0.0.1"},
};
const _objd SDO100A[] = {
    {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName100A, 0, "0.0.1"},
};
const _objd SDO1018[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0, NULL},
    {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 700707, NULL},
    {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 2, NULL},
    {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 1, &Obj.serial},
};
const _objd SDO1600[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x60400008, NULL},
};
const _objd SDO1601[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1601_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x60600008, NULL},
};
const _objd SDO1602[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1602_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_01, 0x60710020, NULL},
};
const _objd SDO1603[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1603_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_01, 0x607A0020, NULL},
};
const _objd SDO1604[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1604_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1604_01, 0x60FF0020, NULL},
};
const _objd SDO1A00[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x20020010, NULL},
};
const _objd SDO1A01[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x20030010, NULL},
};
const _objd SDO1A02[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x20060020, NULL},
};
const _objd SDO1A03[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x200A0020, NULL},
};
const _objd SDO1A04[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60410020, NULL},
};
const _objd SDO1A05[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A05_00, 1, NULL},
    {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_01, 0x60770020, NULL},
};
const _objd SDO1C00[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
    {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
    {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
    {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 5, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_02, 0x1601, NULL},
    {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_03, 0x1602, NULL},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_04, 0x1603, NULL},
    {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_05, 0x1604, NULL},
};
const _objd SDO1C13[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 6, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
    {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, NULL},
    {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_03, 0x1A02, NULL},
    {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_04, 0x1A03, NULL},
    {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_05, 0x1A04, NULL},
    {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_06, 0x1A05, NULL},
};
const _objd SDO2002[] = {
    {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName2002, 0, &Obj.measured_temperature},
};
const _objd SDO2003[] = {
    {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_TXPDO, acName2003, 0, &Obj.measured_motor_voltage},
};
const _objd SDO2006[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName2006, 0x00000000, &Obj.measured_joint_position},
};
const _objd SDO200A[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName200A, 0x00000000, &Obj.measured_joint_velocity},
};
const _objd SDO6040[] = {
    {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName6040, 0, &Obj.controlword},
};
const _objd SDO6041[] = {
    {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO | ATYPE_TXPDO, acName6041, 0, &Obj.statusword},
};
const _objd SDO6060[] = {
    {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName6060, 0, &Obj.mode_of_operation},
};
const _objd SDO6070[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6070, 0x00000000, &Obj.motor_torque_constant},
};
const _objd SDO6071[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName6071, 0x00000000, &Obj.desired_joint_torque},
};
const _objd SDO6072[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6072, 0x00000000, &Obj.motor_current_peak},
};
const _objd SDO6073[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6073, 0x00000000, &Obj.motor_current_nominal},
};
const _objd SDO6074[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6074, 0x00000000, &Obj.motor_time_peak},
};
const _objd SDO6075[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6075, 0x00000000, &Obj.motor_temperature_error},
};
const _objd SDO6076[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6076, 0x00000000, &Obj.motor_temperature_fatal},
};
const _objd SDO6077[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6077, 0x00000000, &Obj.measured_joint_torque},
};
const _objd SDO607A[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName607A, 0x00000000, &Obj.desired_joint_position},
};
const _objd SDO6091[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RW, acName6091, 0x3f800000, &Obj.gear_ratio},
};
const _objd SDO60FF[] = {
    {0x0, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName60FF, 0x00000000, &Obj.desired_joint_velocity},
};
const _objd SDO7000[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7000_00, 3, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RW, acName7000_01, 0x00000000, &Obj.gains_current.kp},
    {0x02, DTYPE_REAL32, 32, ATYPE_RW, acName7000_02, 0x00000000, &Obj.gains_current.ki},
    {0x03, DTYPE_REAL32, 32, ATYPE_RW, acName7000_03, 0x00000000, &Obj.gains_current.kd},
};
const _objd SDO7001[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 3, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RW, acName7001_01, 0x00000000, &Obj.gains_velocity.kp},
    {0x02, DTYPE_REAL32, 32, ATYPE_RW, acName7001_02, 0x00000000, &Obj.gains_velocity.ki},
    {0x03, DTYPE_REAL32, 32, ATYPE_RW, acName7001_03, 0x00000000, &Obj.gains_velocity.kd},
};
const _objd SDO7002[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7002_00, 3, NULL},
    {0x01, DTYPE_REAL32, 32, ATYPE_RW, acName7002_01, 0x00000000, &Obj.gains_position_velocity_torque.kp},
    {0x02, DTYPE_REAL32, 32, ATYPE_RW, acName7002_02, 0x00000000, &Obj.gains_position_velocity_torque.ki},
    {0x03, DTYPE_REAL32, 32, ATYPE_RW, acName7002_03, 0x00000000, &Obj.gains_position_velocity_torque.kd},
};
const _objd SDO7050[] = {
    {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7050, 0, &Obj.calib_mode},
};
const _objd SDO7051[] = {
    {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7051, 0, &Obj.calib_state},
};
const _objd SDO7052[] = {
    {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7052_00, 5, NULL},
    {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName7052_01, 0, &Obj.calib_result.pole_pairs},
    {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7052_02, 0, &Obj.calib_result.direction_phases},
    {0x03, DTYPE_INTEGER32, 32, ATYPE_RO, acName7052_03, 0, &Obj.calib_result.offset_phi_e},
    {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName7052_04, 1, &Obj.calib_result.direction_joint},
    {0x05, DTYPE_REAL32, 32, ATYPE_RO, acName7052_05, 0x00000000, &Obj.calib_result.offset_motor},
};
const _objd SDO70C0[] = {
    {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName70C0, 0, &Obj.error_behavior},
};

const _objectlist SDOobjects[] = {{0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
                                  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
                                  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
                                  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
                                  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
                                  {0x1600, OTYPE_RECORD, 1, 0, acName1600, SDO1600},
                                  {0x1601, OTYPE_RECORD, 1, 0, acName1601, SDO1601},
                                  {0x1602, OTYPE_RECORD, 1, 0, acName1602, SDO1602},
                                  {0x1603, OTYPE_RECORD, 1, 0, acName1603, SDO1603},
                                  {0x1604, OTYPE_RECORD, 1, 0, acName1604, SDO1604},
                                  {0x1A00, OTYPE_RECORD, 1, 0, acName1A00, SDO1A00},
                                  {0x1A01, OTYPE_RECORD, 1, 0, acName1A01, SDO1A01},
                                  {0x1A02, OTYPE_RECORD, 1, 0, acName1A02, SDO1A02},
                                  {0x1A03, OTYPE_RECORD, 1, 0, acName1A03, SDO1A03},
                                  {0x1A04, OTYPE_RECORD, 1, 0, acName1A04, SDO1A04},
                                  {0x1A05, OTYPE_RECORD, 1, 0, acName1A05, SDO1A05},
                                  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
                                  {0x1C12, OTYPE_ARRAY, 5, 0, acName1C12, SDO1C12},
                                  {0x1C13, OTYPE_ARRAY, 6, 0, acName1C13, SDO1C13},
                                  {0x2002, OTYPE_VAR, 0, 0, acName2002, SDO2002},
                                  {0x2003, OTYPE_VAR, 0, 0, acName2003, SDO2003},
                                  {0x2006, OTYPE_VAR, 0, 0, acName2006, SDO2006},
                                  {0x200A, OTYPE_VAR, 0, 0, acName200A, SDO200A},
                                  {0x6040, OTYPE_VAR, 0, 0, acName6040, SDO6040},
                                  {0x6041, OTYPE_VAR, 0, 0, acName6041, SDO6041},
                                  {0x6060, OTYPE_VAR, 0, 0, acName6060, SDO6060},
                                  {0x6070, OTYPE_VAR, 0, 0, acName6070, SDO6070},
                                  {0x6071, OTYPE_VAR, 0, 0, acName6071, SDO6071},
                                  {0x6072, OTYPE_VAR, 0, 0, acName6072, SDO6072},
                                  {0x6073, OTYPE_VAR, 0, 0, acName6073, SDO6073},
                                  {0x6074, OTYPE_VAR, 0, 0, acName6074, SDO6074},
                                  {0x6075, OTYPE_VAR, 0, 0, acName6075, SDO6075},
                                  {0x6076, OTYPE_VAR, 0, 0, acName6076, SDO6076},
                                  {0x6077, OTYPE_VAR, 0, 0, acName6077, SDO6077},
                                  {0x607A, OTYPE_VAR, 0, 0, acName607A, SDO607A},
                                  {0x6091, OTYPE_VAR, 0, 0, acName6091, SDO6091},
                                  {0x60FF, OTYPE_VAR, 0, 0, acName60FF, SDO60FF},
                                  {0x7000, OTYPE_RECORD, 3, 0, acName7000, SDO7000},
                                  {0x7001, OTYPE_RECORD, 3, 0, acName7001, SDO7001},
                                  {0x7002, OTYPE_RECORD, 3, 0, acName7002, SDO7002},
                                  {0x7050, OTYPE_VAR, 0, 0, acName7050, SDO7050},
                                  {0x7051, OTYPE_VAR, 0, 0, acName7051, SDO7051},
                                  {0x7052, OTYPE_RECORD, 5, 0, acName7052, SDO7052},
                                  {0x70C0, OTYPE_VAR, 0, 0, acName70C0, SDO70C0},
                                  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}};
