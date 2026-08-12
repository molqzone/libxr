#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>


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
static const char acName1601[] = "can0_motor_commnads";
static const char acName1601_00[] = "Max SubIndex";
static const char acName1601_01[] = "New array subitem";
static const char acName1601_02[] = "New array subitem";
static const char acName1601_03[] = "New array subitem";
static const char acName1601_04[] = "New array subitem";
static const char acName1601_05[] = "New array subitem";
static const char acName1601_06[] = "New array subitem";
static const char acName1601_07[] = "New array subitem";
static const char acName1601_08[] = "New array subitem";
static const char acName1602[] = "can1_motor_commnads";
static const char acName1602_00[] = "Max SubIndex";
static const char acName1602_01[] = "New array subitem";
static const char acName1602_02[] = "New array subitem";
static const char acName1602_03[] = "New array subitem";
static const char acName1602_04[] = "New array subitem";
static const char acName1602_05[] = "New array subitem";
static const char acName1602_06[] = "New array subitem";
static const char acName1602_07[] = "New array subitem";
static const char acName1602_08[] = "New array subitem";
static const char acName1603[] = "digital_outputs";
static const char acName1603_00[] = "Max SubIndex";
static const char acName1603_01[] = "digital_outputs";
static const char acName1A00[] = "can0_motor_positions";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "New array subitem";
static const char acName1A00_02[] = "New array subitem";
static const char acName1A00_03[] = "New array subitem";
static const char acName1A00_04[] = "New array subitem";
static const char acName1A00_05[] = "New array subitem";
static const char acName1A00_06[] = "New array subitem";
static const char acName1A00_07[] = "New array subitem";
static const char acName1A00_08[] = "New array subitem";
static const char acName1A01[] = "can1_motor_positions";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "New array subitem";
static const char acName1A01_02[] = "New array subitem";
static const char acName1A01_03[] = "New array subitem";
static const char acName1A01_04[] = "New array subitem";
static const char acName1A01_05[] = "New array subitem";
static const char acName1A01_06[] = "New array subitem";
static const char acName1A01_07[] = "New array subitem";
static const char acName1A01_08[] = "New array subitem";
static const char acName1A02[] = "can0_motor_velocities";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "New array subitem";
static const char acName1A02_02[] = "New array subitem";
static const char acName1A02_03[] = "New array subitem";
static const char acName1A02_04[] = "New array subitem";
static const char acName1A02_05[] = "New array subitem";
static const char acName1A02_06[] = "New array subitem";
static const char acName1A02_07[] = "New array subitem";
static const char acName1A02_08[] = "New array subitem";
static const char acName1A03[] = "can1_motor_velocities";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "New array subitem";
static const char acName1A03_02[] = "New array subitem";
static const char acName1A03_03[] = "New array subitem";
static const char acName1A03_04[] = "New array subitem";
static const char acName1A03_05[] = "New array subitem";
static const char acName1A03_06[] = "New array subitem";
static const char acName1A03_07[] = "New array subitem";
static const char acName1A03_08[] = "New array subitem";
static const char acName1A04[] = "can0_motor_currents";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "New array subitem";
static const char acName1A04_02[] = "New array subitem";
static const char acName1A04_03[] = "New array subitem";
static const char acName1A04_04[] = "New array subitem";
static const char acName1A04_05[] = "New array subitem";
static const char acName1A04_06[] = "New array subitem";
static const char acName1A04_07[] = "New array subitem";
static const char acName1A04_08[] = "New array subitem";
static const char acName1A05[] = "can1_motor_currents";
static const char acName1A05_00[] = "Max SubIndex";
static const char acName1A05_01[] = "New array subitem";
static const char acName1A05_02[] = "New array subitem";
static const char acName1A05_03[] = "New array subitem";
static const char acName1A05_04[] = "New array subitem";
static const char acName1A05_05[] = "New array subitem";
static const char acName1A05_06[] = "New array subitem";
static const char acName1A05_07[] = "New array subitem";
static const char acName1A05_08[] = "New array subitem";
static const char acName1A06[] = "can0_motor_temperatures";
static const char acName1A06_00[] = "Max SubIndex";
static const char acName1A06_01[] = "New array subitem";
static const char acName1A06_02[] = "New array subitem";
static const char acName1A06_03[] = "New array subitem";
static const char acName1A06_04[] = "New array subitem";
static const char acName1A06_05[] = "New array subitem";
static const char acName1A06_06[] = "New array subitem";
static const char acName1A06_07[] = "New array subitem";
static const char acName1A06_08[] = "New array subitem";
static const char acName1A07[] = "can1_motor_temperatures";
static const char acName1A07_00[] = "Max SubIndex";
static const char acName1A07_01[] = "New array subitem";
static const char acName1A07_02[] = "New array subitem";
static const char acName1A07_03[] = "New array subitem";
static const char acName1A07_04[] = "New array subitem";
static const char acName1A07_05[] = "New array subitem";
static const char acName1A07_06[] = "New array subitem";
static const char acName1A07_07[] = "New array subitem";
static const char acName1A07_08[] = "New array subitem";
static const char acName1A08[] = "can0_imu_linear_acceleration";
static const char acName1A08_00[] = "Max SubIndex";
static const char acName1A08_01[] = "New array subitem";
static const char acName1A08_02[] = "New array subitem";
static const char acName1A08_03[] = "New array subitem";
static const char acName1A09[] = "can1_imu_linear_acceleration";
static const char acName1A09_00[] = "Max SubIndex";
static const char acName1A09_01[] = "New array subitem";
static const char acName1A09_02[] = "New array subitem";
static const char acName1A09_03[] = "New array subitem";
static const char acName1A0A[] = "can0_imu_angular_velocity";
static const char acName1A0A_00[] = "Max SubIndex";
static const char acName1A0A_01[] = "New array subitem";
static const char acName1A0A_02[] = "New array subitem";
static const char acName1A0A_03[] = "New array subitem";
static const char acName1A0B[] = "can1_imu_angular_velocity";
static const char acName1A0B_00[] = "Max SubIndex";
static const char acName1A0B_01[] = "New array subitem";
static const char acName1A0B_02[] = "New array subitem";
static const char acName1A0B_03[] = "New array subitem";
static const char acName1A0C[] = "digital_inputs";
static const char acName1A0C_00[] = "Max SubIndex";
static const char acName1A0C_01[] = "digital_inputs";
static const char acName1A0D[] = "dbus_data1";
static const char acName1A0D_00[] = "Max SubIndex";
static const char acName1A0D_01[] = "New array subitem";
static const char acName1A0D_02[] = "New array subitem 1";
static const char acName1A0D_03[] = "New array subitem 2";
static const char acName1A0D_04[] = "New array subitem 3";
static const char acName1A0D_05[] = "New array subitem 4";
static const char acName1A0D_06[] = "New array subitem 5";
static const char acName1A0D_07[] = "New array subitem 6";
static const char acName1A0D_08[] = "New array subitem 7";
static const char acName1A0E[] = "dbus_data2";
static const char acName1A0E_00[] = "Max SubIndex";
static const char acName1A0E_01[] = "New array subitem";
static const char acName1A0E_02[] = "New array subitem 1";
static const char acName1A0E_03[] = "New array subitem 2";
static const char acName1A0E_04[] = "New array subitem 3";
static const char acName1A0E_05[] = "New array subitem 4";
static const char acName1A0E_06[] = "New array subitem 5";
static const char acName1A0E_07[] = "New array subitem 6";
static const char acName1A0E_08[] = "New array subitem 7";
static const char acName1A0F[] = "statusword";
static const char acName1A0F_00[] = "Max SubIndex";
static const char acName1A0F_01[] = "statusword";
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
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "PDO Mapping";
static const char acName1C13_03[] = "PDO Mapping";
static const char acName1C13_04[] = "PDO Mapping";
static const char acName1C13_05[] = "PDO Mapping";
static const char acName1C13_06[] = "PDO Mapping";
static const char acName1C13_07[] = "PDO Mapping";
static const char acName1C13_08[] = "PDO Mapping";
static const char acName1C13_09[] = "PDO Mapping";
static const char acName1C13_10[] = "PDO Mapping";
static const char acName1C13_11[] = "PDO Mapping";
static const char acName1C13_12[] = "PDO Mapping";
static const char acName1C13_13[] = "PDO Mapping";
static const char acName1C13_14[] = "PDO Mapping";
static const char acName1C13_15[] = "PDO Mapping";
static const char acName1C13_16[] = "PDO Mapping";
static const char acName2000[] = "max_temperature";
static const char acName2001[] = "can0_imu_trigger";
static const char acName2002[] = "can1_imu_trigger";
static const char acName2003[] = "gpio_modes";
static const char acName6000[] = "can0_motor_positions";
static const char acName6000_00[] = "Max SubIndex";
static const char acName6000_01[] = "New array subitem";
static const char acName6000_02[] = "New array subitem";
static const char acName6000_03[] = "New array subitem";
static const char acName6000_04[] = "New array subitem";
static const char acName6000_05[] = "New array subitem";
static const char acName6000_06[] = "New array subitem";
static const char acName6000_07[] = "New array subitem";
static const char acName6000_08[] = "New array subitem";
static const char acName6001[] = "can1_motor_positions";
static const char acName6001_00[] = "Max SubIndex";
static const char acName6001_01[] = "New array subitem";
static const char acName6001_02[] = "New array subitem";
static const char acName6001_03[] = "New array subitem";
static const char acName6001_04[] = "New array subitem";
static const char acName6001_05[] = "New array subitem";
static const char acName6001_06[] = "New array subitem";
static const char acName6001_07[] = "New array subitem";
static const char acName6001_08[] = "New array subitem";
static const char acName6002[] = "can0_motor_velocities";
static const char acName6002_00[] = "Max SubIndex";
static const char acName6002_01[] = "New array subitem";
static const char acName6002_02[] = "New array subitem";
static const char acName6002_03[] = "New array subitem";
static const char acName6002_04[] = "New array subitem";
static const char acName6002_05[] = "New array subitem";
static const char acName6002_06[] = "New array subitem";
static const char acName6002_07[] = "New array subitem";
static const char acName6002_08[] = "New array subitem";
static const char acName6003[] = "can1_motor_velocities";
static const char acName6003_00[] = "Max SubIndex";
static const char acName6003_01[] = "New array subitem";
static const char acName6003_02[] = "New array subitem";
static const char acName6003_03[] = "New array subitem";
static const char acName6003_04[] = "New array subitem";
static const char acName6003_05[] = "New array subitem";
static const char acName6003_06[] = "New array subitem";
static const char acName6003_07[] = "New array subitem";
static const char acName6003_08[] = "New array subitem";
static const char acName6004[] = "can0_motor_currents";
static const char acName6004_00[] = "Max SubIndex";
static const char acName6004_01[] = "New array subitem";
static const char acName6004_02[] = "New array subitem";
static const char acName6004_03[] = "New array subitem";
static const char acName6004_04[] = "New array subitem";
static const char acName6004_05[] = "New array subitem";
static const char acName6004_06[] = "New array subitem";
static const char acName6004_07[] = "New array subitem";
static const char acName6004_08[] = "New array subitem";
static const char acName6005[] = "can1_motor_currents";
static const char acName6005_00[] = "Max SubIndex";
static const char acName6005_01[] = "New array subitem";
static const char acName6005_02[] = "New array subitem";
static const char acName6005_03[] = "New array subitem";
static const char acName6005_04[] = "New array subitem";
static const char acName6005_05[] = "New array subitem";
static const char acName6005_06[] = "New array subitem";
static const char acName6005_07[] = "New array subitem";
static const char acName6005_08[] = "New array subitem";
static const char acName6006[] = "can0_motor_temperatures";
static const char acName6006_00[] = "Max SubIndex";
static const char acName6006_01[] = "New array subitem";
static const char acName6006_02[] = "New array subitem";
static const char acName6006_03[] = "New array subitem";
static const char acName6006_04[] = "New array subitem";
static const char acName6006_05[] = "New array subitem";
static const char acName6006_06[] = "New array subitem";
static const char acName6006_07[] = "New array subitem";
static const char acName6006_08[] = "New array subitem";
static const char acName6007[] = "can1_motor_temperatures";
static const char acName6007_00[] = "Max SubIndex";
static const char acName6007_01[] = "New array subitem";
static const char acName6007_02[] = "New array subitem";
static const char acName6007_03[] = "New array subitem";
static const char acName6007_04[] = "New array subitem";
static const char acName6007_05[] = "New array subitem";
static const char acName6007_06[] = "New array subitem";
static const char acName6007_07[] = "New array subitem";
static const char acName6007_08[] = "New array subitem";
static const char acName6008[] = "can0_imu_linear_acceleration";
static const char acName6008_00[] = "Max SubIndex";
static const char acName6008_01[] = "New array subitem";
static const char acName6008_02[] = "New array subitem";
static const char acName6008_03[] = "New array subitem";
static const char acName6009[] = "can1_imu_linear_acceleration";
static const char acName6009_00[] = "Max SubIndex";
static const char acName6009_01[] = "New array subitem";
static const char acName6009_02[] = "New array subitem";
static const char acName6009_03[] = "New array subitem";
static const char acName600A[] = "can0_imu_angular_velocity";
static const char acName600A_00[] = "Max SubIndex";
static const char acName600A_01[] = "New array subitem";
static const char acName600A_02[] = "New array subitem";
static const char acName600A_03[] = "New array subitem";
static const char acName600B[] = "can1_imu_angular_velocity";
static const char acName600B_00[] = "Max SubIndex";
static const char acName600B_01[] = "New array subitem";
static const char acName600B_02[] = "New array subitem";
static const char acName600B_03[] = "New array subitem";
static const char acName600C[] = "digital_inputs";
static const char acName600D[] = "dbus_data1";
static const char acName600D_00[] = "Max SubIndex";
static const char acName600D_01[] = "New array subitem";
static const char acName600D_02[] = "New array subitem 1";
static const char acName600D_03[] = "New array subitem 2";
static const char acName600D_04[] = "New array subitem 3";
static const char acName600D_05[] = "New array subitem 4";
static const char acName600D_06[] = "New array subitem 5";
static const char acName600D_07[] = "New array subitem 6";
static const char acName600D_08[] = "New array subitem 7";
static const char acName600E[] = "dbus_data2";
static const char acName600E_00[] = "Max SubIndex";
static const char acName600E_01[] = "New array subitem";
static const char acName600E_02[] = "New array subitem 1";
static const char acName600E_03[] = "New array subitem 2";
static const char acName600E_04[] = "New array subitem 3";
static const char acName600E_05[] = "New array subitem 4";
static const char acName600E_06[] = "New array subitem 5";
static const char acName600E_07[] = "New array subitem 6";
static const char acName600E_08[] = "New array subitem 7";
static const char acName6040[] = "controlword";
static const char acName6041[] = "statusword";
static const char acName7000[] = "can0_motor_commnads";
static const char acName7000_00[] = "Max SubIndex";
static const char acName7000_01[] = "New array subitem";
static const char acName7000_02[] = "New array subitem";
static const char acName7000_03[] = "New array subitem";
static const char acName7000_04[] = "New array subitem";
static const char acName7000_05[] = "New array subitem";
static const char acName7000_06[] = "New array subitem";
static const char acName7000_07[] = "New array subitem";
static const char acName7000_08[] = "New array subitem";
static const char acName7001[] = "can1_motor_commnads";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "New array subitem";
static const char acName7001_02[] = "New array subitem";
static const char acName7001_03[] = "New array subitem";
static const char acName7001_04[] = "New array subitem";
static const char acName7001_05[] = "New array subitem";
static const char acName7001_06[] = "New array subitem";
static const char acName7001_07[] = "New array subitem";
static const char acName7001_08[] = "New array subitem";
static const char acName7002[] = "digital_outputs";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 5001, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName1008, 0, "CAN"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName1009, 0, "0.0.1"},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName100A, 0, "0.0.1"},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 700707, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 2, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 1, &Obj.serial},
};
const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x60400020, NULL},
};
const _objd SDO1601[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1601_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x70000110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_02, 0x70000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_03, 0x70000310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_04, 0x70000410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_05, 0x70000510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_06, 0x70000610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_07, 0x70000710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_08, 0x70000810, NULL},
};
const _objd SDO1602[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1602_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_01, 0x70010110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_02, 0x70010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_03, 0x70010310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_04, 0x70010410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_05, 0x70010510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_06, 0x70010610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_07, 0x70010710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_08, 0x70010810, NULL},
};
const _objd SDO1603[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1603_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_01, 0x70020008, NULL},
};
const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60000110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_02, 0x60000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_03, 0x60000310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_04, 0x60000410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_05, 0x60000510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_06, 0x60000610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_07, 0x60000710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_08, 0x60000810, NULL},
};
const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60010110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_02, 0x60010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_03, 0x60010310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_04, 0x60010410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_05, 0x60010510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_06, 0x60010610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_07, 0x60010710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_08, 0x60010810, NULL},
};
const _objd SDO1A02[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x60020110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_02, 0x60020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_03, 0x60020310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_04, 0x60020410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_05, 0x60020510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_06, 0x60020610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_07, 0x60020710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_08, 0x60020810, NULL},
};
const _objd SDO1A03[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x60030110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x60030210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_03, 0x60030310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_04, 0x60030410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_05, 0x60030510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_06, 0x60030610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_07, 0x60030710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_08, 0x60030810, NULL},
};
const _objd SDO1A04[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60040110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x60040210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_03, 0x60040310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_04, 0x60040410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_05, 0x60040510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_06, 0x60040610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_07, 0x60040710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_08, 0x60040810, NULL},
};
const _objd SDO1A05[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A05_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_01, 0x60050110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_02, 0x60050210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_03, 0x60050310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_04, 0x60050410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_05, 0x60050510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_06, 0x60050610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_07, 0x60050710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_08, 0x60050810, NULL},
};
const _objd SDO1A06[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A06_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_01, 0x60060108, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_02, 0x60060208, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_03, 0x60060308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_04, 0x60060408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_05, 0x60060508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_06, 0x60060608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_07, 0x60060708, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_08, 0x60060808, NULL},
};
const _objd SDO1A07[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A07_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_01, 0x60070108, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_02, 0x60070208, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_03, 0x60070308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_04, 0x60070408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_05, 0x60070508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_06, 0x60070608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_07, 0x60070708, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_08, 0x60070808, NULL},
};
const _objd SDO1A08[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A08_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_01, 0x60080110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_02, 0x60080210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_03, 0x60080310, NULL},
};
const _objd SDO1A09[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A09_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_01, 0x60090110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_02, 0x60090210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_03, 0x60090310, NULL},
};
const _objd SDO1A0A[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0A_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_01, 0x600A0110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_02, 0x600A0210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_03, 0x600A0310, NULL},
};
const _objd SDO1A0B[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0B_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_01, 0x600B0110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_02, 0x600B0210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_03, 0x600B0310, NULL},
};
const _objd SDO1A0C[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0C_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_01, 0x600C0008, NULL},
};
const _objd SDO1A0D[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0D_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_01, 0x600D0110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_02, 0x600D0210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_03, 0x600D0310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_04, 0x600D0410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_05, 0x600D0510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_06, 0x600D0610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_07, 0x600D0710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_08, 0x600D0810, NULL},
};
const _objd SDO1A0E[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0E_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_01, 0x600E0110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_02, 0x600E0210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_03, 0x600E0310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_04, 0x600E0410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_05, 0x600E0510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_06, 0x600E0610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_07, 0x600E0710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0E_08, 0x600E0810, NULL},
};
const _objd SDO1A0F[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0F_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0F_01, 0x60410020, NULL},
};
const _objd SDO1C00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_02, 0x1601, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_03, 0x1602, NULL},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_04, 0x1603, NULL},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 16, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_03, 0x1A02, NULL},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_04, 0x1A03, NULL},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_05, 0x1A04, NULL},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_06, 0x1A05, NULL},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_07, 0x1A06, NULL},
  {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_08, 0x1A07, NULL},
  {0x09, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_09, 0x1A08, NULL},
  {0x0A, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_10, 0x1A09, NULL},
  {0x0B, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_11, 0x1A0A, NULL},
  {0x0C, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_12, 0x1A0B, NULL},
  {0x0D, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_13, 0x1A0C, NULL},
  {0x0E, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_14, 0x1A0D, NULL},
  {0x0F, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_15, 0x1A0E, NULL},
  {0x10, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_16, 0x1A0F, NULL},
};
const _objd SDO2000[] =
{
  {0x0, DTYPE_INTEGER16, 16, ATYPE_RW, acName2000, 100, &Obj.max_temperature},
};
const _objd SDO2001[] =
{
  {0x0, DTYPE_BOOLEAN, 1, ATYPE_RW, acName2001, 0, &Obj.can0_imu_trigger},
};
const _objd SDO2002[] =
{
  {0x0, DTYPE_BOOLEAN, 1, ATYPE_RW, acName2002, 0, &Obj.can1_imu_trigger},
};
const _objd SDO2003[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2003, 0, &Obj.gpio_modes},
};
const _objd SDO6000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_01, 0, &Obj.can0_motor_positions[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_02, 0, &Obj.can0_motor_positions[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_03, 0, &Obj.can0_motor_positions[2]},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_04, 0, &Obj.can0_motor_positions[3]},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_05, 0, &Obj.can0_motor_positions[4]},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_06, 0, &Obj.can0_motor_positions[5]},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_07, 0, &Obj.can0_motor_positions[6]},
  {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6000_08, 0, &Obj.can0_motor_positions[7]},
};
const _objd SDO6001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6001_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_01, 0, &Obj.can1_motor_positions[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_02, 0, &Obj.can1_motor_positions[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_03, 0, &Obj.can1_motor_positions[2]},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_04, 0, &Obj.can1_motor_positions[3]},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_05, 0, &Obj.can1_motor_positions[4]},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_06, 0, &Obj.can1_motor_positions[5]},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_07, 0, &Obj.can1_motor_positions[6]},
  {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_TXPDO, acName6001_08, 0, &Obj.can1_motor_positions[7]},
};
const _objd SDO6002[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6002_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_01, 0, &Obj.can0_motor_velocities[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_02, 0, &Obj.can0_motor_velocities[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_03, 0, &Obj.can0_motor_velocities[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_04, 0, &Obj.can0_motor_velocities[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_05, 0, &Obj.can0_motor_velocities[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_06, 0, &Obj.can0_motor_velocities[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_07, 0, &Obj.can0_motor_velocities[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6002_08, 0, &Obj.can0_motor_velocities[7]},
};
const _objd SDO6003[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6003_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_01, 0, &Obj.can1_motor_velocities[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_02, 0, &Obj.can1_motor_velocities[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_03, 0, &Obj.can1_motor_velocities[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_04, 0, &Obj.can1_motor_velocities[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_05, 0, &Obj.can1_motor_velocities[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_06, 0, &Obj.can1_motor_velocities[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_07, 0, &Obj.can1_motor_velocities[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6003_08, 0, &Obj.can1_motor_velocities[7]},
};
const _objd SDO6004[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6004_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_01, 0, &Obj.can0_motor_currents[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_02, 0, &Obj.can0_motor_currents[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_03, 0, &Obj.can0_motor_currents[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_04, 0, &Obj.can0_motor_currents[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_05, 0, &Obj.can0_motor_currents[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_06, 0, &Obj.can0_motor_currents[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_07, 0, &Obj.can0_motor_currents[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6004_08, 0, &Obj.can0_motor_currents[7]},
};
const _objd SDO6005[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6005_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_01, 0, &Obj.can1_motor_currents[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_02, 0, &Obj.can1_motor_currents[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_03, 0, &Obj.can1_motor_currents[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_04, 0, &Obj.can1_motor_currents[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_05, 0, &Obj.can1_motor_currents[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_06, 0, &Obj.can1_motor_currents[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_07, 0, &Obj.can1_motor_currents[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6005_08, 0, &Obj.can1_motor_currents[7]},
};
const _objd SDO6006[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6006_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_01, 0, &Obj.can0_motor_temperatures[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_02, 0, &Obj.can0_motor_temperatures[1]},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_03, 0, &Obj.can0_motor_temperatures[2]},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_04, 0, &Obj.can0_motor_temperatures[3]},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_05, 0, &Obj.can0_motor_temperatures[4]},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_06, 0, &Obj.can0_motor_temperatures[5]},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_07, 0, &Obj.can0_motor_temperatures[6]},
  {0x08, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6006_08, 0, &Obj.can0_motor_temperatures[7]},
};
const _objd SDO6007[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6007_00, 8, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_01, 0, &Obj.can1_motor_temperatures[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_02, 0, &Obj.can1_motor_temperatures[1]},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_03, 0, &Obj.can1_motor_temperatures[2]},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_04, 0, &Obj.can1_motor_temperatures[3]},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_05, 0, &Obj.can1_motor_temperatures[4]},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_06, 0, &Obj.can1_motor_temperatures[5]},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_07, 0, &Obj.can1_motor_temperatures[6]},
  {0x08, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName6007_08, 0, &Obj.can1_motor_temperatures[7]},
};
const _objd SDO6008[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6008_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6008_01, 0, &Obj.can0_imu_linear_acceleration[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6008_02, 0, &Obj.can0_imu_linear_acceleration[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6008_03, 0, &Obj.can0_imu_linear_acceleration[2]},
};
const _objd SDO6009[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6009_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6009_01, 0, &Obj.can1_imu_linear_acceleration[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6009_02, 0, &Obj.can1_imu_linear_acceleration[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName6009_03, 0, &Obj.can1_imu_linear_acceleration[2]},
};
const _objd SDO600A[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName600A_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600A_01, 0, &Obj.can0_imu_angular_velocity[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600A_02, 0, &Obj.can0_imu_angular_velocity[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600A_03, 0, &Obj.can0_imu_angular_velocity[2]},
};
const _objd SDO600B[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName600B_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600B_01, 0, &Obj.can1_imu_angular_velocity[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600B_02, 0, &Obj.can1_imu_angular_velocity[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600B_03, 0, &Obj.can1_imu_angular_velocity[2]},
};
const _objd SDO600C[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_TXPDO, acName600C, 0, &Obj.digital_inputs},
};
const _objd SDO600D[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName600D_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_01, 0, &Obj.dbus_data1[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_02, 0, &Obj.dbus_data1[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_03, 0, &Obj.dbus_data1[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_04, 0, &Obj.dbus_data1[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_05, 0, &Obj.dbus_data1[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_06, 0, &Obj.dbus_data1[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_07, 0, &Obj.dbus_data1[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600D_08, 0, &Obj.dbus_data1[7]},
};
const _objd SDO600E[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName600E_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_01, 0, &Obj.dbus_data2[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_02, 0, &Obj.dbus_data2[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_03, 0, &Obj.dbus_data2[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_04, 0, &Obj.dbus_data2[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_05, 0, &Obj.dbus_data2[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_06, 0, &Obj.dbus_data2[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_07, 0, &Obj.dbus_data2[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_TXPDO, acName600E_08, 0, &Obj.dbus_data2[7]},
};
const _objd SDO6040[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RW | ATYPE_RXPDO, acName6040, 0, &Obj.controlword},
};
const _objd SDO6041[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RW | ATYPE_TXPDO, acName6041, 0, &Obj.statusword},
};
const _objd SDO7000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7000_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_01, 0, &Obj.can0_motor_commnads[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_02, 0, &Obj.can0_motor_commnads[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_03, 0, &Obj.can0_motor_commnads[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_04, 0, &Obj.can0_motor_commnads[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_05, 0, &Obj.can0_motor_commnads[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_06, 0, &Obj.can0_motor_commnads[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_07, 0, &Obj.can0_motor_commnads[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7000_08, 0, &Obj.can0_motor_commnads[7]},
};
const _objd SDO7001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 8, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_01, 0, &Obj.can1_motor_commnads[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_02, 0, &Obj.can1_motor_commnads[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_03, 0, &Obj.can1_motor_commnads[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_04, 0, &Obj.can1_motor_commnads[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_05, 0, &Obj.can1_motor_commnads[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_06, 0, &Obj.can1_motor_commnads[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_07, 0, &Obj.can1_motor_commnads[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RW | ATYPE_RXPDO, acName7001_08, 0, &Obj.can1_motor_commnads[7]},
};
const _objd SDO7002[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RW | ATYPE_RXPDO, acName7002, 0, &Obj.digital_outputs},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 1, 0, acName1600, SDO1600},
  {0x1601, OTYPE_RECORD, 8, 0, acName1601, SDO1601},
  {0x1602, OTYPE_RECORD, 8, 0, acName1602, SDO1602},
  {0x1603, OTYPE_RECORD, 1, 0, acName1603, SDO1603},
  {0x1A00, OTYPE_RECORD, 8, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 8, 0, acName1A01, SDO1A01},
  {0x1A02, OTYPE_RECORD, 8, 0, acName1A02, SDO1A02},
  {0x1A03, OTYPE_RECORD, 8, 0, acName1A03, SDO1A03},
  {0x1A04, OTYPE_RECORD, 8, 0, acName1A04, SDO1A04},
  {0x1A05, OTYPE_RECORD, 8, 0, acName1A05, SDO1A05},
  {0x1A06, OTYPE_RECORD, 8, 0, acName1A06, SDO1A06},
  {0x1A07, OTYPE_RECORD, 8, 0, acName1A07, SDO1A07},
  {0x1A08, OTYPE_RECORD, 3, 0, acName1A08, SDO1A08},
  {0x1A09, OTYPE_RECORD, 3, 0, acName1A09, SDO1A09},
  {0x1A0A, OTYPE_RECORD, 3, 0, acName1A0A, SDO1A0A},
  {0x1A0B, OTYPE_RECORD, 3, 0, acName1A0B, SDO1A0B},
  {0x1A0C, OTYPE_RECORD, 1, 0, acName1A0C, SDO1A0C},
  {0x1A0D, OTYPE_RECORD, 8, 0, acName1A0D, SDO1A0D},
  {0x1A0E, OTYPE_RECORD, 8, 0, acName1A0E, SDO1A0E},
  {0x1A0F, OTYPE_RECORD, 1, 0, acName1A0F, SDO1A0F},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 4, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 16, 0, acName1C13, SDO1C13},
  {0x2000, OTYPE_VAR, 0, 0, acName2000, SDO2000},
  {0x2001, OTYPE_VAR, 0, 0, acName2001, SDO2001},
  {0x2002, OTYPE_VAR, 0, 0, acName2002, SDO2002},
  {0x2003, OTYPE_VAR, 0, 0, acName2003, SDO2003},
  {0x6000, OTYPE_ARRAY, 8, 0, acName6000, SDO6000},
  {0x6001, OTYPE_ARRAY, 8, 0, acName6001, SDO6001},
  {0x6002, OTYPE_ARRAY, 8, 0, acName6002, SDO6002},
  {0x6003, OTYPE_ARRAY, 8, 0, acName6003, SDO6003},
  {0x6004, OTYPE_ARRAY, 8, 0, acName6004, SDO6004},
  {0x6005, OTYPE_ARRAY, 8, 0, acName6005, SDO6005},
  {0x6006, OTYPE_ARRAY, 8, 0, acName6006, SDO6006},
  {0x6007, OTYPE_ARRAY, 8, 0, acName6007, SDO6007},
  {0x6008, OTYPE_ARRAY, 3, 0, acName6008, SDO6008},
  {0x6009, OTYPE_ARRAY, 3, 0, acName6009, SDO6009},
  {0x600A, OTYPE_ARRAY, 3, 0, acName600A, SDO600A},
  {0x600B, OTYPE_ARRAY, 3, 0, acName600B, SDO600B},
  {0x600C, OTYPE_VAR, 0, 0, acName600C, SDO600C},
  {0x600D, OTYPE_ARRAY, 8, 0, acName600D, SDO600D},
  {0x600E, OTYPE_ARRAY, 8, 0, acName600E, SDO600E},
  {0x6040, OTYPE_VAR, 0, 0, acName6040, SDO6040},
  {0x6041, OTYPE_VAR, 0, 0, acName6041, SDO6041},
  {0x7000, OTYPE_ARRAY, 8, 0, acName7000, SDO7000},
  {0x7001, OTYPE_ARRAY, 8, 0, acName7001, SDO7001},
  {0x7002, OTYPE_VAR, 0, 0, acName7002, SDO7002},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
