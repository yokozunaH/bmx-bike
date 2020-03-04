#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "bno055.h"
#include "bmx_imu.h"
#include "bmx_init.h"
#include "bmx_quaternion.h"
#include "bmx_utilities.h"

int main(void)
{

  InitializeTiva();

  s8 comres = 0;
  struct bno055_t imu_board;

  comres += init_imu(&imu_board);

  // Set to 9 DOF mode
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  u8 gyro_cal = 0, acc_cal = 0, mag_cal = 0, sys_cal = 0;

  struct bno055_quaternion_t q;
  struct bno055_euler_float_t ea;
  int xang[2], yang[2], zang[2];

  while(1)
  {
    comres += bno055_get_gyro_calib_stat(&gyro_cal);
    comres += bno055_get_mag_calib_stat(&mag_cal);
    comres += bno055_get_accel_calib_stat(&acc_cal);
    comres += bno055_get_sys_calib_stat(&sys_cal);

    comres += bno055_read_quaternion_wxyz(&q);
    scale_divide(&q, QUATERNION_SCALING);
    normalize(&q);

    ea = toEuler(&q);

    float_to_2ints(ea.r, xang, 3);
    float_to_2ints(ea.p, yang, 3);
    float_to_2ints(ea.h, zang, 3);

    UARTprintf("%d.%d  %d.%d  %d.%d  %d  %d  %d  %d  \n", xang[0], xang[1], yang[0], yang[1], zang[0], zang[1], gyro_cal, mag_cal, acc_cal, sys_cal);
  }

  return 0;
}
