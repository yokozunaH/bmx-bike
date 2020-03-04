#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "bno055.h"
#include "BNO055Tiva.h"
#include "quaternion.h"
#include "i2cTiva.h"
#include "uartTiva.h"

int main(void)
{

  s8 comres = 0;
  //
  // Enable lazy stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.
  //
  FPULazyStackingEnable();

  //
  // Set the clocking to run directly from the crystal.
  //
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                     SYSCTL_OSC_MAIN);

  ConfigureUART();
  InitI2C0();

  struct bno055_t imu_board;

  comres += init_imu(&imu_board);

  // Set to 9 DOF mode
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  u8 gyro_cal = 0, acc_cal = 0, mag_cal = 0, sys_cal = 0;

  // s16 x_val = 0, y_val = 0, z_val=0, w_val = 0;
  // float eul_z = 0, eul_x = 0, eul_y = 0;
  // int z_int=0, z_dec=0, x_int=0, x_dec=0, y_int=0, y_dec=0;

  struct bno055_quaternion_t q;
  struct bno055_euler_float_t ea;

  // UARTprintf("Gyro | Mag | Acc | Sys | \n");
  while(1)
  {
    comres += bno055_get_gyro_calib_stat(&gyro_cal);
    comres += bno055_get_mag_calib_stat(&mag_cal);
    comres += bno055_get_accel_calib_stat(&acc_cal);
    comres += bno055_get_sys_calib_stat(&sys_cal);

    // comres = bno055_convert_float_euler_h_deg(&eul_z);
    // comres = bno055_convert_float_euler_r_deg(&eul_x);
    // comres = bno055_convert_float_euler_p_deg(&eul_y);
    // z_int = (int) eul_z;
    // z_dec = eul_z*1000 - z_int*1000;
    // x_int = (int) eul_x;
    // x_dec = eul_x*1000 - x_int*1000;


    comres += bno055_read_quaternion_wxyz(&q);
    scale_divide(&q, 16384); // scale by 2^14 per documentation
    normalize(&q);

    ea = toEuler(&q);

    // UARTprintf("Comres: %d \n", comres);
    // sprintf(str, "%3f  %3f  %3f", eul_x, eul_y, eul_z);
    // UARTprintf("%d.%d  %d.%d  %d.%d  %d  %d  %d  %d  \n",x_int, x_dec, y_int, y_dec, z_int, z_dec, gyro_cal, mag_cal, acc_cal, sys_cal);
    // UARTprintf("%d  %d  %d  %d  %d  %d  %d  %d  \n", q.x, q.y, q.z, q.w, x_val, y_val, z_val , w_val);
    UARTprintf("%d  %d  %d  %d  %d  %d  %d  \n", ea.h, ea.r, ea.p, gyro_cal, mag_cal, acc_cal, sys_cal);

  }

  return 0;
}
