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
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"

#include "utils/uartstdio.h"

#include "bno055.h"
#include "bmx_imu.h"
#include "bmx_init.h"
#include "bmx_quaternion.h"
#include "bmx_utilities.h"
#include "encoder.h"
#include "bluetooth.h"

int main(void)
{

  InitializeTiva();

  s8 comres = 0;
  struct bno055_t imu_board;

  comres += init_imu(&imu_board);

  // Set to 9 DOF mode
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  u8 gyro_cal = 0, acc_cal = 0, mag_cal = 0, sys_cal = 0;

  volatile int pos, pos_deg, vel, vel_deg;
  float pos_f, vel_f;

  struct bno055_quaternion_t q;
  struct bno055_euler_float_t ea;
  int xang[2], yang[2], zang[2];

  //Initialize strings for bluetooth printing
  char a[]="Encoder count (pulses):";
  int len_a = sizeof(a);

  char b[]="Encoder count (degrees):";
  int len_b = sizeof(b);

  char c[]="Encoder velocity (pulses):";
  int len_c = sizeof(c);

  char d[]="Encoder velocity (degrees/sec):";
  int len_d = sizeof(d);

  while(1)
  {
    // Turn on the LED.
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

    //
    // Delay for a bit.
    //
    SysCtlDelay(SysCtlClockGet() / 6); //500ms

    //
    // Turn off the BLUE LED.
    //
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

    // Read position from encoder.
    //
    pos = QEIPositionGet(QEI0_BASE);
    vel = QEIVelocityGet(QEI0_BASE);

    //Print counts (pulses)
    printString(a,len_a);
    printInt(pos);

    pos_f = (float) pos;
    pos_deg = (int) pos_f*0.043945;// division doesn't work for some reason  (360/8192) = 0.043945

    //Print counts (degrees)
    printString(b,len_b);
    printInt(pos_deg);

    SysCtlDelay(SysCtlClockGet() / 6); //500ms

    //Print velocity (pulses)
    printString(c,len_c);
    printInt(vel);


    //Print velocity (degrees)
    /*vel_f = (float) vel;
    vel_deg = (int) vel_f*0.043945;// division doesn't work for some reason
    printString(d,len_d);
    printInt(vel_deg);*/


    /*comres += bno055_get_gyro_calib_stat(&gyro_cal);
    comres += bno055_get_mag_calib_stat(&mag_cal);
    comres += bno055_get_accel_calib_stat(&acc_cal);
    comres += bno055_get_sys_calib_stat(&sys_cal);

    comres += bno055_read_quaternion_wxyz(&q);
    scale_divide(&q, QUATERNION_SCALING);
    normalize(&q);*/

    //ea = toEuler(&q);

    //float_to_2ints(q.w, xang, 3);
    //float_to_2ints(q.y, yang, 3);
    //float_to_2ints(q.z, zang, 3);

    /*float_to_2ints(ea.r, xang, 3);
    float_to_2ints(ea.p, yang, 3);
    float_to_2ints(ea.h, zang, 3);*/

    //UARTprintf("%d.%d  %d.%d  %d.%d  %d  %d  %d  %d  \n", xang[0], xang[1], yang[0], yang[1], zang[0], zang[1], gyro_cal, mag_cal, acc_cal, sys_cal);*/
  }

  return 0;
}
