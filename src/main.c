#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
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
#include "i2cTiva.h"
#include "uartTiva.h"

int main(void)
{

  s8 comres = 5;
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

  UARTprintf("Hello, world!\n");
  UARTprintf("Err1, %d, \n", comres);

  InitI2C0();
  UARTprintf("Err2, %d, \n", comres);

  struct bno055_t imu_board;

  comres = init_imu(&imu_board);
  UARTprintf("Err3, %d, \n", comres);

  // Set to 9 DOF mode -- REMEMBER TO CALIBRATE MAGNETOMETOR BY MOVING IN FIGURE 8
  comres = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

  s16 w_val = 0;
  while(1){

  // UARTprintf("Err4, %d, \n", comres);


  comres = bno055_read_quaternion_w(&w_val);

  // UARTprintf("Err, %d, \n", comres);
  UARTprintf("w: %d, \n", w_val);
  }

  return 0;
}
