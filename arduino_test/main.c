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
  InitI2C0();
  UARTprintf("Err1, %d, \n", comres);

  u8 data = 7;

  while(1)
  {
    comres = _imu_i2c_write(0x08, 0x08, &data, 1);
    UARTprintf("Err2, %d, \n", comres);
  }



  return 0;
}
