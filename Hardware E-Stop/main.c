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
#include "driverlib/pwm.h"

#include "utils/uartstdio.h"

#include "bno055.h"
#include "bmx_imu.h"
#include "bmx_init.h"
#include "bmx_quaternion.h"
#include "bmx_utilities.h"
#include "bmx_encoder.h"
#include "bmx_bluetooth.h"
#include "pwmInit.h"
#include "triangle.h"
#include "timerInit.h"
#include "delay.h"
#include "estop.h"


// An example ISR to demo that the ESTOP ISR has higher priority.
void blueISR(void)
{
    uint32_t status=0;
    status = TimerIntStatus(TIMER0_BASE,true);
    TimerIntClear(TIMER0_BASE,status);

    if (ledFlag == 1)
    {
        // Turn on the blue LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }
}


int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    // Configure all the necessary peripherals.
    ConfigureESTOP();
    ConfigureTimerAndInterrupt(blueISR);

  while(1)
  {
      if (ledFlag == 1)
      {
          GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
      }
  }
}
