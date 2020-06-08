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
#include "adcInit.h"
#include "estop.h"


//void ISR(void)
//{
//    uint32_t status=0;
//    status = TimerIntStatus(TIMER5_BASE,true);
//    TimerIntClear(TIMER5_BASE,status);
////    if (flag==1)
////    {
////        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwmOut-60);
////        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, pwmOut-60);
////    }
//
//
//}

void blueISR(void)
{
    uint32_t status=0;
    status = TimerIntStatus(TIMER0_BASE,true);
    TimerIntClear(TIMER0_BASE,status);

    if (ledFlag == 1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }

}


int main(void)
{
  delayMs(3000);

  FPULazyStackingEnable();

  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
  // Enable the GPIO pins for the LED (PF2 & PF3).
     //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
  ConfigureUART();
  ConfigureI2C();
  ConfigureADC();
  ConfigurePWM();
  ConfigureESTOP();
  ConfigureQEI();
  ConfigureQEIVel();
//  ConfigureTimerAndInterrupt(triangle,ISR);

  ConfigureTimerAndInterrupt(blueISR);

  SysCtlDelay(SysCtlClockGet() / 60); //50ms

//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

  flag=1;
  while(1)
  {
  //ledFlag = 1 means ESTOP ASSERTED
  if (ledFlag == 1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 4688);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 4688);
    }
  else
  {
      SysCtlDelay(SysCtlClockGet() / 60); //50ms
  //      delayMs(1000);
      struct sensors s = getADCValue();
      UARTprintf("\rManual input: %d   ",s.POT);
      UARTprintf("Position reading: %d   ",QEIPositionGet(QEI0_BASE));
      UARTprintf("Velocity reading: %d   ",QEIVelocityGet(QEI0_BASE));
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, s.POT);
      PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, s.POT);
  }
//    // Turn on the LED.
//    //
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);


  }

//  return 0;
}
