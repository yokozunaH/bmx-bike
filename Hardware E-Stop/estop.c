#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.c"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.c"
#include "driverlib/gpio.h"

volatile int ledFlag = 0;

void ESTOPISR(void)
{
    uint32_t status=0;
    status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,status);
//    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
    if (ledFlag == 0)
    {
        ledFlag = 1;
    }

}

void ConfigureESTOP(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    //???
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);

    GPIOIntRegister(GPIO_PORTF_BASE,ESTOPISR);

    IntPrioritySet(INT_GPIOF, 0);

    IntEnable(INT_GPIOF);

    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}


