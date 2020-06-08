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

// ESTOP ISR, its job is to raise the ledFlag if it has not been raised
void ESTOPISR(void)
{
    uint32_t status=0;
    status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,status);
    if (ledFlag == 0)
    {
        ledFlag = 1;
    }

}

// ESTOP interrupt configuration. Largely similar to other input based interrupt, but with the highest priority.
void ConfigureESTOP(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);

    GPIOIntRegister(GPIO_PORTF_BASE,ESTOPISR);

    // Setting the priority of this interrupt to the highest, so the stopping flag will be raised in every other place in the code as soon as possible.
    IntPrioritySet(INT_GPIOF, 0);

    IntEnable(INT_GPIOF);

    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}


