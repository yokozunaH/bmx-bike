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

#include "i2cTiva.h"
#include "uartTiva.h"
#include "BNO055Tiva.h"

int main(void)
{

  ConfigureUART();

  InitI2C0();

  return 0;
}
