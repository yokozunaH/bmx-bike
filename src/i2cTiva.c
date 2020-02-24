#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

/*
  See section 15 and 17 of the TivaWare™ Peripheral Driver Library for more usage info.
*/

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

// uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
// {
//     // This function is used to select the device to read from
//     // false == write to slave
//     I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);
//
//     // Set the I2C Bus to tell the device which register is meant to be read
//     I2CMasterDataPut(I2C0_BASE, reg);
//
//     // send control byte and register address byte to slave device
//     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
//     //wait for MCU to finish transaction
//     while(I2CMasterBusy(I2C0_BASE)){
//       // Add in error checking here using I2CMasterErr()
//     };
//
//     // specify that we are going to read from slave device
//     // true == read from slave
//     I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);
//
//     //send control byte and read from the register we
//     //specified
//     I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//
//     //wait for MCU to finish transaction
//     while(I2CMasterBusy(I2C0_BASE));
//
//     //return data pulled from the specified register
//     return I2CMasterDataGet(I2C0_BASE);
// }
