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
#include "BNO055Tiva.h"
#include "bno055.h"


s8 _imu_i2c_read(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count)
{
  s8 comres = 0;
  // This function is used to select the device to read from
  // false == write to slave
  I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, false);

  // Set the I2C Bus to tell the device which first register is meant to be read
  I2CMasterDataPut(I2C0_BASE, reg_address);

  // send slave address, control bit, and register address byte to slave device
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

  //wait for MCU to finish transaction
  while(I2CMasterBusy(I2C0_BASE));

  if(count == 1)
  {
    // specify that we are going to read from slave device
    // true == read from slave
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);

    //send slave address, control bit, and recieve the byte of data
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C0_BASE));

    // write byte to data variable
    arr_data[0] = I2CMasterDataGet(I2C0_BASE);

    // Check for errors
    if (I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
    {
      comres = 0; // success
    }
    else
    {
      comres = -1; // error occured
    }
  }
  else
  {

    const int end = count - 1;
    for(int i = 0; i<count; i++)
    {
      // Decide which i2c macro to use
      if(i == 0)
      {
        // Start reading bytes from the slave
        I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
      }
      else if(i == end)
      {
        // Read final byte from slave
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
      }
      else
      {
        // Continue reading bytes from slave
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
      }

      while(I2CMasterBusy(I2C0_BASE));

      // put read data in data array
      arr_data[i] = I2CMasterDataGet(I2C0_BASE);

      if (I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
      {
        comres = 0;
      }
      else
      {
        comres = -1;
        // Stop Communication and exit
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C0_BASE));
        i = count;
      }
    }
  }
  return comres;
}


s8 _imu_i2c_write(u8 dev_address, u8 reg_address, u8 *var_data, u8 count)
{
    s8 comres = 0;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, false);

    //send the slave address, control bit, and registar address for where to write to
    I2CMasterDataPut(I2C0_BASE, reg_address);

    //Initiate send of data from the MCU
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until MCU is done transferring.
    while(I2CMasterBusy(I2C0_BASE));

    // the BNO055 only ever writes 1 byte of info so if count != 1, throw an error
    if(count == 1)
    {

      // send the information to write
      I2CMasterDataPut(I2C0_BASE, *var_data);

      // Initiate send of data from the MCU
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

      // Wait until MCU is done transferring
      while(I2CMasterBusy(I2C0_BASE));

      if(I2CMasterErr(I2C0_BASE)==I2C_MASTER_ERR_NONE)
      {
        comres = 0; // Success
      }
      else
      {
        comres = -1; // Error
      }
    }
    else
    {
      comres = -1; // Error
    }

    return comres;
}

void _ms_delay(u32 ms)
{
  SysCtlDelay(ms * 5334); // 16000000MHz/3000 ~= 5334 assembly commands per ms
}

s8 init_imu(void)
{
  // initialize setup struct and populate the required information
  struct bno055_t sensor;
  s8 err = 0;
  sensor.bus_write = _imu_i2c_write;
  sensor.bus_read = _imu_i2c_read;
  sensor.delay_msec = _ms_delay;
  sensor.dev_addr = BNO055_I2C_ADDR1;

  // bno055 builtin initialization function
  err = bno055_init(&sensor);
  return err;
}
