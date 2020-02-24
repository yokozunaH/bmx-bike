#ifndef __I2C_TIVA__
#define __I2C_TIVA__

// Used to initialize the I2C0 functionality
void InitI2C0(void);

// Used to retrieve data from a device given an address and registar
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg);


#endif
