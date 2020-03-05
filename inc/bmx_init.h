/// \file bmx_init.h
/// \brief Initialization file. These functions are used to initialize the various
/// features of the tiva.

#ifndef BMX_INIT_HG
#define BMX_INIT_HG

/// \brief main initialization routine. Call this first in main function.
///
void InitializeTiva();

/// \brief Configure UART0. Sets the proper pins and other tiva settings to
/// initialize serial communication.
void ConfigureUART();

/// \brief Configure I2C0. Sets the proper pins and other tiva settings to
/// initialize i2c communication.
void ConfigureI2C();

#endif
