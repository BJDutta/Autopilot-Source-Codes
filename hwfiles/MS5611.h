
#ifndef MS5611_H
#define MS5611_H

#include "lpc17xx.h"
#include "lpc_types.h"

#define MS561101BA_I2C                  LPC_I2C1  //
#define MS561101BA_I2C_Speed            200000 // 200kHz standard

#define MS561101BA_ADDRESS            0x77
#define MS561101BA_DEFAULT_ADDRESS    (MS561101BA_ADDRESS)


// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

void i2c_MS561101BA_reset();
void i2c_MS561101BA_readCalibration();
void  Baro_init();
void i2c_MS561101BA_UT_Start();
void i2c_MS561101BA_UP_Start ();
void i2c_MS561101BA_UP_Read ();
void i2c_MS561101BA_UT_Read();
void i2c_MS561101BA_Calculate();
float Baro_update(uint32_t currentTime);
float vario();
int MS561101BA_I2C_ADDRWrite(uint8_t slaveAddr, uint8_t WriteAddr);
void MS561101BA_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t WriteAddr);
int MS561101BA_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


#endif


