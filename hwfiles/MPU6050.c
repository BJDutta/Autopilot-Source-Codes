//MPU6050 I2C library for ARM STM32F103xx Microcontrollers - Main source file
//Has bit, byte and buffer I2C R/W functions
// 23/05/2012 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2012-05-23 - initial release. Thanks to Jeff Rowberg <jeff@rowberg.net> for his AVR/Arduino
//                  based MPU6050 development which inspired me & taken as reference to develop this.
/* ============================================================================================
MPU6050 device I2C library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2012 Harinadha Reddy Chintalapalli

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
================================================================================================
 */

/* Includes */
#include "MPU6050.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "LED_Blink.h"


/** @defgroup MPU6050_Library
 * @{
 */

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize()
{
	uint8_t tmp = 0x00;

	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_ZGYRO);

//	tmp =  0x02;// for 333 hz, 0x04 for 200hz
	tmp= 0x04;
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &tmp, MPU6050_RA_SMPLRT_DIV); // 200 Hz sampling rate

	tmp = MPU6050_DLPF_BW_10;

	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &tmp, MPU6050_RA_CONFIG);

	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);
	MPU6050_SetSleepModeStatus(DISABLE);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
Bool MPU6050_TestConnection()
{
	if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
		return TRUE;
	else
		return FALSE;
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID()
{
	uint8_t tmp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
	return tmp;
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange()
{
	uint8_t tmp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
	return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange()
{
	uint8_t tmp;
	MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
	return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
	MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
Bool MPU6050_GetSleepModeStatus()
{
	uint8_t tmp;
	MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
	if(tmp == 0x00)
		return FALSE;
	else
		return TRUE;
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState)
{
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro)
{
	int i;
	uint8_t tmpBuffer[14];
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);
	/* Get acceleration */
	for(i=0; i<3; i++)
		AccelGyro[i]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
	/* Get Angular rate */
	for(i=4; i<7; i++)
		AccelGyro[i-1]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t tmp;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t tmp;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t tmp;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	*data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
	uint8_t tmp;
	MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
	*data = tmp & (1 << bitNum);
}

/**
 * @brief  Initializes the I2C peripheral used to drive the MPU6050
 * @param  None
 * @return None
 */
void MPU6050_I2C_Init()
{
	PINSEL_CFG_Type PinCfg;
	/* I2C block ------------------------------------------------------------------- */

	/*
	 * Init I2C pin connect
	 */
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Funcnum = 3;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);

	/* Initialize Slave I2C peripheral */
	I2C_Init(MPU6050_I2C, MPU6050_I2C_Speed);

	/* Enable Slave I2C operation */
	I2C_Cmd(MPU6050_I2C, ENABLE);

}

/**
 * @brief  Writes one byte to the  MPU6050.
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
{
	I2C_M_SETUP_Type transferMCfg;
	uint8_t wbuf[2];
//	Status success;

	wbuf[0] = writeAddr;
	wbuf[1] = pBuffer[0];

	/* Start I2C slave device first */
	transferMCfg.sl_addr7bit = slaveAddr;
	transferMCfg.tx_data = wbuf;
	transferMCfg.tx_length = 2;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 5;
	I2C_MasterTransferData(MPU6050_I2C, &transferMCfg, I2C_TRANSFER_POLLING);

}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */

int MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{

	I2C_M_SETUP_Type transferMCfg;
	uint8_t wbuf[1];
	Status success;


	wbuf[0] = readAddr;

	/* Start I2C slave device first */
	transferMCfg.sl_addr7bit = slaveAddr;
	transferMCfg.tx_data = wbuf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = pBuffer;
	transferMCfg.rx_length = NumByteToRead;
	transferMCfg.retransmissions_max = 5;

	success = I2C_MasterTransferData(MPU6050_I2C, &transferMCfg, I2C_TRANSFER_POLLING);

	if(success==ERROR) return 0;
	else return 1;
}

void MPU6050_ConfigMag(){

	uint8_t tmp = 0;
	uint8_t regAddr = 0;

	//at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
	//now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
	//		      i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
	//		      i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
	//		      i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
	//		      i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|(MAG_ADDRESS>>1));//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
	//		      i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
	//		      i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)


	tmp = 0b00100000; regAddr = MPU6050_RA_USER_CTRL; //0x6A
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&tmp,regAddr);

	tmp = 0x00; 		regAddr = MPU6050_RA_INT_PIN_CFG; //0x37
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&tmp,regAddr);

	tmp = 0x0D; regAddr = MPU6050_RA_I2C_MST_CTRL; //0x24
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&tmp,regAddr);

	tmp = 0b10000000|0x1E; regAddr = MPU6050_RA_I2C_SLV0_ADDR; //0x25
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&tmp,regAddr);

	tmp = 0x03; regAddr = MPU6050_RA_I2C_SLV0_REG; //0x26
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&tmp,regAddr);

	tmp = 0x86; regAddr = MPU6050_RA_I2C_SLV0_CTRL; //0x27
	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS,&tmp,regAddr);


}

void MPU6050_Aux_Bypass_Enable(){

	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, 0);
	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, 1);

}

int MPU6050_GetRawAccelGyroMag(int16_t* Accel, int16_t* Gyro, int16_t* Mag)
{
	int i, success;
	uint8_t tmpBuffer[20];
	success = MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 20);
	/* Get acceleration */
	for(i=0; i<3; i++)
		Accel[i]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
	/* Get Angular rate */
	for(i=4; i<7; i++)
		Gyro[i-4]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
	/* Get Angular rate */
	//for(i=7; i<10; i++)
	Mag[0]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]); i++;
	Mag[2]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]); i++;	// swapping Y and Z axes
	Mag[1]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]); i++;

	return success;
}

int MPU6050_GetRawGyro(int16_t* Gyro)
{
	int i, success;
	uint8_t tmpBuffer[6];
	success = MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_GYRO_XOUT_H, 6);
	/* Get Angular rate */
	for(i=0; i<3; i++)
		Gyro[i]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);

	return success;
}

int MPU6050_GetRawAccel(int16_t* Accel)
{
	int i, success;
	uint8_t tmpBuffer[6];
	success = MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 6);
	/* Get Angular rate */
	for(i=0; i<3; i++)
		Accel[i]=((int16_t)((uint16_t)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);

	return success;


}
void MPU6050_I2C_Reboot(){

	blink();
	//	msg_flags.I2C_reboot=1;
	I2C_DeInit(MPU6050_I2C);

	MPU6050_I2C_Init();

}



/**
 * @}
 */ /* end of group MPU6050_Library */



