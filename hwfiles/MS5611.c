
#include "MS5611.h"
#include "LED_Blink.h"
#include "math.h"
#include "MPU6050.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"

uint32_t pressure=101325;
float BaroAlt = 0; // above MSL in m
float Vz =0;  // variometer in m/s

static struct {
	// sensor registers from the MS561101BA datasheet
	uint16_t c[7];
	union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
	union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
	uint8_t  state;
	uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){

	//  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);

	MS561101BA_I2C_ADDRWrite(MS561101BA_DEFAULT_ADDRESS, MS561101BA_RESET);

}

void i2c_MS561101BA_readCalibration(){
	//	union {uint16_t val; uint8_t raw[2]; } data;
	uint8_t pbuffer[2];
	uint8_t i=0;
	uint8_t success;

	delay(10);

	//  for(uint8_t i=0;i<6;i++) {
	//    i2c_rep_start(MS561101BA_ADDRESS + 0);
	//    i2c_write(0xA2+2*i);
	//    i2c_rep_start(MS561101BA_ADDRESS + 1);//I2C read direction => 1
	//    data.raw[1] = i2c_readAck();  // read a 16 bit register
	//    data.raw[0] = i2c_readNak();
	//    ms561101ba_ctx.c[i+1] = data.val;
	//  }

	for(i=0;i<6;i++) {

		success = MS561101BA_I2C_BufferRead(MS561101BA_DEFAULT_ADDRESS, pbuffer , (0xA2+2*i), 2);
		if(success==0){
			MPU6050_I2C_Reboot();
			i2c_MS561101BA_readCalibration();
			return;
		}

		ms561101ba_ctx.c[i+1] = ((uint16_t)pbuffer[0] <<8) + pbuffer[1];
	}

}

void  Baro_init() {
	delay(10);
	i2c_MS561101BA_reset();
	delay(100);
	i2c_MS561101BA_readCalibration();
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
	//  i2c_rep_start(MS561101BA_ADDRESS+0);      // I2C write direction
	//  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection

	uint8_t success;

	success = MS561101BA_I2C_ADDRWrite(MS561101BA_DEFAULT_ADDRESS, MS561101BA_TEMPERATURE + OSR);

	if(success == 0){
		MPU6050_I2C_Reboot();
		i2c_MS561101BA_UT_Start();
		return;
	}
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
	//	i2c_rep_start(MS561101BA_ADDRESS+0);      // I2C write direction
	//	i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
	uint8_t success;

	success = MS561101BA_I2C_ADDRWrite(MS561101BA_DEFAULT_ADDRESS, MS561101BA_PRESSURE + OSR);

	if(success == 0){
		MPU6050_I2C_Reboot();
		i2c_MS561101BA_UP_Start();
		return;
	}
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {

	uint8_t pbuffer[3];
	uint8_t success;
	//	i2c_rep_start(MS561101BA_ADDRESS + 0);
	//	i2c_write(0);
	//	i2c_rep_start(MS561101BA_ADDRESS + 1);
	//	ms561101ba_ctx.up.raw[2] = i2c_readAck();
	//	ms561101ba_ctx.up.raw[1] = i2c_readAck();
	//	ms561101ba_ctx.up.raw[0] = i2c_readNak();

	success = MS561101BA_I2C_BufferRead(MS561101BA_DEFAULT_ADDRESS, pbuffer , 0x00, 3);

	if(success == 0){
		MPU6050_I2C_Reboot();
		i2c_MS561101BA_UP_Read();
		return;
	}

	ms561101ba_ctx.up.raw[2] = pbuffer[0];
	ms561101ba_ctx.up.raw[1] = pbuffer[1];
	ms561101ba_ctx.up.raw[0] = pbuffer[2];


}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {

	uint8_t pbuffer[3];
	uint8_t success;

	//	i2c_rep_start(MS561101BA_ADDRESS + 0);
	//	i2c_write(0);
	//	i2c_rep_start(MS561101BA_ADDRESS + 1);
	//	ms561101ba_ctx.ut.raw[2] = i2c_readAck();
	//	ms561101ba_ctx.ut.raw[1] = i2c_readAck();
	//	ms561101ba_ctx.ut.raw[0] = i2c_readNak();

	success = MS561101BA_I2C_BufferRead(MS561101BA_DEFAULT_ADDRESS, pbuffer , 0x00, 3);

	if(success == 0){
		MPU6050_I2C_Reboot();
		i2c_MS561101BA_UT_Read();
		return;
	}
	ms561101ba_ctx.ut.raw[2] = pbuffer[0];
	ms561101ba_ctx.ut.raw[1] = pbuffer[1];
	ms561101ba_ctx.ut.raw[0] = pbuffer[2];
}

void i2c_MS561101BA_Calculate() {
	int64_t dT   = ms561101ba_ctx.ut.val - ((uint32_t)ms561101ba_ctx.c[5] << 8);  //int32_t according to the spec, but int64_t here to avoid cast after
	int64_t off  = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
	int64_t sens = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);
	pressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

float Baro_update(uint32_t currentTime) {
	static float lastalt = 0;
	static float dt = 0;
	static int spikecount=0;

	dt+=currentTime;

	if (currentTime < ms561101ba_ctx.deadline) return BaroAlt;
	ms561101ba_ctx.deadline = 10;

	switch (ms561101ba_ctx.state) {
	case 0:
		i2c_MS561101BA_UT_Start();
		ms561101ba_ctx.state++; ms561101ba_ctx.deadline = 10; //according to the specs, the pause should be at least 8.22ms
		break;
	case 1:
		i2c_MS561101BA_UT_Read();
		ms561101ba_ctx.state++;
		break;
	case 2:
		i2c_MS561101BA_UP_Start();
		ms561101ba_ctx.state++; ms561101ba_ctx.deadline = 10; //according to the specs, the pause should be at least 8.22ms
		break;
	case 3:
		i2c_MS561101BA_UP_Read();
		i2c_MS561101BA_Calculate();


		BaroAlt = (1.0f - pow(pressure/101325.0f, 0.190295f)) * 44330.0f;

		ms561101ba_ctx.state = 0; ms561101ba_ctx.deadline = 10;

		if(BaroAlt == NAN) break;

		if(fabs(BaroAlt-lastalt)>35 && spikecount<100){
			BaroAlt = lastalt;
			spikecount++;
		}else{

			spikecount = 0;
		}
		Vz = Vz*0.8 + 0.2*1000*(BaroAlt - lastalt)/dt;
		lastalt = BaroAlt;

		dt = 0;
		break;
	}

	return BaroAlt;
}

float vario(){ // returns variometer in m/s
	return Vz;

}
int MS561101BA_I2C_ADDRWrite(uint8_t slaveAddr, uint8_t WriteAddr)
{
	I2C_M_SETUP_Type transferMCfg;
	uint8_t wbuf[1];

	wbuf[0] = WriteAddr;

	/* Start I2C slave device first */
	transferMCfg.sl_addr7bit = slaveAddr;
	transferMCfg.tx_data = wbuf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 5;

	if(I2C_MasterTransferData(MS561101BA_I2C, &transferMCfg, I2C_TRANSFER_POLLING)==ERROR) return 0;
	else return 1;
}

void MS561101BA_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t WriteAddr)
{
	I2C_M_SETUP_Type transferMCfg;
	uint8_t wbuf[2];

	wbuf[0] = WriteAddr;
	wbuf[1] = pBuffer[0];

	/* Start I2C slave device first */
	transferMCfg.sl_addr7bit = slaveAddr;
	transferMCfg.tx_data = wbuf;
	transferMCfg.tx_length = 2;
	transferMCfg.rx_data = NULL;
	transferMCfg.rx_length = 0;
	transferMCfg.retransmissions_max = 5;
	I2C_MasterTransferData(MS561101BA_I2C, &transferMCfg, I2C_TRANSFER_POLLING);
}

int MS561101BA_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	I2C_M_SETUP_Type transferMCfg;
	uint8_t wbuf[1];


	wbuf[0] = ReadAddr;

	/* Start I2C slave device first */
	transferMCfg.sl_addr7bit = slaveAddr;
	transferMCfg.tx_data = wbuf;
	transferMCfg.tx_length = 1;
	transferMCfg.rx_data = pBuffer;
	transferMCfg.rx_length = NumByteToRead;
	transferMCfg.retransmissions_max = 5;
	if(I2C_MasterTransferData(MS561101BA_I2C, &transferMCfg, I2C_TRANSFER_POLLING)==ERROR) return 0;
	else return 1;
}
