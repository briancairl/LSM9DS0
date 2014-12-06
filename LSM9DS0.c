#include "LSM9DS0.h"


static void delay(volatile uint32_t wait )
{
	wait *= 100000UL;
	while(--wait);
}


static LMS9DS0_Init_t LMS_Device_Defaults =
{
	G_SCALE_245DPS,
	A_SCALE_2G,
 	M_SCALE_2GS,
	G_ODR_95_BW_125, 
	A_ODR_50, 
	M_ODR_50
};


void LSM9DS0_Init( LSM9DS0_t* lsm_t, interface_mode interface, uint8_t gAddr, uint8_t xmAddr )
{
	// interfaceMode will keep track of whether we're using SPI or I2C:
#if(LSM_I2C_SUPPORT==1)
	lsm_t->interfaceMode= interface;
#else
	lsm_t->interfaceMode= MODE_SPI;
#endif
	// xmAddress and lsm_t->gAddress will store the 7-bit I2C address, if using I2C.
	// If we're using SPI, these variables store the chip-select pins.
	lsm_t->xmAddress 	= xmAddr;
	lsm_t->gAddress 	= gAddr;
}



uint16_t LSM9DS0_begin( LSM9DS0_t* lsm_t, LMS9DS0_Init_t* init_t )
{
	uint8_t gTest, xmTest;
	if(init_t==NULL)
		init_t = &LMS_Device_Defaults;

	// Store the given scales in class variables. These scale variables
	// are used throughout to calculate the actual g's, DPS,and Gs's.
	lsm_t->gScale = init_t->gScl;
	lsm_t->aScale = init_t->aScl;
	lsm_t->mScale = init_t->mScl;
	
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(lsm_t); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(lsm_t); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(lsm_t); // Calculate g / ADC tick, stored in aRes variable
	
	// Now, initialize our hardware interface.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)	// If we're using I2C
		LSM_initI2C();					// Initialize I2C
	else if (lsm_t->interfaceMode == MODE_SPI) 	// else, if we're using SPI
	#endif
		LSM_initSPI();							// Initialize SPI
	
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	gTest  = gReadByte(lsm_t,WHO_AM_I_G);		// Read the gyro WHO_AM_I
	xmTest = xmReadByte(lsm_t,WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I
	
	// Gyro initialization stuff:
	initGyro(lsm_t);		// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(lsm_t,init_t->gODR); 		// Set the gyro output data rate and bandwidth.
	setGyroScale(lsm_t,lsm_t->gScale); 	// Set the gyro range
	
	// Accelerometer initialization stuff:
	initAccel(lsm_t); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(lsm_t,init_t->aODR); // Set the accel data rate.
	setAccelScale(lsm_t,lsm_t->aScale); // Set the accel range.
	
	// Magnetometer initialization stuff:
	initMag(lsm_t); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(lsm_t,init_t->mODR); // Set the magnetometer output data rate.
	setMagScale(lsm_t,lsm_t->mScale); // Set the magnetometer's range.
	
	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (xmTest << 8) | gTest;
}


void initGyro( LSM9DS0_t* lsm_t )
{
	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	DR[1:0] - Output data rate selection
		00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	BW[1:0] - Bandwidth selection (sets cutoff frequency)
		 Value depends on ODR. See datasheet table 21.
	PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)	*/
	gWriteByte(lsm_t, CTRL_REG1_G, 0x0F); // Normal mode, enable all axes
	
	/* CTRL_REG2_G sets up the HPF
	Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	HPM[1:0] - High pass filter mode selection
		00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
		10=normal, 11=autoreset on interrupt
	HPCF[3:0] - High pass filter cutoff frequency
		Value depends on data rate. See datasheet table 26.
	*/
	gWriteByte(lsm_t, CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency
	
	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
	I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(lsm_t, CTRL_REG3_G, 0x88); 
	
	/* CTRL_REG4_G sets the scale, update mode
	Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	BDU - Block data update (0=continuous, 1=output not updated until read
	BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	FS[1:0] - Full-scale selection
		00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	ST[1:0] - Self-test enable
		00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	SIM - SPI serial interface mode select
		0=4 wire, 1=3 wire */
	gWriteByte(lsm_t, CTRL_REG4_G, 0x00); // Set scale to 245 dps
	
	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	BOOT - Reboot memory content (0=normal, 1=reboot)
	FIFO_EN - FIFO enable (0=disable, 1=enable)
	HPen - HPF enable (0=disable, 1=enable)
	INT1_Sel[1:0] - Int 1 selection configuration
	Out_Sel[1:0] - Out selection configuration */
	gWriteByte(lsm_t, CTRL_REG5_G, 0x00);
	
	// Temporary !!! For testing !!! Remove !!! Or make useful !!!
	configGyroInt(lsm_t,0x2A, 0, 0, 0, 0); // Trigger interrupt when above 0 DPS...
}



void initAccel(LSM9DS0_t* lsm_t)
{
	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	BOOT - Reboot memory content (0: normal, 1: reboot)
	FIFO_EN - Fifo enable (0: disable, 1: enable)
	WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
	xmWriteByte(lsm_t,CTRL_REG0_XM, 0x00);
	
	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	AODR[3:0] - select the acceleration data rate:
		0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz, 
		0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
		1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	BDU - block data update for accel AND mag
		0: Continuous update
		1: Output registers aren't updated until MSB and LSB have been read.
	AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
		0: Axis disabled, 1: Axis enabled									 */	
	xmWriteByte(lsm_t,CTRL_REG1_XM, 0x57); // 100Hz data rate, x/y/z all enabled
	
	//Serial.println(xmReadByte(CTRL_REG1_XM));
	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
	ABW[1:0] - Accelerometer anti-alias filter bandwidth
		00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
	AFS[2:0] - Accel full-scale selection
		000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
	AST[1:0] - Accel self-test enable
		00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
	SIM - SPI mode selection
		0=4-wire, 1=3-wire													 */
	xmWriteByte(lsm_t,CTRL_REG2_XM, 0x00); // Set scale to 2g
	
	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
	*/
	// Accelerometer data ready on INT1_XM (0x04)
	xmWriteByte(lsm_t,CTRL_REG3_XM, 0x04); 
}



void initMag(LSM9DS0_t* lsm_t)
{	
	/* CTRL_REG5_XM enables temp sensor, sets mag resolution and data rate
	Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
	TEMP_EN - Enable temperature sensor (0=disabled, 1=enabled)
	M_RES[1:0] - Magnetometer resolution select (0=low, 3=high)
	M_ODR[2:0] - Magnetometer data rate select
		000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz, 100=50Hz, 101=100Hz
	LIR2 - Latch interrupt request on INT2_SRC (cleared by reading INT2_SRC)
		0=interrupt request not latched, 1=interrupt request latched
	LIR1 - Latch interrupt request on INT1_SRC (cleared by readging INT1_SRC)
		0=irq not latched, 1=irq latched 									 */
	xmWriteByte(lsm_t,CTRL_REG5_XM, 0x94); // Mag data rate - 100 Hz, enable temperature sensor
	
	/* CTRL_REG6_XM sets the magnetometer full-scale
	Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
	MFS[1:0] - Magnetic full-scale selection
	00:+/-2Gauss, 01:+/-4Gs, 10:+/-8Gs, 11:+/-12Gs							 */
	xmWriteByte(lsm_t,CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS
	
	/* CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters
	AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
	AHPM[1:0] - HPF mode selection
		00=normal (resets reference registers), 01=reference signal for filtering, 
		10=normal, 11=autoreset on interrupt event
	AFDS - Filtered acceleration data selection
		0=internal filter bypassed, 1=data from internal filter sent to FIFO
	MLP - Magnetic data low-power mode
		0=data rate is set by M_ODR bits in CTRL_REG5
		1=data rate is set to 3.125Hz
	MD[1:0] - Magnetic sensor mode selection (default 10)
		00=continuous-conversion, 01=single-conversion, 10 and 11=power-down */
	xmWriteByte(lsm_t,CTRL_REG7_XM, 0x00); // Continuous conversion mode
	
	/* CTRL_REG4_XM is used to set interrupt generators on INT2_XM
	Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
	*/
	xmWriteByte(lsm_t,CTRL_REG4_XM, 0x04); // Magnetometer data ready on INT2_XM (0x08)
	
	/* INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
	Bits[7:0] - XMIEN YMIEN ZMIEN PP_OD IEA IEL 4D MIEN
	XMIEN, YMIEN, ZMIEN - Enable interrupt recognition on axis for mag data
	PP_OD - Push-pull/open-drain interrupt configuration (0=push-pull, 1=od)
	IEA - Interrupt polarity for accel and magneto
		0=active-low, 1=active-high
	IEL - Latch interrupt request for accel and magneto
		0=irq not latched, 1=irq latched
	4D - 4D enable. 4D detection is enabled when 6D bit in INT_GEN1_REG is set
	MIEN - Enable interrupt generation for magnetic data
		0=disable, 1=enable) */
	xmWriteByte(lsm_t,INT_CTRL_REG_M, 0x09); // Enable interrupts for mag, active-low, push-pull
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void calLSM9DS0(LSM9DS0_t* lsm_t, float * gbias, float * abias)
{  
  uint8_t data[6] 	= {0, 0, 0, 0, 0, 0};
  int16_t 
  	gyro_bias[3] 	= {0, 0, 0}, 
  	accel_bias[3] 	= {0, 0, 0};
  int samples, ii;
  
  // First get gyro bias
  uint8_t c = gReadByte(lsm_t,CTRL_REG5_G);
  gWriteByte(lsm_t, CTRL_REG5_G, c | 0x40);         	// Enable gyro FIFO  
  delay(20);                                 	// Wait for change to take effect
  gWriteByte(lsm_t, FIFO_CTRL_REG_G, 0x20 | 0x1F);  	// Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  								// delay 1000 milliseconds to collect FIFO samples
  
  samples = (gReadByte(lsm_t,FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    gReadBytes(lsm_t,OUT_X_L_G,  &data[0], 6);
    gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
  }  

  gyro_bias[0] /= samples; 						// average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  gbias[0] = (float)gyro_bias[0]*lsm_t->gRes; 			 // Properly scale the data to get deg/s
  gbias[1] = (float)gyro_bias[1]*lsm_t->gRes;
  gbias[2] = (float)gyro_bias[2]*lsm_t->gRes;
  
  c = gReadByte(lsm_t,CTRL_REG5_G);
  gWriteByte(lsm_t, CTRL_REG5_G, c & ~0x40);  			// Disable gyro FIFO  
  delay(20);
  gWriteByte(lsm_t, FIFO_CTRL_REG_G, 0x00);   			// Enable gyro bypass mode
  

  //  Now get the accelerometer biases
  c = xmReadByte(lsm_t,CTRL_REG0_XM);
  xmWriteByte(lsm_t,CTRL_REG0_XM, c | 0x40);      	// Enable accelerometer FIFO  
  delay(20);                                	// Wait for change to take effect
  xmWriteByte(lsm_t,FIFO_CTRL_REG, 0x20 | 0x1F);  	// Enable accelerometer FIFO stream mode and set watermark at 32 samples
  delay(1000); 	 								// delay 1000 milliseconds to collect FIFO samples

  samples = (xmReadByte(lsm_t,FIFO_SRC_REG) & 0x1F); 	// Read number of stored accelerometer samples

   for(ii = 0; ii < samples ; ii++) {          	// Read the accelerometer data stored in the FIFO
    xmReadBytes(lsm_t,OUT_X_L_A, &data[0], 6);
    accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1.0f/lsm_t->aRes); // Assumes sensor facing up!
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  abias[0] = (float)accel_bias[0]*lsm_t->aRes; // Properly scale data to get gs
  abias[1] = (float)accel_bias[1]*lsm_t->aRes;
  abias[2] = (float)accel_bias[2]*lsm_t->aRes;

  c = xmReadByte(lsm_t,CTRL_REG0_XM);
  xmWriteByte(lsm_t,CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO  
  delay(20);
  xmWriteByte(lsm_t,FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}

void LSM9DS0_readAccel(LSM9DS0_t* lsm_t)
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	xmReadBytes(lsm_t,OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A
	lsm_t->ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	lsm_t->ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	lsm_t->az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
}

void LSM9DS0_readMag(LSM9DS0_t* lsm_t)
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	xmReadBytes(lsm_t,OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	lsm_t->mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	lsm_t->my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	lsm_t->mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
}

void LSM9DS0_readTemp(LSM9DS0_t* lsm_t)
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	xmReadBytes(lsm_t,OUT_TEMP_L_XM, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L_M
	lsm_t->temperature = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
}

void LSM9DS0_readGyro(LSM9DS0_t* lsm_t)
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	gReadBytes(lsm_t,OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
	lsm_t->gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	lsm_t->gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	lsm_t->gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
}

float calcGyro(LSM9DS0_t* lsm_t, int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return lsm_t->gRes * gyro; 
}

float calcAccel(LSM9DS0_t* lsm_t, int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return lsm_t->aRes * accel;
}

float calcMag(LSM9DS0_t* lsm_t, int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return lsm_t->mRes * mag;
}

void setGyroScale(LSM9DS0_t* lsm_t, gyro_scale gScl)
{
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(lsm_t,CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(lsm_t, CTRL_REG4_G, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	lsm_t->gScale = gScl;
	
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes(lsm_t);
}

void setAccelScale(LSM9DS0_t* lsm_t, accel_scale aScl)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(lsm_t,CTRL_REG2_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	lsm_t->aScale = aScl;
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes(lsm_t);
}

void setMagScale(LSM9DS0_t* lsm_t, mag_scale mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(lsm_t,CTRL_REG6_XM, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	lsm_t->mScale = mScl;
	
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes(lsm_t);
}

void setGyroODR(LSM9DS0_t* lsm_t, gyro_odr gRate)
{
	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(lsm_t,CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(lsm_t, CTRL_REG1_G, temp);
}

void setAccelODR(LSM9DS0_t* lsm_t, accel_odr aRate)
{
	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(lsm_t,CTRL_REG1_XM, temp);
}

void setAccelABW(LSM9DS0_t* lsm_t, accel_abw abwRate)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG2_XM);
	// Then mask out the accel ABW bits:
	temp &= 0xFF^(0x3 << 7);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 7);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(lsm_t,CTRL_REG2_XM, temp);
}

void setMagODR(LSM9DS0_t* lsm_t, mag_odr mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = xmReadByte(lsm_t,CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(lsm_t,CTRL_REG5_XM, temp);
}


void configGyroInt(LSM9DS0_t* lsm_t,  uint8_t int1Cfg, uint16_t int1ThsX, uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration)
{
	gWriteByte(lsm_t, INT1_CFG_G, int1Cfg);
	gWriteByte(lsm_t, INT1_THS_XH_G, (int1ThsX & 0xFF00) >> 8);
	gWriteByte(lsm_t, INT1_THS_XL_G, (int1ThsX & 0xFF));
	gWriteByte(lsm_t, INT1_THS_YH_G, (int1ThsY & 0xFF00) >> 8);
	gWriteByte(lsm_t, INT1_THS_YL_G, (int1ThsY & 0xFF));
	gWriteByte(lsm_t, INT1_THS_ZH_G, (int1ThsZ & 0xFF00) >> 8);
	gWriteByte(lsm_t, INT1_THS_ZL_G, (int1ThsZ & 0xFF));
	if (duration)
		gWriteByte(lsm_t, INT1_DURATION_G, 0x80 | duration);
	else
		gWriteByte(lsm_t, INT1_DURATION_G, 0x00);
}


void calcgRes(LSM9DS0_t* lsm_t)
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:
	switch (lsm_t->gScale)
	{
	case G_SCALE_245DPS:
		lsm_t->gRes = 245.0f / 32768.0f;
		break;
	case G_SCALE_500DPS:
		lsm_t->gRes = 500.0f / 32768.0f;
		break;
	case G_SCALE_2000DPS:
		lsm_t->gRes = 2000.0f / 32768.0f;
		break;
	}
}

void calcaRes(LSM9DS0_t* lsm_t)
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an 
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:
	lsm_t->aRes = lsm_t->aScale == A_SCALE_16G ? 16.0f / 32768.0f : 
		   (((float)lsm_t->aScale + 1.0f) * 2.0f) / 32768.0f;
}

void calcmRes(LSM9DS0_t* lsm_t)
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	lsm_t->mRes = lsm_t->mScale == M_SCALE_2GS ? 2.0f / 32768.0f : 
	       (float) (lsm_t->mScale << 2) / 32768.0f;
}
	
void gWriteByte(LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)
		I2CwriteByte(lsm_t->lsm_t->gAddress, subAddress, data);
	else if (lsm_t->interfaceMode == MODE_SPI)
	#endif
		SPIwriteByte(lsm_t->gAddress, subAddress, data,'g');
}

void xmWriteByte(LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)
		I2CwriteByte(lsm_t->xmAddress, subAddress, data);
	else if (lsm_t->interfaceMode == MODE_SPI)
	#endif
		SPIwriteByte(lsm_t->xmAddress, subAddress, data,'a');
}

uint8_t gReadByte(LSM9DS0_t* lsm_t, uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)
		return I2CreadByte(lsm_t->gAddress, subAddress);
	else if (lsm_t->interfaceMode == MODE_SPI)
	#endif
		return SPIreadByte(lsm_t->gAddress, subAddress,'g');
}

void gReadBytes(LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)
		I2CreadBytes(lsm_t->gAddress, subAddress, dest, count);
	else if (lsm_t->interfaceMode == MODE_SPI)
	#endif
		SPIreadBytes(lsm_t->gAddress, subAddress, dest, count,'g');
}

uint8_t xmReadByte(LSM9DS0_t* lsm_t, uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)
		return I2CreadByte(lsm_t->xmAddress, subAddress);
	else if (lsm_t->interfaceMode == MODE_SPI)
	#endif
		return SPIreadByte(lsm_t->xmAddress, subAddress,'a');
}

void xmReadBytes(LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	#if(LSM_I2C_SUPPORT==1)
	if (lsm_t->interfaceMode == MODE_I2C)
		I2CreadBytes(lsm_t->xmAddress, subAddress, dest, count);
	else if (lsm_t->interfaceMode == MODE_SPI)
	#endif
		SPIreadBytes(lsm_t->xmAddress, subAddress, dest, count, 'a');
}

void LSM_initSPI()
{

	/* Initialize SPI */
	init_SPI();
	
	/* CS HIGH */
	LSM_CSG_HIGH;
	LSM_CSXM_HIGH;
}

void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data,char chip)
{
	if(chip=='g') // Initiate communication
	{
		set_SPI_Data(subAddress, G, data);
	} 
	else
	{
		set_SPI_Data(subAddress, XM, data);
	}
}

uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress,char chip)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte. 
	// Value is returned to `temp`.
	SPIreadBytes(csPin, subAddress, &temp, 1,chip);
	return temp;
}

void SPIreadBytes(uint8_t csPin, uint8_t subAddress, uint8_t * dest, uint8_t count,char chip)
{
	
	if(chip=='g') // Initiate communication
	{
		int i;
		for(i = 0; i < count; i++)
		{
			dest[i] = get_SPI_Data(subAddress + i, G);
		}
	}
	else
	{
		int i;
		for(i = 0; i < count; i++)
		{
			dest[i] = get_SPI_Data(subAddress + i, XM);
		}
	}

}



#if(LSM_I2C_SUPPORT==1)
void LSM_initI2C()
{
	//Wire.begin();	// Initialize I2C library
}

// Wire.h read and write protocols
void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	//Wire.beginTransmission(address);  // Initialize the Tx buffer
	//Wire.write(subAddress);           // Put slave register address in Tx buffer
	//Wire.write(data);                 // Put data in Tx buffer
	//Wire.endTransmission();           // Send the Tx buffer
}

uint8_t I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	//Wire.beginTransmission(address);         // Initialize the Tx buffer
	//Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	//Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	//Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
	//data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

void I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{  
	//Wire.beginTransmission(address);   // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	//Wire.write(subAddress | 0x80);     // Put slave register address in Tx buffer
	//Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	//uint8_t i = 0;
	//Wire.requestFrom(address, count);  // Read bytes from slave register address 
	//while (Wire.available()) 
	//{
	//	dest[i++] = Wire.read(); // Put read results in the Rx buffer
	//}
}
#endif
