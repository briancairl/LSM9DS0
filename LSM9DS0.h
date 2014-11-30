#ifndef __SFE_LSM9DS0_H__
#define __SFE_LSM9DS0_H__

#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "SPI.h"


#ifndef LSM_I2C_SUPPORT
#define LSM_I2C_SUPPORT 	0
///		Im porting this for SPI, only.
///		For now and maybe forever
///		Huck the Faters
///		Sorry, not sorry
///		#BlumpkinSpiceLatte
///		Go ahead, make this 1
///		I dare you...
#endif


/* SPI on STM32F4-Discovery board */
#define LSM_SPI						SPI1
#define LSM_SPI_PINSPACK	TM_SPI_PinsPack_1


/* CS pin on to IMU */
#ifndef LSM_CSG_PIN
#define LSM_CSG_RCC				RCC_AHB1Periph_GPIOE
#define LSM_CSG_PORT			GPIOE
#define LSM_CSG_PIN				GPIO_Pin_8
#endif

#ifndef LSM_CSXM_PIN
#define LSM_CSXM_RCC			RCC_AHB1Periph_GPIOE
#define LSM_CSXM_PORT			GPIOE
#define LSM_CSXM_PIN			GPIO_Pin_7
#endif

/* CS pin settings */
#define LSM_CSG_LOW				LSM_CSG_PORT->BSRRH  = LSM_CSG_PIN
#define LSM_CSG_HIGH			LSM_CSG_PORT->BSRRL  = LSM_CSG_PIN
#define LSM_CSXM_LOW			LSM_CSXM_PORT->BSRRH = LSM_CSXM_PIN
#define LSM_CSXM_HIGH			LSM_CSXM_PORT->BSRRL = LSM_CSXM_PIN


////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_XM			0x0F
#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_XM		0x1F
#define CTRL_REG1_XM		0x20
#define CTRL_REG2_XM		0x21
#define CTRL_REG3_XM		0x22
#define CTRL_REG4_XM		0x23
#define CTRL_REG5_XM		0x24
#define CTRL_REG6_XM		0x25
#define CTRL_REG7_XM		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

// The LSM9DS0 functions over both I2C or SPI. This library supports both.
// But the interface mode used must be sent to the LSM9DS0 constructor. Use
// one of these two as the first parameter of the constructor.
typedef enum interface_mode_x
{
	MODE_SPI = 0,
#if(LSM_I2C_SUPPORT==1)
	MODE_I2C = 1
#endif
} interface_mode;


// gyro_scale defines the possible full-scale ranges of the gyroscope:
typedef enum gyro_scale_x
{
	G_SCALE_245DPS,		// 00:  245 degrees per second
	G_SCALE_500DPS,		// 01:  500 dps
	G_SCALE_2000DPS,	// 10:  2000 dps
} gyro_scale;

// accel_scale defines all possible FSR's of the accelerometer:
typedef enum accel_scale_x
{
	A_SCALE_2G,	// 000:  2g
	A_SCALE_4G,	// 001:  4g
	A_SCALE_6G,	// 010:  6g
	A_SCALE_8G,	// 011:  8g
	A_SCALE_16G	// 100:  16g
} accel_scale;

// mag_scale defines all possible FSR's of the magnetometer:
typedef enum mag_scale_x
{
	M_SCALE_2GS,	// 00:  2Gs
	M_SCALE_4GS, 	// 01:  4Gs
	M_SCALE_8GS,	// 10:  8Gs
	M_SCALE_12GS,	// 11:  12Gs
} mag_scale;

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
typedef enum gyro_odr_x
{							// ODR (Hz) --- Cutoff
	G_ODR_95_BW_125  = 0x0, //   95         12.5
	G_ODR_95_BW_25   = 0x1, //   95          25
	// 0x2 and 0x3 define the same data rate and bandwidth
	G_ODR_190_BW_125 = 0x4, //   190        12.5
	G_ODR_190_BW_25  = 0x5, //   190         25
	G_ODR_190_BW_50  = 0x6, //   190         50
	G_ODR_190_BW_70  = 0x7, //   190         70
	G_ODR_380_BW_20  = 0x8, //   380         20
	G_ODR_380_BW_25  = 0x9, //   380         25
	G_ODR_380_BW_50  = 0xA, //   380         50
	G_ODR_380_BW_100 = 0xB, //   380         100
	G_ODR_760_BW_30  = 0xC, //   760         30
	G_ODR_760_BW_35  = 0xD, //   760         35
	G_ODR_760_BW_50  = 0xE, //   760         50
	G_ODR_760_BW_100 = 0xF, //   760         100
} gyro_odr;

// accel_oder defines all possible output data rates of the accelerometer:
typedef enum accel_odr_x
{
	A_POWER_DOWN, 	// Power-down mode (0x0)
	A_ODR_3125,		// 3.125 Hz	(0x1)
	A_ODR_625,		// 6.25 Hz (0x2)
	A_ODR_125,		// 12.5 Hz (0x3)
	A_ODR_25,		// 25 Hz (0x4)
	A_ODR_50,		// 50 Hz (0x5)
	A_ODR_100,		// 100 Hz (0x6)
	A_ODR_200,		// 200 Hz (0x7)
	A_ODR_400,		// 400 Hz (0x8)
	A_ODR_800,		// 800 Hz (9)
	A_ODR_1600		// 1600 Hz (0xA)
} accel_odr;

  // accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
typedef enum accel_abw_x
{
	A_ABW_773,		// 773 Hz (0x0)
	A_ABW_194,		// 194 Hz (0x1)
	A_ABW_362,		// 362 Hz (0x2)
	A_ABW_50,		//  50 Hz (0x3)
} accel_abw;


// mag_oder defines all possible output data rates of the magnetometer:
typedef enum mag_odr_x
{
	M_ODR_3125,	// 3.125 Hz (0x00)
	M_ODR_625,	// 6.25 Hz (0x01)
	M_ODR_125,	// 12.5 Hz (0x02)
	M_ODR_25,	// 25 Hz (0x03)
	M_ODR_50,	// 50 (0x04)
	M_ODR_100,	// 100 Hz (0x05)
} mag_odr;


typedef struct LMS9DS0_x
{		
	// xmAddress and gAddress store the I2C address or SPI chip select pin
	// for each sensor.
	uint8_t xmAddress;
	uint8_t gAddress;
	
	// interfaceMode keeps track of whether we're using SPI or I2C to talk
	interface_mode interfaceMode;
	
	// gScale, aScale, and mScale store the current scale range for each 
	// sensor. Should be updated whenever that value changes.
	gyro_scale 	gScale;
	accel_scale aScale;
	mag_scale 	mScale;
	
	// gRes, aRes, and mRes store the current resolution for each sensor. 
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float 	gRes;
	float 	aRes;
	float 	mRes;

	// We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!
	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx;
	int16_t	gy; 
	int16_t	gz; // x, y, and z axis readings of the gyroscope
	int16_t ax;
	int16_t	ay; 
	int16_t	az; // x, y, and z axis readings of the accelerometer
	int16_t mx; 
	int16_t	my; 
	int16_t	mz; // x, y, and z axis readings of the magnetometer
    
    int16_t temperature;
	float 	abias[3];
    float 	gbias[3];

} LSM9DS0_t;


// LSM9DS0 -- LSM9DS0 class constructor
// The constructor will set up a handful of private variables, and set the
// communication mode as well.
// Input:
//	- interface = Either MODE_SPI or MODE_I2C, whichever you're using
//				to talk to the IC.
//	- gAddr = If MODE_I2C, this is the I2C address of the gyroscope.
// 				If MODE_SPI, this is the chip select pin of the gyro (CSG)
//	- xmAddr = If MODE_I2C, this is the I2C address of the accel/mag.
//				If MODE_SPI, this is the cs pin of the accel/mag (CSXM)
void LSM9DS0_Init( LSM9DS0_t* lsm_t, interface_mode interface, uint8_t gAddr, uint8_t xmAddr );

// begin() -- Initialize the gyro, accelerometer, and magnetometer.
// This will set up the scale and output rate of each sensor. It'll also
// "turn on" every sensor and every axis of every sensor.
// Input:
//	- gScl = The scale of the gyroscope. This should be a gyro_scale value.
//	- aScl = The scale of the accelerometer. Should be a accel_scale value.
//	- mScl = The scale of the magnetometer. Should be a mag_scale value.
//	- gODR = Output data rate of the gyroscope. gyro_odr value.
//	- aODR = Output data rate of the accelerometer. accel_odr value.
//	- mODR = Output data rate of the magnetometer. mag_odr value.
// Output: The function will return an unsigned 16-bit value. The most-sig
//		bytes of the output are the WHO_AM_I reading of the accel. The
//		least significant two bytes are the WHO_AM_I reading of the gyro.
// All parameters have a defaulted value, so you can call just "begin()".
// Default values are FSR's of:  245DPS, 2g, 2Gs; ODRs of 95 Hz for 
// gyro, 100 Hz for accelerometer, 100 Hz for magnetometer.
// Use the return value of this function to verify communication.
typedef struct LMS9DS0_Init_x
{
	gyro_scale 	gScl;
	accel_scale aScl;
	mag_scale 	mScl;
	gyro_odr 		gODR; 
	accel_odr 	aODR; 
	mag_odr 		mODR;
} LMS9DS0_Init_t;
uint16_t LSM9DS0_begin( LSM9DS0_t* lsm_t, LMS9DS0_Init_t* init_t );

// readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling readGyro().
void LSM9DS0_readGyro( LSM9DS0_t* lsm_t );

// readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling readAccel().
void LSM9DS0_readAccel( LSM9DS0_t* lsm_t );

// readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling readMag().
void LSM9DS0_readMag( LSM9DS0_t* lsm_t );

// readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling readTemp().
void LSM9DS0_readTemp( LSM9DS0_t* lsm_t );

// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//	- gyro = A signed 16-bit raw reading from the gyroscope.
float calcGyro( LSM9DS0_t* lsm_t, int16_t gyro);

// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//	- accel = A signed 16-bit raw reading from the accelerometer.
float calcAccel( LSM9DS0_t* lsm_t, int16_t accel);

// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//	- mag = A signed 16-bit raw reading from the magnetometer.
float calcMag( LSM9DS0_t* lsm_t, int16_t mag);

// setGyroScale() -- Set the full-scale range of the gyroscope.
// This function can be called to set the scale of the gyroscope to 
// 245, 500, or 200 degrees per second.
// Input:
// 	- gScl = The desired gyroscope scale. Must be one of three possible
//		values from the gyro_scale enum.
void setGyroScale( LSM9DS0_t* lsm_t, gyro_scale gScl);

// setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
// 	- aScl = The desired accelerometer scale. Must be one of five possible
//		values from the accel_scale enum.
void setAccelScale( LSM9DS0_t* lsm_t, accel_scale aScl);

// setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
// 	- mScl = The desired magnetometer scale. Must be one of four possible
//		values from the mag_scale enum.
void setMagScale( LSM9DS0_t* lsm_t, mag_scale mScl);

// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//	- gRate = The desired output rate and cutoff frequency of the gyro.
//		Must be a value from the gyro_odr enum (check above, there're 14).
void setGyroODR( LSM9DS0_t* lsm_t, gyro_odr gRate);

// setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//	- aRate = The desired output rate of the accel.
//		Must be a value from the accel_odr enum (check above, there're 11).
void setAccelODR( LSM9DS0_t* lsm_t, accel_odr aRate); 	

    // setAccelABW() -- Set the anti-aliasing filter rate of the accelerometer
// Input:
//	- abwRate = The desired anti-aliasing filter rate of the accel.
//		Must be a value from the accel_abw enum (check above, there're 4).
void setAccelABW( LSM9DS0_t* lsm_t, accel_abw abwRate);



// setMagODR() -- Set the output data rate of the magnetometer
// Input:
//	- mRate = The desired output rate of the mag.
//		Must be a value from the mag_odr enum (check above, there're 6).
void setMagODR( LSM9DS0_t* lsm_t, mag_odr mRate);

// configGyroInt() -- Configure the gyro interrupt output.
// Triggers can be set to either rising above or falling below a specified
// threshold. This function helps setup the interrupt configuration and 
// threshold values for all axes.
// Input:
//	- int1Cfg = A 8-bit value that is sent directly to the INT1_CFG_G
//		register. This sets AND/OR and high/low interrupt gen for each axis
//	- int1ThsX = 16-bit interrupt threshold value for x-axis
//	- int1ThsY = 16-bit interrupt threshold value for y-axis
//	- int1ThsZ = 16-bit interrupt threshold value for z-axis
//	- duration = Duration an interrupt holds after triggered. This value
// 		is copied directly into the INT1_DURATION_G register.
// Before using this function, read about the INT1_CFG_G register and
// the related INT1* registers in the LMS9DS0 datasheet.
void configGyroInt( LSM9DS0_t* lsm_t, 
	uint8_t 		int1Cfg, 
	uint16_t 		int1ThsX,
	uint16_t 		int1ThsY,
	uint16_t 		int1ThsZ,
	uint8_t 		duration
);


void calLSM9DS0( LSM9DS0_t* lsm_t, float gbias[3], float abias[3]);



// initGyro() -- Sets up the gyroscope to begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
//		95 Hz ODR, 12.5 Hz cutoff frequency.
//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//		set to 7.2 Hz (depends on ODR).
//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//		active high). Data-ready output enabled on DRDY_G.
//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
void initGyro( LSM9DS0_t* lsm_t );

// initAccel() -- Sets up the accelerometer to begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//		all axes enabled.
//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
void initAccel( LSM9DS0_t* lsm_t );

// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void initMag( LSM9DS0_t* lsm_t );

// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
// 	- subAddress = Register to be read from.
// Output:
// 	- An 8-bit value read from the requested address.
uint8_t gReadByte( LSM9DS0_t* lsm_t, uint8_t subAddress);

// gReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the gyroscope.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void gReadBytes( LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t * dest, uint8_t count);

// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void gWriteByte( LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t data);

// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//	- subAddress = Register to be read from.
// Output:
//	- An 8-bit value read from the requested register.
uint8_t xmReadByte( LSM9DS0_t* lsm_t, uint8_t subAddress);

// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void xmReadBytes( LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t * dest, uint8_t count);

// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void xmWriteByte( LSM9DS0_t* lsm_t, uint8_t subAddress, uint8_t data);

// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void calcgRes( LSM9DS0_t* lsm_t );

// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void calcmRes( LSM9DS0_t* lsm_t );

// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void calcaRes(  LSM9DS0_t* lsm_t );

/// @todo ? needed ?

///////////////////
// SPI Functions //
///////////////////
// initSPI() -- Initialize the SPI hardware.
// This function will setup all SPI pins and related hardware.
void LSM_initSPI(void);

// SPIwriteByte() -- Write a byte out of SPI to a register in the device
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
void SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data, char chip);

// SPIreadByte() -- Read a single byte from a register over SPI.
// Input:
//	- csPin = The chip select pin of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress, char chip);

// SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
// Input:
//	- csPin = The chip select pin of a slave device.
//	- subAddress = The register to begin reading.
// 	- * dest = Pointer to an array where we'll store the readings.
//	- count = Number of registers to be read.
// Output: No value is returned by the function, but the registers read are
// 		all stored in the *dest array given.
void SPIreadBytes(uint8_t csPin, uint8_t subAddress, uint8_t * dest, uint8_t count, char chip);

#if(LSM_I2C_SUPPORT==1)
///////////////////
// I2C Functions //
///////////////////
// initI2C() -- Initialize the I2C hardware.
// This function will setup all I2C pins and related hardware.
void LSM_initI2C();

// I2CwriteByte() -- Write a byte out of I2C to a register in the device
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

// I2CreadByte() -- Read a single byte from a register over I2C.
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);

// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to begin reading.
// 	- * dest = Pointer to an array where we'll store the readings.
//	- count = Number of registers to be read.
// Output: No value is returned by the function, but the registers read are
// 		all stored in the *dest array given.
void I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
#endif

#endif // SFE_LSM9DS0_H //
