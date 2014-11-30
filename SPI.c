#include "SPI.h"

//LSM9DS0 register definition
#define OUT_X_L_G 0x28
#define OUT_X_H_G 0x29
#define OUT_Y_L_G 0x2A
#define OUT_Y_H_M 0x2B
#define OUT_Z_L_G 0x2C
#define OUT_Z_H_M 0x2D


void init_SPI(void)
{
	uint16_t pinSources[3];
	SPI_InitTypeDef SPI_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	// set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	// transmit in master mode, NSS pin has to be always high
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master; 
  // one packet of data is 8 bits wide	
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; 
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_High; 
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	// SPI frequency is APB2 frequency / 4
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; 
	// data is transmitted MSB first
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStruct); 
	
	pinSources[0] = GPIO_PinSource5;
	pinSources[1] = GPIO_PinSource6;
	pinSources[2] = GPIO_PinSource7;
	
	init_GPIO_AF(GPIOA, RCC_AHB1Periph_GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7, GPIO_AF_SPI1, pinSources, 3);
	init_GPIO(GPIOE, RCC_AHB1Periph_GPIOE, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8, GPIO_Mode_OUT);


	select_SPI_Channel(NONE);

	// enable SPI1
	SPI_Cmd(SPI1, ENABLE); 
}

void select_SPI_Channel(uint32_t channel)
{
	switch(channel)
	{
		case NONE:
		{
			set_GPIO_Pin(GPIOE, GPIO_Pin_6, 1);
			set_GPIO_Pin(GPIOE, GPIO_Pin_7, 1);
			set_GPIO_Pin(GPIOE, GPIO_Pin_8, 1);
		}
			break;
		case DISP:
		{
			set_GPIO_Pin(GPIOE, GPIO_Pin_6, 0);
		}
			break;
		case XM:
		{
			set_GPIO_Pin(GPIOE, GPIO_Pin_7, 0);
		}
			break;
		case G:
		{
			set_GPIO_Pin(GPIOE, GPIO_Pin_8, 0);
		}
			break;
	}
}

// read register from SPI channel
uint8_t get_SPI_Data(uint8_t adress, uint8_t channel)
{
	//select channel
	select_SPI_Channel(channel); 
	//set adress to "write" mode by adding 1000 0000 (0x80 or 128)
  adress = 0x80 | adress; 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	//send adress
	SPI_I2S_SendData(SPI1, adress); 

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	//Clear RXNE bit
	SPI_I2S_ReceiveData(SPI1); 

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	//Dummy byte to generate clock
	SPI_I2S_SendData(SPI1, 0x00); 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));

	//deselect channel
	select_SPI_Channel(NONE); 
	//read data
	return  SPI_I2S_ReceiveData(SPI1); 
}

//write register to SPI channel
void set_SPI_Data(uint8_t adress, uint8_t channel, uint8_t data)
{
	//select chanel
	select_SPI_Channel(channel); 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	//send adress
	SPI_I2S_SendData(SPI1, adress); 

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	//Clear RXNE bit
	SPI_I2S_ReceiveData(SPI1); 

	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	//send data
	SPI_I2S_SendData(SPI1, data); 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
	//Clear RXNE bit
	SPI_I2S_ReceiveData(SPI1); 
	//deselect all
	select_SPI_Channel(NONE); 
}
