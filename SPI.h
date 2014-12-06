#ifndef SPI
#define SPI

#include <misc.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include "GPIO.h"

//channels
#define NONE 0
#define DISP 1
#define XM 2
#define G 3

void init_SPI(void);
void select_SPI_Channel(uint32_t channel);
uint8_t get_SPI_Data(uint8_t adress, uint8_t channel);
void set_SPI_Data(uint8_t adress, uint8_t channel, uint8_t data);

#endif //SPI
