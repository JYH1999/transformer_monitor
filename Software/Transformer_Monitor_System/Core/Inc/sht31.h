#ifndef __SHT31_H_
#define __SHT31_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"

#define SHT_ADDR (0x44<<1)//SHT31 ADDR引脚接地，地址0x88

uint8_t sht31_init(void);

uint8_t sht31_sample(float *t, float *h);

#endif 
