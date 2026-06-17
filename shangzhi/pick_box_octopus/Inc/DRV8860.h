#ifndef __DRV8860_H
#define __DRV8860_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#define ENTER_CRITICAL() __disable_irq()
#define EXIT_CRITICAL() __enable_irq()
typedef struct
{
	GPIO_TypeDef *latch_port;
	uint16_t latch_pin;

	GPIO_TypeDef *din_port;
	uint16_t din_pin;

	GPIO_TypeDef *clk_port;
	uint16_t clk_pin;
	// uint8_t state;              // 缓存 OUT1~OUT8 状态, 0000 0101，OUT1 和 OUT3 打开
	uint8_t out_mask;
} DRV8860_HandleTypeDef;

typedef enum
{
	DRV8860_CHANNEL_1 = 1,
	DRV8860_CHANNEL_2,
	DRV8860_CHANNEL_3,
	DRV8860_CHANNEL_4,
	DRV8860_CHANNEL_5,
	DRV8860_CHANNEL_6,
	DRV8860_CHANNEL_7,
	DRV8860_CHANNEL_8,
	DRV8860_CHANNEL_9,
	DRV8860_CHANNEL_10,
	DRV8860_CHANNEL_11,
	DRV8860_CHANNEL_12,
	DRV8860_CHANNEL_13,
	DRV8860_CHANNEL_14,
	DRV8860_CHANNEL_15,
	DRV8860_CHANNEL_16,
	DRV8860_CHANNEL_17,
	DRV8860_CHANNEL_18,
	DRV8860_CHANNEL_19,
	DRV8860_CHANNEL_20,
	DRV8860_CHANNEL_21,
	DRV8860_CHANNEL_22,
	DRV8860_CHANNEL_23,
	DRV8860_CHANNEL_24,
	DRV8860_CHANNEL_25,
	DRV8860_CHANNEL_26,
	DRV8860_CHANNEL_27,
	DRV8860_CHANNEL_28,
	DRV8860_CHANNEL_29,
	DRV8860_CHANNEL_30,
	DRV8860_CHANNEL_31,
	DRV8860_CHANNEL_32,
} DRV8860_ch_e;

extern DRV8860_HandleTypeDef drv_1;
extern DRV8860_HandleTypeDef drv_2;
extern DRV8860_HandleTypeDef drv_3;

void DRV8860_init(DRV8860_HandleTypeDef *dev);
void channel_set(DRV8860_HandleTypeDef *dev, uint8_t channel, uint8_t value);

#endif
