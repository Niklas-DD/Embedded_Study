#include "DRV8860.h"
#include "gpio.h"

// MOSI->DIN		SCK->CLK	CS->LATCH
DRV8860_HandleTypeDef drv_1 = {
    .latch_port = SPI_CS_GPIO_Port,
    .latch_pin = SPI_CS_Pin,
    .din_port = SPI_MOSI_GPIO_Port,
    .din_pin = SPI_MOSI_Pin,
    .clk_port = SPI_SCLK_GPIO_Port,
    .clk_pin = SPI_SCLK_Pin,
};
DRV8860_HandleTypeDef drv_2 = {
	.latch_port = SPI1_CS_GPIO_Port,
    .latch_pin = SPI1_CS_Pin,
    .din_port = SPI1_MOSI_GPIO_Port,
    .din_pin = SPI1_MOSI_Pin,
    .clk_port = SPI1_SCLK_GPIO_Port,
    .clk_pin = SPI1_SCLK_Pin,
};
static void DIN_Write(DRV8860_HandleTypeDef *dev, uint8_t bit_value)
{
    HAL_GPIO_WritePin(dev->din_port, dev->din_pin, (GPIO_PinState)bit_value);
}

static void CLK_Write(DRV8860_HandleTypeDef *dev, uint8_t bit_value)
{
    HAL_GPIO_WritePin(dev->clk_port, dev->clk_pin, (GPIO_PinState)bit_value);
}

static void LATCH_Write(DRV8860_HandleTypeDef *dev, uint8_t bit_value)
{
    HAL_GPIO_WritePin(dev->latch_port, dev->latch_pin, (GPIO_PinState)bit_value);
}

static void LATCH_Pulse(DRV8860_HandleTypeDef *dev)
{
    LATCH_Write(dev, 1);
    for (volatile int i = 0; i < 80; i++)
    {
    }
    LATCH_Write(dev, 0);
    for (volatile int i = 0; i < 80; i++)
    {
    }
}

void DRV_WriteBit(DRV8860_HandleTypeDef *dev, uint8_t bit)
{
    DIN_Write(dev, bit);
    for (volatile int i = 0; i < 80; i++)
    {
    }
    CLK_Write(dev, 1);
    for (volatile int i = 0; i < 80; i++)
    {
    }
    CLK_Write(dev, 0);
    for (volatile int i = 0; i < 80; i++)
    {
    }
}

void DRV8860_init(DRV8860_HandleTypeDef *dev)
{
    LATCH_Write(dev, 0);
    CLK_Write(dev, 0);
    DIN_Write(dev, 0);
}

static void DRV_WriteByte_MSB(DRV8860_HandleTypeDef *dev, uint8_t data)
{
    for (int i = 7; i >= 0; i--)
        DRV_WriteBit(dev, (data >> i) & 0x01);
}

void DRV8860_Write(DRV8860_HandleTypeDef *dev, uint8_t bit) // 8路输出
{
    LATCH_Write(dev, 0);
    CLK_Write(dev, 0);
    DRV_WriteByte_MSB(dev, bit);
    LATCH_Pulse(dev);
}

void channel_set(DRV8860_HandleTypeDef *dev, uint8_t channel, uint8_t value)
{
    if (channel < 1 || channel > 8)
        return;

    uint8_t bit = (uint8_t)(1u << (channel - 1));

    ENTER_CRITICAL();

    if (value)
        dev->out_mask |= bit;
    else
        dev->out_mask &= (uint8_t)~bit;

    DRV8860_Write(dev, dev->out_mask);

    EXIT_CRITICAL();
}
