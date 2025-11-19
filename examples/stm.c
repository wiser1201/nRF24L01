#include "nRF24L01.h"
#include "main.h"


extern SPI_HandleTypeDef hspi1;

static void spi_cs(const bool level);
void spi_tx(const uint8_t* data, const unsigned int size, const bool cs);
void spi_rx(uint8_t* buff, const unsigned int size, const bool cs);
void ex_delay_us(const uint32_t us);
uint32_t ex_get_ms(void);
void gpio_set_ce(const bool level);
extern void rf_irq(void);
void EXT2_Callback(void);

void nrf_app_init(void)
{
    nRF24L01_Init_t tx_init = {0};
    tx_init.arc = 3;
    tx_init.ard = ARD_1000US;
    tx_init.data_rate = DR_1MBPS;
    tx_init.rf_freq = 1;
    tx_init.tx_pwr = TX_PWR_0dBm;
    nRF24L01_init(&tx_init);
}

bool nrf_app_send(const uint8_t* data, const uint8_t size)
{
    RF_Return_e ret = nRF24L01_tx(data, size);
    return ret == RF_RET_OK ? true : false;
}

void spi_cs(const bool level)
{
    if (level)
    {
		if (!HAL_GPIO_ReadPin(CS_PORT, CS_PIN))
		{
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
			delay_us(1);
		}
    }
    else
    {
		if (HAL_GPIO_ReadPin(CS_PORT, CS_PIN))
		{
			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
			delay_us(1);
		}
    }
}

void spi_tx(const uint8_t* data, const unsigned int size, const bool cs)
{
	spi_cs(GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, size, 0xFF);
    spi_cs(cs);
}

void spi_rx(uint8_t* buff, const unsigned int size, const bool cs)
{
    const uint32_t nop = 0xFFFFFFFF;
	spi_cs(GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&nop, buff, size, 0xFF);
    spi_cs(cs);
}

void ex_delay_us(const uint32_t us)
{
	delay_us(us);
}

uint32_t ex_get_ms(void)
{
	return HAL_GetTick();
}

void gpio_set_ce(const bool level)
{
	if (level)
	{
		HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
	}
}

void EXT2_Callback(void)
{
	rf_irq();
}