# nRF24L01 Driver

Simple cross-platform driver for the LCD displays that use nRF24L01
receiver-transmitter. MCU with SPI communication and one IRQ line is required.

# nRF24L01 Integration

User must implement a couple of platform-specific functions:

extern void spi_tx(const uint8_t* data, const unsigned int size, const bool cs);
extern void spi_rx(uint8_t* buff, const unsigned int size, const bool cs);
extern void ex_delay_us(const uint32_t us);
extern uint32_t ex_get_ms(void);
extern void gpio_set_ce(const bool level);
void rf_irq(void);

All this functions can be found in nRF24L01.c file.
Example with implementation platform specific functions
is in the examples folder.