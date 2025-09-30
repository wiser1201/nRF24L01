#include "nRF24L01.h"

#define REG_CONFIG 0x0
#define REG_EN_AA 0x1
#define REG_EN_RXADDR 0x2
#define REG_SETUP_AW 0x3
#define REG_SETUP_RETR 0x4
#define REG_RF_CH 0x5
#define REG_RF_SETUP 0x6
#define REG_STATUS 0x7
#define REG_OBSERVE_TX 0x8
#define REG_RPD 0x9
#define REG_RX_ADDR_P0 0xA
#define REG_RX_ADDR_P1 0xB
#define REG_RX_ADDR_P2 0xC
#define REG_RX_ADDR_P3 0xD
#define REG_RX_ADDR_P4 0xE
#define REG_RX_ADDR_P5 0xF
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0

#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0

#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0

#define AW_MSB 1
#define AW_LSB 0

#define ARD_MSB 7
#define ARD_LSB 4
#define ARC_MSB 3
#define ARC_LSB 0

#define CONT_WAVE 7
#define RF_DR_LOW 5
#define PLL_LOCK 4
#define RF_DR_HIGH 3
#define RF_PWR_MSB 2
#define RF_PWR_LSB 1

#define RX_DR_IF 6
#define TX_DS_IF 5
#define MAX_RT_IF 4
#define RX_P_NO_3 3
#define RX_P_NO_2 2
#define RX_P_NO_1 1
#define TX_FULL 0

#define TX_REUSE 6
#define TX_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0

#define DPL_P5 5
#define DPL_P4 4
#define DPL_P3 3
#define DPL_P2 2
#define DPL_P1 1
#define DPL_P0 0

#define EN_DPL 2
#define EN_ACK_PAY 1
#define EN_DYN_ACK 0

#define LOW 0
#define HIGH 1

#define RF_FREQ_MIN 1
#define RF_FREQ_MAX 124

#define ALL_PIPES_MASK 0b111111

#define US_IN_MS 1000UL

#define FLAG_IS_SET(reg, flag) ((reg) & (1 << (flag)))
#define FLAG_SET(reg, flag)    ((reg) |= (1 << (flag)))
#define FLAG_CLEAR(reg, flag)  ((reg) &= ~(1 << (flag)))
#define FLAG_TOGGLE(reg, flag) ((reg) ^= (1 << (flag)))

typedef enum
{
    OP_READ_REG = 0b0000000,
    OP_WRITE_REG = 0b0010000,
    OP_RX_PL = 0b01100001,
    OP_TX_PL = 0b10100000,
    OP_FLUSH_RX = 0b11100010,
    OP_FLUSH_TX = 0b11100001,
    OP_REUSE_TX_PL = 0b11100011,
    OP_RX_PL_WIDTH = 0b01100000,
    OP_RX_ACK_TX_PL = 0b10101000,
    OP_TX_PL_NO_ACK = 0b10110000,
    OP_NOP = 0b11111111

} Operation_e;

typedef enum
{
    MODE_UNDEFINED,
    MODE_POWER_DOWN,
    MODE_STANDBY_I,
    MODE_STANDBY_II,
    MODE_RX_TX,
    MODE_RX_DR,
    MODE_TX_DS,
    MODE_IRQ
} ChipMode_e;

static volatile ChipMode_e chip_mode;
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
extern void spi_tx(const uint8_t* data, const unsigned int size, const bool cs);
extern void spi_rx(uint8_t* buff, const unsigned int size, const bool cs);
extern void ex_delay_us(const uint32_t us);
extern void gpio_set_ce(const bool level);
void rf_irq(void);

static uint32_t read_reg(const uint8_t reg_addr, const uint8_t reg_size);
static void write_reg(const uint8_t reg_addr, const uint32_t reg_data, const uint8_t reg_size);
static void rx_pl(uint8_t* buff, const uint8_t size);
static void tx_pl(const uint8_t* data, const uint8_t size);
static void flush_rx();
static void flush_tx();
static void reuse_tx_pl();
static uint8_t rx_pl_width();
static void rx_ack_tx_pl();
static void tx_pl_no_ack();
static uint8_t nop();

static void tx_op(const Operation_e op, const uint8_t reg_addr);

static RF_Return_e switch_chip_mode(const ChipMode_e mode);
static void to_standby(void);
static void to_powerdown(void);
static void to_active(void);
static void rf_irq_handler(void);
static uint32_t parse_buff(const uint8_t* buff, const uint8_t size);


RF_Return_e nRF24L01_init(const nRF24L01_Init_t *config)
{
    gpio_set_ce(LOW);

    uint8_t config_reg = 0;
    /* if (config->enable_rx_dr_irq == false)
    {
        config_reg |= (1 << MASK_RX_DR);
    }
    if (config->enable_tx_ds_irq == false)
    {
        config_reg |= (1 << MASK_TX_DS);
    }
    if (config->enable_max_rt_irq == false)
    {
        config_reg |= (1 << MASK_MAX_RT);
    } */
    config_reg |= (1 << EN_CRC);

    uint8_t rx_pipe_reg = 0;
    if (config->rx_config.rx_active_pipes > 0 && config->rx_config.rx_active_pipes <= ALL_PIPES_MASK)
    {
        rx_pipe_reg = config->rx_config.rx_active_pipes;
    }

    uint8_t addr_width_reg = 0;
    addr_width_reg |= (1 << AW_MSB); // aw is 4 bytes

    uint8_t retr_reg = 0;
    retr_reg |= (0b0011 << ARD_LSB); // retry every 1 ms
    retr_reg |= (0b0011 << ARC_LSB); // retry 3 times

    uint8_t rf_freq_reg = 0;
    if (config->rf_freq >= RF_FREQ_MIN && config->rf_freq <= RF_FREQ_MAX)
    {
        rf_freq_reg = config->rf_freq;
    }
    else
    {
        rf_freq_reg = 2;
    }

    uint8_t rf_config_reg = 0;
    switch (config->data_rate)
    {
    case DR_250KBPS:
        rf_config_reg |= (1 << RF_DR_LOW);
        break;
    case DR_2MBPS:
        rf_config_reg |= (1 << RF_DR_HIGH);
        break;
    default:
    case DR_1MBPS:
        break;
    }
    switch (config->tx_config.tx_pwr)
    {
    case TX_PWR_NEG_18dBm:
        break;
    case TX_PWR_NEG_12dBm:
        rf_config_reg |= (1 << RF_PWR_LSB);
        break;
    case TX_PWR_NEG_6dBm:
        rf_config_reg |= (1 << RF_PWR_MSB);
        break;
    default:
    case TX_PWR_0dBm:
        rf_config_reg |= (1 << RF_PWR_LSB);
        rf_config_reg |= (1 << RF_PWR_MSB);
        break;
    }

    uint32_t rx_addr_p0_reg = config->rx_config.rx_p0_addr;
#ifdef USE_RX_MULTI
    uint32_t rx_addr_p1_reg = config->rx_config.rx_p1_addr;
    uint8_t rx_addr_p2_reg = config->rx_config.rx_p2_addr;
    uint8_t rx_addr_p3_reg = config->rx_config.rx_p3_addr;
    uint8_t rx_addr_p4_reg = config->rx_config.rx_p4_addr;
    uint8_t rx_addr_p5_reg = config->rx_config.rx_p5_addr;
#endif

    uint8_t rx_pl_p0_size_reg = config->rx_config.rx_pl_p0_size;
#ifdef USE_RX_MULTI
    uint8_t rx_pl_p1_size_reg = config->rx_config.rx_pl_p1_size;
    uint8_t rx_pl_p2_size_reg = config->rx_config.rx_pl_p2_size;
    uint8_t rx_pl_p3_size_reg = config->rx_config.rx_pl_p3_size;
    uint8_t rx_pl_p4_size_reg = config->rx_config.rx_pl_p4_size;
    uint8_t rx_pl_p5_size_reg = config->rx_config.rx_pl_p5_size;
#endif

    uint8_t feat_reg = 0;
    uint8_t rx_dpl_reg = 0;
    if (config->rx_config.rx_dpl > 0 && config->rx_config.rx_dpl < ALL_PIPES_MASK)
    {
        rx_dpl_reg = config->rx_config.rx_dpl;
        feat_reg |= (1 << EN_DPL);
    }

    write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
    write_reg(REG_EN_RXADDR, rx_pipe_reg, sizeof(rx_pipe_reg));
    write_reg(REG_SETUP_AW, addr_width_reg, sizeof(addr_width_reg));
    write_reg(REG_SETUP_RETR, retr_reg, sizeof(retr_reg));
    write_reg(REG_RF_CH, rf_freq_reg, sizeof(rf_freq_reg));
    write_reg(REG_RF_SETUP, rf_config_reg, sizeof(rf_config_reg));
    write_reg(REG_RX_ADDR_P0, rx_addr_p0_reg, sizeof(rx_addr_p0_reg));
#ifdef USE_RX_MULTI
    write_reg(REG_RX_ADDR_P1, rx_addr_p1_reg, sizeof(rx_addr_p1_reg));
    write_reg(REG_RX_ADDR_P2, rx_addr_p2_reg, sizeof(rx_addr_p2_reg));
    write_reg(REG_RX_ADDR_P3, rx_addr_p3_reg, sizeof(rx_addr_p3_reg));
    write_reg(REG_RX_ADDR_P4, rx_addr_p4_reg, sizeof(rx_addr_p4_reg));
    write_reg(REG_RX_ADDR_P5, rx_addr_p5_reg, sizeof(rx_addr_p5_reg));
#endif
    write_reg(REG_RX_PW_P0, rx_pl_p0_size_reg, sizeof(rx_pl_p0_size_reg));
#ifdef USE_RX_MULTI
    write_reg(REG_RX_PW_P1, rx_pl_p1_size_reg, sizeof(rx_pl_p1_size_reg));
    write_reg(REG_RX_PW_P2, rx_pl_p2_size_reg, sizeof(rx_pl_p2_size_reg));
    write_reg(REG_RX_PW_P3, rx_pl_p3_size_reg, sizeof(rx_pl_p3_size_reg));
    write_reg(REG_RX_PW_P4, rx_pl_p4_size_reg, sizeof(rx_pl_p4_size_reg));
    write_reg(REG_RX_PW_P5, rx_pl_p5_size_reg, sizeof(rx_pl_p5_size_reg));
#endif
    write_reg(REG_FEATURE, feat_reg, sizeof(feat_reg));
    write_reg(REG_DYNPD, rx_dpl_reg, sizeof(rx_dpl_reg));

    switch_chip_mode(MODE_POWER_DOWN);
    return RF_RET_OK;
}

RF_Return_e switch_chip_mode(const ChipMode_e mode)
{
    switch (chip_mode)
    {
    case MODE_UNDEFINED:
    {
        switch (mode)
        {
        case MODE_POWER_DOWN:
        {
            chip_mode = mode;
            break;
        }
        }
        break;
    }
    case MODE_POWER_DOWN:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            standby();
            chip_mode = mode;
            break;
        }
        }
        break;
    }
    case MODE_STANDBY_I:
    {
        switch (mode)
        {
        case MODE_POWER_DOWN:
        {
            powerdown();
            chip_mode = mode;
            break;
        }
        case MODE_RX_TX:
        {
            to_active();
            chip_mode = mode;
            break;
        }
        }
        break;
    }
    case MODE_RX_TX:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            gpio_set_ce(LOW);
            chip_mode = mode;
            break;
        }
        case MODE_IRQ:
        {
            chip_mode = mode;
            break;
        }
        }
        break;
    }
    case MODE_IRQ:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            gpio_set_ce(LOW);
            chip_mode = mode;
            break;
        }
        case MODE_TX_DS:
        case MODE_RX_DR:
        {
            chip_mode = mode;
            break;
        }
        }
        break;
    }
    case MODE_TX_DS:
    case MODE_RX_DR:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            gpio_set_ce(LOW);
            chip_mode = mode;
            break;
        }
        }
        break;
    }
    }
    if (chip_mode != mode)
    {
        return RF_RET_FAIL;
    }
    return RF_RET_OK;
}

void to_standby(void)
{
    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(uint8_t));
    FLAG_SET(config_reg, PWR_UP);
    write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
    ex_delay_us(US_IN_MS * 10);
}

void to_powerdown(void)
{
    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(uint8_t));
    FLAG_CLEAR(config_reg, PWR_UP);
    write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
}

void to_active(void)
{
    gpio_set_ce(HIGH);
    ex_delay_us(20);
    gpio_set_ce(LOW);
}

RF_Return_e nRF24L01_tx(const uint32_t tx_addr, const uint8_t* data, const uint8_t size)
{
    if (switch_chip_mode(MODE_STANDBY_I) != RF_RET_OK)
    {
        return RF_RET_NO_INIT;
    }
    
    gpio_set_ce(LOW);

    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(uint8_t));
    if (FLAG_IS_SET(config_reg, PRIM_RX))
    {
        FLAG_CLEAR(config_reg, PRIM_RX);
        write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
    }   

    uint8_t rx_pipe_reg = read_reg(REG_EN_RXADDR, sizeof(uint8_t));
    if (FLAG_IS_SET(rx_pipe_reg, ERX_P0) == false)
    {
        FLAG_SET(rx_pipe_reg, ERX_P0);
        write_reg(REG_EN_RXADDR, rx_pipe_reg, sizeof(rx_pipe_reg));
    }    

    write_reg(REG_TX_ADDR, tx_addr, sizeof(tx_addr));
    write_reg(REG_RX_ADDR_P0, tx_addr, sizeof(tx_addr));
    tx_pl(data, size);

    switch_chip_mode(MODE_RX_TX);
    
    while (chip_mode == MODE_RX_TX) {}
    
    rf_irq_handler();

    const bool success = chip_mode == MODE_TX_DS;
    switch_chip_mode(MODE_STANDBY_I);

    if (!success) return RF_RET_FAIL;
    return RF_RET_OK;
}

void rf_irq(void)
{
    chip_mode = MODE_IRQ;
}

void rf_irq_handler(void)
{
    uint8_t status_reg = nop();
    if (FLAG_SET(status_reg, RX_DR_IF))
    {
        FLAG_CLEAR(status_reg, RX_DR_IF);
        switch_chip_mode(MODE_RX_DR);
    }
    if (FLAG_SET(status_reg, TX_DS_IF))
    {
        FLAG_CLEAR(status_reg, TX_DS_IF);
        switch_chip_mode(MODE_TX_DS);
    }
    if (FLAG_SET(status_reg, MAX_RT_IF))
    {
        FLAG_CLEAR(status_reg, MAX_RT_IF);
        switch_chip_mode(MODE_STANDBY_I);
    }
    write_reg(REG_STATUS, status_reg, sizeof(status_reg));
}

/**
 * @brief  No operation. NOP = send 0xFF
 *          and receive status register in parallel
 * @retval Status register
 */
uint8_t nop()
{
    uint8_t status_reg;
    spi_rx(&status_reg, sizeof(status_reg), true);
    return status_reg;
}

uint32_t read_reg(const uint8_t reg_addr, const uint8_t reg_size)
{
    if (reg_addr == 0 || reg_size == 0) return 0;

    uint8_t buff[reg_size];
    tx_op(OP_READ_REG, reg_addr);
    spi_rx(buff, reg_size, true);
    return parse_buff(buff, reg_size);
}

void write_reg(const uint8_t reg_addr, const uint32_t reg_data, const uint8_t reg_size)
{
    if (reg_addr == 0 || reg_size == 0) return 0;

    tx_op(OP_WRITE_REG, reg_addr);
    spi_tx((uint8_t*)&reg_data, reg_size, true);
}

void rx_pl(uint8_t *buff, const uint8_t size)
{
    if (!buff || size == 0) return 0;

    tx_op(OP_RX_PL, 0);
    spi_rx(buff, size, true);
}

void tx_pl(const uint8_t *data, const uint8_t size)
{
    if (!data || size == 0) return 0;

    tx_op(OP_TX_PL, 0);
    spi_tx(data, size, true);
}

void tx_op(const Operation_e op, const uint8_t reg_addr)
{
    const uint8_t operation = (uint8_t)op;
    const uint8_t spi_tx_op = operation | reg_addr;
    spi_tx(&spi_tx_op, sizeof(spi_tx_op), false);
}

uint32_t parse_buff(const uint8_t* buff, const uint8_t size)
{
    uint32_t ret = 0;
    for (int byte_n = 0; byte_n < size; ++byte_n)
    {
        ret |= ((uint32_t)buff[0] << (byte_n * 8));
    }
    return ret;
}