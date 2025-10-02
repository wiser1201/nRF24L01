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
#define RX_P_NO_MSB 3
#define RX_P_NO_LSB 1
#define TX_FULL 0

#define FIFO_TX_REUSE 6
#define FIFO_TX_FULL 5
#define FIFO_TX_EMPTY 4
#define FIFO_RX_FULL 1
#define FIFO_RX_EMPTY 0

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
#define RF_FREQ_DEF 2

#define ARD_STEP 250
#define ARD_MIN ARD_STEP
#define ARD_MAX (ARD_STEP * 16)

#define ALL_PIPES_MASK 0b111111

#define US_IN_MS 1000UL

#define FLAG_IS_SET(reg, flag) ((reg) & (1 << (flag)))
#define FLAG_SET(reg, flag)    ((reg) |= (1 << (flag)))
#define FLAG_CLEAR(reg, flag)  ((reg) &= ~(1 << (flag)))
#define FLAG_TOGGLE(reg, flag) ((reg) ^= (1 << (flag)))

typedef enum
{
    OP_READ_REG = 0b00000000,
    OP_WRITE_REG = 0b00100000,
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
    MODE_RX,
    MODE_TX,
    MODE_IRQ
} ChipMode_e;

typedef enum
{
    IRQ_NONE,
    IRQ_RX_DR,
    IRQ_TX_DS,
    IRQ_MAX_RT
} IRQ_State_e;

static volatile ChipMode_e chip_mode;
static volatile IRQ_State_e irq_state;

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
extern uint32_t ex_get_ms(void);
extern void gpio_set_ce(const bool level);
void rf_irq(void);

static uint32_t read_reg(const uint8_t reg_addr, const uint8_t reg_size);
static void write_reg(const uint8_t reg_addr, const uint32_t reg_data, const uint8_t reg_size);
static void rx_pl(uint8_t* buff, const uint8_t size);
static void tx_pl(const uint8_t* data, const uint8_t size);
static void flush_rx(void);
static void flush_tx(void);
static void reuse_tx_pl(void);
static uint8_t rx_pl_width(void);
static void rx_ack_tx_pl(void);
static void tx_pl_no_ack(void);
static uint8_t nop(void);

static void tx_op(const Operation_e op, const uint8_t reg_addr);

static RF_Return_e switch_chip_mode(const ChipMode_e mode);
static bool chip_mode_is(const ChipMode_e mode);
static bool irq_state_is(const IRQ_State_e state);
static void irq_state_set(const IRQ_State_e state);
static void irq_state_clear(void);
static void pwrd_to_standby(void);
static void standby_to_pwrd(void);
static void rf_irq_handler(void);
static uint32_t parse_buff(const uint8_t* buff, const uint8_t size);


RF_Return_e nRF24L01_init(const nRF24L01_Init_t *config)
{
    uint8_t addr_width_reg = 0;
    FLAG_SET(addr_width_reg, AW_MSB); // aw is 4 bytes

    uint8_t retr_reg = 0;
    if ((uint8_t)config->ard <= 0xF)
    {
        retr_reg |= ((uint8_t)config->ard << ARD_LSB);
    }
    if (config->arc <= 0xF)
    {
        retr_reg |= (config->arc << ARC_LSB);
    }

    uint8_t rf_freq_reg = 0;
    if (config->rf_freq >= RF_FREQ_MIN && config->rf_freq <= RF_FREQ_MAX)
    {
        rf_freq_reg = config->rf_freq;
    }
    else
    {
        rf_freq_reg = RF_FREQ_DEF;
    }

    uint8_t rf_config_reg = 0;
    switch (config->data_rate)
    {
    case DR_250KBPS:
        FLAG_SET(rf_config_reg, RF_DR_LOW);
        break;
    case DR_2MBPS:
        FLAG_SET(rf_config_reg, RF_DR_HIGH);
        break;
    default:
    case DR_1MBPS:
        break;
    }
    if ((uint8_t)config->tx_pwr <= 0x3)
    {
        rf_config_reg |= ((uint8_t)config->tx_pwr << RF_PWR_LSB);
    }

    write_reg(REG_SETUP_AW, addr_width_reg, sizeof(addr_width_reg));
    write_reg(REG_SETUP_RETR, retr_reg, sizeof(retr_reg));
    write_reg(REG_RF_CH, rf_freq_reg, sizeof(rf_freq_reg));
    write_reg(REG_RF_SETUP, rf_config_reg, sizeof(rf_config_reg));
    
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
        default: break;
        }
        break;
    }
    case MODE_POWER_DOWN:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            pwrd_to_standby();
            chip_mode = mode;
            break;
        }
        default: break;
        }
        break;
    }
    case MODE_STANDBY_I:
    {
        switch (mode)
        {
        case MODE_POWER_DOWN:
        {
            standby_to_pwrd();
            chip_mode = mode;
            break;
        }
        case MODE_RX:
        {
            gpio_set_ce(true);
            ex_delay_us(130);
            chip_mode = mode;
            break;
        }
        case MODE_TX:
        {
            gpio_set_ce(true);
            ex_delay_us(130);
            gpio_set_ce(false);
            chip_mode = mode;
            break;
        }
        default: break;
        }
        break;
    }
    case MODE_RX:
    case MODE_TX:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            gpio_set_ce(false);
            chip_mode = mode;
            break;
        }
        case MODE_IRQ:
        {
            chip_mode = mode;
            break;
        }
        default: break;
        }
        break;
    }
    case MODE_IRQ:
    {
        switch (mode)
        {
        case MODE_STANDBY_I:
        {
            gpio_set_ce(false);
            chip_mode = mode;
            break;
        }
        default: break;
        }
        break;
    }
    default: break;
    }
    if (chip_mode != mode)
    {
        return RF_RET_FAIL;
    }
    return RF_RET_OK;
}

void pwrd_to_standby(void)
{
    gpio_set_ce(LOW);
    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(config_reg));
    FLAG_SET(config_reg, PWR_UP);
    write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
    ex_delay_us(US_IN_MS * 10);
}

void standby_to_pwrd(void)
{
    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(uint8_t));
    FLAG_CLEAR(config_reg, PWR_UP);
    write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
}

bool chip_mode_is(const ChipMode_e mode)
{
    return chip_mode == mode;
}

bool irq_state_is(const IRQ_State_e state)
{
    return irq_state == state;
}

void irq_state_set(const IRQ_State_e state)
{
    irq_state = state;
}

void irq_state_clear(void)
{
    irq_state = IRQ_NONE;
}

RF_Return_e nRF24L01_tx(const TxConfig_t* tx)
{
    if (!tx || !tx->data || tx->size <= 0 || tx->size > 32)
    {
        return RF_RET_NO_INIT;
    }
    if (switch_chip_mode(MODE_STANDBY_I) != RF_RET_OK)
    {
        return RF_RET_NO_INIT;
    }

    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(config_reg));
    if (FLAG_IS_SET(config_reg, PRIM_RX))
    {
        FLAG_CLEAR(config_reg, PRIM_RX);
        write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
    }

    uint8_t rx_pipe_reg = read_reg(REG_EN_RXADDR, sizeof(rx_pipe_reg));
    if (FLAG_IS_SET(rx_pipe_reg, ERX_P0) == false)
    {
        FLAG_SET(rx_pipe_reg, ERX_P0);
        write_reg(REG_EN_RXADDR, rx_pipe_reg, sizeof(rx_pipe_reg));
    }

    write_reg(REG_TX_ADDR, tx->tx_addr, sizeof(tx->tx_addr));
    write_reg(REG_RX_ADDR_P0, tx->tx_addr, sizeof(tx->tx_addr));
    tx_pl(tx->data, tx->size);

    switch_chip_mode(MODE_TX);
    
    while (chip_mode_is(MODE_TX)) {}
    
    rf_irq_handler();

    const bool success = irq_state_is(IRQ_TX_DS);
    switch_chip_mode(MODE_STANDBY_I);
    irq_state_clear();
    flush_tx();

    if (!success) return RF_RET_FAIL;
    return RF_RET_OK;
}

RF_Return_e nRF24L01_rx(const RxConfig_t* rx)
{
    if (!rx || !rx->buff || rx->size <= 0 || rx->size > 32 || rx->ms <= 0)
    {
        return RF_RET_NO_INIT;
    }
    if (switch_chip_mode(MODE_STANDBY_I) != RF_RET_OK)
    {
        return RF_RET_NO_INIT;
    }

    uint8_t config_reg = read_reg(REG_CONFIG, sizeof(config_reg));
    if (FLAG_IS_SET(config_reg, PRIM_RX) == false)
    {
        FLAG_SET(config_reg, PRIM_RX);
        write_reg(REG_CONFIG, config_reg, sizeof(config_reg));
    }

    uint8_t reg_shift = (uint8_t)rx->rx_pipe;
    if (reg_shift > (uint8_t)RX_P5)
    {
        return RF_RET_NO_INIT;
    }
    
    uint8_t rx_pipe_reg = read_reg(REG_EN_RXADDR, sizeof(rx_pipe_reg));
    if (FLAG_IS_SET(rx_pipe_reg, ERX_P0 + reg_shift) == false)
    {
        FLAG_SET(rx_pipe_reg, ERX_P0 + reg_shift);
        write_reg(REG_EN_RXADDR, rx_pipe_reg, sizeof(rx_pipe_reg));
    }

    write_reg(REG_RX_ADDR_P0 + reg_shift, rx->rx_addr, sizeof(rx->rx_addr));    

    if (rx->rx_dpl)
    {
        uint8_t rx_dpl_reg = read_reg(REG_DYNPD, sizeof(rx_dpl_reg));
        uint8_t feat_reg = read_reg(REG_FEATURE, sizeof(feat_reg));
        if (FLAG_IS_SET(feat_reg, EN_DPL) == false)
        {
            FLAG_SET(feat_reg, EN_DPL);
            write_reg(REG_FEATURE, feat_reg, sizeof(feat_reg));
        }
        if (FLAG_IS_SET(rx_dpl_reg, DPL_P0 + reg_shift) == false)
        {
            FLAG_SET(rx_dpl_reg, DPL_P0 + reg_shift);
            write_reg(REG_DYNPD, rx_dpl_reg, sizeof(rx_dpl_reg));
        }
    }
    else
    {
        write_reg(REG_RX_PW_P0 + reg_shift, rx->size, sizeof(rx->size));
    }

    switch_chip_mode(MODE_RX);
    
    const uint32_t time_ms = ex_get_ms();
    while (chip_mode_is(MODE_RX))
    {
        if ((ex_get_ms() - time_ms) > rx->ms)
        {
            switch_chip_mode(MODE_STANDBY_I);
            return RF_RET_FAIL;
        }
    }

    rf_irq_handler();
    
    const bool success = irq_state_is(IRQ_RX_DR);
    switch_chip_mode(MODE_STANDBY_I);
    irq_state_clear();

    if (!success)
    {
        return RF_RET_FAIL;
    }

    uint8_t status_reg = nop();
    uint8_t data_in_pipe = ((status_reg >> RX_P_NO_LSB) & 0b111);    

    if (data_in_pipe != (uint8_t)rx->rx_pipe)
    {
        return RF_RET_FAIL;
    }

    rx_pl(rx->buff, rx->size);
    return RF_RET_OK;
}

void rf_irq(void)
{
    chip_mode = MODE_IRQ;
}

void rf_irq_handler(void)
{
    uint8_t status_reg = nop();
    uint8_t status_clear = 0;
    if (FLAG_IS_SET(status_reg, RX_DR_IF))
    {
        irq_state_set(IRQ_RX_DR);
        status_clear |= (1 << RX_DR_IF);
    }
    if (FLAG_IS_SET(status_reg, TX_DS_IF))
    {
        irq_state_set(IRQ_TX_DS);
        status_clear |= (1 << TX_DS_IF);     
    }
    if (FLAG_IS_SET(status_reg, MAX_RT_IF))
    {
        irq_state_set(IRQ_MAX_RT);
        status_clear |= (1 << MAX_RT_IF);
    }
    write_reg(REG_STATUS, status_clear, sizeof(status_reg));
}

/**
 * @brief  No operation. NOP = send 0xFF
 *          and receive status register in parallel
 * @retval Status register
 */
uint8_t nop(void)
{
    uint8_t status_reg;
    spi_rx(&status_reg, sizeof(status_reg), true);
    return status_reg;
}

uint32_t read_reg(const uint8_t reg_addr, const uint8_t reg_size)
{
    if (reg_size == 0) return 0;

    uint8_t buff[reg_size];
    tx_op(OP_READ_REG, reg_addr);
    spi_rx(buff, reg_size, true);
    return parse_buff(buff, reg_size);
}

void write_reg(const uint8_t reg_addr, const uint32_t reg_data, const uint8_t reg_size)
{
    if (reg_size == 0) return;

    tx_op(OP_WRITE_REG, reg_addr);
    spi_tx((uint8_t*)&reg_data, reg_size, true);
}

void rx_pl(uint8_t *buff, const uint8_t size)
{
    if (!buff || size == 0) return;

    tx_op(OP_RX_PL, 0);
    spi_rx(buff, size, true);
}

void tx_pl(const uint8_t *data, const uint8_t size)
{
    if (!data || size == 0) return;

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
    for (int i = 0; i < size; ++i)
    {
        ret |= ((uint32_t)buff[i] << (i * 8));
    }
    return ret;
}

void flush_rx(void)
{
    if (chip_mode_is(MODE_POWER_DOWN) || chip_mode_is(MODE_STANDBY_I))
    {
        tx_op(OP_FLUSH_RX, 0);
    }
}

void flush_tx(void)
{
    if (chip_mode_is(MODE_POWER_DOWN) || chip_mode_is(MODE_STANDBY_I))
    {
        tx_op(OP_FLUSH_TX, 0);
    }
}

void reuse_tx_pl(void)
{
    tx_op(OP_REUSE_TX_PL, 0);
}

uint8_t rx_pl_width(void)
{
    uint8_t ret = 0;
    tx_op(OP_RX_PL_WIDTH, 0);
    spi_rx(&ret, sizeof(ret), true);
    return ret;
}

void rx_ack_tx_pl(void)
{

}

void tx_pl_no_ack(void)
{

}
// TEST SECTION
// void nRF_test_check_config(void)
// {
//     read_reg(REG_SETUP_AW, 1);
//     read_reg(REG_SETUP_RETR, 1);
//     read_reg(REG_RF_CH, 1);
//     read_reg(REG_RF_SETUP, 1);
// }

void nRF_test_status_reg(void)
{
    uint8_t status_reg = nop();
}

// void nRF_test_rx_config(void)
// {
//     write_reg(REG_RX_ADDR_P0 + 1, 2147483611UL, 4);
//     read_reg(REG_RX_ADDR_P1, 4);
// }
uint8_t nRF_read_fifo_status(void)
{
    return read_reg(REG_FIFO_STATUS, 1);
}