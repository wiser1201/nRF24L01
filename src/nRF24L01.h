#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>

#define USE_RX_MULTI

typedef enum
{
    RX_P0 = 0x1,
#ifdef USE_RX_MULTI
    RX_P1 = 0x2,
    RX_P2 = 0x4,
    RX_P3 = 0x8,
    RX_P4 = 0x16,
    RX_P5 = 0x32
#endif
} RxPipe_e;

typedef enum
{
    RX_DPL_P0 = 0x1,
#ifdef USE_RX_MULTI
    RX_DPL_P1 = 0x2,
    RX_DPL_P2 = 0x4,
    RX_DPL_P3 = 0x8,
    RX_DPL_P4 = 0x16,
    RX_DPL_P5 = 0x32
#endif
} RxDynPayl_e;

typedef enum
{
    AW_3 = 0x1,
    AW_4 = 0x2,
    AW_5 = 0x4
} AddressWidth_e;

typedef enum
{
    DR_1MBPS,
    DR_2MBPS,
    DR_250KBPS
} DataRate_e;

typedef enum
{
    TX_PWR_NEG_18dBm,
    TX_PWR_NEG_12dBm,
    TX_PWR_NEG_6dBm,
    TX_PWR_0dBm
} TxPower_e;

typedef struct
{
    uint8_t rx_active_pipes;        // Values of RxPipe_e
    uint32_t rx_p0_addr;
#ifdef USE_RX_MULTI
    uint32_t rx_p1_addr;
    uint8_t rx_p2_addr;             // Only LSB. MSBytes are equal to rx_p1_addr
    uint8_t rx_p3_addr;             // Only LSB. MSBytes are equal to rx_p1_addr
    uint8_t rx_p4_addr;             // Only LSB. MSBytes are equal to rx_p1_addr
    uint8_t rx_p5_addr;             // Only LSB. MSBytes are equal to rx_p1_addr
#endif
    uint8_t rx_pl_p0_size;          // rx payload size for rx_pipe, 0 to 32 bytes, 0 - rx disabled
#ifdef USE_RX_MULTI
    uint8_t rx_pl_p1_size;          // rx payload size for rx_pipe, 0 to 32 bytes, 0 - rx disabled
    uint8_t rx_pl_p2_size;          // rx payload size for rx_pipe, 0 to 32 bytes, 0 - rx disabled
    uint8_t rx_pl_p3_size;          // rx payload size for rx_pipe, 0 to 32 bytes, 0 - rx disabled
    uint8_t rx_pl_p4_size;          // rx payload size for rx_pipe, 0 to 32 bytes, 0 - rx disabled
    uint8_t rx_pl_p5_size;          // rx payload size for rx_pipe, 0 to 32 bytes, 0 - rx disabled
#endif
    uint8_t rx_dpl;                 // enable or disable dynamic payload length for pipes, RxDynPayl_e
} RxConfig_t;

typedef struct
{
    uint32_t tx_addr;
    TxPower_e tx_pwr;
    uint8_t arc;                    // auto-retransmit count, 0 disabled, 15 max
    uint16_t ard;                   // auto-retransmit delay, 250-4000 us, multiple of 250 
} TxConfig_t;

typedef struct
{
    bool enable_rx_dr_irq;
    bool enable_tx_ds_irq;
    bool enable_max_rt_irq;
    uint8_t rf_freq;                // freq of module. Value is 1 to 124 MHz
    DataRate_e data_rate;
    RxConfig_t rx_config;
    TxConfig_t tx_config;
} nRF24L01_Init_t;

void nRF24L01_init(const nRF24L01_Init_t* config);


#endif // NRF24L01_H