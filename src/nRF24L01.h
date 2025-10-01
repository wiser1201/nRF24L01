#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    RX_P0 = 0,
    RX_P1,
    RX_P2,
    RX_P3,
    RX_P4,
    RX_P5
} RxPipe_e;

typedef enum
{
    DR_1MBPS,
    DR_2MBPS,
    DR_250KBPS
} DataRate_e;

typedef enum
{
    TX_PWR_NEG_18dBm = 0x0,
    TX_PWR_NEG_12dBm = 0x1,
    TX_PWR_NEG_6dBm = 0x2,
    TX_PWR_0dBm = 0x3
} TxPower_e;

typedef enum
{
    ARD_250US = 0x0,
    ARD_500US = 0x1,
    ARD_1000US = 0x3,
    ARD_2000US = 0x7,
    ARD_4000US = 0xF
} ARD_e;

typedef struct
{
    RxPipe_e rx_pipe;
    uint32_t rx_addr;               // 0 & 1 pipe 32-bit, the rest are LSByte + 3 bytes of 1st pipe
    uint8_t* buff;
    uint8_t size;                   // 1 - 32 bytes
    uint32_t ms;
    bool rx_dpl;                    // enable or disable dynamic payload length
} RxConfig_t;

typedef struct
{    
    uint32_t tx_addr;
    uint8_t* data;
    uint8_t size;
} TxConfig_t;

typedef struct
{
    TxPower_e tx_pwr;
    uint8_t arc;                    // auto-retransmit count, 0 disabled, 15 max
    ARD_e ard;                      // auto-retransmit delay
    uint8_t rf_freq;                // freq of module. Value is 1 to 124 MHz
    DataRate_e data_rate;
} nRF24L01_Init_t;

typedef enum
{
    RF_RET_OK,
    RF_RET_NO_INIT,
    RF_RET_FAIL
} RF_Return_e;

RF_Return_e nRF24L01_init(const nRF24L01_Init_t* config);
RF_Return_e nRF24L01_tx(const TxConfig_t* tx);
RF_Return_e nRF24L01_rx(const RxConfig_t* rx);
bool nRF_test(void);

#endif // NRF24L01_H