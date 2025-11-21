#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    RX_P0 = 0x0,
    RX_P1,
    RX_P2,
    RX_P3,
    RX_P4,
    RX_P5
} RxPipe_e;

typedef enum
{
    DR_1MBPS = 0x0,
    DR_2MBPS,
    DR_250KBPS
} DataRate_e;

typedef enum
{
    TX_PWR_NEG_18dBm = 0x0,
    TX_PWR_NEG_12dBm,
    TX_PWR_NEG_6dBm,
    TX_PWR_0dBm
} TxPower_e;

typedef enum
{
    ARD_250US = 0x0,
    ARD_500US = 0x1,
    ARD_1000US = 0x3,
    ARD_2000US = 0x7,
    ARD_4000US = 0xF
} ARD_e;

typedef enum
{
    RF_RET_OK = 0x0,
    RF_RET_NO_INIT,
    RF_RET_FAIL
} RF_Return_e;

typedef struct
{
    TxPower_e tx_pwr;               // power of transmitter. Matters only if tx mode is used
    uint8_t arc;                    // auto-retransmit count, 0 disabled, 15 max. Matters only if tx mode is used
    ARD_e ard;                      // auto-retransmit delay, value of ARD_e. Matters only if tx mode is used
    uint8_t rf_freq;                // freq of RM. Value from 1 to 124 MHz. Must match on both sides
    DataRate_e data_rate;           // transmition speed. Must match on both sides
} nRF24L01_Init_t;

typedef struct
{
    RxPipe_e rx_pipe;               // 5 pipes can be used. Value of RxPipe_e
    uint32_t rx_addr;               // 0 & 1 pipe 32-bit, the rest are LSByte + 3 bytes of 1st pipe. Must match on both sides
    uint8_t* buff;                  // ptr to the array where the received bytes will be written
    uint8_t size;                   // 1 - 32 bytes, size of the buff
    uint32_t ms;                    // timeout, milliseconds
    bool rx_dpl;                    // enable dynamic payload length. If you know the expected data size, leave it false
} RxConfig_t;

typedef struct
{    
    uint32_t tx_addr;               // address of the receiver
    uint8_t* data;                  // the array of bytes to send
    uint8_t size;                   // size of the data
} TxConfig_t;

RF_Return_e nRF24L01_init(const nRF24L01_Init_t* config);
RF_Return_e nRF24L01_tx(const TxConfig_t* tx);
RF_Return_e nRF24L01_rx(const RxConfig_t* rx);

#endif // NRF24L01_H