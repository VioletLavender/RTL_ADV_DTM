#include <string.h>
#include <stdio.h>
#include "knl_pblc.h"
#include "config.h"
#include "systick.h"
#include "bsp_misc.h"
#include "bsp_spi_ble.h"
#include "hci.h"
#include "LL.h"
#include "rf_phy.h"
#include "_rafael_phy.h"
#include "_ble_host.h"

extern void delay_us(u16 us) ;

#define ID_TESTFUNC_NONE        0
#define ID_TESTFUNC_TEMP        1
#define ID_TESTFUNC_MSG2UART    2
#define ID_TESTFUNC_RAFAEL      3


#define PLL_LOCK_TIME            80   //R91
#define FAST_PLL_LOCK_TIME       80   //R92
#define NEXT_RX_TIME              1   //R93
#define NEXT_TX_TIME             17   //R94     //1M:15  2M:23 (PLL=80,NextTX=7,PA=15, total=150 by Frontline)
#define RX_TIMEOUT_PERIOD        255  //R95
#define PA_ON_TIME               15   //R96[7:4]= 5us
#define XTAL_TURNON_TIME         96   //R97, 96*31.25us = 3ms; need to modified delay time in RF_External_Wakeup()
//#define XTAL_TURNON_TIME         32   //R97, 32*31.25us = 1ms; need to modified delay time in RF_External_Wakeup()


Uint8 TAB_ZERO_128[] =
{
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};  //Actually total 136 bytes.


uint8_t ChipVer = 102;
#if (ENABLE_RF_VOLT_DETECT==1)
    uint8_t RfVolt_new = 1;
    uint8_t Init_flag = 0;
#endif

uint32_t Timeline24 = 0;
uint16_t Tmr16Interval;
uint8_t Content_ioInt;
uint32_t addr_TxFIFO;

uint8_t rssiOffsetValue = 26; // default is set to 26 dB

//MP IC, 0121 VBAT, init LDO
uint8_t RFIP_init_reg[RADIO_TOTAL_REG_NUM] =        //start reg addr=8
{
    0,   0,   0,   0,   0,   0,   0,   0,               //R0~R7     (read only)
    28, 128, 223, 164, 224, 243, 127,   5,  33,  16,     //R8~R17
    240, 160,  63, 187, 239, 194, 143, 122,   5, 128,     //R18~R27
    255, 255, 255,  39,  98,   0, 112, 210, 136, 172,     //R28~R37
    //145, 222, 192,  20,  96,  66, 252, 63, 106,   0,     //R38~R47   //R47[0]=0, CLK output enable
    145, 222, 192,  20,  96,  66, 252,  63, 106,   1,     //R38~R47   //R47[0]=1, CLK output disable; 32K high freq (R43[7]=0,R44=11111100,R45[4:0]=11111)
    0,   0,   0,   0,   0,   0, 240,   0,   0,   6,     //R48~R57   //R53=0x00
    162, 106,   0,   0, 255, 121, 112, 210, 234,  10,     //R58~R67   //Be carefaul to set R61=0 !
    30,  10,  35,   0, 160,  31,  85, 161,   0,  68,     //R68~R77   //R74[3]=1, edge trigger; 0:level mode * use level mode
#ifndef _HCI_HW_
    0,  82,   0,   6,  30,  30,   0,   6, 142,  36,     //R78~R87   //R82=30; R83=30 in FPGA_V17
#else
    0,  82,   0,   6,  32,  32,   0,   6, 142,  36,     //R78~R87   //R82=30; R83=30 in FPGA_V17
#endif
    1,  68,   0,  PLL_LOCK_TIME,  FAST_PLL_LOCK_TIME,   NEXT_RX_TIME,  NEXT_TX_TIME, RX_TIMEOUT_PERIOD, ((PA_ON_TIME << 4) | REG_96_WHITEN_EN),  XTAL_TURNON_TIME, //R88~R97   //R96[7:4] PA on time=15,  R97:Xtal_turn_on time; R88=1(PLL linear mode)
    0,   0,   0,   0,   0,   0,   0, 128,   0,   0,     //R98~R107  //R105[7]=1,enable TX payload check
    0,   0,   0,   0, 0x55, 0x55, 0x55, 0xFF, 0xFF, 0x4F, //R108~R117 //R117(RTC)
    0,   0,   4 | (CRC_IN_RX_FIFO << 6), 128 & 0, 0x29, 0x41, 0x76, 0x71,   0,   0 //R118~R127  //Be careful to set R121=0 !
};


/* TX POWER */
uint8_t txpower_0dbm_reg[4]   = { 66, 129, 64, 42}; // R23-R26
uint8_t txpower_4dbm_reg[4]   = {130, 164, 90,  5}; // R23-R26
uint8_t txpower_8dbm_reg[4]   = {210, 142, 122,  5}; // R23-R26
uint8_t txpower_10dbm_reg[4]  = {194, 143, 122,  5}; // R23-R26


uint8_t RFIP_REG_IDX[SIZE_RFIP_REG] =
{
    RFIP_REG_23,
    RFIP_REG_36,
    RFIP_REG_40,
    RFIP_REG_41,
    RFIP_REG_42,
    RFIP_REG_43,
    RFIP_REG_57,
    RFIP_REG_61,
    //RFIP_REG_62,
    RFIP_REG_105,
    RFIP_REG_107,
    //RFIP_REG_115,
    //RFIP_REG_116,
    //RFIP_REG_118,
    RFIP_REG_119,
    RFIP_REG_120,
    RFIP_REG_121,
    RFIP_REG_166,
};


uint8_t TIMELINE24_3750US_IDX[] =
{
    0x0F, 0x1E, 0x2D, 0x3C, 0x4B, 0x5A, 0x69, 0x78,
    0x87, 0x96, 0xA5, 0xB4, 0xC3, 0xD2, 0xE1, 0xF0,
    0xFF,
};

uint8_t TAB_4_FIFO_CLR[] = {0x00, 0x80, 0x00, 0x80};
uint8_t CH_SEL_TABLE[] =    //no use now
{
    /*
      8,  12,  16,  20,  24,  28,  32,  36,     //0~7
     40,  44,  48,  56,  60,  64,  68,  72,     //8~15
     76,  80,  84,  88,  92,  96, 100, 104,     //16~23
    108, 112, 116, 120, 124, 128, 132, 136,     //24~31
    140, 144, 148, 152, 156,   4,  52, 160      //32~39
    */
    1,  2,  3,  4,  5,  6,   7,    8,
    9,  10, 11, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33,
    34, 35, 36, 37, 38, 0,  12, 39
};                                          //For the ch step = 0.5MHz, 2400MHz based.

uint8_t CH_SEL_TABLE_INV[] =
{
    /*
      8,  12,  16,  20,  24,  28,  32,  36,     //0~7
     40,  44,  48,  56,  60,  64,  68,  72,     //8~15
     76,  80,  84,  88,  92,  96, 100, 104,     //16~23
    108, 112, 116, 120, 124, 128, 132, 136,     //24~31
    140, 144, 148, 152, 156,   4,  52, 160      //32~39
    */
    37, 0,  1,  2,  3,  4,  5,   6,
    7,  8,  9,  10, 38, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 39
};                                          //For the ch step = 0.5MHz, 2400MHz based.


volatile uint8_t RFIP_reg_MEM[SIZE_RFIP_REG];


typedef struct _RT568_Gain_Info
{
    uint8_t   Reg38_val;
    uint8_t   Reg39_val;
    uint8_t   LNA_gain;
    uint8_t   TIA_gain;
    uint8_t   VGA_gain;
    uint8_t   BB_rssi_dbm;
} RT568_Gain_Info;


//gain table(gain*128) to calculate RSSI
uint16_t  R568_lna_agc[16] = {0, 219, 462, 717,  944, 1191, 1504, 1785, 1986, 2231, 2417, 2792, 3182, 3442, 3619, 3724};
uint16_t  R568_tia_agc[16] = {0, 385, 758, 1161, 1526, 1914, 1913, 2304, 2711, 3116, 3513, 3914, 4319, 4748, 4749, 4749};
uint16_t  R568_vga_agc[16] = {0, 379, 757, 1147, 1511, 1884, 2551, 2616, 2976, 3320, 3653, 3959, 4241, 4488, 4489, 4491};


//PLL Setting
uint8_t RFIP_PLL_Reg_R34[40] =
{
    0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0,              //CH0 ~ CH6
    0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0,        //CH7 ~ CH14
    0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0,          //CH15 ~ CH22
    0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0,          //CH23 ~ CH30
    0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0,          //CH31 ~ CH38
    0x00
};

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef _HCI_HW_
Uint8 NUM_OGF_TABLE[] =
{
    MAX_OGF_INVALID_NO,
    MAX_OGF_LINK_CONTROL_NO,
    MAX_OGF_INVALID_NO,
    MAX_OGF_CONTROLLER_BASEBAND_NO,
    MAX_OGF_INFORMATIONAL_PARAM_NO,
    MAX_OGF_STATUS_PARAM_NO,
    MAX_OGF_INVALID_NO,
    MAX_OGF_INVALID_NO,
    MAX_OGF_LE_CONTROLLER_NO,
};


Uint8 HCI_OGF_LINK_CONTROL[] =
{
    HCI_CMD_OGF_INVALID,
    HCI_CMD_OGF_DISCONNECT,
    HCI_CMD_OGF_READ_REMOTE_VERSION_INFORMATION,
};


Uint8 HCI_OGF_CONTROLLER_BASEBAND[] =
{
    HCI_CMD_OGF_INVALID,
    HCI_CMD_OGF_SET_EVENT_MASK,
    HCI_CMD_OGF_RESET,
    HCI_CMD_OGF_READ_TRANSMIT_POWER_LEVEL,
    HCI_CMD_OGF_SET_CONTROLLER_TO_HOST_FLOW_CONTROL,
    HCI_CMD_OGF_HOST_BUFFER_SIZE,
    HCI_CMD_OGF_HOST_NUMBER_OF_COMPLETED_PACKETS,
    HCI_CMD_OGF_SET_EVENT_MASK_PAGE_2,
    HCI_CMD_OGF_READ_LE_HOST_SUPPORT,
    HCI_CMD_OGF_WRITE_LE_HOST_SUPPORT,
    HCI_CMD_OGF_READ_AUTHENTICATED_PAYLOAD_TIMEOUT,
    HCI_CMD_OGF_WRITE_AUTHENTICATED_PAYLOAD_TIMEOUT,
};


Uint8 HCI_OGF_INFO_PARAM[] =
{
    HCI_CMD_OGF_INVALID,
    HCI_CMD_OGF_READ_LOCAL_VERSION_INFORMATION,
    HCI_CMD_OGF_READ_LOCAL_SUPPORTED_COMMANDS,
    HCI_CMD_OGF_READ_LOCAL_SUPPORTED_FEATURES,
    HCI_CMD_OGF_READ_BUFFER_SIZE,
    HCI_CMD_OGF_READ_BD_ADDR,
};


Uint8 HCI_OGF_STATUS_PARAM[] =
{
    HCI_CMD_OGF_INVALID,
    HCI_CMD_OGF_READ_RSSI,
};


Uint8 HCI_OGF_INVALID[] =
{
    HCI_CMD_OGF_INVALID,
};



Uint8 HCI_OGF_LE_CTRLER[] =
{
    HCI_CMD_OGF_INVALID,
    HCI_CMD_OGF_LE_SET_EVENT_MASK,
    HCI_CMD_OGF_LE_READ_BUFFER_SIZE,
    HCI_CMD_OGF_LE_READ_LOCAL_SUPPORTED_FEATURES,
    HCI_CMD_OGF_LE_SET_RANDOM_ADDRESS,
    HCI_CMD_OGF_LE_SET_ADVERTISING_PARAMETERS,
    HCI_CMD_OGF_LE_READ_ADVERTISING_CHANNEL_TX_POWER,
    HCI_CMD_OGF_LE_SET_ADVERTISING_DATA,
    HCI_CMD_OGF_LE_SET_SCAN_RESPONSE_DATA,
    HCI_CMD_OGF_LE_SET_ADVERTISE_ENABLE,
    HCI_CMD_OGF_LE_SET_SCAN_PARAMETERS,
    HCI_CMD_OGF_LE_SET_SCAN_ENABLE,
    HCI_CMD_OGF_LE_CREATE_CONNECTION,
    HCI_CMD_OGF_LE_CREATE_CONNECTION_CANCEL,
    HCI_CMD_OGF_LE_READ_WHITE_LIST_SIZE,
    HCI_CMD_OGF_LE_CLEAR_WHITE_LIST,
    HCI_CMD_OGF_LE_ADD_DEVICE_TO_WHITE_LIST,
    HCI_CMD_OGF_LE_REMOVE_DEVICE_FROM_WHITE_LIST_,
    HCI_CMD_OGF_LE_CONNECTION_UPDATE,
    HCI_CMD_OGF_LE_SET_HOST_CHANNEL_CLASSIFICATION,
    HCI_CMD_OGF_LE_READ_CHANNEL_MAP,
    HCI_CMD_OGF_LE_READ_REMOTE_USED_FEATURES,
    HCI_CMD_OGF_LE_ENCRYPT,
    HCI_CMD_OGF_LE_RAND,
    HCI_CMD_OGF_LE_START_ENCRYPTION,
    HCI_CMD_OGF_LE_LONG_TERM_KEY_REQUEST_REPLY,
    HCI_CMD_OGF_LE_LONG_TERM_KEY_REQUEST_NEGATIVE_REPLY,
    HCI_CMD_OGF_LE_READ_SUPPORTED_STATES_,
    HCI_CMD_OGF_LE_RECEIVER_TEST,
    HCI_CMD_OGF_LE_TRANSMITTER_TEST,
    HCI_CMD_OGF_LE_TEST_END,
    HCI_CMD_OGF_LE_REMOTE_CONN_PARAM_REQUEST_REPLY,
    HCI_CMD_OGF_LE_REMOTE_CONN_PARAM_REQUEST_NEG_REPLY,
    HCI_CMD_OGF_LE_READ_PHY,
    HCI_CMD_OGF_LE_SET_DEFAULT_PHY,
    HCI_CMD_OGF_LE_SET_PHY,
    HCI_CMD_OGF_LE_SET_DATA_LENGTH,
};


uint8_t *HCI_OGF_TABLE[] =
{
    HCI_OGF_INVALID,
    HCI_OGF_LINK_CONTROL,
    HCI_OGF_INVALID,
    HCI_OGF_CONTROLLER_BASEBAND,
    HCI_OGF_INFO_PARAM,
    HCI_OGF_STATUS_PARAM,
    HCI_OGF_INVALID,
    HCI_OGF_INVALID,
    HCI_OGF_LE_CTRLER,
};


Uint8 HCI_CMD_OGF_LINK_CONTROL[] =
{
    LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND,  HCLL_DISCONNECT,
    HCLL_READ_REMOTE_VER_INFO,
};


Uint8 HCI_LENGTH_CMD_OGF_LINK_CONTROL[] =
{
    //HCLL_DISCONNECT, HCLL_READ_REMOTE_VER_INFO,
    0,              3,              2,
};


Uint8 HCI_CMD_OGF_CONTROLLER_BASEBAND[] =
{
    LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND,  HCLL_SET_EVENT_MASK,
    HCLL_RESET,                         HCLL_READ_TRANSMIT_PWR_LEVEL,
    HCLL_SET_CTRLER_TO_HOST_FLOW_CTRL,  HCLL_HOST_BUFFER_SIZE,
    HCLL_HOST_NUM_OF_COMPLETED_PACKETS, HCLL_SET_EVENT_MASK_PAGE_2,
    HCLL_READ_LE_HOST_SUPPORT,          HCLL_WRITE_LE_HOST_SUPPORT,
    HCLL_READ_AUTHEN_PAYLOAD_TIMEOUT,   HCLL_WRITE_AUTHEN_PAYLOAD_TIMEOUT,
};


Uint8 HCI_LENGTH_CMD_OGF_CONTROLLER_BASEBAND[] =
{
    //MLL_HCI_NULL,                       HCLL_SET_EVENT_MASK,
    0,                                  8,
    //HCLL_RESET,                         HCLL_READ_TRANSMIT_PWR_LEVEL,
    0,                                  3,
    //HCLL_SET_CTRLER_TO_HOST_FLOW_CTRL,  HCLL_HOST_BUFFER_SIZE,
    1,                                  7,
    //HCLL_HOST_NUM_OF_COMPLETED_PACKETS, HCLL_SET_EVENT_MASK_PAGE_2,
    255,                                8,
    //HCLL_READ_LE_HOST_SUPPORT,          HCLL_WRITE_LE_HOST_SUPPORT,
    0,                                  2,
    //HCLL_READ_AUTHEN_PAYLOAD_TIMEOUT,   HCLL_WRITE_AUTHEN_PAYLOAD_TIMEOUT,
    2,                                  4,
};


Uint8 HCI_CMD_OGF_INFO_PARAM[] =
{
    LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND,  HCLL_READ_LOCAL_VER_INFO,
    HCLL_READ_LOCAL_SUPPORTED_CMD,      HCLL_READ_LOCAL_SUPPORTED_FEAT,
    HCLL_READ_BUFFER_SIZE,              HCLL_READ_BD_ADDR,
};


Uint8 HCI_LENGTH_CMD_OGF_INFO_PARAM[] =
{
    //MLL_HCI_NULL,                       HCLL_READ_LOCAL_VER_INFO,
    0,                                  0,
    //HCLL_READ_LOCAL_SUPPORTED_CMD,      HCLL_READ_LOCAL_SUPPORTED_FEAT,
    0,                                  0,
    //HCLL_READ_BUFFER_SIZE,              HCLL_READ_BD_ADDR,
    0,                                  0,
};


Uint8 HCI_CMD_OGF_STATUS_PARAM[] =
{
    LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND,   HCLL_READ_RSSI,
};


Uint8 HCI_LENGTH_CMD_OGF_STATUS_PARAM[] =
{
    //MLL_HCI_NULL,                       HCLL_READ_RSSI,
    0,                                  2,
};


Uint8 HCI_CMD_OGF_NULL[] =
{
    LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND,
};


Uint8 HCI_LENGTH_CMD_OGF_NULL[] =
{
    //MLL_HCI_NULL,
    0,
};


Uint8 HCI_CMD_OGF_LE_CTRLER[] =
{
    LLHC_ERR_CODE_UNKNOWN_HCI_COMMAND,  HCLL_LE_SET_EVENT_MASK,
    HCLL_LE_READ_BUFFER_SIZE,           HCLL_LE_READ_LOCAL_SUPPORTED_FEAT,
    HCLL_LE_SET_RANDOM_ADDRESS,         HCLL_LE_SET_ADV_PARAM,
    HCLL_LE_READ_ADV_CH_TX_PWR,         HCLL_LE_SET_ADV_DATA,
    HCLL_LE_SET_SCAN_RESPONSE_DATA,     HCLL_LE_SET_ADVERTISE_ENABLE,
    HCLL_LE_SET_SCAN_PARAM,             HCLL_LE_SET_SCAN_ENABLE,
    HCLL_LE_CREATE_CONN,                HCLL_LE_CREATE_CONN_CANCEL,
    HCLL_LE_READ_WHITE_LIST_SIZE,       HCLL_LE_CLEAR_WHITE_LIST,
    HCLL_LE_ADD_DEVICE_TO_WHITE_LIST,   HCLL_LE_RMV_DEVICE_FROM_WHITE_LIST,
    HCLL_LE_CONN_UPDATE,                HCLL_LE_SET_HOST_CH_CLASSIFICATION,
    HCLL_LE_READ_CH_MAP,                HCLL_LE_READ_REMOTE_USED_FEAT,
    HCLL_LE_ENCRYPT,                    HCLL_LE_RAND,
    HCLL_LE_START_ENCRYPTION,           HCLL_LE_LONG_TERM_KEY_REQ_REPLY,
    HCLL_LE_LONG_TERM_KEY_REQ_NEG_REPLY, HCLL_LE_READ_SUPPORTED_STATES_,
    HCLL_LE_RECEIVER_TEST,              HCLL_LE_TRANSMITTER_TEST,
    HCLL_LE_TEST_END,                   HCLL_LE_REMOTE_CONN_PARAM_REQ_REPLY,
    HCLL_LE_REMOTE_CONN_PARAM_REQ_NEG_REPLY,
    HCLL_LE_READ_PHY,                   HCLL_LE_SET_DEFAULT_PHY,
    HCLL_LE_SET_PHY,                    HCLL_LE_SET_DATA_LENGTH,
};


Uint8 HCI_LENGTH_CMD_OGF_LE_CTRLER[] =
{
    //MLL_HCI_NULL,                       HCLL_LE_SET_EVENT_MASK,
    0,                                  8,
    //HCLL_LE_READ_BUFFER_SIZE,           HCLL_LE_READ_LOCAL_SUPPORTED_FEAT,
    0,                                  0,
    //HCLL_LE_SET_RANDOM_ADDRESS,         HCLL_LE_SET_ADV_PARAM,
    6,                                  15,
    //HCLL_LE_READ_ADV_CH_TX_PWR,         HCLL_LE_SET_ADV_DATA,
    0,                                  32,
    //HCLL_LE_SET_SCAN_RESPONSE_DATA,     HCLL_LE_SET_ADVERTISE_ENABLE,
    32,                                 1,
    //HCLL_LE_SET_SCAN_PARAM,             HCLL_LE_SET_SCAN_ENABLE,
    7,                                  2,
    //HCLL_LE_CREATE_CONN,                HCLL_LE_CREATE_CONN_CANCEL,
    25,                                 0,
    //HCLL_LE_READ_WHITE_LIST_SIZE,       HCLL_LE_CLEAR_WHITE_LIST,
    0,                                  0,
    //HCLL_LE_ADD_DEVICE_TO_WHITE_LIST,   HCLL_LE_RMV_DEVICE_FROM_WHITE_LIST,
    7,                                  7,
    //HCLL_LE_CONN_UPDATE,                HCLL_LE_SET_HOST_CH_CLASSIFICATION,
    14,                                 5,
    //HCLL_LE_READ_CH_MAP,                HCLL_LE_READ_REMOTE_USED_FEAT,
    2,                                  2,
    //HCLL_LE_ENCRYPT,                    HCLL_LE_RAND,
    32,                                 0,
    //HCLL_LE_START_ENCRYPTION,           HCLL_LE_LONG_TERM_KEY_REQ_REPLY,
    28,                                 18,
    //HCLL_LE_LONG_TERM_KEY_REQ_NEG_REPLY,HCLL_LE_READ_SUPPORTED_STATES_,
    2,                                  0,
    //HCLL_LE_RECEIVER_TEST,              HCLL_LE_TRANSMITTER_TEST,
    1,                                  3,
    //HCLL_LE_TEST_END,                   HCLL_LE_REMOTE_CONN_PARAM_REQ_REPLY,
    0,                                  14,
    //HCLL_LE_REMOTE_CONN_PARAM_REQ_NEG_REPLY,
    3,
    //HCLL_LE_READ_PHY                    HCLL_LE_SET_DEFAULT_PHY
    2,                                  3,
    //HCLL_LE_SET_PHY                   HCLL_LE_SET_DATA_LENGTH
    7,                                  6,
    255,
};



Uint8 *HCI_CMD_TABLE[] =                          //MAIN__001
{
    HCI_CMD_OGF_NULL,
    HCI_CMD_OGF_LINK_CONTROL,
    HCI_CMD_OGF_NULL,
    HCI_CMD_OGF_CONTROLLER_BASEBAND,
    HCI_CMD_OGF_INFO_PARAM,
    HCI_CMD_OGF_STATUS_PARAM,
    HCI_CMD_OGF_NULL,
    HCI_CMD_OGF_NULL,
    HCI_CMD_OGF_LE_CTRLER,
};


Uint8 *HCI_LENGTH_CMD_TABLE[] =                  //MAIN__001
{
    HCI_LENGTH_CMD_OGF_NULL,
    HCI_LENGTH_CMD_OGF_LINK_CONTROL,
    HCI_LENGTH_CMD_OGF_NULL,
    HCI_LENGTH_CMD_OGF_CONTROLLER_BASEBAND,
    HCI_LENGTH_CMD_OGF_INFO_PARAM,
    HCI_LENGTH_CMD_OGF_STATUS_PARAM,
    HCI_LENGTH_CMD_OGF_NULL,
    HCI_LENGTH_CMD_OGF_NULL,
    HCI_LENGTH_CMD_OGF_LE_CTRLER,
};


uint8_t UARTBufferT[SIZE_UART_BUFFER];  //UART buffer for Tx
uint8_t UARTBufIdxR;                    //UART buffer index for Rx
uint8_t UARTBufIdxT;                    //UART buffer index for Tx
uint8_t UART_Tx_size;
uint8_t UART_Rx_size;

uint8_t UARTBufValidT;

#endif //(_HCI_HW_)



//#pragma GCC optimize ("O0")


/***********************************************
 * Function
 ***********************************************/
void RF_SymbolRate_set(uint8_t zSymbol_1M);
void setChannel_BLE(uint8_t ch);
void RF_Tmr_Periodic_set_ISR(uint16_t period_tick);
void RF_RxLengthLimit(uint8_t maxPDU);
void RF_TxFIFO_OriginAddr_set(uint16_t OriginAddr);
void RF_WTR_intOn(void);
void RF_Tmr_Periodic_initial(uint32_t period_tick, uint8_t sleep_mode);
void rafael_spi_clear_tx_fifo_cnt(void);
uint8_t RF_Voltage_Det(void);


extern  Uint8 TAB_ZERO_128[];

void rafael_reset_phy_fsm(void)
{
    uint8_t i;

    //RFIP_reg_MEM[RFIP_REG_MEM_121] |= 0x80;      //1: reset MAC state to READY
    //rafael_spi_write(spi_instance, 121, RFIP_init_reg+121, 1);
    while (1)
    {
        InterruptDisable();
        i = RFIP_reg_MEM[RFIP_REG_MEM_121];
        InterruptEnable();
        SPI_1BYT_SetTx(RFIP_REG_121, (i | REG_121_RST_STATE));
        //Write 1 to end all TX & RX events and reset the MAC state to READY STATE, user must write 0 to this bit, so PMU state machine would go normally.
        InterruptDisable();
        //rafael_spi_write(spi_instance, 121, RFIP_init_reg+121, 1);
        if (i == RFIP_reg_MEM[RFIP_REG_MEM_121])
        {
            SPI_1BYT_SetTx(RFIP_REG_121, i);
        }
        InterruptEnable();
        InterruptDisable();
        if (i == RFIP_reg_MEM[RFIP_REG_MEM_121])
        {
            InterruptEnable();
            break;
        }
        InterruptEnable();
    }
}


void rafael_spi_clear_fifo(void)
{

    RFIP_reg_MEM[RFIP_REG_MEM_107] = (RFIP_reg_MEM[RFIP_REG_MEM_107] | 0x80);
    SPI_1BYT_SetTx(RFIP_REG_107, RFIP_reg_MEM[RFIP_REG_MEM_107]);     //B7=1, clear RX_fifo

    RFIP_reg_MEM[RFIP_REG_MEM_107] = (RFIP_reg_MEM[RFIP_REG_MEM_107] & 0x7F);
    SPI_1BYT_SetTx(RFIP_REG_107, RFIP_reg_MEM[RFIP_REG_MEM_107]);     //B7=0
}

void rafael_spi_clear_tx_fifo_cnt(void)
{
    RFIP_reg_MEM[RFIP_REG_MEM_107] = (RFIP_reg_MEM[RFIP_REG_MEM_107] | 0x40);
    SPI_1BYT_SetTx(RFIP_REG_107, RFIP_reg_MEM[RFIP_REG_MEM_107]);     //B6=1, clear TX_buffer_cnt

    RFIP_reg_MEM[RFIP_REG_MEM_107] = (RFIP_reg_MEM[RFIP_REG_MEM_107] & 0xBF);
    SPI_1BYT_SetTx(RFIP_REG_107, RFIP_reg_MEM[RFIP_REG_MEM_107]);     //B6=0
}


// PLL calibration
#define NUM_PLL_BANK    5
#define NUM_BLE_CH      40

static uint16_t pll_freq_mhz[NUM_PLL_BANK] = {2402, 2422, 2442, 2462, 2480}; //0:2402MHz, 10:2422MHz, 20:2442MHz, 30:2462MHz, 39:2480MHz
uint8_t   Phy_pll_bank_freqIdx[5] = {37, 9, 18, 28, 39};                    // initial value
//static uint8_t  Phy_pll_bank[NUM_PLL_BANK]={32, 32, 32, 32, 32};                // initial value
uint8_t  Phy_pll_bank[NUM_PLL_BANK] = {32, 32, 32, 32, 32};
#if (ENABLE_VCO_CAL==1)
    static uint8_t  pll_cal_done_flag = 0;
#endif //(ENABLE_VCO_CAL==1)
uint8_t CH_PLL_bank_Table[NUM_BLE_CH];
void rafael_pll_bank_get(uint16_t freq_mhz, uint8_t *p_bank_result);

void RF_PLL_VCO_Bank_set(uint8_t valueVCO)                                    //Set PLL VCO Bank
{
#if (ENABLE_VCO_CAL==1)
    SPI_1BYT_SetTx(RFIP_REG_12, (valueVCO | (RFIP_init_reg[RFIP_REG_12] & 0xC0)));
#endif  //(ENABLE_VCO_CAL==1)
}

//---------------------------------//
//Rafael API:
//PLL Calibration
//---------------------------------//
void rafael_pll_calibration(void)
{
#if 1
    uint8_t pll_cal_cnt;
    uint8_t j = 0;
    uint16_t i16;

    //1.disable INT
    SPI_1BYT_SetTx(RFIP_REG_61, 0x00);
    SPI_1BYT_SetTx(RFIP_REG_62, 0xFF);

    //2.Set PLL lock Time (R91[7:0]) = 255, set maximum value first
    SPI_1BYT_SetTx(RFIP_REG_91, 0xFF);

    //3.Set PLL linear mode(R36[2]==1)
    //R88[4]=0, manual mode by R36[2]
    SPI_1BYT_SetTx(RFIP_REG_88, 0x01);

    RFIP_reg_MEM[RFIP_REG_MEM_36] = RFIP_reg_MEM[RFIP_REG_MEM_36] | 0x04;
    SPI_1BYT_SetTx(RFIP_REG_36, RFIP_reg_MEM[RFIP_REG_MEM_36]);

    //4.Set PLL VCO Bank = 32(R12[5:0]=32)
    RF_PLL_VCO_Bank_set(32);

    //5.Set Symbol rate = 1MHz(R120 = 0x08)
    RF_SymbolRate_set(0);

    InterruptDisable();
    for (pll_cal_cnt = 0; pll_cal_cnt < NUM_PLL_BANK; pll_cal_cnt++)
    {
        //6.Set Channel Num R121[5:0]
        setChannel_BLE(Phy_pll_bank_freqIdx[pll_cal_cnt]);

        //7.Enable RX (R119 = 0x00, then R119 = 0x80)
        SPI_1BYT_SetTx(RFIP_REG_119, 0x00);
        SPI_1BYT_SetTx(RFIP_REG_119, REG_119_MANUAL_TRX_EN);

        //8.Wait 450us, then read R1[5:0], get VCO bank num for 2402 Mhz
        Tiny_Delay(1000);  //1ms

        //read VCO bank
        Phy_pll_bank[pll_cal_cnt] = (SPI_1BYT_SetRx(RFIP_REG_1) & 0x3F);
        //printf("Phy_pll_bank[%d]=%d\n", pll_cal_cnt, Phy_pll_bank[pll_cal_cnt]);
        //printf("%d\n", Phy_pll_bank[pll_cal_cnt]);
        /*
                //This part is time consuming? => calculate all channel VCO bank, do once in initial, do not have to calculate when set channel!
                if(pll_cal_cnt) {
                    i16 = pll_freq_mhz[(pll_cal_cnt-1)];  //get freq, i16
                    while(i16 < pll_freq_mhz[(pll_cal_cnt)]) {
                        rafael_pll_bank_get(i16, &CH_PLL_bank_Table[CH_SEL_TABLE_INV[j]]); // transfer i16 to PLL VCO Bank, store to CH_PLL_bank_Table[CH_SEL_TABLE_INV[j]]
                        i16 = i16+2;
                        j++;
                    };
                };
        */
        //8.Disable Rx (R121[7] = 1)
        rafael_reset_phy_fsm();
    } //end for(pll_cal_cnt=0;pll_cal_cnt<5;pll_cal_cnt++)

    if ((Phy_pll_bank[1] <= Phy_pll_bank[0]) || (Phy_pll_bank[2] <= Phy_pll_bank[1]) || (Phy_pll_bank[3] <= Phy_pll_bank[2]) || (Phy_pll_bank[4] <= Phy_pll_bank[3]))
    {
        //Give a fixed value as a protection mechanism.
        Phy_pll_bank[0] = 36;
        Phy_pll_bank[1] = 38;
        Phy_pll_bank[2] = 41;
        Phy_pll_bank[3] = 43;
        Phy_pll_bank[4] = 45;
    }

    for (pll_cal_cnt = 0; pll_cal_cnt < NUM_PLL_BANK; pll_cal_cnt++)
    {
        //This part is time consuming? => calculate all channel VCO bank, do once in initial, do not have to calculate when set channel!
        if (pll_cal_cnt)
        {
            i16 = pll_freq_mhz[(pll_cal_cnt - 1)]; //get freq, i16
            while (i16 < pll_freq_mhz[(pll_cal_cnt)])
            {
                rafael_pll_bank_get(i16, &CH_PLL_bank_Table[CH_SEL_TABLE_INV[j]]); // transfer i16 to PLL VCO Bank, store to CH_PLL_bank_Table[CH_SEL_TABLE_INV[j]]
                i16 = i16 + 2;
                j++;
            };
        };
    }
    CH_PLL_bank_Table[39] = Phy_pll_bank[NUM_PLL_BANK - 1]; //last channel

    //enable INT
    SPI_1BYT_SetTx(RFIP_REG_62, 0xFF);
    RF_WTR_intOn();
    InterruptEnable();

    //Set PLL Lock Time(R91), PLL Fast Lock Time(R92)
    SPI_PDMA_SetTx(RFIP_REG_91, ((uint32_t)(RFIP_init_reg + 91)), 2);
#endif //(1)
}

//---------------------------------//
//Rafael API:
//Return PLL VCO Bank using interpolation method
//---------------------------------//
void rafael_pll_bank_get(uint16_t freq_mhz, uint8_t *p_bank_result)
{
    //For example,  2403 < N < 2423,
    //VCO_Bank (N) = VCO_BANK(2403) + {[ VCO_BANK(2403)  - VCO_BANK(2423) ] * [(N-2403) / 20]};

    //static uint16_t pll_freq_mhz[5]={2402, 2422, 2442, 2462, 2480}; //0:2402MHz, 10:2422MHz, 20:2442MHz, 30:2462MHz, 39:2480MHz
    //static uint8_t pll_bank[5]={36, 39, 41, 43, 44};

    uint8_t pll_bank_cnt = 0;
    uint16_t pll_cal;

    while (pll_bank_cnt < NUM_PLL_BANK)
    {
        if (freq_mhz == pll_freq_mhz[pll_bank_cnt])
        {
            *p_bank_result = Phy_pll_bank[pll_bank_cnt];
            return;
        }
        else if (freq_mhz > pll_freq_mhz[pll_bank_cnt] && freq_mhz < pll_freq_mhz[pll_bank_cnt + 1])
        {
            //*p_bank_result = pll_bank[pll_bank_cnt] + (((pll_bank[pll_bank_cnt+1] - pll_bank[pll_bank_cnt]) * (freq_mhz-pll_freq_mhz[pll_bank_cnt])) / (pll_freq_mhz[pll_bank_cnt+1] - pll_freq_mhz[pll_bank_cnt]));
            pll_cal = (((Phy_pll_bank[pll_bank_cnt + 1] - Phy_pll_bank[pll_bank_cnt]) * (freq_mhz - pll_freq_mhz[pll_bank_cnt]) * 10) / (pll_freq_mhz[pll_bank_cnt + 1] - pll_freq_mhz[pll_bank_cnt]));
            if ((pll_cal % 10) >= 5)
            {
                *p_bank_result = Phy_pll_bank[pll_bank_cnt] + (pll_cal / 10) + 1;
            }
            else
            {
                *p_bank_result = Phy_pll_bank[pll_bank_cnt] + (pll_cal / 10);
            }
            return;
        }
        else
        {
            pll_bank_cnt++;
        }
    } //while(pll_bank_cnt<5)
}

void Ble_SW_Init(void)
{
    extern void initInt(void);



    InterruptDisable();

    initInt();

    RF_Tmr_Periodic_initial(DUR_LL_WAKEUP_MIN, RF_SLEEP_DISABLE);


    //enable RT568 timer wakeup INT
    RF_WTR_intOn();

    InterruptEnable();
}

void RF_Init(void)
{
    uint32_t u32i;

#if (ENABLE_RF_VOLT_DETECT==1)
    uint8_t volt;
#endif

    MCU_GpioIntDisable();

#ifdef _DEBUG_PINS_
//    Debug_Pins_Init();
#endif  //(#ifdef _DEBUG_PINS_)

#ifdef _HCI_HW_
    UART_Init();
#endif //(_HCI_HW_)

    //  MUST init global device parameter first
    //initBleDeviceParam(&ble_device_param);
    ChipVer = 0x66;

    /* test SPI read */
    //ble_device_param.ble_deviceChipId = SPI_1BYT_SetRx(RFIP_REG_0);   //Must read ChipID first
    ChipVer  = SPI_1BYT_SetRx(RFIP_REG_0);   //Must read ChipID first
//#ifndef _HCI_HW_
//    printf("Chip_ID=%d\n",ble_device_param.ble_deviceChipId);
//#endif


    //Initial RFIP_reg_MEM[] ram
    for (u32i = 0; u32i < SIZE_RFIP_REG; u32i++)
    {
        RFIP_reg_MEM[u32i] = RFIP_init_reg[RFIP_REG_IDX[u32i]];
    }


//#if (_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_) || (_BOARD_SELECTION_==_BOARD_NUVOTON_M487JIDAE_B3_)  //?
    //addr_TxFIFO = 2;
    //SPI_PDMA_SetTx(TX_BUFFER_WRITE_PORT, (uint32_t)(TAB_ZERO_128), LEN_CONN_DATA_MAX);
    //To avoid one strange problem on next calling function SPI_PDMA_SetTx(RFIP_REG_8,......
    //The SS pin will go high after the first byte sent, the discontinuous SS signal will cause RT568 failing to initialize.
//#endif  //(_BOARD_SELECTION_ == _BOARD_NUVOTON_M032SE3AE_) || (_BOARD_SELECTION_==_BOARD_NUVOTON_M487JIDAE_B3_)

    //RF_init
    SPI_PDMA_SetTx(RFIP_REG_8, ((uint32_t)(RFIP_init_reg + 8)), RADIO_RF_INIT_REG_NUM);      //R8~R46
    SPI_PDMA_SetTx(RFIP_REG_47, ((uint32_t)(RFIP_init_reg + 47)), RADIO_BB_INIT_REG_NUM);    //R48~R127

    if (ChipVer == MP_A1)
    {
        SPI_1BYT_SetTx(RFIP_REG_74, (RFIP_init_reg[RFIP_REG_74] | REG_74_EDGE));  //use edge trigger mode
    }

#if (ENABLE_TX_PAYLOAD_CHECK==1)
    //R105[7]=1, Enable Tx payload check
    RFIP_reg_MEM[RFIP_REG_MEM_105] = (RFIP_reg_MEM[RFIP_REG_MEM_105] | 0x80);    //enable TX payload check. When write payload into TX_buffer, must fill 4th byte b7=1
#else
    //R105[7]=0, Disable Tx payload check
    RFIP_reg_MEM[RFIP_REG_MEM_105] = (RFIP_reg_MEM[RFIP_REG_MEM_105] & 0x7F);    //disable TX payload check
#endif //(ENABLE_TX_PAYLOAD_CHECK==1)
    SPI_1BYT_SetTx(RFIP_REG_105, RFIP_reg_MEM[RFIP_REG_MEM_105]);

    //------- If (received header[1]<MAX_PAYLOAD_LEN), CRC_err! -------//
    //SPI_1BYT_SetTx(RFIP_REG_165, 0xFF); //()
    RF_RxLengthLimit(MAX_PAYLOAD_LEN);  //255

    //R172=0x0D. //b0=1:system_normal (0 is reset_n); b1=0, b2=1; b3=1:wait RC32K cal_done before Sleep; b5:4=00; b7:6=0/1/2(cal_time=1ms/2ms/4ms)
    SPI_1BYT_SetTx(RFIP_REG_172, 0x0D);

    //TX low_IF, R175=0x40
    SPI_1BYT_SetTx(RFIP_REG_175, 0x40);

    //normal SPI, R250=0x00
    SPI_1BYT_SetTx(RFIP_REG_250, 0x00);

    //-------Init BB/MAC register--------//
    //clear IRQ status
    SPI_1BYT_SetTx(RFIP_REG_62, 0x7f);                      //write 1 to clear status

    //R98~R102'b0 are RTC timer count, initial are all 0
    //R102[7]=1, update RTC
    SPI_1BYT_SetTx(RFIP_REG_102, 0x80);

    // clear RX FIFO
    rafael_spi_clear_fifo();

    // clear TX_buffer_cnt
    rafael_spi_clear_tx_fifo_cnt();


    RFIP_reg_MEM[RFIP_REG_MEM_119] &= (~(REG_119_MANUAL_TRX_EN | REG_119_TR_TRIG_MODE));
    SPI_1BYT_SetTx(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);   //RFIP_init_reg[119] is 0 as init..

    //Auto switch TX/RX => R120[2]

#if (ENABLE_AUTOMATIC_SWITCH_MODE==1)
    RFIP_reg_MEM[RFIP_REG_MEM_120] |= REG_120_AUTO_TRX;  //enable auto TR switch
#else
    RFIP_reg_MEM[RFIP_REG_MEM_120] &= (~REG_120_AUTO_TRX);
#endif  //(ENABLE_AUTOMATIC_SWITCH_MODE==1)

    //clear bit7 first before set R120, disable RX access search time-out function.
    RFIP_reg_MEM[RFIP_REG_MEM_120] &= (~(REG_120_UPD_WAKE_TMR | REG_120_RX_TMROUT_EN));
    //rafael_spi_write(SPI_MASTER_PORT, 120, RFIP_init_reg+120, 1);
    SPI_1BYT_SetTx(RFIP_REG_120, RFIP_reg_MEM[RFIP_REG_MEM_120]);

    //reset MAC state write 1->0
    rafael_reset_phy_fsm();

    // Tx_buffer start address = 0
    RF_TxFIFO_OriginAddr_set(0x0000);

    // PLL Calibration
#if (ENABLE_VCO_CAL==1)
    //PLL linear mode & VCO calibration
    if (pll_cal_done_flag == 0)
    {
        rafael_pll_calibration();
        pll_cal_done_flag = 1;
        /*
             //if(Phy_pll_bank[0]==Phy_pll_bank[4])
             if((Phy_pll_bank[1]<=Phy_pll_bank[0]) || (Phy_pll_bank[2]<=Phy_pll_bank[1]) || (Phy_pll_bank[3]<=Phy_pll_bank[2]) || (Phy_pll_bank[4]<=Phy_pll_bank[3]))
             {
               //Give a fixed value as a protection mechanism.
               Phy_pll_bank[0] = 36;
               Phy_pll_bank[1] = 38;
               Phy_pll_bank[2] = 41;
               Phy_pll_bank[3] = 43;
               Phy_pll_bank[4] = 45;

               pll_cal_done_flag=0;
             }
             else{
               pll_cal_done_flag=1;
             }
        */
    }

    //R36[2]=1, linear mode
    RFIP_reg_MEM[RFIP_REG_MEM_36] = RFIP_reg_MEM[RFIP_REG_MEM_36] | 0x04;  //linear mode
    //RFIP_reg_MEM[RFIP_REG_MEM_36] = RFIP_reg_MEM[RFIP_REG_MEM_36] & 0xFB;  //binary mode
    SPI_1BYT_SetTx(RFIP_REG_36, RFIP_reg_MEM[RFIP_REG_MEM_36]);

#else  //!ENABLE_VCO_CAL
    //PLL hybrid mode(binary->linear), (R88[4]=1,so set R88=0x11)
    //SPI_1BYT_SetTx(RFIP_REG_88, 0x11);

    //PLL binary mode
    RFIP_reg_MEM[RFIP_REG_MEM_36] = RFIP_reg_MEM[RFIP_REG_MEM_36] & 0xFB;  //binary mode
    SPI_1BYT_SetTx(RFIP_REG_36, RFIP_reg_MEM[RFIP_REG_MEM_36]);
    SPI_1BYT_SetTx(RFIP_REG_88, 0x01);    //PLL mode depend on R36[2]
#endif  //(ENABLE_VCO_CAL==1)

    //no encryption at initialization
    SPI_1BYT_SetTx(RFIP_REG_155, REG_155_AES_MODE_BYPASS);

    MCU_GpioIntEnable();

    Ble_SW_Init();

#if (ENABLE_RF_VOLT_DETECT==1)
    volt = RF_Voltage_Det();
    Init_flag = 1;
#ifndef _HCI_HW_
    printf("volt=%d\n", volt); //removed in SDK
#endif
#endif  //(ENABLE_RF_VOLT_DETECT==1)  

}


void initInt(void)          //initial interrupt
{
    Content_ioInt = 0;      //initialization
}



uint8_t RF_WTR_TRxChk(void)     //0: Rx, Not 0: Tx
{
    return 0;
}


uint8_t RF_WTR_EndChk(void)
{
    return 0;
}


uint8_t RF_Sync_Chk(void)
{
    return 0;
}

void RF_RxFIFOrst(void)     //=> use rafael_spi_clear_fifo() for bit_change
{
    SPI_1BYT_SetTx_Isr((RFIP_REG_107), 0x80);
    SPI_1BYT_SetTx_Isr((RFIP_REG_107), 0x00);
}


uint8_t RF_Tmr16_FlagChk(void)
{
    return 0;
}


void RF_Tmr16_FlagClr_Isr(void)
{
    __NOP();
}



//#endif  //(#ifdef _TIMER_)

#pragma push
#pragma Otime
//#pragma Ospace
void rafael_reset_phy_fsm_Isr(void)
{
    uint8_t i;

    i = RFIP_reg_MEM[RFIP_REG_MEM_121];    //1: reset MAC state to READY
    //RFIP_reg_MEM[RFIP_REG_MEM_121] |= 0x80;
    //rafael_spi_write(spi_instance, 121, RFIP_init_reg+121, 1);
    SPI_1BYT_SetTx_Isr(RFIP_REG_121, (i | REG_121_RST_STATE));

    //Write 1 to end all TX & RX events and reset the MAC state to READY STATE, user must write 0 to this bit, so PMU state machine would go normally.
    //rafael_spi_write(spi_instance, 121, RFIP_init_reg+121, 1);
    SPI_1BYT_SetTx_Isr(RFIP_REG_121, i);
}


void setChannel_BLE(uint8_t ch)     //used in Isr
{
    uint8_t i;

    i = RFIP_reg_MEM[RFIP_REG_MEM_121];
    //i = (i & 0xC0)|CH_SEL_TABLE[ch];
    i = ((i & (~REG_121_MSK_CHN_IDX)) | (ch & REG_121_MSK_CHN_IDX));
    SPI_1BYT_SetTx_Isr(RFIP_REG_121, i);
    RFIP_reg_MEM[RFIP_REG_MEM_121] = i;
}


void RF_WTR_intOn(void)
{
    uint8_t i;

    //RFIP_reg_MEM[RFIP_REG_MEM_61] = (RFIP_reg_MEM[RFIP_REG_MEM_61]|INT_SETTING_W);
    //SPI_1BYT_SetTx_Isr(RFIP_REG_61, RFIP_reg_MEM[RFIP_REG_MEM_61]);
    i = (RFIP_reg_MEM[RFIP_REG_MEM_61] | INT_SETTING_W_WTR);
    RFIP_reg_MEM[RFIP_REG_MEM_61] = i;
    SPI_1BYT_SetTx_Isr(RFIP_REG_61, i);
}


void RF_WTR_intOff(void)
{
    uint8_t i;

    //RFIP_reg_MEM[RFIP_REG_MEM_61] = (RFIP_reg_MEM[RFIP_REG_MEM_61]&(~INT_SETTING_W));
    //SPI_1BYT_SetTx_Isr(RFIP_REG_61, RFIP_reg_MEM[RFIP_REG_MEM_61]);
    i = (RFIP_reg_MEM[RFIP_REG_MEM_61] & (~INT_SETTING_W_WTR));
    RFIP_reg_MEM[RFIP_REG_MEM_61] = i;
    SPI_1BYT_SetTx_Isr(RFIP_REG_61, i);
}


void RF_TxAutoAckOn(void)       //Not only for Tx
{
    uint8_t i;

    i = (RFIP_reg_MEM[RFIP_REG_MEM_120] | REG_120_AUTO_TRX);
    RFIP_reg_MEM[RFIP_REG_MEM_120] = i;
    SPI_1BYT_SetTx_Isr(RFIP_REG_120, i);
}


void RF_TxAutoAckOff(void)      //Not only for Tx
{
    uint8_t i;

    i = (RFIP_reg_MEM[RFIP_REG_MEM_120] & (~REG_120_AUTO_TRX));
    RFIP_reg_MEM[RFIP_REG_MEM_120] = i;
    SPI_1BYT_SetTx_Isr(RFIP_REG_120, i);
}


void RF_SymbolRate_set(uint8_t zSymbol_1M)    //0: 1M, not 0: 2M
{
    uint8_t i;

    if (zSymbol_1M)
    {
        i = (RFIP_reg_MEM[RFIP_REG_MEM_120] | REG_120_DATA_RATE);
    }
    else
    {
        i = (RFIP_reg_MEM[RFIP_REG_MEM_120] & (~REG_120_DATA_RATE));
    }
    RFIP_reg_MEM[RFIP_REG_MEM_120] = i;
    SPI_1BYT_SetTx_Isr(RFIP_REG_120, i);
}


void RF_SymbolRate_Patch_1M_2M(uint8_t zSymbol_1M)    //0: 1M, not 0: 2M
{
    uint8_t i;

    if (zSymbol_1M)
    {
        i = (NEXT_TX_TIME + 14);
        RFIP_reg_MEM[RFIP_REG_MEM_120] |= REG_120_DATA_RATE;
    }
    else
    {
        i = NEXT_TX_TIME;
        RFIP_reg_MEM[RFIP_REG_MEM_120] &= (~REG_120_DATA_RATE);
    }
    SPI_1BYT_SetTx_Isr(RFIP_REG_94, i);
}



void RF_TxFIFO_LeData_set(uint8_t *SrcAddr, uint8_t length)
{
    addr_TxFIFO = 2;
    SPI_PDMA_SetTx(TX_BUFFER_WRITE_PORT, (uint32_t)(SrcAddr), length);
}


void RF_LE_HeaderStsLen_Tx(uint8_t *SrcAddr)
{
    addr_TxFIFO = 0;

    //SPI_PDMA_SetTx(TX_BUFFER_WRITE_PORT, (uint32_t)(SrcAddr), 2);
    SPI_2BYT_SetTx_Isr(TX_BUFFER_WRITE_PORT, SrcAddr);  //write header into TX_buffer
}


void RF_RxLengthLimit(uint8_t maxPDU)
{
    SPI_1BYT_SetTx_Isr(RFIP_REG_165, maxPDU);
}

/*
void RF_RTC_WkpVal_set(uint64_t valueUpd)
{
    //__INT64
    uint64_t i64;

    i64 = valueUpd;
    //Little endian
    //((uint32_t *) &i64)[1] &= 0x0000001F;      //bit[36:32]
    //((uint32_t *) &i64)[1] |= 0x00000080;      //bit[39]

    //i = ((uint8_t *) &i64)[4] & 0x1F;
    //((uint8_t *) &i64)[4] = i | 0x80;
    SPI_PDMA_SetTx(RFIP_REG_115, (uint32_t)&i64, 5);
    ((uint32_t *) &i64)[1] |= 0x00000080;                   //bit[39]
    ((uint32_t *) &i64)[1] &= (0x0000001F|0x00000080);      //bit[39, 36:32]
    RFIP_reg_MEM[RFIP_REG_MEM_119] = ((uint8_t *) &i64)[4];
    SPI_PDMA_waitFinish();
}
*/

void RF_RTC_initVal_set(uint64_t valueUpd)
{
    //__INT64
    uint64_t i64;

    i64 = valueUpd;
    //Little endian
    //((uint32_t *) &i64)[1] &= 0x0000001F;      //bit[36:32]
    //((uint32_t *) &i64)[1] |= 0x00000080;      //bit[39]

    //i = ((uint8_t *) &i64)[4] & 0x1F;
    //((uint8_t *) &i64)[4] = i | 0x80;
    SPI_PDMA_SetTx(RFIP_REG_98, (uint32_t)&i64, 5);
    ((uint32_t *) &i64)[1] |= 0x00000080;                   //bit[39]
    ((uint32_t *) &i64)[1] &= (0x0000001F | 0x00000080);    //bit[39, 36:32]
    SPI_PDMA_waitFinish();
}


uint64_t RF_RTCvalue_get(void)
{
    //__INT64
    uint64_t i64;

    SPI_PDMA_SetRx_Isr(RFIP_REG_98, (uint32_t)&i64, 5);
    SPI_PDMA_waitFinish();
    return i64;
}


void RF_TxFIFO_OriginAddr_set(uint16_t OriginAddr)
{
    OriginAddr &= 0x01FF;
    SPI_2BYT_SetTx_Isr(RFIP_REG_104, (uint8_t *)&OriginAddr);
}


uint16_t RF_RxFIFO_Count_get(void)
{
    //__INT64
    uint16_t i16;

    SPI_PDMA_SetRx_Isr(RFIP_REG_106, (uint32_t)&i16, 2);
    SPI_PDMA_waitFinish();
    return (i16 & 0x01FF);
}


/*
void RF_CCM_NonceIV_Key16_set(uint8_t *addrIV)
{
    SPI_PDMA_SetTx(RFIP_REG_131, (uint32_t)&addrIV, LEN_SMP_IVS+LEN_SMP_IVM+LEN_AES_KEY);
}


void RF_CCM_NoncePktCnt_set(uint8_t *addrPktCnt)
{
    SPI_PDMA_SetTx(RFIP_REG_126, (uint32_t)&addrPktCnt, LEN_SMP_PKTCNT);
}
*/


void RF_CCM_AES_Mode_set(uint8_t setting)
{
    uint8_t LL_SMP_DATACH_CONV[] =
    {
        REG_155_AES_MODE_BYPASS,
        REG_155_AES_MODE_CCM_TX_ENCRY,
        REG_155_AES_MODE_CCM_RX_DECRY,
        REG_155_AES_MODE_CCM_ENABLE,
    };
    //Ref. LL_SMP_DATA_CH_R_T_NORMAL, LL_SMP_DATA_CH_T_CCM, LL_SMP_DATA_CH_R_CCM & LL_SMP_DATA_CH_R_T_CCM

    SPI_1BYT_SetTx_Isr(RFIP_REG_155, LL_SMP_DATACH_CONV[(setting & REG_155_AES_MODE)]);
}


#if (ENABLE_RF_VOLT_DETECT==1)
uint8_t RF_Voltage_Det(void)
{
    uint8_t volt_read;

    volt_read = SPI_1BYT_SetRx(RFIP_REG_4);

    volt_read = (volt_read & 0x20) >> 5; //R4[5]: VBAT volt
    if ((RfVolt_new != volt_read) || (Init_flag == 0))
    {

        //set both buck,ldo on
        RFIP_reg_MEM[RFIP_REG_MEM_40] = (RFIP_reg_MEM[RFIP_REG_MEM_40] & 0xAF) | 0x50;         //R40[4]=1(buck on);R40[6]=1(ldo on)
        SPI_1BYT_SetTx(RFIP_REG_40, RFIP_reg_MEM[RFIP_REG_MEM_40]);
        Tiny_Delay(10);

        if (volt_read == 1) //VBAT>2.5V
        {
            RFIP_reg_MEM[RFIP_REG_MEM_40] = (RFIP_reg_MEM[RFIP_REG_MEM_40] & 0xBF);                //R40[6]=0(ldo off)
            SPI_1BYT_SetTx(RFIP_REG_40, RFIP_reg_MEM[RFIP_REG_MEM_40]);
        }
        else    //VBAT<2.5V
        {
            RFIP_reg_MEM[RFIP_REG_MEM_40] = (RFIP_reg_MEM[RFIP_REG_MEM_40] & 0xEF);                //R40[4]=0(buck off)
            SPI_1BYT_SetTx(RFIP_REG_40, RFIP_reg_MEM[RFIP_REG_MEM_40]);
        }
    }
    RfVolt_new = volt_read;

    return volt_read;
}
#endif  //(ENABLE_RF_VOLT_DETECT==1)


void RF_Tmr_Periodic_initial(uint32_t period_tick, uint8_t sleep_mode) //1st time enable periodic timer
{
    uint8_t temp;
    uint8_t RTC_reg[5] = {0, 0, 0, 0, 0};
    uint32_t periodUS_Set;
//    uint32_t RTC_timer_us_low, RTC_timer_us_high;

    //disable RT568 timer wakeup INT
    temp = (RFIP_reg_MEM[RFIP_REG_MEM_61] & (~REG_61_WAKEUP));
    RFIP_reg_MEM[RFIP_REG_MEM_61] = temp;
    SPI_1BYT_SetTx(RFIP_REG_61, temp);

    //set one shot mode. R159[7]=0
    SPI_1BYT_SetTx(RFIP_REG_159, 0);

    //update RTC timer R98~R102 as 0
    RTC_reg[4] = 0x80; //update RTC
    SPI_PDMA_SetTx(RFIP_REG_98, (uint32_t)(RTC_reg), 5);
    SPI_PDMA_waitFinish();      //need to wait SPI done. or RTC_reg will be updated later

    periodUS_Set = period_tick * 125;
    /*
        //Sleep or not. Set R120[7]
        if(sleep_mode==RF_SLEEP_ENABLE){  //Sleep
           temp = RFIP_reg_MEM[RFIP_REG_MEM_120] | REG_120_UPD_WAKE_TMR;    //R120[7]=1
        }
        else{               //Non-Sleep
           temp = RFIP_reg_MEM[RFIP_REG_MEM_120] & (~REG_120_UPD_WAKE_TMR); //R120[7]=0
        }
        temp = temp & 0xFD;    //clear bit1, use internal RTC sleep mode. 32k on
        RFIP_reg_MEM[RFIP_REG_MEM_120] = temp;
        SPI_1BYT_SetTx(RFIP_REG_120, temp);
    */
    //Write RTC timer (R115~R119), periodUS timeup
    RTC_reg[0] = periodUS_Set & 0xFF;
    RTC_reg[1] = (periodUS_Set >> 8) & 0xFF;
    RTC_reg[2] = (periodUS_Set >> 16) & 0xFF;
    RTC_reg[3] = (periodUS_Set >> 24) & 0xFF;

    if (ChipVer == MP_A1)
    {
        RTC_reg[4] = RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xE0;
        SPI_PDMA_SetTx(RFIP_REG_115, (uint32_t)(RTC_reg), 5);
        SPI_PDMA_waitFinish();
    }
    else
    {
        RTC_reg[4] = (RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xE0) | 0x20;   //R119[5] write 1 and write 0 to update RTC_compare
        SPI_PDMA_SetTx(RFIP_REG_115, (uint32_t)(RTC_reg), 5);
        SPI_PDMA_waitFinish();

        SPI_1BYT_SetTx(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xDF);  //R119[5]=0
    }

    //enable RT568 timer wakeup INT
    temp = (RFIP_reg_MEM[RFIP_REG_MEM_61] | REG_61_WAKEUP);
    RFIP_reg_MEM[RFIP_REG_MEM_61] = temp;
    SPI_1BYT_SetTx(RFIP_REG_61, temp);

    //Write period timer
    RF_Tmr_Periodic_set_ISR(period_tick);  //time unit:125us. enable periodic timer, INT in bit[6]

    //Sleep or not
    if (sleep_mode == 1) //Sleep
    {
        temp = RFIP_reg_MEM[RFIP_REG_MEM_120] | REG_120_UPD_WAKE_TMR;    //R120[7]=1
    }
    else                //Non-Sleep
    {
        temp = RFIP_reg_MEM[RFIP_REG_MEM_120] & (~REG_120_UPD_WAKE_TMR); //R120[7]=0
    }
    RFIP_reg_MEM[RFIP_REG_MEM_120] = temp;
    SPI_1BYT_SetTx(RFIP_REG_120, temp);
}


/*
void RF_Tmr_Periodic_set(uint32_t periodUS)  //change periodic timer
{
    extern uint32_t Timeline24;

    uint32_t periodUS_Set;

    periodUS_Set = periodUS*125;
    //periodUS_Set |= (1<<31);       //R159[7]=1, enable periodic INT
    periodUS_Set |= 0x80000000;
    SPI_PDMA_SetTx(RFIP_REG_156, (uint32_t)&periodUS_Set, 4);

    Tmr16Interval = periodUS;
    Timeline24 += periodUS;
    if(((uint8_t *)&Timeline24)[3] > 0xEF) {
        //Timeline24 &= 0x0FFFFFFF;
        ((uint8_t *)&Timeline24)[3] &= 0x0F;
    }

    //SPI_PDMA_SetTx(RFIP_REG_156, (uint32_t)&periodUS, 4);
    //((uint8_t *)periodUS)[3] |= REG_159_PERIOD_INT;
    SPI_PDMA_waitFinish();
}
*/


#pragma push
#pragma O1
#pragma Otime
void RF_Tmr_Periodic_set_ISR(uint16_t period_tick)  //change periodic timer
{
    extern uint32_t Timeline24;
    uint32_t periodUS_Set;

    periodUS_Set = period_tick * 125;
    periodUS_Set |= 0x80000000;
    SPI_PDMA_SetTx(RFIP_REG_156, (uint32_t)&periodUS_Set, 4);

    Timeline24 += period_tick;
    if (((uint8_t *)&Timeline24)[3] > 0xEF)
    {
        //Timeline24 &= 0x0FFFFFFF;
        ((uint8_t *)&Timeline24)[3] &= 0x0F;
    }

    Tmr16Interval = period_tick;

    SPI_PDMA_waitFinish();
}
#pragma pop

/*
void RF_Tmr_OneShot_set(uint64_t durUS)
{
    uint64_t i64;

    i64  = durUS;

    SPI_PDMA_SetTx(RFIP_REG_115, (uint32_t)&i64, 5);
    ((uint32_t *) &i64)[1] |= 0x00000080;                   //bit[39]
    ((uint32_t *) &i64)[1] &= (0x0000001F|0x00000080);      //bit[39, 36:32]
    RFIP_reg_MEM[RFIP_REG_MEM_119] = ((uint8_t *) &i64)[4];
    SPI_1BYT_SetTx_Isr(RFIP_REG_159, (0&~REG_159_PERIOD_INT));
    SPI_PDMA_waitFinish();
}
*/


void RF_PowerSaving_En_Isr(void)
{

#if (ENABLE_RF_VOLT_DETECT==1)
    //Monitor RF Voltage here
    RF_Voltage_Det();
#endif

}


void RF_LE_HeaderStsLen_Rx(void)
{
}

int8_t RF_Get_LastRssi(void)
{


#if (ENABLE_RF_RSSI==1)  //do not calculate RSSI in RX->TIFS critical section

    RT568_Gain_Info RT568_rf_gain;
    int16_t rssi_dbm;
    uint16_t lna_gain;
    uint16_t tia_gain;
    uint16_t vga_gain;
    uint8_t rssi_read_data[3];

    SPI_PDMA_SetRx_Isr(RFIP_REG_166, (uint32_t)&rssi_read_data[0], 3);
    SPI_PDMA_waitFinish();

    RT568_rf_gain.LNA_gain = (uint8_t)((rssi_read_data[0] & 0xF0) >> 4);
    RT568_rf_gain.TIA_gain = (uint8_t)(rssi_read_data[1] & 0x0F);
    RT568_rf_gain.VGA_gain = (uint8_t)((rssi_read_data[1] & 0xF0) >> 4);
    RT568_rf_gain.BB_rssi_dbm = (uint8_t)rssi_read_data[2];

    lna_gain = R568_lna_agc[RT568_rf_gain.LNA_gain]; //gain*128
    tia_gain = R568_tia_agc[RT568_rf_gain.TIA_gain]; //gain*128
    vga_gain = R568_vga_agc[RT568_rf_gain.VGA_gain]; //gain*128

    rssi_dbm = (int16_t)(0 - (lna_gain + tia_gain + vga_gain)) - (int16_t)((255 - RT568_rf_gain.BB_rssi_dbm) << 7);

    // Compensation 3.5 db
    if (rssi_read_data[2] == 254)
    {
        //rssi_dbm += 350;   //3.5*100
        rssi_dbm += 448;   //3.5*128
    }

    rssi_dbm = ((rssi_dbm >> 7) + rssiOffsetValue); // Add offset value

    return rssi_dbm;
#else
    return 0x1F;
#endif

}

uint8_t RF_Set_TxPowerLevel_Isr(int8_t power)
{

    // 2020/03/11 Neil 40pin
    switch (power)
    {
    case TX_POWER_0_DBM:
        SPI_PDMA_SetTx(RFIP_REG_23, (uint32_t)&txpower_0dbm_reg, 4); // R23-R26
        break;

    case TX_POWER_4_DBM:
        SPI_PDMA_SetTx(RFIP_REG_23, (uint32_t)&txpower_4dbm_reg, 4); // R23-R26
        break;

    case TX_POWER_8_DBM:
        SPI_PDMA_SetTx(RFIP_REG_23, (uint32_t)&txpower_8dbm_reg, 4); // R23-R26
        break;

    case TX_POWER_10_DBM:
        SPI_PDMA_SetTx(RFIP_REG_23, (uint32_t)&txpower_10dbm_reg, 4); // R23-R26
        break;

    default:
        return FAIL; // error tx power
    }

    return SUCCESS;
}

int8_t RF_Get_TxPowerLevel(void)
{

    uint8_t txpower_read_data[4];

    SPI_PDMA_SetRx_Isr(RFIP_REG_23, (uint32_t)&txpower_read_data[0], 4);
    SPI_PDMA_waitFinish();

    if (memcmp(txpower_0dbm_reg, txpower_read_data, 4) == 0)
    {
        return (int8_t)TX_POWER_0_DBM;
    }
    else if (memcmp(txpower_4dbm_reg, txpower_read_data, 4) == 0)
    {
        return (int8_t)TX_POWER_4_DBM;
    }
    else if (memcmp(txpower_8dbm_reg, txpower_read_data, 4) == 0)
    {
        return (int8_t)TX_POWER_8_DBM;
    }
    else if (memcmp(txpower_10dbm_reg, txpower_read_data, 4) == 0)
    {
        return (int8_t)TX_POWER_10_DBM;
    }

    return TX_POWER_10_DBM;
}

int8_t RF_Get_MaximumTxPowerLevel(void)
{
    return (int8_t)TX_POWER_10_DBM;
}

void RF_Enter_DeepSleep(void)
{
    //Enter Deep Sleep mode (32K off)
    uint8_t temp41, temp42;

    //R41[6:5]=11(N power smallest), R41[7],42[0]=11 (P power smallest)
    temp41 = (RFIP_reg_MEM[RFIP_REG_MEM_41] & 0x1F) | 0xE0;
    temp42 = (RFIP_reg_MEM[RFIP_REG_MEM_42] & 0x7E) | 0x81;

    SPI_1BYT_SetTx(RFIP_REG_41, temp41);
    SPI_1BYT_SetTx(RFIP_REG_42, temp42);

    //set R120 bit1, bit7
    SPI_1BYT_SetTx(RFIP_REG_120, (RFIP_reg_MEM[RFIP_REG_MEM_120] | 0x82));
}

void RF_External_Wakeup(void)
{
    //external wakeup from Sleep or Deep Sleep, by SPI write R0 any value
    SPI_1BYT_SetTx(RFIP_REG_0, 0);
    //Tiny_Delay(2000);  // XTAL_TURNON_TIME is set to 1ms, delay must > 1ms
    Tiny_Delay(3500);    // XTAL_TURNON_TIME is set to 3ms, delay must > 3ms


    /* ====== Re-enable BG ============================ */
    SPI_1BYT_SetTx(RFIP_REG_28, 0);  // disable
    Tiny_Delay(500);
    SPI_1BYT_SetTx(RFIP_REG_28, 1);  // enable

    //Set noraml R41,R42 values
    //SPI_2BYT_SetTx_Isr(RFIP_REG_41, RFIP_init_reg+41);
    SPI_1BYT_SetTx(RFIP_REG_41, RFIP_reg_MEM[RFIP_REG_MEM_41]);
    SPI_1BYT_SetTx(RFIP_REG_42, RFIP_reg_MEM[RFIP_REG_MEM_42]);

}

void RT568_tx_rf_continuous_mode(uint8_t new_tx_channel)
{
    uint8_t temp32, temp34;

    //PLL Frequency change
    if (new_tx_channel < 7)
    {
        temp32 = 0x62;
    }
    else if (new_tx_channel < 15)
    {
        temp32 = 0xA2;
    }
    else if (new_tx_channel < 23)
    {
        temp32 = 0xE2;
    }
    else if (new_tx_channel < 31)
    {
        temp32 = 0x23;
    }
    else if (new_tx_channel < 39)
    {
        temp32 = 0x63;
    }
    else
    {
        temp32 = 0xA3;
    }

    temp34 = RFIP_PLL_Reg_R34[new_tx_channel];

    SPI_1BYT_SetTx(32, temp32);
    SPI_1BYT_SetTx(34, temp34);

    SPI_1BYT_SetTx(53, 0xC0);

}




#pragma pop


uint8_t CONN_IND_SAMPLE[] =
{
    0x05,                                       //header
    0x22,                                       //Payload Length: 34
    0x2e, 0x8b, 0xbf, 0x25, 0xde, 0x7e,         //Initiator Address: 0x7ede25bf8b2e
    0x43, 0xa8, 0x5d, 0x93, 0x7a, 0x18,         //Advertiser Address: 0x187a935da843
    0xbe, 0x2b, 0x6f, 0xf4,                     //Access Address: 0xf46f2bbe
    0x11, 0x22, 0x33,                           //CRC initialization value: 0x332211
    0x05,                                       //transmitWindowSize: 6.25  ms
    0x00, 0x00,                                 //transmitWindowOffset: 0.00  ms
    0x10, 0x00,                                 //connInterval: 20.00  ms
    0x00, 0x00,                                 //connSlaveLatency: 0
    0x2c, 0x01,                                 //connSupervisionTimeout: 3000.00  ms
    0xff, 0xff, 0xff, 0xff, 0x1f,               //Channel Map: 0001111111111111111111111111111111111111
    0x25                                        //masterSCA: 151 ppm to 250 ppm:0x001xxxxx, hopIncrement: 5:0xaaa00101
};


#pragma push
#pragma O0
//#pragma Otime
//#pragma Ospace


uint8_t ChipId_Get(void)
{
    return ChipVer;
}


uint32_t BleTxFIFOAddr_Get(void)
{
    return addr_TxFIFO;
}



#pragma pop

