#include <stdbool.h>
#include <string.h>
#include <stdlib.h> /* Random */
#include "ble_dtm.h"
#include "_ble_rf_dtm.h"
#include "rf_phy.h"
#include "_rafael_phy.h"
#include "config.h"
#include "bsp_misc.h"
#include "bsp_spi_ble.h"

extern void delay_us(u16 us) ;
/*----- BW=2M, use IF=2M ------*/
#define RADIO_PHY_1M             (0UL)
#define RADIO_PHY_2M             (1UL)
#define RADIO_PHY_CONST_CARRIER  (2UL)

#define DTM_PRINT_REGISTER       0
#define DTM_HEADER_OFFSET        0                                         /**< Index where the header of the pdu is located. */
#define DTM_HEADER_SIZE          2                                         /**< Size of PDU header. */
#define DTM_PAYLOAD_MAX_SIZE     255                                       /**< Maximum payload size allowed during dtm execution. */
#define DTM_LENGTH_OFFSET        (DTM_HEADER_OFFSET + 1)                   /**< Index where the length of the payload is encoded. */
#define DTM_PDU_MAX_MEMORY_SIZE  (DTM_HEADER_SIZE + DTM_PAYLOAD_MAX_SIZE)  /**< Maximum PDU size allowed during dtm execution. */
//------- Rafael define ----------//
#define DTM_PREAMBLE_SIZE        1
#define DTM_ACCESS_ADDR_SIZE     4
#define DTM_CRC_SIZE             3
#define TOTAL_PHY_MAX_SIZE      (DTM_PREAMBLE_SIZE + DTM_ACCESS_ADDR_SIZE + DTM_HEADER_SIZE + DTM_PAYLOAD_MAX_SIZE + DTM_CRC_SIZE)
//---------------------------------//
#define RX_MODE          1   /**< Constant defining RX mode for radio during dtm test. */
#define TX_MODE          0  /**< Constant defining TX mode for radio during dtm test. */

#define PHYS_CH_MAX      39     /**< Maximum number of valid channels in BLE. */

// Values that for now are "constants" - they could be configured by a function setting them,
// but most of these are set by the BLE DTM standard, so changing them is not relevant.
#define RFPHY_TEST_0X0F_REF_PATTERN  0x0f  /**<  RF-PHY test packet patterns, for the repeated octet packets. */
#define RFPHY_TEST_0X55_REF_PATTERN  0x55  /**<  RF-PHY test packet patterns, for the repeated octet packets. */
#define RFPHY_TEST_0XFF_REF_PATTERN  0xFF  /**<  RF-PHY test packet patterns, for the repeated octet packets. */

#define PRBS9_CONTENT  {0xFF, 0xC1, 0xFB, 0xE8, 0x4C, 0x90, 0x72, 0x8B,   \
                        0xE7, 0xB3, 0x51, 0x89, 0x63, 0xAB, 0x23, 0x23,   \
                        0x02, 0x84, 0x18, 0x72, 0xAA, 0x61, 0x2F, 0x3B,   \
                        0x51, 0xA8, 0xE5, 0x37, 0x49, 0xFB, 0xC9, 0xCA,   \
                        0x0C, 0x18, 0x53, 0x2C, 0xFD, 0x45, 0xE3, 0x9A,   \
                        0xE6, 0xF1, 0x5D, 0xB0, 0xB6, 0x1B, 0xB4, 0xBE,   \
                        0x2A, 0x50, 0xEA, 0xE9, 0x0E, 0x9C, 0x4B, 0x5E,   \
                        0x57, 0x24, 0xCC, 0xA1, 0xB7, 0x59, 0xB8, 0x87,   \
                        0xFF, 0xE0, 0x7D, 0x74, 0x26, 0x48, 0xB9, 0xC5,   \
                        0xF3, 0xD9, 0xA8, 0xC4, 0xB1, 0xD5, 0x91, 0x11,   \
                        0x01, 0x42, 0x0C, 0x39, 0xD5, 0xB0, 0x97, 0x9D,   \
                        0x28, 0xD4, 0xF2, 0x9B, 0xA4, 0xFD, 0x64, 0x65,   \
                        0x06, 0x8C, 0x29, 0x96, 0xFE, 0xA2, 0x71, 0x4D,   \
                        0xF3, 0xF8, 0x2E, 0x58, 0xDB, 0x0D, 0x5A, 0x5F,   \
                        0x15, 0x28, 0xF5, 0x74, 0x07, 0xCE, 0x25, 0xAF,   \
                        0x2B, 0x12, 0xE6, 0xD0, 0xDB, 0x2C, 0xDC, 0xC3,   \
                        0x7F, 0xF0, 0x3E, 0x3A, 0x13, 0xA4, 0xDC, 0xE2,   \
                        0xF9, 0x6C, 0x54, 0xE2, 0xD8, 0xEA, 0xC8, 0x88,   \
                        0x00, 0x21, 0x86, 0x9C, 0x6A, 0xD8, 0xCB, 0x4E,   \
                        0x14, 0x6A, 0xF9, 0x4D, 0xD2, 0x7E, 0xB2, 0x32,   \
                        0x03, 0xC6, 0x14, 0x4B, 0x7F, 0xD1, 0xB8, 0xA6,   \
                        0x79, 0x7C, 0x17, 0xAC, 0xED, 0x06, 0xAD, 0xAF,   \
                        0x0A, 0x94, 0x7A, 0xBA, 0x03, 0xE7, 0x92, 0xD7,   \
                        0x15, 0x09, 0x73, 0xE8, 0x6D, 0x16, 0xEE, 0xE1,   \
                        0x3F, 0x78, 0x1F, 0x9D, 0x09, 0x52, 0x6E, 0xF1,   \
                        0x7C, 0x36, 0x2A, 0x71, 0x6C, 0x75, 0x64, 0x44,   \
                        0x80, 0x10, 0x43, 0x4E, 0x35, 0xEC, 0x65, 0x27,   \
                        0x0A, 0xB5, 0xFC, 0x26, 0x69, 0x3F, 0x59, 0x99,   \
                        0x01, 0x63, 0x8A, 0xA5, 0xBF, 0x68, 0x5C, 0xD3,   \
                        0x3C, 0xBE, 0x0B, 0xD6, 0x76, 0x83, 0xD6, 0x57,   \
                        0x05, 0x4A, 0x3D, 0xDD, 0x81, 0x73, 0xC9, 0xEB,   \
                        0x8A, 0x84, 0x39, 0xF4, 0x36, 0x0B, 0xF7}           /**< The PRBS9 sequence used as packet payload.
                                                                                 The bytes in the sequence is in the right order, but the bits of each byte in the array is reverse.
                                                                                 of that found by running the PRBS9 algorithm. This is because of the endianess of the nRF5 radio. */



//static uint8_t tx_data_init_tx[260];
/**@brief Structure holding the PDU used for transmitting/receiving a PDU.
 */
typedef struct
{
    uint8_t content[DTM_HEADER_SIZE + DTM_PAYLOAD_MAX_SIZE];                 /**< PDU packet content. */
} pdu_type_t;

/*rafael define*/
typedef struct
{
    uint8_t content[4 + TOTAL_PHY_MAX_SIZE]; /**77+buf_addr1+buf_addr2+< Air packet content.>+dummy */
} air_packet_type_t;

/**@brief States used for the DTM test implementation.
 */
typedef enum
{
    STATE_UNINITIALIZED = 0,                                                   /**< The DTM is uninitialized. */
    STATE_IDLE = 1,                                                            /**< State when system has just initialized, or current test has completed. */
    STATE_TRANSMITTER_TEST = 2,                                                /**< State used when a DTM Transmission test is running. */
    STATE_CARRIER_TEST = 3,                                                    /**< State used when a DTM Carrier test is running (Vendor specific test). */
    STATE_RECEIVER_TEST = 4                                                    /**< State used when a DTM Receive test is running. */
} state_t;

// Internal variables set as side effects of commands or events.
static state_t           m_state = STATE_UNINITIALIZED;                      /**< Current machine state. */
static uint16_t          m_rx_pkt_count;                                     /**< Number of valid packets received. */
static pdu_type_t        m_pdu;                                              /**< PDU to be sent. */
static uint16_t          m_event;                                            /**< current command status - initially "ok", may be set if error detected, or to packet count. */
static uint8_t           m_new_event;                                        /**< Command has been processed - number of not yet reported event bytes. */
static uint32_t          m_packet_length;                                    /**< Payload length of transmitted PDU, bits 2:7 of 16-bit dtm command. */
static dtm_pkt_type_t    m_packet_type;                                      /**< Bits 0..1 of 16-bit transmit command, or 0xFFFFFFFF. */
static dtm_freq_t        m_phys_ch;                                          /**< 0..39 physical channel number (base 2402 MHz, Interval 2 MHz), bits 8:13 of 16-bit dtm command. */
static uint32_t          m_current_time = 0;

static uint8_t           m_phys_tx_ch_random_flag = 0;
static uint8_t           ch_random_min = 0;
static uint8_t           ch_random_max = 39;
static int               ch_random_val = 0;
static uint8_t           m_phys_tx_package_number_by_user_flag = 0;
static uint16_t          tx_user_defined_package_number_target = 0;
static uint16_t          tx_user_defined_package_number_real = 0;

/**< Counter for interrupts from timer to ensure that the 2 bytes forming a DTM command are received within the time window. */

// Definition of initial values found in ble_dtm.h

static uint8_t     m_prbs_content[]    = PRBS9_CONTENT;                /**< Pseudo-random bit sequence defined by the BLE standard. */

static uint8_t           m_radio_mode        = RADIO_PHY_1M;                 //TBD
static uint32_t          m_txIntervaluS      = 2500;                         /**< Time between start of Tx packets (in uS). */  //init value, it depends on packet length
//static uint32_t          m_txInterval125uS      = 5;                                                /**< Time between start of Tx packets (in 125uS). */  //init value, it depends on packet length

static uint8_t           m_timer_event[2] = {0, 0};
static uint8_t           m_rxcomplete_event = 0;

static uint8_t           m_crc_status = 0;                        //1:RX crc_check ok
static air_packet_type_t m_air_tx_packet;
static air_packet_type_t m_air_rx_packet;

static uint8_t  tx_continuous_mode_flag = 0;

#if (DTM_PRINT_REGISTER==1)
    uint8_t tx_data[300] ;
#endif
uint8_t DTM_init_reg[RADIO_TOTAL_REG_NUM];

//-----------------------------------------------//
static uint8_t  gui_get_rssi_flag = 0;

extern void  RF_Enter_DeepSleep(void);
extern void  RF_Tmr_Periodic_initial(uint32_t period_tick, uint8_t sleep_mode);
extern void RT568_tx_rf_continuous_mode(uint8_t new_tx_channel);
extern uint8_t RF_Voltage_Det(void);
extern uint8_t RFIP_init_reg[RADIO_TOTAL_REG_NUM];


static uint8_t check_pdu(void)
{
    uint8_t        k;                // Byte pointer for running through PDU payload
    uint8_t        pattern;          // Repeating octet value in payload
    dtm_pkt_type_t pdu_packet_type;  // Note: PDU packet type is a 4-bit field in HCI, but 2 bits in BLE DTM
    uint32_t       length = 0;

    pdu_packet_type = (dtm_pkt_type_t)(m_pdu.content[DTM_HEADER_OFFSET] & 0x0F); //* from received data */
    //length          = m_pdu.content[DTM_LENGTH_OFFSET];                          //* from received data */
    //pdu_packet_type = DTM_PKT_PRBS9;
    length = m_packet_length;

    // Check that the length is valid.
    if (length > DTM_PAYLOAD_MAX_SIZE)
    {
        return false;
    }

    //If the 1Mbit or 2Mbit radio mode is active, check that one of the three valid uncoded DTM packet types are selected.
    if ((m_radio_mode == RADIO_PHY_1M || m_radio_mode == RADIO_PHY_2M) && (pdu_packet_type > (dtm_pkt_type_t)DTM_PKT_0X55))
    {
        return false;
    }

    if (pdu_packet_type == DTM_PKT_PRBS9)
    {
        //Payload does not consist of one repeated octet; must compare ir with entire block into
        return (memcmp(m_pdu.content + DTM_HEADER_SIZE, m_prbs_content, length) == 0);
    }

    if (pdu_packet_type == DTM_PKT_0X0F)
    {
        pattern = RFPHY_TEST_0X0F_REF_PATTERN;
    }
    else if (pdu_packet_type == DTM_PKT_0X55)
    {
        pattern = RFPHY_TEST_0X55_REF_PATTERN;
    }
    else if (pdu_packet_type == DTM_PKT_0XFF)
    {
        pattern = RFPHY_TEST_0XFF_REF_PATTERN;
    }
    else
    {
        //No valid packet type set.
        return false;
    }

    for (k = 0; k < length; k++)
    {
        //Check repeated pattern filling the PDU payload
        if (m_pdu.content[k + 2] != pattern)
        {
            return false;
        }
    }


    return true;
}


/**@brief Function for turning off the radio after a test.
 *        Also called after test done, to be ready for next test.
 */
static void radio_reset(void)
{
    //-- called by radio_init() and dtm_test_done() --//
    rafael_reset_phy_fsm_Isr();
    //clear received packet count
    m_rx_pkt_count = 0;
}


/**@brief Function for initializing the radio for DTM.
 */
static uint32_t radio_init(void)
{
    // Turn off radio before configuring it
    radio_reset();
    RF_Init();
    return DTM_SUCCESS;
}


/**@brief Function for preparing the rt568 radio. At start of each test: Turn off RF, clear interrupt flags of RF, initialize the radio
 *        at given RF channel.
 *
 *@param[in] rx     boolean indicating if radio should be prepared in rx mode(1) or tx mode(0).
 */
static void radio_prepare_rt568(uint8_t rx)
{
    //uint8_t i;
    uint8_t parameterLSO[4];
    uint8_t rtcprepare_temp[5];
    uint8_t wakeupprepare_temp[6];
    uint8_t read_register_val = 0;
    //===========================================================
    RFIP_reg_MEM[RFIP_REG_MEM_61] = 0x3C;
    SPI_1BYT_SetTx(RFIP_REG_61, RFIP_reg_MEM[RFIP_REG_MEM_61]);

    //from initial function remove to radio_prepare_rt568 function
    //RFIP_init_reg[120] = (RFIP_init_reg[120] | 0x40); //R120[6]=1 Receive CRC/MIC in RX_FIFO
    RFIP_reg_MEM[RFIP_REG_MEM_120] = (RFIP_reg_MEM[RFIP_REG_MEM_120] | 0x40);
    SPI_1BYT_SetTx(RFIP_REG_120, RFIP_reg_MEM[RFIP_REG_MEM_120]);


    RFIP_reg_MEM[RFIP_REG_MEM_105] = (RFIP_reg_MEM[RFIP_REG_MEM_105] & 0x7F); //R105[7]=0, Disable Tx payload check
    SPI_1BYT_SetTx(RFIP_REG_105, RFIP_reg_MEM[RFIP_REG_MEM_105]);

    SPI_1BYT_SetTx(RFIP_REG_172, 0x0D);

    SPI_1BYT_SetTx(RFIP_REG_250, 0x00); //normal SPI

    SPI_1BYT_SetTx(RFIP_REG_165, 255); //MAX_PAYLOAD_LEN

    SPI_1BYT_SetTx(RFIP_REG_155, RAFAEL_AES_MODE_BYPASS);

    read_register_val = SPI_1BYT_SetRx((uint8_t)96);
    read_register_val = (((read_register_val & 0xF8) | 0x01) | (m_packet_type << 1)); //R96[0]=1, Enable DTM
    SPI_1BYT_SetTx((uint8_t)96, read_register_val);

    //Disable RTC periodic mode
    SPI_1BYT_SetTx(159, 0);

    //R98~R103'b0 are RTC timer count, initial are all 0.
    SPI_1BYT_SetTx(RFIP_REG_102, 0x80); //R102[7]=1, update RTC.

    //T/R Trigger Mode: manual mode (R119[6]=0)
    RFIP_reg_MEM[RFIP_REG_MEM_119] = RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xBF;
    SPI_1BYT_SetTx(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);

    //Auto switch TX/RX => R120[2]=0 disable Auto T/R
    RFIP_reg_MEM[RFIP_REG_MEM_120] = RFIP_reg_MEM[RFIP_REG_MEM_120] & 0xFB;
    SPI_1BYT_SetTx(RFIP_REG_120, RFIP_reg_MEM[RFIP_REG_MEM_120]);

    //===========================================================
    //called by RX: dtm_cmd()-LE_RECEIVER_TEST

    //reset MAC state write 1->0
    rafael_reset_phy_fsm_Isr();

    //write 1 to clear INT status
    SPI_1BYT_SetTx(RFIP_REG_62, 0x7F);

    //Set RF channel
    setChannel_BLE(m_phys_ch);

    //Set BW
    RFIP_reg_MEM[RFIP_REG_MEM_120] = ((RFIP_reg_MEM[RFIP_REG_MEM_120] & 0xF7) | (m_radio_mode << 3));
    SPI_1BYT_SetTx(RFIP_REG_120, RFIP_reg_MEM[RFIP_REG_MEM_120]);


    //Auto T/R disable
    RFIP_reg_MEM[RFIP_REG_MEM_120] = (RFIP_reg_MEM[RFIP_REG_MEM_120] & 0xFB);
    SPI_1BYT_SetTx(RFIP_REG_120, RFIP_reg_MEM[RFIP_REG_MEM_120]);

    RF_PLL_VCO_Bank_set(CH_PLL_bank_Table[m_phys_ch]);

    //Set Access Code
    parameterLSO[0] = 0x29;
    parameterLSO[1] = 0x41;
    parameterLSO[2] = 0x76;
    parameterLSO[3] = 0x71;
    //rafael_spi_write(122, parameterLSO, 4);
    SPI_PDMA_SetTx(RFIP_REG_122, (uint32_t)parameterLSO, 4);

    //clear FIFO
    RF_RxFIFOrst();

    //SPI_1BYT_SetTx(RFIP_REG_120, (SPI_1BYT_SetRx(RFIP_REG_120) & 0xDF));
    RFIP_reg_MEM[RFIP_REG_MEM_120] = (RFIP_reg_MEM[RFIP_REG_MEM_120] & 0xDF);
    SPI_1BYT_SetTx(RFIP_REG_120, RFIP_reg_MEM[RFIP_REG_MEM_120]);

    if (rx == true)
    {
        //RTC updata
        rtcprepare_temp[0] = 0x00;
        rtcprepare_temp[1] = 0x00;
        rtcprepare_temp[2] = 0x00;
        rtcprepare_temp[3] = 0x00;
        //Update RTC(ver T3) from Reg103[7] moves to Reg102[7]
        rtcprepare_temp[4] = 0x80;


        //Wake up time and
        //T/R Trigger Mode: Auto mode (R119[6]=1)
        //Auto T/R enable (R120[2]=1)
        //TR_TRIG_MODE[6], MAN_EN_TR[7]
        wakeupprepare_temp[0] = 0x80; //Reg115
        wakeupprepare_temp[1] = 0x00; //Reg116
        wakeupprepare_temp[2] = 0x00; //Reg117

        if (ChipVer > MP_A1)
        {
            wakeupprepare_temp[4] = 0x20; //Reg119
        }
        else
        {
            wakeupprepare_temp[4] = 0x40; //Reg119
        }

        //RFIP_init_reg[118] = 0x00;
        wakeupprepare_temp[3] = 0x00; //Reg118
        //RFIP_init_reg[120] = 0x04; //[bit 7] 1:Enable sleep
        wakeupprepare_temp[5] = 0x04; //Reg120
        //Set BW
        wakeupprepare_temp[5] = ((wakeupprepare_temp[5] & 0xF7) | (m_radio_mode << 3)); //Reg120
        SPI_PDMA_SetTx(RFIP_REG_115, ((uint32_t)(wakeupprepare_temp)), 6);        //R115~R120
        SPI_PDMA_SetTx(RFIP_REG_98, ((uint32_t)(rtcprepare_temp)), 5);        //R98~R103

        if (ChipVer > MP_A1)
        {
            //20200206   R119[5] Low->High->Low
            RFIP_reg_MEM[RFIP_REG_MEM_119] = 0x00;
            SPI_1BYT_SetTx(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);

            //RFIP_init_reg[119] = 0x40;
            RFIP_reg_MEM[RFIP_REG_MEM_119] = 0x40;
            SPI_1BYT_SetTx(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);
        }

        //First T/R=R
        RFIP_reg_MEM[RFIP_REG_MEM_121] = RFIP_reg_MEM[RFIP_REG_MEM_121] | 0x40;
        SPI_1BYT_SetTx(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);
    }
    else // tx
    {
        //First T/R=T
        RFIP_reg_MEM[RFIP_REG_MEM_121] = RFIP_reg_MEM[RFIP_REG_MEM_121] & 0xBF;
        SPI_1BYT_SetTx(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

        //TR_TRIG_MODE[6], MAN_EN_TR[7]
        //T/R Trigger Mode: Manual mode (R119[6]=0)
        RFIP_reg_MEM[RFIP_REG_MEM_119] = (RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xBF);
        SPI_1BYT_SetTx(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);
    }
    /*
        SPI_PDMA_SetRx_Isr(RFIP_REG_0, (uint32_t)&tx_data[0], 201);
        SPI_PDMA_waitFinish();
        for(print_count=0; print_count<201; print_count++)
        {
          printf("Reg%d=0x%02x\n",print_count,tx_data[print_count]);
        }
    */

}

/**@brief Function for terminating the ongoing test (if any) and closing down the radio.
 */
static void dtm_test_done(void)
{
    //called by dtm_cmd()-LE_TEST_SETUP & LE_TEST_END

    //TX/RX_close & m_rx_pkt_count=0
    radio_reset();

    m_state = STATE_IDLE;
}

//-----------------------------------------------------------------------------//
//@brief Function for configuring the timer for 260us and N*625us cycle time.
//called by dtm_init()
//-----------------------------------------------------------------------------//
static uint32_t timer_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    TIMER_Open(TIM16, 1, 2500) ;  //(2500 - 1); //2500us per interrupt
    TIMER_Open(TIM17, 1, 260)  ;//(260 - 1); //260us per interrupt

    /* Enable TIM16 TIM17 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority  = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd  = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
    NVIC_Init(&NVIC_InitStructure);
//    /* Open Timer0 in periodic mode, enable interrupt and 400 interrupt ticks per second */
//    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 400);  //2500us per interrupt
    TIMER_EnableInt(TIMER0);  //enable timer0 interrupt

//    /* Open Timer2 in periodic mode, enable interrupt and 3846 interrupt ticks per second */
//    TIMER_Open(TIMER2, TIMER_PERIODIC_MODE, 3846);  //3846Hz <-> 260us per interrupt
    TIMER_EnableInt(TIMER2);  //enable timer2 interrupt

//    /* Enable Timer0 ~ Timer1 NVIC */
//    NVIC_EnableIRQ(TIM16_IRQn);
//    NVIC_EnableIRQ(TIM17_IRQn);

    TIMER_Start(TIMER0);      //start timer0 counting
    TIMER_Start(TIMER2);      //start timer2 counting

    m_current_time        = 0;
    return DTM_SUCCESS;
}

static uint32_t dtm_packet_interval_calculate(uint32_t test_payload_length, uint32_t mode)
{
    uint32_t test_packet_length = 0; // [us] NOTE: bits are us at 1Mbit
    uint32_t packet_interval    = 0; // us
    uint32_t overhead_bits      = 0; // bits

    uint32_t i       = 0;
    uint32_t timeout = 0;

    /* packet overhead
     * see BLE [Vol 6, Part F] page 213
     * 4.1 LE TEST PACKET FORMAT */
    if (mode == RADIO_PHY_2M)
    {
        // 16 preamble
        // 32 sync word
        //  8 PDU header, actually packetHeaderS0len * 8
        //  8 PDU length, actually packetHeaderLFlen
        // 24 CRC
        overhead_bits = 88; // 11 bytes
    }
    else if (mode == RADIO_PHY_1M)
    {
        //  8 preamble
        // 32 sync word
        //  8 PDU header, actually packetHeaderS0len * 8
        //  8 PDU length, actually packetHeaderLFlen
        // 24 CRC
        overhead_bits = 80; // 10 bytes
    }

    /* add PDU payload test_payload length */
    test_packet_length = (test_payload_length * 8); // in bits

    // add overhead calculated above
    test_packet_length += overhead_bits;
    // we remember this bits are us in 1Mbit

    if (mode == RADIO_PHY_2M)
    {
        test_packet_length /= 2; // double speed
    }

    /*
     * packet_interval = ceil((test_packet_length+249)/625)*625
     * NOTE: To avoid floating point an equivalent calculation is used.
     */
    do
    {
        i++;
        timeout = i * 625;
    }
    while (test_packet_length + 249 > timeout);
    packet_interval = i * 625;

    return packet_interval;
}

uint32_t dtm_init(void)
{
    if ((timer_init() != DTM_SUCCESS) || (radio_init() != DTM_SUCCESS))
    {
        return DTM_ERROR_ILLEGAL_CONFIGURATION;
    }
    m_new_event     = false;     //set true in dtm_cmd()
    m_state         = STATE_IDLE;
    m_packet_length = 0;

    // Enable wake-up on event
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
    /*
    Send Event on Pending bit:
    0 = only enabled interrupts or events can wakeup the processor, disabled interrupts are excluded
    1 = enabled events and all interrupts, including disabled interrupts, can wakeup the processor.
    When an event or interrupt enters pending state, the event signal wakes up the processor from WFE. If the processor is not waiting for an event, the event is registered and affects the next WFE.
    The processor also wakes up on execution of an SEV instruction or an external event.
    */
    return DTM_SUCCESS;
}

uint32_t dtm_wait(void)
{
    // Enable wake-up on event
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    for (;;)
    {
        // Event may be the reception of a packet -
        // handle radio first, to give it highest priority:
        if (m_rxcomplete_event == 1)
        {
            m_rxcomplete_event = 0;

            if (m_state == STATE_RECEIVER_TEST)
            {
                if ((m_crc_status == 1) && check_pdu())  // CRC is ok, so the entire payload is received //
                {
                    m_crc_status = 0;
                    // Count the number of successfully received packets
                    m_rx_pkt_count++;
                }
                // Note that failing packets are simply ignored (CRC or contents error).

                // Zero fill all pdu fields to avoid stray data
                memset(&m_pdu, 0, DTM_PDU_MAX_MEMORY_SIZE);   // max size: 2+255=257 //
            }
        }

        if (m_timer_event[0] == 1)
        {
            m_timer_event[0] = 0;
        }
        else if (m_timer_event[1] == 1)
        {
            m_timer_event[1] = 0;
            return ++m_current_time;              // only return at here (every 260us) /
        }
        // Other events: No processing

    } //end of for(;;)
}

/**@brief Function for Get the RSSI flag,
 *
 * @return 1: The last RSSI value will be provided after dtm_event_get, 0: no No RSSI value provided.
*/
uint8_t dtm_get_gui_get_rssi_flag(void)
{
    return gui_get_rssi_flag;
}

/**@brief Function for splitting UART command bit fields into separate command parameters for the DTM library.
 *
 * @param[in]   command : The packed UART command.
 * @return      result status.
 */
uint32_t dtm_cmd_put(uint16_t command)
{
    uint32_t result;

    dtm_cmd_t      command_code = (command >> 14) & 0x03;
    dtm_freq_t     freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    dtm_pkt_type_t payload      = command & 0x03;

    result = dtm_cmd(command_code, freq, length, payload);

    return result;
}

uint32_t dtm_cmd(dtm_cmd_t cmd, dtm_freq_t freq, uint32_t length, dtm_pkt_type_t payload)
{
    // Save specified packet in uint8_t variable for tx/rx functions to use.
    // Note that BLE conformance testers always use full length packets.
    m_packet_length = (m_packet_length & 0xC0) | ((uint8_t)length & 0x3F); //* payload length infomation from CMD (not include header length) */
    m_packet_type   = payload;  //* type infomation from CMD */
    m_phys_ch       = freq;     //* frequency infomation from CMD */

    // Clean out any non-retrieved event that might linger from an earlier test
    m_new_event     = true;

    // Set default event; any error will set it to LE_TEST_STATUS_EVENT_ERROR
    m_event         = LE_TEST_STATUS_EVENT_SUCCESS;

    if (m_state == STATE_UNINITIALIZED)
    {
        // Application has not explicitly initialized DTM,
        return DTM_ERROR_UNINITIALIZED;
    }
    //Byte1 = cmd(7:6) + freq(5:0)
    //Byte2 = lengte(7:2) + payload(1:0)
    if (cmd == LE_TEST_SETUP)  //0x00
    {
        //* freq = control 6 bits     */
        //* length = parameter 6 bits */

        // Note that timer will continue running after a reset
        dtm_test_done();
        if (freq == LE_TEST_SETUP_RESET)  //* control = 0x00 (Reset) */
        {
            if (length != 0x00)   //* parameter = 0x00 (Reset) */
            {
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            // Reset the packet length upper bits.
            m_packet_length = 0;

            // Reset the selected PHY to 1Mbit
            m_radio_mode        = RADIO_PHY_1M;

            //reset flag
            m_phys_tx_ch_random_flag = 0;
            m_phys_tx_package_number_by_user_flag = 0; //0:unlimited, 1:package number by user
            tx_user_defined_package_number_target = 0;
            tx_user_defined_package_number_real = 0;
        }
        else if (freq == LE_TEST_SETUP_SET_UPPER) //* control = 0x01 */
        {
            if (length > 0x03)     //* parameter = 0x00~0x03 (set upper 2 bits of data length) */
            {
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            m_packet_length = length << 6;
        }
        else if (freq == LE_TEST_SETUP_SET_PHY)  //* control = 0x02 */
        {
            switch (length)      //* parameter */
            {
            case LE_PHY_1M:  //* 0x01 (set PHY to LE 1M) */
                m_radio_mode        = RADIO_PHY_1M;
                return radio_init();

            case LE_PHY_2M:  //* 0x02 (set PHY to LE 2M) */
                m_radio_mode        = RADIO_PHY_2M;
                return radio_init();

            case LE_PHY_LE_CODED_S8:  //* 0x03 (set PHY to LE coded S=8) */
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;

            case LE_PHY_LE_CODED_S2:  //* 0x04 (set PHY to LE Coded S=2) */
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;

            default:
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;
            }
        }
        else if (freq == LE_TEST_SETUP_SELECT_MODULATION) //* control = 0x03 */
        {
            if (length > 0x01)     //* parameter = 0x00~0x01 (assume TX has modulation index) */
            {
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            // Only standard modulation is supported.
        }
        else if (freq == LE_TEST_SETUP_READ_SUPPORTED)   //* control = 0x04 */
        {
            if (length != 0x00)    //* parameter = 0x00 (read test case support features) */
            {
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            // 0XXXXXXXXXXX0110 indicate that 2Mbit and DLE is supported and stable modulation is not supported (No nRF5 device supports this).
            m_event = 0x0006;
        }
        else if (freq == LE_TEST_SETUP_READ_MAX)    //* control = 0x05 */
        {
            // Read max supported value.
            switch (length)
            {
            case 0x00:
                // Read supportedMaxTxOctets
                m_event = 0x01FE;
                break;

            case 0x01:
                // Read supportedMaxTxTime
                m_event = 0x4290;
                break;

            case 0x02:
                // Read supportedMaxRxOctets
                m_event = 0x01FE;
                break;

            case 0x03:
                // Read supportedMaxRxTime
                m_event = 0x4290;
                break;

            default:
                m_event = LE_TEST_STATUS_EVENT_ERROR;
                return DTM_ERROR_ILLEGAL_CONFIGURATION;
            }
        }
        else
        {
            m_event = LE_TEST_STATUS_EVENT_ERROR;
            return DTM_ERROR_ILLEGAL_CONFIGURATION;
        }
        return DTM_SUCCESS;
    }

    if ((cmd == LE_TEST_END) && (freq != 0))
    {
        if (freq == DTM_VENDORSPECIFIC_TX_RANDOM_CH_DISABLE) //Random CH Disable
        {
            m_phys_tx_ch_random_flag = 0;
        }
        else if (freq == DTM_VENDORSPECIFIC_TX_RANDOM_CH_ENABLE) //Random CH enable
        {
            m_phys_tx_ch_random_flag = 1;
        }
        else if (freq == DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_UNLIMITED) // package number unlimited
        {
            m_phys_tx_package_number_by_user_flag = 0; //unlimited
            tx_user_defined_package_number_target = 0;
        }
        else if (freq == DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_BY_USER) //package number by user
        {

            m_phys_tx_package_number_by_user_flag = 1; //by user
            tx_user_defined_package_number_target = ((((length & 0x3F) << 2) | (payload & 0x03)) * 10);
        }
        else if (freq == DTM_VENDORSPECIFIC_TX_POWER_SELECT)
        {
            //Byte1 = cmd(7:6) + freq(5:0)
            //Byte2 = lengte(7:2) + payload(1:0)
            //length = 0(0dB),4(4dB),8(8dB),10(10dB)
            //payload = 0(20pin type), 1(40pin type)
            //dtm_set_txpower((uint8_t)length, (uint8_t)payload);
            RF_Set_TxPowerLevel_Isr((int8_t)length);//40pin only
        }
        else if (freq == DTM_VENDORSPECIFIC_RX_GET_RSSI)
        {
            gui_get_rssi_flag = (uint8_t)payload; //payload => 0:no print RSSI, 1:print RSSI after report PER.
        }
        else if (freq == DTM_VENDORSPECIFIC_GO_SLEEP_MODE)
        {
            RF_Tmr_Periodic_initial(0x1FFFFF, RF_SLEEP_ENABLE);
        }
        else if (freq == DTM_VENDORSPECIFIC_GO_DEEP_SLEEP_MODE)
        {
            RF_Enter_DeepSleep();
        }
        else if (freq == DTM_VENDORSPECIFIC_GO_TXRF_CONTINUOUS_MODE)
        {
            //R53[7:6] = 'b11
            tx_continuous_mode_flag = 1;
        }
        else if (freq == DTM_VENDORSPECIFIC_GO_TXRF_DTM_MODE)
        {
            tx_continuous_mode_flag = 0;
        }
        else
        {
            m_phys_tx_ch_random_flag = 0;
            m_phys_tx_package_number_by_user_flag = 0; //unlimited
            tx_user_defined_package_number_target = 0;
            tx_continuous_mode_flag = 0;
        }

        return DTM_SUCCESS;
    }

    if ((cmd == LE_TEST_END) && (freq == 0)) //0x03
    {
        if (m_state == STATE_IDLE)
        {
            // Sequencing error - only rx or tx test may be ended!
            m_event = LE_TEST_STATUS_EVENT_ERROR;
            return DTM_ERROR_INVALID_STATE;
        }
        m_event = LE_PACKET_REPORTING_EVENT | m_rx_pkt_count;   //* m_rx_pkt_count=0 in radio_reset() (/
        dtm_test_done();
        m_phys_tx_ch_random_flag = 0;

        return DTM_SUCCESS;
    }

    if (m_state != STATE_IDLE)
    {
        // Sequencing error - only TEST_END/RESET are legal while test is running
        // Note: State is unchanged; ongoing test not affected
        m_event = LE_TEST_STATUS_EVENT_ERROR;
        return DTM_ERROR_INVALID_STATE;
    }

    // Check for illegal values of m_phys_ch. Skip the check if the packet is vendor spesific.
    if (m_phys_ch > PHYS_CH_MAX)  //>39
    {
        // Parameter error
        // Note: State is unchanged; ongoing test not affected
        m_event = LE_TEST_STATUS_EVENT_ERROR;

        return DTM_ERROR_ILLEGAL_CHANNEL;
    }

    m_rx_pkt_count = 0;

    if (cmd == LE_RECEIVER_TEST)    //0x01
    {
        // Zero fill all pdu fields to avoid stray data from earlier test run
        memset(&m_pdu, 0, DTM_PDU_MAX_MEMORY_SIZE);
        radio_prepare_rt568(RX_MODE);                       // Reinitialize "everything"; RF interrupts OFF

        //m_cmd_payload_length = m_packet_length;      //*update payload length from cmd*/

        //R112: DTM payload length
        //RFIP_init_reg[112] = m_packet_length;        //R112, DTM test payload length
        //rafael_spi_write(112, RFIP_init_reg+112, 1);
        SPI_1BYT_SetTx(RFIP_REG_112, m_packet_length);

        m_state = STATE_RECEIVER_TEST;

#if (DTM_PRINT_REGISTER==1)
        SPI_PDMA_SetRx_Isr(RFIP_REG_0, (uint32_t)&tx_data[0], 201);
        SPI_PDMA_waitFinish();
        for (print_count = 0; print_count < 201; print_count++)
        {
            printf("RX.Reg%d=0x%02x\n", print_count, tx_data[print_count]);
        }
#endif
        return DTM_SUCCESS;
    }

    if (cmd == LE_TRANSMITTER_TEST)  //0x02
    {
        // Check for illegal values of m_packet_length. Skip the check if the packet is vendor spesific.
        if (m_packet_length > DTM_PAYLOAD_MAX_SIZE)   //>255
        {
            // Parameter error
            m_event = LE_TEST_STATUS_EVENT_ERROR;
            return DTM_ERROR_ILLEGAL_LENGTH;
        }

        // Note that PDU uses 4 bits even though BLE DTM uses only 2 (the HCI SDU uses all 4)
        m_pdu.content[DTM_HEADER_OFFSET] = ((uint8_t)m_packet_type & 0x0F);
        m_pdu.content[DTM_LENGTH_OFFSET] = m_packet_length;

        //pack whole air packet
        switch (m_packet_type)
        {
        case DTM_PKT_PRBS9:
        case DTM_PKT_0X0F:
        case DTM_PKT_0X55:
        case DTM_PKT_0XFF:
            //DTM mode: only write header
            memcpy(m_air_tx_packet.content, m_pdu.content, DTM_HEADER_SIZE); //2 bytes
            //R112: DTM payload length
            SPI_1BYT_SetTx(RFIP_REG_112, m_packet_length);
            break;
        default:

            break;
        }
        if (tx_continuous_mode_flag == 0) //DTM mode
        {
            // Initialize CRC value, set channel:
            radio_prepare_rt568(TX_MODE);

            // Set the timer to the correct period. The delay between each packet is described in the
            // Bluetooth Core Spsification version 4.2 Vol. 6 Part F Section 4.1.6.
            TIMER_Stop(TIMER0);       //stop timer counting
            TIMER_DisableInt(TIMER0); //disable timer interrupt

            m_txIntervaluS = dtm_packet_interval_calculate(m_packet_length, m_radio_mode);  // new, I(L)=N*625us /
            //m_txInterval125uS = (m_txIntervaluS/125);
            //RF_Tmr_Periodic_initial((m_txInterval125uS), RF_SLEEP_DISABLE);
            TIMER_Open(TIMER0, 1, m_txIntervaluS) ;  //(2500 - 1); //2500us per interrupt
            TIMER_EnableInt(TIMER0);  //enable timer interrupt
            TIMER_Start(TIMER0);      //start timer counting
        }
        else
        {
            RT568_tx_rf_continuous_mode(m_phys_ch);
        }

#if (DTM_PRINT_REGISTER==1)
        SPI_PDMA_SetRx_Isr(RFIP_REG_0, (uint32_t)&tx_data[0], 201);
        SPI_PDMA_waitFinish();
        for (print_count = 0; print_count < 201; print_count++)
        {
            printf("TX.Reg%d=0x%02x\n", print_count, tx_data[print_count]);
        }
#endif
        m_state            = STATE_TRANSMITTER_TEST;
    } //end of if(cmd==LE_TRANSMITTER_TEST)
    return DTM_SUCCESS;
}

uint8_t dtm_event_get(dtm_event_t *p_dtm_event)
{
    uint8_t was_new = m_new_event;  //set to true in dtm_cmd()
    // mark the current event as retrieved
    m_new_event  = false;
    *p_dtm_event = m_event;
    // return value indicates whether this value was already retrieved.
    return was_new;
}

//2500US timer interrupr
//--------------Interrupt Service Routine ---------------//
void TIM16_IRQHandler(void)
{
    if (TIM_GetITStatus(TIMER0, TIM_IT_Update) != RESET)
    {
        /* Clear TIM16 Update Interrupt Flag */
        TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
//        GPIOA ->ODR ^= GPIO_Pin_0 ;  //TEST PIN
        m_timer_event[0] = 1;
        //-------- add Tx action here ----------//
        if (m_state == STATE_TRANSMITTER_TEST)
        {
            if (m_phys_tx_package_number_by_user_flag == 1)
            {
                //m_phys_tx_package_number_by_user_flag = 0; //0:unlimited, 1:package number by user
                //tx_user_defined_package_number_target = 0;
                //tx_user_defined_package_number_real = 0;
                tx_user_defined_package_number_real += 1;
                if (tx_user_defined_package_number_real > tx_user_defined_package_number_target)
                {
                    return;
                }
            }

            if (m_phys_tx_ch_random_flag == 1)
            {
                //ch_random_min=0;
                //ch_random_max=39;
                //channel change
                ch_random_val = (rand() % (ch_random_max - ch_random_min + 1) + ch_random_min);
                //Set RF channel
                setChannel_BLE(ch_random_val);
            }

#if (ENABLE_RF_VOLT_DETECT==1)
            //Voltage detector by Read R4
            RF_Voltage_Det();
#endif  //(ENABLE_RF_VOLT_DETECT==1)                 

            //write only header 2 bytes to FIFO in DTM
            RF_LE_HeaderStsLen_Tx(m_air_tx_packet.content);

            SPI_1BYT_SetTx_Isr(RFIP_REG_119, (RFIP_reg_MEM[RFIP_REG_MEM_119] | REG_119_MANUAL_TRX_EN));

        }
        //---------------------------------------//
    }
}

//260us timer interrupr
void TIM17_IRQHandler(void)
{
    if (TIM_GetITStatus(TIMER2, TIM_IT_Update) != RESET)
    {
        /* Clear TIM17 Update Interrupt Flag */
        TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
//        GPIOA ->ODR ^= GPIO_Pin_1 ;  //TEST PIN
        //test
        m_timer_event[1] = 1;

    }
}

void DTM_Isr(void)
{
    uint8_t interrupt_sataus;
    uint8_t rxFifo[2];
    uint16_t rx_packet_len;

    //read IC INT status
    //rafael_spi_read(62, &interrupt_sataus, 1);
    interrupt_sataus = SPI_1BYT_SetRx(RFIP_REG_62);

    //clear IRQ status
    SPI_1BYT_SetTx(RFIP_REG_62, interrupt_sataus);  //write 1 to clear status

    //b5: access code searched
    //if(interrupt_sataus & REG_62_ACCESS_MATCH)
    //{
    //}

    //b4: RX packet received
    if (interrupt_sataus & REG_62_RX_END)
    {
        m_rxcomplete_event = 1;

        //read CRC_check result
        //rafael_spi_read(155, rxFifo, 1);
        rxFifo[0] = SPI_1BYT_SetRx_Isr(RFIP_REG_155);
        m_crc_status = (rxFifo[0] & 0x20) >> 5;   //R155[5]

        if (m_crc_status == 0)
        {
            m_rxcomplete_event = 0;

            //Step 1. STOP T/R FSM (R121[7] = 1, R121[6] = 1)
            RFIP_reg_MEM[RFIP_REG_MEM_121] = (RFIP_reg_MEM[RFIP_REG_MEM_121] | 0xC0);
            SPI_1BYT_SetTx_Isr(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

            //Step 2. PLL Lock Time = 2(us) (R91 = 2)
            SPI_1BYT_SetTx_Isr(RFIP_REG_91, 0x02);

            //Step 3. START T/R FSM, R119[7]=1
            RFIP_reg_MEM[RFIP_REG_MEM_119] = (RFIP_reg_MEM[RFIP_REG_MEM_119] | 0x80);
            SPI_1BYT_SetTx_Isr(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);

            //Step 4. wait 14us, let CCM start
            Tiny_Delay(14);

            //Step 5. STOP T/R FSM (R121[7] = 1, R121[6] = 1)
            RFIP_reg_MEM[RFIP_REG_MEM_121] = (RFIP_reg_MEM[RFIP_REG_MEM_121] | 0xC0);
            SPI_1BYT_SetTx_Isr(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

            //Step 6.Clear RX FIFO, R107[7]=1
            RF_RxFIFOrst();

            //Step 7.Read RX 1Byte FIFO to clear (FIFO<->SPI) buffer.
            SPI_PDMA_SetRx_Isr(RX_BUFFER_READ_PORT, (uint32_t)rxFifo, 1);

            //Step 8. Restore PLL Lock Time (R91=80)
            SPI_1BYT_SetTx_Isr(RFIP_REG_91, 80);

            RFIP_reg_MEM[RFIP_REG_MEM_121] = ((RFIP_reg_MEM[RFIP_REG_MEM_121] & 0x7F) | 0x40);
            SPI_1BYT_SetTx_Isr(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

            //Step 9. START T/R FSM Manually (R119[6]=0, Then R119[7]=1)
            RFIP_reg_MEM[RFIP_REG_MEM_119] = ((RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xBF) | 0x80);
            SPI_1BYT_SetTx_Isr(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);

        }
        else  //CRC =1
        {
            //Read RX FIFO(header, 2Byte)
            SPI_PDMA_SetRx_Isr(RX_BUFFER_READ_PORT, (uint32_t)m_air_rx_packet.content, 2);

            if (m_air_rx_packet.content[1] != m_packet_length)
            {
                m_rxcomplete_event = 0;

                //Step 1. STOP T/R FSM (R121[7] = 1, R121[6] = 1)
                RFIP_reg_MEM[RFIP_REG_MEM_121] = (RFIP_reg_MEM[RFIP_REG_MEM_121] | 0xC0);
                SPI_1BYT_SetTx_Isr(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

                //Step 2. PLL Lock Time = 2(us) (R91 = 2)
                SPI_1BYT_SetTx_Isr(RFIP_REG_91, 0x02);//default is 80us, apply to step 8.

                //Step 3. START T/R FSM, R119[7]=1
                SPI_1BYT_SetTx(RFIP_REG_119, (SPI_1BYT_SetRx(RFIP_REG_119) | 0x80));

                //Step 4. wait 14us, let CCM start
                Tiny_Delay(14);

                //Step 5. STOP T/R FSM (R121[7] = 1, R121[6] = 1)
                //rafael_reset_phy_fsm_Isr();
                RFIP_reg_MEM[RFIP_REG_MEM_121] = (RFIP_reg_MEM[RFIP_REG_MEM_121] | 0xC0);
                SPI_1BYT_SetTx_Isr(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

                //Step 6.Clear RX FIFO, R107[7]=1
                RF_RxFIFOrst();

                //Step 7.Read RX 1Byte FIFO to clear (FIFO<->SPI) buffer.
                SPI_PDMA_SetRx_Isr(RX_BUFFER_READ_PORT, (uint32_t)rxFifo, 1);

                //Step 8. Restore PLL Lock Time (R91=80)
                SPI_1BYT_SetTx_Isr(RFIP_REG_91, 80);

                RFIP_reg_MEM[RFIP_REG_MEM_121] = ((RFIP_reg_MEM[RFIP_REG_MEM_121] & 0x7F) | 0x40);
                SPI_1BYT_SetTx_Isr(RFIP_REG_121, RFIP_reg_MEM[RFIP_REG_MEM_121]);

                //Step 9. START T/R FSM Manually (R119[6]=0, Then R119[7]=1)
                RFIP_reg_MEM[RFIP_REG_MEM_119] = ((RFIP_reg_MEM[RFIP_REG_MEM_119] & 0xBF) | 0x80);
                SPI_1BYT_SetTx_Isr(RFIP_REG_119, RFIP_reg_MEM[RFIP_REG_MEM_119]);
            }
            else //Len = 37
            {
                rx_packet_len = m_air_rx_packet.content[1] + 3; //crc 3byte

                //Read RX FIFO
                SPI_PDMA_SetRx_Isr(RX_BUFFER_READ_PORT, (uint32_t)m_air_rx_packet.content + 2, rx_packet_len);

                rx_packet_len = rx_packet_len + 2;
                //memcpy(m_pdu.content, m_air_rx_packet.content, rx_packet_len);
                if (rx_packet_len >= 3)
                {
                    memcpy(m_pdu.content, m_air_rx_packet.content, rx_packet_len - 3); //only header+payload in m_pdu.content
                }

            }
        } //if(m_crc_status==0)
    }
    /*
        else if(interrupt_sataus & REG_62_WAKEUP)   //do TX Enable
        {
            m_timer_event[0] = 1;
            //-------- add Tx action here ----------//
            if(m_state== STATE_TRANSMITTER_TEST)
            {
                if(m_phys_tx_package_number_by_user_flag==1)
                {
                    //m_phys_tx_package_number_by_user_flag = 0; //0:unlimited, 1:package number by user
                    //tx_user_defined_package_number_target = 0;
                    //tx_user_defined_package_number_real = 0;
                    tx_user_defined_package_number_real+=1;
                    if(tx_user_defined_package_number_real > tx_user_defined_package_number_target)
                    {
                        return;
                    }
                }
                if(m_phys_tx_ch_random_flag == 1)
                {
                    //ch_random_min=0;
                    //ch_random_max=39;
                    //channel change
                    ch_random_val = (rand() % (ch_random_max - ch_random_min + 1) + ch_random_min);
                    //RFIP_init_reg[121] = (RFIP_init_reg[121] & 0xC0) | ch_random_val;
                    //rafael_spi_write(121, RFIP_init_reg+121, 1);
                    SPI_1BYT_SetTx(RFIP_REG_121, ((SPI_1BYT_SetRx(RFIP_REG_121) & 0xC0)|ch_random_val));
                }

    #if (ENABLE_RF_VOLT_DETECT==1)
                //Monitor RF Voltage here
                RF_Voltage_Det();
    #endif

                RF_Tmr_Periodic_initial((m_txInterval125uS), RF_SLEEP_DISABLE);

                //write only header 2 bytes to FIFO in DTM
                RF_LE_HeaderStsLen_Tx(m_air_tx_packet.content);

                //SPI set TX enable
                SPI_1BYT_SetTx_Isr(RFIP_REG_119, (RFIP_reg_MEM[RFIP_REG_MEM_119] | REG_119_MANUAL_TRX_EN));

            }//if(m_state== STATE_TRANSMITTER_TEST)
        }
    */
}
