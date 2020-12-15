/******************************************************************************
 * @file    main.c
 * @author  MM32 AE Team
 * @version V1.1.3
 * @date    26-August-2020
 * @brief   ......
 ******************************************************************************
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 * <H2><CENTER>&COPY; COPYRIGHT 2020 MINDMOTION </CENTER></H2>
******************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------*/
#define __MAIN_C__


/* Includes -----------------------------------------------------------------*/
#include "main.h"
#include "rf_phy.h"
#include "_rafael_phy.h"
#include "ble_dtm.h"

/* Includes -----------------------------------------------------------------*/
#include "systick.h"
#include "uart.h"
#include "spi.h"

/* Includes -----------------------------------------------------------------*/
#include "bsp_misc.h"
#include "bsp_spi_ble.h"


extern void delay_us(u16 us) ;
/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/

#if (BLE_DEMO==DEMO_DTM)
// @note: The BLE DTM 2-wire UART standard specifies 8 data bits, 1 stop bit, no flow control.
//        These parameters are not configurable in the BLE standard.
/**@details Maximum iterations needed in the main loop between stop bit 1st byte and start bit 2nd
 * byte. DTM standard allows 5000us delay between stop bit 1st byte and start bit 2nd byte.
 * As the time is only known when a byte is received, then the time between between stop bit 1st
 * byte and stop bit 2nd byte becomes:
 *      5000us + transmission time of 2nd byte.
 *
 * Byte transmission time is (Baud rate of 19200):
 *      10bits * 1/19200 = approx. 520 us/byte (8 data bits + start & stop bit).
 *
 * Loop time on polling UART register for received byte is defined in ble_dtm.c as:
 *   UART_POLL_CYCLE = 260 us
 *
 * The max time between two bytes thus becomes (loop time: 260us / iteration):
 *      (5000us + 520us) / 260us / iteration = 21.2 iterations.
 *
 * This is rounded down to 21.
 *
 * @note If UART bit rate is changed, this value should be recalculated as well.
 */
#define MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE ((5000 + 2 * UART_POLL_CYCLE) / UART_POLL_CYCLE)
#endif

#if (BLE_DEMO==DEMO_DTM)
void DTM_Main_Loop(void);
#endif

/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/

#if (BLE_DEMO==DEMO_DTM)
void DTM_Main_Loop(void)
{
    uint32_t    current_time;
    uint32_t    msb_time          = 0;     // Time when MSB of the DTM command was read. Used to catch stray bytes from "misbehaving" testers.
    uint8_t     is_msb_read       = 0;     // True when MSB of the DTM command has been read and the application is waiting for LSB.
    uint16_t    dtm_cmd_from_uart = 0;     // Packed command containing command_code:freqency:length:payload in 2:6:6:2 bits.
    uint8_t     rx_byte;                   // Last byte read from UART.
    dtm_event_t result;                    // Result of a DTM operation.

    dtm_init();

    for (;;)
    {
        TIMER_EnableInt(TIMER0);
        TIMER_EnableInt(TIMER2);

        // Will return every timeout, 625 us.
        current_time = dtm_wait();           // return every 260us //

        if(UART_GetFlagStatus(DEBUG_UART, UART_IT_RXIEN) == RESET) 
        {   
            // Nothing read from the UART.            
            continue;
        }
        UART_ClearITPendingBit(DEBUG_UART, UART_IT_RXIEN);
        rx_byte = (u8)UART_ReceiveData(DEBUG_UART);

        if (!is_msb_read)
        {
            // This is first byte of two-byte command.
            is_msb_read       = 1;
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;

            // Go back and wait for 2nd byte of command word.
            continue;
        }

        // This is the second byte read; combine it with the first and process command
        if (current_time > (msb_time + MAX_ITERATIONS_NEEDED_FOR_NEXT_BYTE))
        {
            // More than ~5mS after msb: Drop old byte, take the new byte as MSB.
            // The variable is_msb_read will remains true.
            // Go back and wait for 2nd byte of the command word.
            dtm_cmd_from_uart = ((dtm_cmd_t)rx_byte) << 8;
            msb_time          = current_time;
            continue;
        }

        // 2-byte UART command received.
        is_msb_read        = 0;
        dtm_cmd_from_uart |= (dtm_cmd_t)rx_byte;

        if (dtm_cmd_put(dtm_cmd_from_uart) != DTM_SUCCESS)
        {
            // Extended error handling may be put here.
            // Default behavior is to return the event on the UART (see below);
            // the event report will reflect any lack of success.
        }

        // Retrieve result of the operation. This implementation will busy-loop
        // for the duration of the byte transmissions on the UART.
        if (dtm_event_get(&result)) //"true" after running dtm_cmd()
        {
            // Report command status on the UART. (Send EVENT out)
            // Transmit MSB of the result.
            UART_SendData(DEBUG_UART, (result >> 8) & 0xFF);
            // Wait until MSB is sent.
            while(!UART_GetFlagStatus(DEBUG_UART, UART_FLAG_TXEPT));
            UART_ClearITPendingBit(DEBUG_UART, UART_FLAG_TXEPT);
//            UART0->INTSTS = UART0->INTSTS & (~UART_INTSTS_THREIF_Msk);

            // Transmit LSB of the result.
            UART_SendData(DEBUG_UART, result & 0xFF);
            // Wait until LSB is sent.
            while(!UART_GetFlagStatus(DEBUG_UART, UART_FLAG_TXEPT));
            UART_ClearITPendingBit(DEBUG_UART, UART_FLAG_TXEPT);

            //no support rssi value
            //if(dtm_get_gui_get_rssi_flag()==1)
            //{
            //printf("RSSI:%d\n", RF_Get_LastRssi()); //dBm
            //}
        }//if (dtm_event_get(&result)) //"true" after running dtm_cmd()
    } //end of for(;;)
}
#endif

void BleApp_Main(void)
{
#if (BLE_DEMO==DEMO_DTM)
    DTM_Main_Loop();
#endif
} //end of BleApp_Main()


void spiGpioWriteReg(const unsigned char regAddr, const unsigned char regData)
{

    unsigned char SPICount;                               // Counter used to clock out the data
    unsigned char SPIData;                                // Define a data structure for the SPI data.

    SPI_CS = 1;                                           // Make sure we start with /CS high
    SPI_CK = 0;                                           // and CK low

    SPI_CS = 0;                                           // Set /CS low to start the SPI cycle 25nS
    // Although SPIData could be implemented as an "int", resulting in one
    // loop, the routines run faster when two loops are implemented with
    // SPIData implemented as two "char"s.

    delay_us(1);

    //Address 1th byte
    SPIData = regAddr & 0x7F;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        delay_us(1);                                    // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }                                                     // and loop back to send the next bit
    // Repeat for the Data byte
    //Address 2nd byte
    SPIData = (regAddr & 0x80) >> 7;
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
    {
        if (SPIData & 0x80)                                 // Check for a 1
            SPI_MOSI = 1;                                     // and set the MOSI line appropriately
        else
            SPI_MOSI = 0;

        delay_us(1);                                     // delay half clk cycle
        SPI_CK = 1;                                         // Toggle the clock line
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;                                      // Rotate to get the next bit
    }

    //Data
    SPIData = regData;                                    // Preload the data to be sent with Data
    for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Data
    {
        if (SPIData & 0x80)
            SPI_MOSI = 1;
        else
            SPI_MOSI = 0;

        delay_us(1);
        SPI_CK = 1;
        delay_us(1);
        SPI_CK = 0;
        SPIData <<= 1;
    }

    delay_us(1);

    SPI_CS = 1;
    SPI_MOSI = 0;
}

/******************************************************************************
 * @brief    SPI IO mapping. Must do this after Power ON
 * @param
 * @retval
 * @attention
******************************************************************************/
void SPI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOB Peripheral clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RF_IRQ_CLOCK, ENABLE);
    /*Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_IRQ_PIN | GPIO_Pin_12 | GPIO_Pin_13 |  \
                                    GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(RF_IRQ_PORT, &GPIO_InitStructure);
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 * @attention  SPI IO mapping. Must do this after Power ON
******************************************************************************/
void RF_SpiIoMapping(void)
{
    /*
     *------------------------------------------------
     *          NSS     SCK     MISO    MOSI   IRQ  AF
     *------------------------------------------------
     *   SPI2   PB12    PB13    PB14    PB15   PB8  AF0
     *------------------------------------------------
     */
    /*(1) Here set all CLK,CS,MOSI,NISO,INT as output */
    SPI_GPIO_Init();
    SPI_CS = 1;
    SPI_CK = 0;
    SPI_MOSI = 0;
    SPI_MISO = 0;
    DEFAULT_INT = 0;

    //(2) Write R248, R249 GPIO select
    //Write R248=8'b10,001,000, R249=8'b0,100,011,0
    //GPIO0[2:0]=0 - CS
    //GPIO1[2:0]=1 - CLK
    //GPIO2[2:0]=2 - MOSI
    //GPIO3[2:0]=3 - MISO
    //GPIO4[2:0]=4 - INT
    delay_us(25);
    spiGpioWriteReg(248, 0x88);
    delay_us(25);
    spiGpioWriteReg(249, 0x46);
    delay_us(25);

    /* (3) Output all pin as HIGH, last 10ms(>1ms). trigger HW take effect */
    SPI_GPIO_Init();
    SPI_CS = 1;
    SPI_CK = 1;
    SPI_MOSI = 1;
    SPI_MISO = 1;
    DEFAULT_INT = 1;
    SysTick_DelayMs(10);  //Delay 10ms

    //(4) Init GPIO & SPI
    MCU_GpioPinInit();        //Set GPIO interrupt pin as input
    /* Init SPI_BLE : SPI master, clk=12M, mode 0, 8-bit, MSB first  */
    BSP_SPI_BLE_Init();

    SPI_1BYT_SetTx(249, (0x46 | 0x80));  //set RF MISO, INT as output

    //manual control
    SPI_1BYT_SetTx(53, 0xC0); //To gurantee DC/DC power on when set R40=0x90

    //enable LDO
//    SPI_1BYT_SetTx(40, 0xC0);
    SPI_1BYT_SetTx(40, 0xC0);// 截取正常波形中是 0x90 而不是 0xc0
    SysTick_DelayMs(25);        //Put delay after LDO_enable, or set register may have strange behavior!

    //enable chip
    SPI_1BYT_SetTx(53, 0x80); //Enable chip
    SysTick_DelayMs(10);   //Put delay after chip_enable, or set register may have strange behavior!
}

/******************************************************************************
 * @brief
 * @param
 * @retval
 * @attention
******************************************************************************/
static void RF_Open()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Wait RF PHY stable ,delay 25ms */
    SysTick_DelayMs(25);

    /* RF_RESET_CLOCK  enable */
    RCC_AHBPeriphClockCmd(RF_RESET_CLOCK, ENABLE);
    /* Configure RF_RESET_PIN in output pushpull mode */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_RESET_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(RF_RESET_PORT, &GPIO_InitStructure);

    /* Reset RF by using RF_RESET_PIN */
    GPIO_SetBits(RF_RESET_PORT, RF_RESET_PIN);
    SysTick_DelayMs(1);
    GPIO_ResetBits(RF_RESET_PORT, RF_RESET_PIN);
    SysTick_DelayMs(1);
    GPIO_SetBits(RF_RESET_PORT, RF_RESET_PIN);


    SysTick_DelayMs(50);

    /* SPI IO remapping */
    RF_SpiIoMapping();

    /* Initialize RF PHY */
    RF_Init();                   //EnableGpioInterrupt in the end of this function
}

/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/


/******************************************************************************
 * @brief  main loop for initialization and BLE kernel
 * @param
 * @retval
 * @attention
******************************************************************************/
int main(void)
{
#ifdef _HW_PRG_RESET_
#ifdef _HCI_HW_
    uint8_t RSP_HCI_RESET[] = {0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00};
#endif
#endif  //(#ifdef _HW_PRG_RESET_)

    /* Init System Clock 48M , HCLK2 = 24M */
    /* Init SysTick timer 1ms for SysTick_DelayMs */
    SysTick_Init(1000);

    /* Config UART1 with parameter(115200, N, 8, 1) for printf */
    UARTx_Configure(DEBUG_UART, 9600, UART_WordLength_8b, UART_StopBits_1,  \
                    UART_Parity_No);

    /* Enable the BLE RF PHY */
    RF_Open();

    printf("\r\nRafael_BLE_MM32 Demo run. Now is %s %s\r\n", __DATE__, __TIME__);
    printf("-------------------\r\n");
    printf("  BLE Start.....\r\n");
    printf("-------------------\r\n");

    printf("Chip_ID=0x%x\r\n", SPI_1BYT_SetRx(RFIP_REG_0));

//#ifdef _HW_PRG_RESET_
//    if (WDT_GET_RESET_FLAG() == 1)
//    {
//        WDT_CLEAR_RESET_FLAG();
//#ifdef _HCI_HW_
//        setUART_Tx(RSP_HCI_RESET, 7);
//#endif
//    }
//#endif  //(#ifdef _HW_PRG_RESET_)
   

    while (1)
    {
        /* SPI BLE Demo run */
        BleApp_Main();
    }
}


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

