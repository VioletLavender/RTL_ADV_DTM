/******************************************************************************
 * @file    bsp_irq_gpio.c
 * @author  King
 * @version V1.00
 * @date    20-May-2020
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
#define __bsp_misc_C__


/* Includes -----------------------------------------------------------------*/
#include "bsp_misc.h"
#include "rf_phy.h"
#include "ble_dtm.h"

/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/
/* Private macro ------------------------------------------------------------*/
/* Private variables --------------------------------------------------------*/
/* Private function prototypes ----------------------------------------------*/
/* Private functions --------------------------------------------------------*/


/* Exported variables -------------------------------------------------------*/
/* Exported function prototypes ---------------------------------------------*/

/******************************************************************************
 * @brief    Assign GPIO INT pin
 * @param
 * @retval
 * @attention
******************************************************************************/
void MCU_GpioPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* RF_IRQ_CLOCK enable */
    RCC_AHBPeriphClockCmd(RF_IRQ_CLOCK, ENABLE);
    /*Be careful not to set 5 pins as HIGH, it will trigger HW mapping SPI pin*/
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = RF_IRQ_PIN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_FLOATING ;//GPIO_Mode_IPD ; //GPIO_Mode_FLOATING
    GPIO_Init(RF_IRQ_PORT, &GPIO_InitStructure);
    
    
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//    
//    GPIO_StructInit(&GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 ;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP ;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void MCU_GpioIntEnable(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;

    // Configure PB8 as Input mode and enable interrupt by rising edge trigger
    MCU_GpioPinInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //IRQ - pb8
    SYSCFG_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void MCU_GpioIntDisable(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    EXTI_InitTypeDef  EXTI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //IRQ - pb8
    SYSCFG_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/* GPIO Interrupt Handler */

void EXTI4_15_IRQHandler(void)
{
    volatile uint32_t temp;
    extern void LL_GPIO_Isr(void);
    extern void delay_us(u16 us);
#ifdef BLE_DEMO
#if (BLE_DEMO==DEMO_DTM)
    extern void DTM_Isr(void);
#endif //#if (BLE_DEMO==DEMO_DTM)
#endif //#ifdef BLE_DEMO
    //Clear MCU GPIO Int status

    if (EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line8); // Key modification!!! Put this here
        while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1)
        {
            #ifdef BLE_DEMO
            #if (BLE_DEMO==DEMO_DTM)
            DTM_Isr();
            #else
            LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
            #endif
            #else
            LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
            #endif
            Tiny_Delay(1);
        }
    }
//    LL_GPIO_Isr();      // Put this on the top following the clearing MCU GPIO Int status process.
}

//------------------- CLK function -----------------------//
void _CLK_Idle(void)
{
    extern void SystemInit(void);
    //important notes
    //need to set those GPIO Pins as AIN exclude the wake up pin;
    //need to disalbe those interrupts (Systick, Timer), exclude the wake interrupt
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_PWREN, ENABLE);
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    //After enter STOP mode, the SYSCLK changed to HSI/6
    //Need re-initialize the SYSCLK , set HSI/4+PLL to 48Mhz as SYSCLK
    SystemInit();
    //re-initialize those GPIO Pins as previous setting before the wake up pin;
    //re-enalbe those interrupts (Systick, Timer) as previous setting before
    //the wake up pin
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for diasble internal analog POR circuit                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Disable_AnalogPORCircuit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All ^ GPIO_Pin_13 ^ GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All ^ GPIO_Pin_8 ^ GPIO_Pin_9       \
                                  ^ GPIO_Pin_12 ^ GPIO_Pin_13 ^ GPIO_Pin_14 ^ GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}


#ifdef _HW_PRG_RESET_
void MCU_WDTmr_En(void)
{
    extern void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

    InterruptDisable();
   
//    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, 0);
//    CLK_EnableModuleClock(WDT_MODULE);
//    SystemCoreClockUpdate();

//    WDT_Open(WDT_TIMEOUT_2POW4, WDT_RESET_DELAY_18CLK, TRUE, FALSE);
//    NVIC_EnableIRQ(WDT_IRQn);
//    WDT_EnableInt();

    InterruptEnable();
}
#endif  //(#ifdef _HW_PRG_RESET_)

void TIMER_DisableInt(TIM_TypeDef* TIMx)
{
    /* DISABLE TIMx Interrupt */
    TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
}

void TIMER_EnableInt(TIM_TypeDef* TIMx)
{
    /* Enable TIMx Interrupt */
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
}

void TIMER_Stop(TIM_TypeDef* TIMx)
{
    /* DISABLE TIMx Count */   
    TIM_Cmd(TIMx, DISABLE);
    TIM_SetCounter(TIMx,0) ;
}

void TIMER_Start(TIM_TypeDef* TIMx)
{
    /* ENABLE TIMx Count */   
    TIM_Cmd(TIMx, ENABLE);   
}
void TIMER_Open(TIM_TypeDef* TIMx, uint32_t u32Mode, uint32_t u32Freq)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   
    /* Enable TIM16 TIM17 Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17, ENABLE);
    
    TIM_DeInit(TIMx);   
    /* Config TIM16 Every 1ms Generate Interrupt */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler             = (RCC_GetSysClockFreq() / 1000000 - 1);
    TIM_TimeBaseStructure.TIM_CounterMode           = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period                = (u32Freq - 1); //2500us per interrupt
    TIM_TimeBaseStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter     = 0;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    /* Clear TIMx  Update Flag */
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
}

extern uint32_t SystemCoreClock ;
#if defined(ewarm) || defined(DOXYGEN)
void SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}
#endif

#if defined(codered) || defined(gcc) || defined(sourcerygxx)
void __attribute__((naked))
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne     SysCtlDelay\n"
          "    bx      lr");
}
#endif

#if defined(rvmdk) || defined(__ARMCC_VERSION)
__asm void
SysCtlDelay(uint32_t ui32Count)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#endif

void delay_us(u16 us)
{
    u32 xus = us;
    SysCtlDelay(xus * (SystemCoreClock / 4000000));
}

void delay_ms(u16 ms)
{
    u32 xms = ms;
    SysCtlDelay(xms * (1000 * (SystemCoreClock / 4000000)));
}


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

