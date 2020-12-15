/******************************************************************************
 * @file    config.h
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
#ifndef __CONFIG_H__
#define __CONFIG_H__


#ifdef __cplusplus
extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


/* Includes -----------------------------------------------------------------*/
#include "HAL_conf.h"
#include "HAL_device.h"

/* Exported typedef -------------------------------------------------------*/
typedef struct _16_Bits_Struct
{
    u16 bit0 : 1;
    u16 bit1 : 1;
    u16 bit2 : 1;
    u16 bit3 : 1;
    u16 bit4 : 1;
    u16 bit5 : 1;
    u16 bit6 : 1;
    u16 bit7 : 1;
    u16 bit8 : 1;
    u16 bit9 : 1;
    u16 bit10 : 1;
    u16 bit11 : 1;
    u16 bit12 : 1;
    u16 bit13 : 1;
    u16 bit14 : 1;
    u16 bit15 : 1;
} Bits_16_TypeDef;

/* Exported constants -------------------------------------------------------*/
#define TIMER0      TIM16   /* Used for printf */
#define TIMER2      TIM17   /* Used for printf */

#define DEBUG_UART      UART1   /* Used for printf */

#define _MCU_ARM_CORTEX_   //no actual use, just for InterruptDisable
#define Tiny_Delay(x)    delay_us(x)//SysTick_DelayMs(x)

#ifndef BIT
#define BIT(x)    (1 << (x))
#endif

#define RF_IRQ_PORT     GPIOB
#define RF_IRQ_PIN      GPIO_Pin_8
#define RF_IRQ_CLOCK    RCC_AHBPeriph_GPIOB

#define RF_RESET_PORT   GPIOB
#define RF_RESET_PIN    GPIO_Pin_9
#define RF_RESET_CLOCK  RCC_AHBPeriph_GPIOB

#define SPI2_PORT_OUT    ((Bits_16_TypeDef *)(&(GPIOB->ODR)))
#define SPI_CS            (SPI2_PORT_OUT->bit12)
#define SPI_CK            (SPI2_PORT_OUT->bit13)
#define SPI_MOSI          (SPI2_PORT_OUT->bit15)
#define SPI_MISO          (SPI2_PORT_OUT->bit14)
#define RF_IRQ_PORT_OUT  ((Bits_16_TypeDef *)(&(RF_IRQ_PORT->ODR)))
#define DEFAULT_INT       (RF_IRQ_PORT_OUT->bit8)

/* Platform Select */
#define _BOARD_NUVOTON_M031TD2AE_QFN33_ 3   //M031 daughter board
#define _BOARD_SELECTION_              _BOARD_NUVOTON_M031TD2AE_QFN33_

/* Ble App Setting */
#define  DEMO_HRS                   0
#define  DEMO_TRSPX_LED_SLAVE       1
#define  DEMO_TRSPX_UART_SLAVE      2      //use UART0
#define  DEMO_DTM                   9      //use UART0
//Select a demo application
#define  BLE_DEMO                   DEMO_DTM

/* ************************************
 * Select a demo BLE Server Profile
 **************************************/


/* ------------------------------------
 * Select BLE Client Profile
  -------------------------------------*/
#ifdef _HOST_CLIENT_
//#define _PROFILE_CLI_HRP_
//#define _PROFILE_CLI_HTP_
//#define _PROFILE_CLI_CSCP_
//#define _PROFILE_CLI_BLP_
//#define _PROFILE_CLI_FMP_
//#define _PROFILE_CLI_SCPP_
//#define _PROFILE_CLI_PXP_
//#define _PROFILE_CLI_RSCP_
//#define _PROFILE_CLI_BAS_
//#define _PROFILE_CLI_IAS_
//#define _PROFILE_CLI_LLS_
//#define _PROFILE_CLI_TPS_
//#define _PROFILE_CLI_LNS_
//#define _PROFILE_CLI_GLS_
//#define _PROFILE_CLI_CPS_
//#define _PROFILE_CLI_CSTM_UDF01S_
//#define _PROFILE_CLI_CSTM_UDF02S_
//#define _PROFILE_CLI_CSTM_UDF03S_
#endif



enum
{
    BLE_LL_SLAVE_ONLY = 0U,
    BLE_LL_MASTER_ONLY = 1U,
    BLE_LL_SLAVE_MASTER = 2U
};


//#define _HCI_ON_       //remove in SDK
#define _HCI_NEW_        //define in SDK
////#define _HCI_HW_       //remove in SDK. However, it is a must for test
//#define _HCI_HW_MSG_   //remove in SDK. (msg2uart)
//#define _LE_SCAN_ON_   //no use

#ifndef _HCI_NEW_
#undef _HCI_HW_
#endif
#ifndef _HCI_HW_
#undef _HCI_HW_MSG_      //remove in SDK
#endif


//#define _DEBUG_DISPLAY_      //remove in SDK
//#define _DEBUG_ICE_          //remove in SDK, no use
#define _DEBUG_PINS_           //remove in SDK
//#define _DEBUG_MSG_USER_       //define in SDK
//#define _DEBUG_MSG_USER_SUB1 //remove in SDK

#ifdef _HCI_HW_
#undef _DEBUG_MSG_USER_
#endif  //(#ifdef _HCI_HW_)

#ifndef _DEBUG_MSG_USER_
#undef _DEBUG_MSG_USER_SUB1    //detailed debug message
#endif  //(#ifndef _DEBUG_MSG_USER_)


//#define _USED_EXCEPT_SDK_      //remove in SDK


// Define BLE Version
//#define BLE_VERSION             0x06    //Bluetooth Core Specification 4.0
//#define BLE_VERSION             0x07    //Bluetooth Core Specification 4.1
//#define BLE_VERSION             0x08    //Bluetooth Core Specification 4.2
#define BLE_VERSION             0x09    //Bluetooth Core Specification 5.0
//#define BLE_VERSION             0x0A    //Bluetooth Core Specification 5.1
#define BLE_COMPANY_ID_L        0x64    //HCI__004
#define BLE_COMPANY_ID_H        0x08

// Settings
#define MAX_SIZE_ATT_VALUE_OVER_MTU_WR      45    //Maximum Attribute Data Size

//---- For future application use ----//
//#define _PROFILE_HOGP_
//#define _PROFILE_HOGP_MOUSE_
//#define _PROFILE_HOGP_KEYBOARD_
//#define _PROFILE_HOGP_COMSUMER_
//#define _PROFILE_HOGP_RUEMANN_

#ifdef _PROFILE_HOGP_
#ifndef _PROFILE_HOGP_KEYBOARD_
#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_
#define _PROFILE_HOGP_KEYBOARD_
#endif  //(#ifdef _PROFILE_HOGP_KEYBOARD_PHOTO_)
#endif  //(#ifndef _PROFILE_HOGP_KEYBOARD_)

#ifdef _PROFILE_HOGP_MOUSE_
#if defined _PROFILE_HOGP_KEYBOARD_
#define _PROFILE_HOGP_MULTI_
#elif defined _PROFILE_HOGP_COMSUMER_
#define _PROFILE_HOGP_MULTI_
#endif  //(#if defined _PROFILE_HOGP_KEYBOARD_)
#endif  //(#ifdef _PROFILE_HOGP_MOUSE_)

#ifndef _PROFILE_HOGP_MULTI_
#ifdef _PROFILE_HOGP_KEYBOARD_
#ifdef _PROFILE_HOGP_COMSUMER_
#define _PROFILE_HOGP_MULTI_
#endif  //(#ifdef _PROFILE_HOGP_COMSUMER_)
#endif  //(#ifdef _PROFILE_HOGP_KEYBOARD_)
#else   //(#ifndef _PROFILE_HOGP_MULTI_)
#endif  //(#ifndef _PROFILE_HOGP_MULTI_)

#ifndef _PROFILE_HOGP_MULTI_
#define __PROFILE_HOGP_KEYBOARD_RPIDOFST_   0
#define __PROFILE_HOGP_CS_RPIDOFST_         0
#define __PROFILE_HOGP_MS_RPIDOFST_         0
#else   //(#ifndef _PROFILE_HOGP_MULTI_)
#define __PROFILE_HOGP_CS_RPIDOFST_         3
#define __PROFILE_HOGP_MS_RPIDOFST_         2
#ifdef _PROFILE_HOGP_EMC_1501_
#define __PROFILE_HOGP_KEYBOARD_RPIDOFST_   2
#else   //(#ifdef _PROFILE_HOGP_EMC_1501_)
#define __PROFILE_HOGP_KEYBOARD_RPIDOFST_   1
#endif  //(#ifdef _PROFILE_HOGP_EMC_1501_)
#endif  //(#ifndef _PROFILE_HOGP_MULTI_)
#endif  //(#ifdef _PROFILE_HOGP_)


/* InterruptEnable */
#ifdef _MCU_ARM_CORTEX_
#define InterruptDisable      __disable_irq
#define InterruptEnable       __enable_irq
#endif

#define E_SUCCESS (0)

#ifndef NULL
#define NULL      (0)                 ///< NULL pointer
#endif

#define TRUE      (1UL)               ///< Boolean true, define to use in API parameters or return value
#define FALSE     (0UL)               ///< Boolean false, define to use in API parameters or return value


#ifdef __cplusplus
}
#endif


#endif


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

