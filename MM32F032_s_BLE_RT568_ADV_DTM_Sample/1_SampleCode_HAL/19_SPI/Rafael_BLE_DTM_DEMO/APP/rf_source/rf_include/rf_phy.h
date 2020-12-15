/**************************************************************************//**
* @file       rf_phy.h
* @brief      This file provides the declaration that RF IC control needed.
*
* @defgroup rf_phy RF PHY
* @{
* @details  Provides the declaration that RF IC control needed. (rf_phy.h).
* @}
*****************************************************************************/

#ifndef __RF_PHY_H__
#define __RF_PHY_H__

#include "config.h"
#include "ble_cmd.h"


/**************************************************************************
 * Definitions
 **************************************************************************/
/** @defgroup rf_phy_def RF PHY Definition.
 * @{
 * @details Here shows the definitions of RF PHY.
 * @ingroup rf_phy
 * @}
 */

/** @defgroup rf_phy_def_chipId Chip ID Definition
 * @{
 * @ingroup rf_phy_def
 */
#define MP_A1      102u                  /**< chip_id = 102 (0x66). */
#define MP_A2      103u                  /**< chip_id = 103 (0x67). */
/** @} */


/** @defgroup rf_phy_def_TRxWritePort TX/RX Buffer Write Port Definition
 * @{
 * @ingroup rf_phy_def
 */
#define RFIP_REG_254            254U          /**< Register 254. */
#define RFIP_REG_255            255U          /**< Register 255. */


#define TX_BUFFER_WRITE_PORT    RFIP_REG_254  /**< TX Buffer Write Port. */
#define RX_BUFFER_READ_PORT     RFIP_REG_255  /**< RX Buffer Write Port. */
/** @} */



/**
 * @ingroup ble_cmd_definition
 * @defgroup ble_cmd_bleTxPower BLE TX Power Level Definition
 * @{
 * @details BLE TX power level definition.
 * @name BleTxPower
 * @brief  The definition of BLE TX Power Level.
 * @{
 */
#define TX_POWER_0_DBM          0     /**< 0 dBm.   */
#define TX_POWER_4_DBM          4     /**< +4 dBm.  */
#define TX_POWER_8_DBM          8     /**< +8 dBm.  */
#define TX_POWER_10_DBM         10    /**< +10 dBm. */


//========== SW Feature options =========//
// will be moved to _rafael_phy.h in SDK
//==========================================

/** use RF timer as LL timerU
 * @ingroup rf_phy_def
 */
#define _TMR_USE_INTERNAL_      //Use RF internal RTC as timer
#ifndef _HCI_HW

/** Enable Auto-Sleep Function by LL.
 * @ingroup rf_phy_def
 */

#endif


#define _SW_PATCH_MP_
#ifndef _TMR_USE_INTERNAL_
#undef _SW_PATCH_MP_
#endif

#ifdef _TMR_USE_INTERNAL_
//#define DUR_LL_WAKEUP_MIN       32+6    //4us, 1.6
//#define DUR_LL_WAKEUP_MIN       800     //OK
//#define DUR_LL_WAKEUP_MIN       240     //OK
//#define DUR_LL_WAKEUP_MIN       1600    //OK
//#define DUR_LL_WAKEUP_MIN       80    //6us
//#define DUR_LL_WAKEUP_MIN       60    //16us, 4
//#define DUR_LL_WAKEUP_MIN       70    //10us
//#define DUR_LL_WAKEUP_MIN       75    //11, 3us
//#define DUR_LL_WAKEUP_MIN       78    //6us
//#define DUR_LL_WAKEUP_MIN       76    //10us
#define DUR_LL_WAKEUP_MIN       1600    //1600*125u=200ms, sleep mode 33us deviation

#else   //(#ifdef _TMR_USE_INTERNAL_)

#endif


/** Enable WDT Reset.
 * @ingroup rf_phy_def
 */
#define _HW_PRG_RESET_


/** LL Power-Saving Duration. (1600*125u=200ms)
 * @ingroup rf_phy_def
 */
#define DUR_LL_WAKEUP_MIN                 1600    //1600*125u=200ms, LL power-saving duration


/** Free Time Period to the Next Event over 10ms will make the IC Go Sleep. (example: 80*125us=10ms)
 * @ingroup rf_phy_def
 */
#define DUR_PWR_DOWN_PERIOD_FCTR_125MULT  80   //125*n (us), example: 80*125us=10ms, free time period to the next event over 10ms will make the IC go sleep.
//================================================



/**************************************************************************
 * Functions
 **************************************************************************/

/** @defgroup rf_phy_function RF PHY Function
 * @{
 * @details Here shows the functions of RF PHY.
 * @ingroup rf_phy
 * @}
 */

/** Initializes the BLE stack.
 *  Must initial BLE stack after @ref RF_SpiIoMapping.
 *
 * @ingroup rf_phy_function
 */
void RF_Init(void);



/** Get BLE Device Chip ID
 *
 *  This function is used to get BLE Device Chip ID.
 *
 * @ingroup rf_phy_function
 *
 * @return  @ref rf_phy_def_chipId "Chip ID"
 */
uint8_t ChipId_Get(void);



/** Get TX FIFI Address
 *
 *  This function is used to get TX FIFO RAM start address.
 *
 * @ingroup rf_phy_function
 *
 * @return  TX FIFO address.
 */
uint32_t BleTxFIFOAddr_Get(void);



extern uint8_t ChipVer;


#endif
