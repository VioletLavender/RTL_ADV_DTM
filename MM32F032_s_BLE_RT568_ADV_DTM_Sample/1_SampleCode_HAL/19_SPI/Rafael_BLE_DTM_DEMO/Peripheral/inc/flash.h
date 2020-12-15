/******************************************************************************
 * @file    flash.h
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
#ifndef __FLASH_H__
#define __FLASH_H__


#ifdef __cplusplus
extern "C" {
#endif


#undef  EXTERN


#ifdef  __FLASH_C__
#define EXTERN
#else
#define EXTERN extern
#endif


/* Includes -----------------------------------------------------------------*/
#include "config.h"


/* Exported constants -------------------------------------------------------*/
//Flash address of bonding information and data

#define BONDING_INFORMATION_ADDRESS                    0x0800F800       /**< Address of bonding information block.*/
#define BONDING_DATA_ADDRESS                           0x0800FC00       /**< Address of bonding data block.*/

//Do not modify below definition
#define SIZE_OF_FLASH_PAGE                             512              /**< Size of Flash Page Type.*/
#define SIZE_OF_BONDING_INFORMATION                    1024             /**< Total size of BLE bonding information array.*/
#define SIZE_OF_BONDING_DATA                           1024             /**< Total size of BLE bonding Data array.*/
#define SIZE_OF_INFO_BLK                               64               /**< Size of BLE bonding information block.*/
#define SIZE_OF_DAT_GRP_BLK                            128              /**< Size of BLE bonding Data block.*/
#define NUM_OF_FLASH_PAGE_FOR_BONDING_INFO_BLK         (SIZE_OF_BONDING_INFORMATION/SIZE_OF_FLASH_PAGE) /**< Total number of flash size of bound information block.*/
#define NUM_OF_FLASH_PAGE_FOR_BONDING_DAT_GRP_BLK      (SIZE_OF_BONDING_DATA/SIZE_OF_FLASH_PAGE)        /**< Total number of flash size of bound data block.*/
#define NUM_OF_INFO_BLK_ONE_PAGE                       (SIZE_OF_FLASH_PAGE/SIZE_OF_INFO_BLK)            /**< Amount of information bound using flash page.*/
#define NUM_OF_DAT_GRP_BLK_ONE_PAGE                    (SIZE_OF_FLASH_PAGE/SIZE_OF_DAT_GRP_BLK)         /**< Amount of data bound using flash page.*/
#define TAB_FLASH_BONDING_INFO_BLK                     SIZE_OF_INFO_BLK                                 /**< Tab Size of BLE bonding information block.*/
#define TAB_FLASH_BONDING_DATA_GRP_BLK                 SIZE_OF_DAT_GRP_BLK                              /**< Tab size of BLE bonding Data block.*/

/* Exported types -----------------------------------------------------------*/
/* Exported macro -----------------------------------------------------------*/


/* Exported functions -------------------------------------------------------*/
EXTERN void FLASH_ProgramEntry(void);


#ifdef __cplusplus
}
#endif


#endif


/******************* (C) COPYRIGHT 2020 ************************END OF FILE***/

