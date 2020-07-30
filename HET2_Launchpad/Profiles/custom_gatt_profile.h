/**********************************************************************************************
 * Filename:       custom_gatt_profile.h
 *
 * Description:    This file contains the CUSTOM service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


#ifndef _CUSTOMGATTPROFILE_H_
#define _CUSTOMGATTPROFILE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <bcomdef.h>

/*********************************************************************
* CONSTANTS
*/

// Service UUID
#define CUSTOM_SERV_UUID 0x2642
#define CUSTOM_CHAR_LEN    100

//  Characteristic defines
#define SYS_CFG_ID   0
#define SYS_CFG_UUID 0xABCD
#define SYS_CFG_LEN  10

//  Characteristic defines
#define CHAR1_ID   1
#define CHAR1_UUID 0x62D2
#define CHAR1_LEN  8

//  Characteristic defines
#define CHAR2_ID   2
#define CHAR2_UUID 0x44DC
#define CHAR2_LEN  82

//  Characteristic defines
#define CHAR3_ID   3
#define CHAR3_UUID 0x3C36
#define CHAR3_LEN  CUSTOM_CHAR_LEN

//  Characteristic defines
#define CHAR4_ID   4
#define CHAR4_UUID 0x3A36
#define CHAR4_LEN  CUSTOM_CHAR_LEN

//  Characteristic defines
#define CHAR5_ID   5
#define CHAR5_UUID 0x30D8
#define CHAR5_LEN  CUSTOM_CHAR_LEN

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*BLE_Change_t)( uint8 paramID );
//(uint16_t connHandle, uint8_t paramID, uint16_t len, uint8_t *pValue);

typedef struct
{
  BLE_Change_t        pfnChangeCb;  // Called when characteristic value changes
  BLE_Change_t        pfnCfgChangeCb;
} BLE_CBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * CUSTOM_AddService- Initializes the CUSTOM service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t CUSTOM_AddService();

/*
 * CUSTOM_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t CUSTOM_RegisterAppCBs( BLE_CBs_t *appCallbacks );

/*
 * CUSTOM_SetParameter - Set a CUSTOM parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t CUSTOM_SetParameter(uint8_t param, uint16_t len, void *value);

/*
 * CUSTOM_GetParameter - Get a CUSTOM parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t CUSTOM_GetParameter(uint8_t param, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _CUSTOMGATTPROFILE_H_ */
