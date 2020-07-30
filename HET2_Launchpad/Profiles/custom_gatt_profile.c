/**********************************************************************************************
 * Filename:       custom_gatt_profile.c
 *
 * Description:    This file contains the implementation of the service.
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


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <icall.h>
#include <Profiles/custom_gatt_profile.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// CAPSULE Service UUID
const uint8_t CUSTOMUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(CUSTOM_SERV_UUID)
};

// SYS_CFG UUID
const uint8_t SYS_CFGUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SYS_CFG_UUID)
};
// CHAR1 UUID
const uint8_t CHAR1UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(CHAR1_UUID)
};
// CHAR2 UUID
const uint8_t CHAR2UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(CHAR2_UUID)
};
// CHAR3 UUID
const uint8_t CHAR3UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(CHAR3_UUID)
};
// CHAR4 UUID
const uint8_t CHAR4UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(CHAR4_UUID)
};
// CHAR5 UUID
const uint8_t CHAR5UUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(CHAR5_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static BLE_CBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static const gattAttrType_t CUSTOMDecl = { ATT_UUID_SIZE, CUSTOMUUID };

// Characteristic "SYS_CFG" Properties (for declaration)
static uint8_t SYS_CFGProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "SYS_CFG" Value variable
uint8_t SYSdata[SYS_CFG_LEN] = {0};

// Characteristic "SYSconfig" Description
static uint8_t sysDesc[8] = "CP__CFG";


// Characteristic 1 Properties (for declaration)
static uint8_t Char1Props = GATT_PROP_NOTIFY;

// Characteristic 1 Value variable
uint8_t CHAR1data[CHAR1_LEN] = {0};

// Characteristic 1 CCCD
static gattCharCfg_t *CHAR1Config;

// Characteristic 1 Description
static uint8_t Char1Desc[8] = "CP_CHAR1";


// Characteristic 2 Properties (for declaration)
static uint8_t Char2Props = GATT_PROP_NOTIFY;

// Characteristic 2 Value variable
uint8_t CHAR2data[CHAR2_LEN] = {0};

// Characteristic 2 CCCD
static gattCharCfg_t *CHAR2Config;

// Characteristic 2 Description
static uint8_t Char2Desc[8] = "CP_CHAR2";


// Characteristic 3 Properties (for declaration)
static uint8_t Char3Props = GATT_PROP_NOTIFY;

// Characteristic 3 Value variable
uint8_t CHAR3data[CHAR3_LEN] = {0};

// Characteristic 3 CCCD
static gattCharCfg_t *CHAR3Config;

// Characteristic 3 Description
static uint8_t Char3Desc[8] = "CP_CHAR3";


// Characteristic 4 Properties (for declaration)
static uint8_t Char4Props = GATT_PROP_NOTIFY;

// Characteristic 4 Value variable
uint8_t CHAR4data[CHAR4_LEN] = {0};

// Characteristic 4 CCCD
static gattCharCfg_t *CHAR4Config;

// Characteristic 4 Description
static uint8_t Char4Desc[8] = "CP_CHAR4";


// Characteristic 5 Properties (for declaration)
static uint8_t Char5Props = GATT_PROP_NOTIFY;

// Characteristic 5 Value variable
uint8_t CHAR5data[CHAR5_LEN] = {0};

// Characteristic 5 CCCD
static gattCharCfg_t *CHAR5Config;

// Characteristic 5 Description
static uint8_t Char5Desc[8] = "CP_CHAR5";



/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t CUSTOMAttrTbl[] =
{
  // CUSTOM Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&CUSTOMDecl
  },
    // SYS_CFG Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &SYS_CFGProps
    },
      // SYS_CFG Characteristic Value
      {
        { ATT_UUID_SIZE, SYS_CFGUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        SYSdata
      },
      // SYS_CONFIG Characteristic Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sysDesc
      },
    //  Characteristic 1 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Char1Props
    },
      //  Characteristic 1 Value
      {
        { ATT_UUID_SIZE, CHAR1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        CHAR1data
      },
      // Characteristic 1 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&CHAR1Config
      },
      //  Characteristic 1 Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        Char1Desc
      },
    //  Characteristic 2 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Char2Props
    },
      // Characteristic 2 Value
      {
        { ATT_UUID_SIZE, CHAR2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        CHAR2data
      },
      // Characteristic 2 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&CHAR2Config
      },
      // Characteristic 2 Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        Char2Desc
      },
    //  Characteristic 3 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Char3Props
    },
      // Characteristic 3 Value
      {
        { ATT_UUID_SIZE, CHAR3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        CHAR3data
      },
      // Characteristic 3 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&CHAR3Config
      },
      // Characteristic 3 Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        Char3Desc
      },
    //  Characteristic 4 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Char4Props
    },
      // Characteristic 4 Value
      {
        { ATT_UUID_SIZE, CHAR4UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        CHAR4data
      },
      // Characteristic 4 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&CHAR4Config
      },
      // Characteristic 4 Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        Char4Desc
      },
    //  Characteristic 5 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &Char5Props
    },
      // Characteristic 5 Value
      {
        { ATT_UUID_SIZE, CHAR5UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        CHAR5data
      },
      // Characteristic 5 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&CHAR5Config
      },
      // Characteristic 5 Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        Char5Desc
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t CUSTOM_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t CUSTOM_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Custom Profile Service Callbacks
const gattServiceCBs_t CUSTOMCBs =
{
  CUSTOM_ReadAttrCB,  // Read callback function pointer
  CUSTOM_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * CUSTOM_AddService- Initializes the CUSTOM service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t CUSTOM_AddService()
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  CHAR1Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( CHAR1Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, CHAR1Config );
  // Allocate Client Characteristic Configuration table
  CHAR2Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( CHAR2Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, CHAR2Config );
  // Allocate Client Characteristic Configuration table
  CHAR3Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( CHAR3Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, CHAR3Config );
  // Allocate Client Characteristic Configuration table
  CHAR4Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( CHAR4Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, CHAR4Config );
  // Allocate Client Characteristic Configuration table
  CHAR5Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( CHAR5Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, CHAR5Config );
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( CUSTOMAttrTbl,
                                        GATT_NUM_ATTRS( CUSTOMAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &CUSTOMCBs );

  return ( status );
}

/*
 * CUSTOM_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t CUSTOM_RegisterAppCBs( BLE_CBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

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
bStatus_t CUSTOM_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SYS_CFG_ID:
      if ( len == SYS_CFG_LEN )
      {
          if(value != NULL) memcpy(SYSdata, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CHAR1_ID:
      if ( len == CHAR1_LEN )
      {
          if(value != NULL) memcpy(CHAR1data, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( CHAR1Config, (uint8_t *)&CHAR1data, FALSE,
                                    CUSTOMAttrTbl, GATT_NUM_ATTRS( CUSTOMAttrTbl ),
                                    INVALID_TASK_ID,  CUSTOM_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CHAR2_ID:
      if ( len == CHAR2_LEN )
      {
          if(value != NULL) memcpy(CHAR2data, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( CHAR2Config, (uint8_t *)&CHAR2data, FALSE,
                                    CUSTOMAttrTbl, GATT_NUM_ATTRS( CUSTOMAttrTbl ),
                                    INVALID_TASK_ID,  CUSTOM_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CHAR3_ID:
      if ( len == CHAR3_LEN )
      {
          if(value != NULL) memcpy(CHAR3data, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( CHAR3Config, (uint8_t *)&CHAR3data, FALSE,
                                    CUSTOMAttrTbl, GATT_NUM_ATTRS( CUSTOMAttrTbl ),
                                    INVALID_TASK_ID,  CUSTOM_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CHAR4_ID:
      if ( len == CHAR4_LEN )
      {
          if(value != NULL) memcpy(CHAR4data, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( CHAR4Config, (uint8_t *)&CHAR4data, FALSE,
                                    CUSTOMAttrTbl, GATT_NUM_ATTRS( CUSTOMAttrTbl ),
                                    INVALID_TASK_ID,  CUSTOM_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case CHAR5_ID:
      if ( len == CHAR5_LEN )
      {
          if(value != NULL) memcpy(CHAR5data, value, len);

        // Try to send notification.
        GATTServApp_ProcessCharCfg( CHAR5Config, (uint8_t *)&CHAR5data, FALSE,
                                    CUSTOMAttrTbl, GATT_NUM_ATTRS( CUSTOMAttrTbl ),
                                    INVALID_TASK_ID,  CUSTOM_ReadAttrCB);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*
 * CUSTOM_GetParameter - Get a CUSTOM parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t CUSTOM_GetParameter( uint8_t param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SYS_CFG_ID:
      memcpy(value, SYSdata, SYS_CFG_LEN);
      break;

    /*case TMP_RtR_ID:
      memcpy(value, TMPdata, TMP_RtR_LEN);
      break;

    case PPG_ALL_ID:
      memcpy(value, PPGdata, PPG_ALL_LEN);
      break;

    case ACC_ALL_ID:
      memcpy(value, ACCdata, ACC_ALL_LEN);
      break;

    case MAG_ALL_ID:
      memcpy(value, MAGdata, MAG_ALL_LEN);
      break;

    case MAX_ALL_ID:
      memcpy(value, MAXdata, MAX_ALL_LEN);
      break;*/

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}


/*********************************************************************
 * @fn          CUSTOM_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t CUSTOM_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;

  // See if request is regarding the SYS_CFG Characteristic Value
if ( ! memcmp(pAttr->type.uuid, SYS_CFGUUID, pAttr->type.len) )
  {
    if ( offset > SYS_CFG_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, SYS_CFG_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the TMP_RtR Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, CHAR1UUID, pAttr->type.len) )
  {
    if ( offset > CHAR1_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CHAR1_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the PPG_ALL Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, CHAR2UUID, pAttr->type.len) )
  {
    if ( offset > CHAR2_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CHAR2_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the ACC_ALL Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, CHAR3UUID, pAttr->type.len) )
  {
    if ( offset > CHAR3_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CHAR3_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the MAG_ALL Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, CHAR4UUID, pAttr->type.len) )
  {
    if ( offset > CHAR4_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CHAR4_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  // See if request is regarding the MAX_ALL Characteristic Value
else if ( ! memcmp(pAttr->type.uuid, CHAR5UUID, pAttr->type.len) )
  {
    if ( offset > CHAR5_LEN )  // Prevent malicious ATT ReadBlob offsets.
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      *pLen = MIN(maxLen, CHAR5_LEN - offset);  // Transmit as much as possible
      memcpy(pValue, pAttr->pValue + offset, *pLen);
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has READ permissions.
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}


/*********************************************************************
 * @fn      CUSTOM_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t CUSTOM_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if ( ! memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len) )
  {
    // Allow only notifications.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY);
  }
  // See if request is regarding the SYS_CFG Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, SYS_CFGUUID, pAttr->type.len) )
  {
    if ( offset + len > SYS_CFG_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == SYS_CFG_LEN)
        paramID = SYS_CFG_ID;
    }
  }
  // See if request is regarding the TMP_RtR Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, CHAR1UUID, pAttr->type.len) )
  {
    if ( offset + len > CHAR1_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == CHAR1_LEN)
        paramID = CHAR1_ID;
    }
  }
  // See if request is regarding the PPG_ALL Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, CHAR2UUID, pAttr->type.len) )
  {
    if ( offset + len > CHAR2_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == CHAR2_LEN)
        paramID = CHAR2_ID;
    }
  }
  // See if request is regarding the ACC_ALL Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, CHAR3UUID, pAttr->type.len) )
  {
    if ( offset + len > CHAR3_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == CHAR3_LEN)
        paramID = CHAR3_ID;
    }
  }
  // See if request is regarding the MAG_ALL Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, CHAR4UUID, pAttr->type.len) )
  {
    if ( offset + len > CHAR4_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == CHAR4_LEN)
        paramID = CHAR4_ID;
    }
  }
  // See if request is regarding the MAX_ALL Characteristic Value
  else if ( ! memcmp(pAttr->type.uuid, CHAR5UUID, pAttr->type.len) )
  {
    if ( offset + len > CHAR5_LEN )
    {
      status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
      // Copy pValue into the variable we point to from the attribute table.
      memcpy(pAttr->pValue + offset, pValue, len);

      // Only notify application if entire expected value is written
      if ( offset + len == CHAR5_LEN)
        paramID = CHAR5_ID;
    }
  }
  else
  {
    // If we get here, that means you've forgotten to add an if clause for a
    // characteristic value attribute in the attribute table that has WRITE permissions.
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  // Let the application know something changed (if it did) by using the
  // callback it registered earlier (if it did).
  if (paramID != 0xFF)
    if ( pAppCBs && pAppCBs->pfnChangeCb )
      pAppCBs->pfnChangeCb( paramID );
  //(connHandle, paramID, len, pValue); // Call app function from stack task context.

  return status;
}

