/******************************************************************************

 @file  custom_peripheral.c

 @brief This file contains the Custom Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************

 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdint.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/display/Display.h>
//#include <ti/drivers/GPIO.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
//#include <ti/drivers/SD.h>
//#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/drivers/sd/SDSPI.h>
//#include <ti/drivers/apps/LED.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
//#include <simple_gatt_profile.h>
#include <Profiles/custom_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>
//#include <board_key.h>

//#include <menu/two_btn_menu.h>

//#include "simple_peripheral_menu.h"
#include "custom_peripheral.h"
#include "ti_ble_config.h"
//#include "myGPIOs.h"

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26X2DMA.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <Drivers/AD5941/ad5940.h>
#include <Drivers/AD5941/HET2_CAPH.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PIN.h>
#include <Drivers/HET2_EChem.h>
#include <util.h>


#ifdef PTM_MODE
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#endif // PTM_MODE


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// How often to perform periodic event (in ms)
#define CP_PERIODIC_EVT_PERIOD               10000

// Task configuration
#define CP_TASK_PRIORITY                     1

#ifndef CP_TASK_STACK_SIZE
#define CP_TASK_STACK_SIZE                   1024
#endif

// Application events
#define CP_STATE_CHANGE_EVT                  0
#define CP_CHAR_CHANGE_EVT                   1
#define CP_KEY_CHANGE_EVT                    2
#define CP_ADV_EVT                           3
#define CP_PAIR_STATE_EVT                    4
#define CP_PASSCODE_EVT                      5
#define CP_PERIODIC_EVT                      6
#define CP_READ_RPA_EVT                      7
#define CP_SEND_PARAM_UPDATE_EVT             8
#define CP_CONN_EVT                          9
#define CP_DATA_EVT                          10

// Internal Events for RTOS application
#define CP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define CP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define CP_ALL_EVENTS                        (CP_ICALL_EVT             | \
                                              CP_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define CP_ADDR_STR_SIZE     15

//#define DATA_LEN                            (CUSTOM_CHAR_LEN-4)
//#define PN(X)                               PIN_ID(gpioPinConfigs[X])

// Row numbers for two-button menu
//#define TBM_ROW_APP             0
#define CP_ROW_SEPARATOR_1   0
#define CP_ROW_STATUS_1      1
#define CP_ROW_STATUS_2      2
#define CP_ROW_CONNECTION    3
#define CP_ROW_ADVSTATE      4
#define CP_ROW_RSSI          5
#define CP_ROW_IDA           6
#define CP_ROW_RPA           7
#define CP_ROW_DEBUG         8
#define CP_ROW_AC            9

// For storing the active connections
#define CP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define CP_MAX_RSSI_STORE_DEPTH    5
#define CP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30
#define RSSI_1M_THRSHLD           -40
#define RSSI_S2_THRSHLD           -50
#define RSSI_S8_THRSHLD           -60
#define CP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define CUSTOMPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

//AD5941 Attributes
#define APPBUFF_SIZE               1000
#define n                          2
#define SAMPLES                    10
/*********************************************************************
 * TYPEDEFS
 */

// Auto connect availble groups
enum
{
  AUTOCONNECT_DISABLE = 0,              // Disable
  AUTOCONNECT_GROUP_A = 1,              // Group A
  AUTOCONNECT_GROUP_B = 2               // Group B
};


// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} cpEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} cpPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} cpPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} cpGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} cpClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} cpConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t              connHandle;                        // Connection Handle
  cpClockEventData_t*   pParamUpdateEventData;
  Clock_Struct*         pUpdateClock;                      // pointer to clock struct
  int8_t                rssiArr[CP_MAX_RSSI_STORE_DEPTH];
  uint8_t               rssiCntr;
  int8_t                rssiAvg;
  bool                  phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t               currPhy;
  uint8_t               rqPhy;
  uint8_t               phyRqFailCnt;                      // PHY change request count
  bool                  isAutoPHYEnable;                   // Flag to indicate auto phy change
} cpConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// Task configuration
Task_Struct cpTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(cpTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t cpTaskStack[CP_TASK_STACK_SIZE];

//SPI Globals
SPI_Handle      spi;
SPI_Params      spiParams;
SPI_Transaction spiTransaction;

bool            transferOK;

#define Board_AD_GPIO_0   IOID_25


//AD5941 Globals
uint32_t IntCount = 3;
AD5940Err ADerror;
uint32_t AppBuff[n][APPBUFF_SIZE];
AppCHRONOAMPCfg_Type *pAMPCfg;
uint32_t g_period = 0.5;

//HET2 Device Globals
EChem_Device_Type *HET2;
uint16_t g_TempBuffer[10] = {0,3,6,9,12,15,18,21,24,27};
uint8_t g_BattRead;

/*********************************************************************
 * LOCAL VARIABLES
 */

extern uint8_t  SYSdata[];
extern uint8_t  CHAR1data[];
extern uint8_t  CHAR2data[];
extern uint8_t  CHAR3data[];
extern uint8_t  CHAR4data[];
extern uint8_t  CHAR5data[];

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
cpClockEventData_t argPeriodic =
{ .event = CP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
cpClockEventData_t argRpaRead =
{ .event = CP_READ_RPA_EVT };

// Per-handle connection info
static cpConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Advertising handles
static uint8_t advHandleLegacy;
static uint8_t advHandleLongRange;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8_t rpa[B_ADDR_LEN] = {0};

static PIN_State  hStateADrdy;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void CustomPeripheral_init( void );
static void CustomPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t CustomPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t CustomPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void CustomPeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void CustomPeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void CustomPeripheral_processAdvEvent(cpGapAdvEventData_t *pEventData);
static void CustomPeripheral_processAppMsg(cpEvt_t *pMsg);
static void CustomPeripheral_processCharValueChangeEvt(uint8_t paramId);
static void CustomPeripheral_performPeriodicTask(void);
static void CustomPeripheral_updateRPA(void);
static void CustomPeripheral_clockHandler(UArg arg);
static void CustomPeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void CustomPeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void CustomPeripheral_processPairState(cpPairStateData_t *pPairState);
static void CustomPeripheral_processPasscode(cpPasscodeData_t *pPasscodeData);
static void CustomPeripheral_charValueChangeCB(uint8_t paramId);
static status_t CustomPeripheral_enqueueMsg(uint8_t event, void *pData);
//static void CustomPeripheral_keyChangeHandler(uint8_t keys);
static void CustomPeripheral_handleKeys(uint8_t keys);
static void CustomPeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void CustomPeripheral_initPHYRSSIArray(void);
static void CustomPeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t CustomPeripheral_addConn(uint16_t connHandle);
static uint8_t CustomPeripheral_getConnIndex(uint16_t connHandle);
static uint8_t CustomPeripheral_removeConn(uint16_t connHandle);
static void CustomPeripheral_processParamUpdate(uint16_t connHandle);
static status_t CustomPeripheral_startAutoPhyChange(uint16_t connHandle);
static status_t CustomPeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t CustomPeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t CustomPeripheral_clearConnListEntry(uint16_t connHandle);
//static void CustomPeripheral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
//                                          tbmMenuObj_t* pMenuObjNext);
static void CustomPeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void CustomPeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

static void CustomPeripheral_INT_CB(PIN_Handle handle, PIN_Id pinId);

#ifdef PTM_MODE
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg);  // Declaration
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len);  // Declaration
#endif // PTM_MODE


/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t CustomPeripheral_BondMgrCBs =
{
  CustomPeripheral_passcodeCb,       // Passcode callback
  CustomPeripheral_pairStateCb       // Pairing/Bonding state Callback
};

// Custom GATT Profile Callbacks
static BLE_CBs_t CustomPeripheral_CBs =
{
  CustomPeripheral_charValueChangeCB // Custom GATT Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

#ifdef PTM_MODE
/*********************************************************************
* @fn      simple_peripheral_handleNPIRxInterceptEvent
*
* @brief   Intercept an NPI RX serial message and queue for this application.
*
* @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
*
* @return  none.
*/
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
 // Send Command via HCI TL
 HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

 // The data is stored as a message, free this first.
 ICall_freeMsg(((NPIMSG_msg_t *)pMsg)->pBuf);

 // Free container.
 ICall_free(pMsg);
}

/*********************************************************************
* @fn      simple_peripheral_sendToNPI
*
* @brief   Create an NPI packet and send to NPI to transmit.
*
* @param   buf - pointer HCI event or data.
*
* @param   len - length of buf in bytes.
*
* @return  none
*/
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len)
{
 npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

 if (pNpiPkt)
 {
   pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
   pNpiPkt->hdr.status = 0xFF;
   pNpiPkt->pktLen = len;
   pNpiPkt->pData  = (uint8_t *)(pNpiPkt + 1);

   memcpy(pNpiPkt->pData, buf, len);

   // Send to NPI
   // Note: there is no need to free this packet.  NPI will do that itself.
   NPITask_sendToHost((uint8_t *)pNpiPkt);
 }
}
#endif // PTM_MODE

/*********************************************************************
 * @fn      CustomPeripheral_createTask
 *
 * @brief   Task creation function for the Custom Peripheral.
 */
void CustomPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = cpTaskStack;
  taskParams.stackSize = CP_TASK_STACK_SIZE;
  taskParams.priority = CP_TASK_PRIORITY;

  Task_construct(&cpTask, CustomPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      CustomPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void CustomPeripheral_init(void)
{
  // Create the menu
  //CustomPeripheral_buildMenu();

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
  HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, CustomPeripheral_clockHandler,
                      CP_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  dispHandle = Display_open(Display_Type_ANY, NULL);

  /*RLEDhandle = LED_open(Board_RLED, &LED_defaultParams);
  GLEDhandle = LED_open(Board_GLED, &LED_defaultParams);
  LED_setOn(RLEDhandle, 0);
  LED_setOn(GLEDhandle, 0);*/

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide
  setBondManagerParameters();

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  CUSTOM_AddService(); // Custom GATT Profile

  // Setup the CUSTOM Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/

  {
      CUSTOM_SetParameter(CHAR1_ID, CHAR1_LEN, NULL);
      CUSTOM_SetParameter(CHAR2_ID, CHAR2_LEN, NULL);
      CUSTOM_SetParameter(CHAR3_ID, CHAR3_LEN, NULL);
      CUSTOM_SetParameter(CHAR4_ID, CHAR4_LEN, NULL);
      CUSTOM_SetParameter(CHAR5_ID, CHAR5_LEN, NULL);
  }

  // Register callback with CustomGATTprofile
  CUSTOM_RegisterAppCBs(&CustomPeripheral_CBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&CustomPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Init key debouncer
  //Board_initKeys(CustomPeripheral_keyChangeHandler);

  // Initialize Connection List
  CustomPeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, NULL);

  // Initialize array to store connection handle and RSSI values
  CustomPeripheral_initPHYRSSIArray();

  // Initialize Two-Button Menu module
  /*TBM_SET_TITLE(&spMenuMain, "Custom Peripheral");
  tbm_setItemStatus(&spMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);

  tbm_initTwoBtnMenu(dispHandle, &spMenuMain, 5, CustomPeripheral_menuSwitchCb);
  Display_printf(dispHandle, CP_ROW_SEPARATOR_1, 0, "====================");*/

  //Initialize SPI Bus
    SPI_init();  // Initialize the SPI driver
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.dataSize = 8;       // 8-bit data size

     spi = SPI_open(CONFIG_SPI_0, &spiParams);
      if (spi == NULL) {
          while (1);  // SPI_open() failed
     }


       //Set up AD5941 in Chronoamperometric Mode
      AppCAPHGetCfg(&pAMPCfg);
      AD5940PlatformCfg();

        AD5940AMPStructInit();  //Configure your parameters in this function

        ADerror = AppCAPHInit(AppBuff[0], APPBUFF_SIZE);    /* Initialize AMP application. Provide a buffer, which is used to store sequencer commands */
       printf ("AD CFG COMPLETE\n");


  // Set up interrupt
        const PIN_Config ADrdypin[] = {
                                     Board_AD_GPIO_0 | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE,    // GPIO_0
                                     PIN_TERMINATE
                                    };

        PIN_open(&hStateADrdy, ADrdypin);

        if(PIN_registerIntCb(&hStateADrdy, CustomPeripheral_INT_CB) != 0) {
            printf("Interrupt Init Failed\n");
        }
        printf("Custom Peripheral Init\n");

        //ADerror = AppCAPHCtrl(CHRONOAMPCTRL_START, 0);


#ifdef PTM_MODE
  // Intercept NPI RX events.
  NPITask_registerIncomingRXEventAppCB(simple_peripheral_handleNPIRxInterceptEvent, INTERCEPT);

  // Register for Command Status information
  HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) simple_peripheral_sendToNPI, NULL, selfEntity);

  // Register for Events
  HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity));

  // Inform Stack to Initialize PTM
  HCI_EXT_EnablePTMCmd();
#endif // PTM_MODE
}

/*********************************************************************
 * @fn      CustomPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Custom Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void CustomPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  CustomPeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, CP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = CustomPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & CP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          cpEvt_t *pMsg = (cpEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            CustomPeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t CustomPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      CustomPeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = CustomPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          CustomPeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
                Display_printf(dispHandle, CP_ROW_STATUS_1, 0,
                        "PHY Change failure, peer does not support this");
              }
              else
              {
                Display_printf(dispHandle, CP_ROW_STATUS_1, 0,
                               "PHY Update Status Event: 0x%x",
                               pMyMsg->cmdStatus);
              }

              CustomPeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {
              Display_printf(dispHandle, CP_ROW_STATUS_1, 0,
                             "PHY Change failure");
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
              Display_printf(dispHandle, CP_ROW_STATUS_2, 0,
                             "PHY Updated to %s",
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value");
            }

            CustomPeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }
          break;
        }

        default:
          break;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

#ifdef PTM_MODE
  // Check for NPI Messages
  hciPacket_t *pBuf = (hciPacket_t *)pMsg;

  // Serialized HCI Event
  if (pBuf->hdr.event == HCI_CTRL_TO_HOST_EVENT)
  {
    uint16_t len = 0;

    // Determine the packet length
    switch(pBuf->pData[0])
    {
      case HCI_EVENT_PACKET:
        len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
        break;

      case HCI_ACL_DATA_PACKET:
        len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
        break;

      default:
        break;
    }

    // Send to Remote Host.
    simple_peripheral_sendToNPI(pBuf->pData, len);

    // Free buffers if needed.
    switch (pBuf->pData[0])
    {
      case HCI_ACL_DATA_PACKET:
      case HCI_SCO_DATA_PACKET:
        BM_free(pBuf->pData);
      default:
        break;
    }
  }
#endif // PTM_MODE

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      CustomPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t CustomPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      CustomPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void CustomPeripheral_processAppMsg(cpEvt_t *pMsg)
{
  static uint8_t packetCount = 0;
  bool dealloc = TRUE;

  switch (pMsg->event)
  {
      //User events
  case CP_DATA_EVT:
//      printf("Processing AD Data\n");
//      GPIO_write(LED_RED_GPIO,1);
//      AppCAPHISR(CHAR2data); /* Deal with it and provide a buffer to store data we got */
//      GPIO_write(LED_RED_GPIO,0);
      CustomPeripheral_performPeriodicTask();
      break;

      //System events
    case CP_CHAR_CHANGE_EVT:
      CustomPeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case CP_KEY_CHANGE_EVT:
      CustomPeripheral_handleKeys(*(uint8_t*)(pMsg->pData));
      break;

    case CP_ADV_EVT:
      CustomPeripheral_processAdvEvent((cpGapAdvEventData_t*)(pMsg->pData));
      break;

    case CP_PAIR_STATE_EVT:
      CustomPeripheral_processPairState((cpPairStateData_t*)(pMsg->pData));
      break;

    case CP_PASSCODE_EVT:
      CustomPeripheral_processPasscode((cpPasscodeData_t*)(pMsg->pData));
      break;

    case CP_PERIODIC_EVT:
      CustomPeripheral_performPeriodicTask();
      break;

    case CP_READ_RPA_EVT:
      CustomPeripheral_updateRPA();
      break;

    case CP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((cpClockEventData_t *)pMsg->pData)->data);

      CustomPeripheral_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case CP_CONN_EVT:
      CustomPeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
      break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void CustomPeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Initialized");

        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&CustomPeripheral_advCallback, &advParams1,
                               &advHandleLegacy);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData1), advData1);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanResData1), scanResData1);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&CustomPeripheral_advCallback, &advParams2,
                               &advHandleLongRange);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData2), advData2);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        CUSTOMPERIPHERAL_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, CP_ROW_IDA, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        if (addrMode > ADDRMODE_RANDOM)
        {
          CustomPeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, CustomPeripheral_clockHandler,
                              READ_RPA_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
        //tbm_setItemStatus(&spMenuMain, CP_ITEM_AUTOCONNECT, TBM_ITEM_NONE);
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, CP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        CustomPeripheral_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Connected to %s",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        // Enable connection selection option
        //tbm_setItemStatus(&spMenuMain, CP_ITEM_SELECT_CONN,CP_ITEM_AUTOCONNECT);

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
      }
      if ((numActive < MAX_NUM_BLE_CONNS) && (autoConnect == AUTOCONNECT_DISABLE))
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
      }

      ADerror = AppCAPHCtrl(CHRONOAMPCTRL_START, 0); //Start CA/PH measurement

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Device Disconnected!");
      Display_printf(dispHandle, CP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection from the list and disable RSSI if needed
      CustomPeripheral_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        //Util_stopClock(&clkPeriodic);

        // Disable Connection Selection option
        //tbm_setItemStatus(&spMenuMain, CP_ITEM_AUTOCONNECT, CP_ITEM_SELECT_CONN);
      }

      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      Display_clearLine(dispHandle, CP_ROW_CONNECTION);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {
        // Display the address of the connection update
        Display_printf(dispHandle, CP_ROW_STATUS_2, 0, "Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        // Display the address of the connection update failure
        Display_printf(dispHandle, CP_ROW_STATUS_2, 0,
                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));
      }

      // Check if there are any queued parameter updates
      cpConnHandleEntry_t *connHandleEntry = (cpConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        CustomPeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

    default:
      Display_clearLines(dispHandle, CP_ROW_STATUS_1, CP_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void CustomPeripheral_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    Display_printf(dispHandle, CP_ROW_RPA, 0, "RP Addr: %s",
                   Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void CustomPeripheral_clockHandler(UArg arg)
{
  cpClockEventData_t *pData = (cpClockEventData_t *)arg;

 if (pData->event == CP_PERIODIC_EVT)
 {
   // Start the next period
   Util_startClock(&clkPeriodic);

   // Post event to wake up the application
   CustomPeripheral_enqueueMsg(CP_PERIODIC_EVT, NULL);
 }
 else if (pData->event == CP_READ_RPA_EVT)
 {
   // Start the next period
   Util_startClock(&clkRpaRead);

   // Post event to read the current RPA
   CustomPeripheral_enqueueMsg(CP_READ_RPA_EVT, NULL);
 }
 else if (pData->event == CP_SEND_PARAM_UPDATE_EVT)
 {
    // Send message to app
    CustomPeripheral_enqueueMsg(CP_SEND_PARAM_UPDATE_EVT, pData);
 }
}

/*********************************************************************
 * @fn      CustomPeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
/*static void CustomPeripheral_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    if(CustomPeripheral_enqueueMsg(CP_KEY_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}*/

/*********************************************************************
 * @fn      CustomPeripheral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 */
static void CustomPeripheral_handleKeys(uint8_t keys)
{
  /*if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(CONFIG_PIN_BTN1) == 0)
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed. Workaround for possible bouncing.
    if (PIN_getInputValue(CONFIG_PIN_BTN2) == 0)
    {
      tbm_buttonRight();
    }
  }*/
}

/*********************************************************************
 * @fn      CustomPeripheral_doSetConnPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool CustomPeripheral_doSetConnPhy(uint8_t index)
{
  bool status = TRUE;

  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
    AUTO_PHY_UPDATE
  };

  uint8_t connIndex = CustomPeripheral_getConnIndex(menuConnHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
    return FALSE;
  }

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  // If auto PHY update is not selected and if auto PHY update is enabled, then
  // stop auto PHY update
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  if(phy[index] != AUTO_PHY_UPDATE)
  {
    // Cancel RSSI reading  and auto phy changing
    CustomPeripheral_stopAutoPhyChange(connList[connIndex].connHandle);

    CustomPeripheral_setPhy(menuConnHandle, 0, phy[index], phy[index], 0);

    //Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "PHY preference: %s",
    //               TBM_GET_ACTION_DESC(&spMenuConnPhy, index));
  }
  else
  {
    // Start RSSI read for auto PHY update (if it is disabled)
    CustomPeripheral_startAutoPhyChange(menuConnHandle);
  }

  return status;
}
/*********************************************************************
 * @fn      CustomPeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void CustomPeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  cpGapAdvEventData_t *pData = ICall_malloc(sizeof(cpGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(CustomPeripheral_enqueueMsg(CP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void CustomPeripheral_processAdvEvent(cpGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      Display_printf(dispHandle, CP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      Display_printf(dispHandle, CP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

      Display_printf(dispHandle, CP_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
                     advSetTerm->handle, advSetTerm->connHandle );
#endif
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      CustomPeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void CustomPeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  cpPairStateData_t *pData = ICall_malloc(sizeof(cpPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(CustomPeripheral_enqueueMsg(CP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void CustomPeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  cpPasscodeData_t *pData = ICall_malloc(sizeof(cpPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(CustomPeripheral_enqueueMsg(CP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void CustomPeripheral_processPairState(cpPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void CustomPeripheral_processPasscode(cpPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      CustomPeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void CustomPeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(CustomPeripheral_enqueueMsg(CP_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void CustomPeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Get index from handle
  uint8_t connIndex = CustomPeripheral_getConnIndex(pReport->handle);

  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
    return;
  }

  // If auto phy change is enabled
  if (connList[connIndex].isAutoPHYEnable == TRUE)
  {
    // Read the RSSI
    HCI_ReadRssiCmd(pReport->handle);
  }
}


/*********************************************************************
 * @fn      CustomPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t CustomPeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  cpEvt_t *pMsg = ICall_malloc(sizeof(cpEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      CustomPeripheral_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool CustomPeripheral_doSelectConn(uint8_t index)
{
  menuConnHandle = connList[index].connHandle;

  // Set the menu title and go to this connection's context
  //TBM_SET_TITLE(&spMenuPerConn, TBM_GET_ACTION_DESC(&spMenuSelectConn, index));

  // Clear non-connection-related message
  Display_clearLine(dispHandle, CP_ROW_CONNECTION);

  //tbm_goTo(&spMenuPerConn);

  return (true);
}
/*********************************************************************
 * @fn      CustomPeripheral_doAutoConnect
 *
 * @brief   Enable/Disable peripheral as AutoConnect node.
 *
 * @param   index - 0 : Disable AutoConnect
 *                  1 : Enable Group A
 *                  2 : Enable Group B
 *
 * @return  always true
 */
bool CustomPeripheral_doAutoConnect(uint8_t index)
{
    if (index == 1)
    {
      if (autoConnect != AUTOCONNECT_GROUP_A)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'A';
        advData2[2] = 'G';
        advData2[3] = 'A';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_GROUP_A;
      }
      Display_printf(dispHandle, CP_ROW_AC, 0, "AutoConnect enabled: Group A");
    }
    else if (index == 2)
    {
      if (autoConnect != AUTOCONNECT_GROUP_B)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'G';
        advData1[3] = 'B';
        advData2[2] = 'G';
        advData2[3] = 'B';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_GROUP_B;
      }
      Display_printf(dispHandle, CP_ROW_AC, 0, "AutoConnect enabled: Group B");
    }
    else
    {
      if (autoConnect)
      {
        GapAdv_disable(advHandleLongRange);
        GapAdv_disable(advHandleLegacy);
        advData1[2] = 'S';
        advData1[3] = 'P';
        advData2[2] = 'S';
        advData2[3] = 'P';
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        autoConnect = AUTOCONNECT_DISABLE;
      }
      Display_printf(dispHandle, CP_ROW_AC, 0, "AutoConnect disabled");
    }
    //tbm_goTo(&spMenuMain);

    return (true);
}


/*********************************************************************
 * @fn      CustomPeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t CustomPeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

      // Allocate data to send through clock handler
      connList[i].pParamUpdateEventData = ICall_malloc(sizeof(cpClockEventData_t) +
                                                       sizeof (uint16_t));
      if(connList[i].pParamUpdateEventData)
      {
        connList[i].pParamUpdateEventData->event = CP_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)connList[i].pParamUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              CustomPeripheral_clockHandler,
                              SEND_PARAM_UPDATE_DELAY, 0, true,
                              (UArg) (connList[i].pParamUpdateEventData));
        }
        else
        {
            ICall_free(connList[i].pParamUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      CustomPeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t CustomPeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      CustomPeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t CustomPeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = CustomPeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
    {
      return(bleInvalidRange);
    }
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, CP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      CustomPeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void CustomPeripheral_clearPendingParamUpdate(uint16_t connHandle)
{
  List_Elem *curr;

  for (curr = List_head(&paramUpdateList); curr != NULL; curr = List_next(curr))
  {
    if (((cpConnHandleEntry_t *)curr)->connHandle == connHandle)
    {
      List_remove(&paramUpdateList, curr);
    }
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t CustomPeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = CustomPeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
      // Free ParamUpdateEventData
      ICall_free(connList[connIndex].pParamUpdateEventData);
    }
    // Clear pending update requests from paramUpdateList
    CustomPeripheral_clearPendingParamUpdate(connHandle);
    // Stop Auto PHY Change
    CustomPeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    CustomPeripheral_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      CustomPeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void CustomPeripheral_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = CustomPeripheral_getConnIndex(connHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
    return;
  }

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct, only in case it is not NULL
  if (connList[connIndex].pUpdateClock != NULL)
  {
      ICall_free(connList[connIndex].pUpdateClock);
      connList[connIndex].pUpdateClock = NULL;
  }
  // Free ParamUpdateEventData, only in case it is not NULL
  if (connList[connIndex].pParamUpdateEventData != NULL)
      ICall_free(connList[connIndex].pParamUpdateEventData);

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    cpConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(cpConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      CustomCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void CustomPeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      int8 rssi = (int8)pMsg->pReturnParam[3];

      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = CustomPeripheral_getConnIndex(handle);
        if (index >= MAX_NUM_BLE_CONNS)
        {
          Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
          return;
        }

        if (rssi != LL_RSSI_NOT_AVAILABLE)
        {
          connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
          connList[index].rssiCntr %= CP_MAX_RSSI_STORE_DEPTH;

          int16_t sum_rssi = 0;
          for(uint8_t cnt=0; cnt<CP_MAX_RSSI_STORE_DEPTH; cnt++)
          {
            sum_rssi += connList[index].rssiArr[cnt];
          }
          connList[index].rssiAvg = (uint32_t)(sum_rssi/CP_MAX_RSSI_STORE_DEPTH);

          uint8_t phyRq = CP_PHY_NONE;
          uint8_t phyRqS = CP_PHY_NONE;
          uint8_t phyOpt = LL_PHY_OPT_NONE;

          if(connList[index].phyCngRq == FALSE)
          {
            if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
            (connList[index].currPhy != HCI_PHY_2_MBPS) &&
                 (connList[index].currPhy != CP_PHY_NONE))
            {
              // try to go to higher data rate
              phyRqS = phyRq = HCI_PHY_2_MBPS;
            }
            else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                    (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                    (connList[index].currPhy != CP_PHY_NONE))
            {
              // try to go to legacy regular data rate
              phyRqS = phyRq = HCI_PHY_1_MBPS;
            }
            else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                    (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != CP_PHY_NONE))
            {
              // try to go to lower data rate S=2(500kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S2;
              phyRq = BLE5_CODED_S2_PHY;
            }
            else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
            {
              // try to go to lowest data rate S=8(125kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S8;
              phyRq = BLE5_CODED_S8_PHY;
            }
            if((phyRq != CP_PHY_NONE) &&
               // First check if the request for this phy change is already not honored then don't request for change
               (((connList[index].rqPhy == phyRq) &&
                 (connList[index].phyRqFailCnt < 2)) ||
                 (connList[index].rqPhy != phyRq)))
            {
              //Initiate PHY change based on RSSI
              CustomPeripheral_setPhy(connList[index].connHandle, 0,
                                      phyRqS, phyRqS, phyOpt);
              connList[index].phyCngRq = TRUE;

              // If it a request for different phy than failed request, reset the count
              if(connList[index].rqPhy != phyRq)
              {
                // then reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
              }

              if(phyOpt == LL_PHY_OPT_NONE)
              {
                connList[index].rqPhy = phyRq;
              }
              else if(phyOpt == LL_PHY_OPT_S2)
              {
                connList[index].rqPhy = BLE5_CODED_S2_PHY;
              }
              else
              {
                connList[index].rqPhy = BLE5_CODED_S8_PHY;
              }

            } // end of if ((phyRq != CP_PHY_NONE) && ...
          } // end of if (connList[index].phyCngRq == FALSE)
        } // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

        Display_printf(dispHandle, CP_ROW_RSSI, 0,
                       "RSSI:%d dBm, AVG RSSI:%d dBm",
                       (uint32_t)(rssi),
                       connList[index].rssiAvg);

      } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, CP_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      CustomPeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void CustomPeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = CP_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      CustomPeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t CustomPeripheral_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = CustomPeripheral_getConnIndex(connHandle);
  CUSTOMPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  status = Gap_RegisterConnEventCb(CustomPeripheral_connEvtCB, GAP_CB_REGISTER, connHandle);

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      CustomPeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t CustomPeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = CustomPeripheral_getConnIndex(connHandle);
  CUSTOMPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      CustomPeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t CustomPeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  cpConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(cpConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      CustomPeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void CustomPeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      cpConnHandleEntry_t *connHandleEntry =
                           (cpConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = CustomPeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = CustomPeripheral_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
 * @fn      CustomPeripheral_menuSwitchCb
 *
 * @brief   Detect menu context switching
 *
 * @param   pMenuObjCurr - the current menu object
 * @param   pMenuObjNext - the menu object the context is about to switch to
 *
 * @return  none
 */
/*static void CustomPeripheral_menuSwitchCb(tbmMenuObj_t* pMenuObjCurr,
                                       tbmMenuObj_t* pMenuObjNext)
{
  uint8_t NUMB_ACTIVE_CONNS = linkDB_NumActive();

  // interested in only the events of
  // entering scMenuConnect, spMenuSelectConn, and scMenuMain for now
  if (pMenuObjNext == &spMenuSelectConn)
  {
    static uint8_t* pAddrs;
    uint8_t* pAddrTemp;

    if (pAddrs != NULL)
    {
      ICall_free(pAddrs);
    }

    // Allocate buffer to display addresses
    pAddrs = ICall_malloc(NUMB_ACTIVE_CONNS * CP_ADDR_STR_SIZE);

    if (pAddrs == NULL)
    {
      TBM_SET_NUM_ITEM(&spMenuSelectConn, 0);
    }
    else
    {
      uint8_t i;

      TBM_SET_NUM_ITEM(&spMenuSelectConn, MAX_NUM_BLE_CONNS);

      pAddrTemp = pAddrs;

      // Add active connection info to the menu object
      for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
      {
        if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID)
        {
          // Get the address from the connection handle
          linkDBInfo_t linkInfo;
          linkDB_GetInfo(connList[i].connHandle, &linkInfo);
          // This connection is active. Set the corresponding menu item with
          // the address of this connection and enable the item.
          memcpy(pAddrTemp, Util_convertBdAddr2Str(linkInfo.addr),
                 CP_ADDR_STR_SIZE);
          TBM_SET_ACTION_DESC(&spMenuSelectConn, i, pAddrTemp);
          tbm_setItemStatus(&spMenuSelectConn, (1 << i), CP_ITEM_NONE);
          pAddrTemp += CP_ADDR_STR_SIZE;
        }
        else
        {
          // This connection is not active. Disable the corresponding menu item.
          tbm_setItemStatus(&spMenuSelectConn, CP_ITEM_NONE, (1 << i));
        }
      }
    }
  }
  else if (pMenuObjNext == &spMenuMain)
  {
    // Now we are not in a specific connection's context

    // Clear connection-related message
    Display_clearLine(dispHandle, CP_ROW_CONNECTION);
  }
}*/

/*********************************************************************
 * @fn      CustomPeripheral_charValueChangeCB
 *
 * @brief   Callback from Custom Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void CustomPeripheral_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (CustomPeripheral_enqueueMsg(CP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Custom Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void CustomPeripheral_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue[SYS_CFG_LEN];

  switch(paramId)
  {
    case SYS_CFG_ID:
      CUSTOM_GetParameter(SYS_CFG_ID, &newValue);

      Display_printf(dispHandle, CP_ROW_STATUS_1, 0, "Char SYS: 0x%x%x%x%x",
                     newValue[3],newValue[2],newValue[1],newValue[0]);
      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      CustomPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every two seconds (CP_PERIODIC_EVT_PERIOD). In this project,
 *          the temperature data is read and transmitted over BLE. It also
 *          process the magnetometer data, if the AFE4404 is not active.
 *          When the device is disconnected from the host, all the reseting
 *          activity is performed here, after the disconnection is completed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void CustomPeripheral_performPeriodicTask(void)
{
    static uint8_t i = 0;
    const uint32_t NotifTime = ClockP_getSystemTicks();
    Display_printf(dispHandle, CP_ROW_CONNECTION, 0, "periodic: %d.%05d", NotifTime/100000,NotifTime%100000);

    CHAR1data[0] = i;
    CHAR3data[0] = i+1;
    CHAR4data[0] = i+2;
    CHAR5data[0] = i+3;


    CUSTOM_SetParameter(CHAR1_ID, CHAR1_LEN, NULL);
    CUSTOM_SetParameter(CHAR2_ID, CHAR2_LEN, NULL);
    CUSTOM_SetParameter(CHAR3_ID, CHAR3_LEN, NULL);
    CUSTOM_SetParameter(CHAR4_ID, CHAR4_LEN, NULL);
    CUSTOM_SetParameter(CHAR5_ID, CHAR5_LEN, NULL);

    i++;

}

static void CustomPeripheral_INT_CB(PIN_Handle handle, PIN_Id pinId)
{
    //printf("Processing AD Data\n");
    GPIO_write(LED_RED_GPIO,1);
    AppCAPHISR(CHAR2data); /* Deal with it and provide a buffer to store data we got */
    GPIO_write(LED_RED_GPIO,0);
    //printf("Interrupt!\n");

//    if(AD5940_GetMCUIntFlag()){
//          AD5940_ClrMCUIntFlag(); /* Clear this flag */
//          temp[IntCount] = APPBUFF_SIZE;
//          AppCHRONOAMPISR(AppBuff[IntCount], &temp[IntCount]); /* Deal with it and provide a buffer to store data we got */
//                if(pAMPCfg->bMeasureTransient == bFALSE)
//                {
//                    AMPShowResult((float*)AppBuff[0], temp[0]);
//                }
//    }
//    else{
//     printf("No Flag\n");
//    }



       //AMPShowResult((float*)AppBuff[0], temp);

        // Read battery
        //g_BattRead++;   // dummy battery read

        // Send data
        //CustomPeripheral_enqueueMsg(CP_DATA_EVT,NULL); //handle interrupt
        //CUSTOM_SetParameter(CHAR2_ID, CHAR2_LEN, NULL);
}


