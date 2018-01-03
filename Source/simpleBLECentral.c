/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
//#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"

//�������ڳ���
#include "SerialApp.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      800

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           100

// Default passcode
#define DEFAULT_PASSCODE                      123456

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     TRUE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE

// Checking period for pressing key if this key is still be pressed 
#define WORKING_REQ_CHECK_PERIOD              100

// Open debug log in uart port, you should also open HAL_UART=TRUE
//#define UART_DEBUG_MODE                       TRUE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// Working states
enum
{
  WORKKEY_PRESSED,
  WORKKEY_RELEASE
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "TJGQ glove";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;
static uint8 simpleBLEScanSelectedIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
static uint16 simpleBLECharHdl = 0;

// Value to write
static uint8 simpleBLECharVal = 0;

// Value read/write toggle, TRUE to write and FALSE to read
static bool simpleBLEDoWrite = TRUE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

// GAP scan response data
static uint8 defaultDeviceName[] = { 0x54, 0x4A, 0x47, 0x51 };  // 'T', 'J', 'G', 'Q'

// GAP scan response data length
static uint8 defaultDeviceNameLength = 4;

static attWriteReq_t workingReq;

static uint8 workingState = WORKKEY_RELEASE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void SendWorkingState( void );
static void simpleBLECentralSearchDevice( void );
static void simpleBLECentralSelectDevice( void );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType, int8 rssi);
static void simpleBLECentralConnectDevice( void );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static bool simpleBLEFindDeviceName( uint8* pData, uint8 dataLen);

char *bdAddr2Str ( uint8 *pAddr );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

#if defined ( HAL_UART)
  SerialApp_Init(simpleBLETaskId);
#endif

#if defined ( UART_DEBUG_MODE )
  SerialPrintString("\r\nSimpleBLECentral_SerialPrint Start init.");
#endif

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }

  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

#if ( defined POWER_SAVING )
  // Power saving mode
  osal_pwrmgr_device( PWRMGR_BATTERY );
#endif

#if defined ( UART_DEBUG_MODE )
  SerialPrintString("\r\nReady to Starting");
#endif
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\nBLE Stack is running");
#endif

    osal_set_event( simpleBLETaskId, START_SEARCH_EVT );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_SEARCH_EVT )
  {
    simpleBLECentralSearchDevice( );

    return ( events ^ START_SEARCH_EVT );
  }

  if ( events & DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );

    return ( events ^ DISCOVERY_EVT );
  }

  if ( events & SELECT_EVT )
  {
    simpleBLECentralSelectDevice( );
    
    return ( events ^ SELECT_EVT );
  }

  if ( events & CONNECT_EVT )
  {
    simpleBLECentralConnectDevice( );

    return ( events ^ CONNECT_EVT );
  }
  if ( events & WORKING_REQ_EVT )
  {
    SendWorkingState();

    return ( events ^ WORKING_REQ_EVT );
  }
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_6 )
  {
#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\n[KEY SW 6 pressed!]");
#endif

    //if ( simpleBLEState == BLE_STATE_CONNECTED &&
    //          simpleBLECharHdl != 0 &&
    //          simpleBLEProcedureInProgress == FALSE )
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      osal_set_event(simpleBLETaskId, WORKING_REQ_EVT);
    }
  }

  if ( keys & HAL_KEY_UP )
  {
#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\n[KEY UP pressed!]");
#endif

    osal_set_event( simpleBLETaskId, START_SEARCH_EVT );
  }

  if ( keys & HAL_KEY_RIGHT )
  {
#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\n[KEY RIGHT pressed!]");
#endif

    // Connection update
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      GAPCentralRole_UpdateLink( simpleBLEConnHandle,
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    }
  }

  if ( keys & HAL_KEY_LEFT )
  {
  }

  if ( keys & HAL_KEY_CENTER )
  {
#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\n[KEY CENTER pressed!]");
#endif
    // disconnect
    if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink( simpleBLEConnHandle );

#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nDisconnecting");
#endif
    }
  }

  if ( keys & HAL_KEY_DOWN )
  {
#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\n[KEY DOWN pressed!]");
#endif
    // Start or cancel RSSI polling
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )
      {
        simpleBLERssi = TRUE;
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
      }
      else
      {
        simpleBLERssi = FALSE;
        GAPCentralRole_CancelRssi( simpleBLEConnHandle );

#if defined ( UART_DEBUG_MODE )
        SerialPrintString("\r\nRSSI Cancelled");
#endif
      }
    }
  }
}

/*********************************************************************
 * @fn      SendWorkingState
 *
 * @brief   Send current key state to remote device
 *
 * @param   void
 *
 * @return  none
 */
void SendWorkingState( void )
{
  bool isPressing = HalKeyPressing(HAL_KEY_SW_6);

  workingReq.handle = simpleBLECharHdl;
  workingReq.len = 1;
  workingReq.sig = 0;
  workingReq.cmd = 0;

  // 如果希望始终发送按键信息，则应该关掉workingState状态的判断，并且在isPressing的外边启动定时器，一直不关闭
  if ( isPressing )
  {
    if ( workingState == WORKKEY_RELEASE )
    {
      workingReq.value[0] = 1;
      //SerialPrintString("\r\nsending press");

      uint8 tmp = GATT_WriteCharValue(simpleBLEConnHandle, &workingReq, simpleBLETaskId);
      if (tmp == SUCCESS)
      {
        simpleBLEProcedureInProgress = TRUE;
        workingState = WORKKEY_PRESSED;
        //SerialPrintString("\r\nsend press success");
      }
    }
    osal_start_timerEx( simpleBLETaskId, WORKING_REQ_EVT, WORKING_REQ_CHECK_PERIOD);
  }
  else
  {
    if ( workingState == WORKKEY_PRESSED )
    {
      workingReq.value[0] = 0;
      //SerialPrintString("\r\nsending release");

      uint8 tmp = GATT_WriteCharValue(simpleBLEConnHandle, &workingReq, simpleBLETaskId);
      if (tmp == SUCCESS)
      {
        simpleBLEProcedureInProgress = TRUE;
        workingState = WORKKEY_RELEASE;
        //SerialPrintString("\r\nsend release success");
      }
    }
    osal_stop_timerEx( simpleBLETaskId, WORKING_REQ_EVT);
  }
  
  //osal_start_timerEx( simpleBLETaskId, WORKING_REQ_EVT, WORKING_REQ_CHECK_PERIOD);

  // other cases do nothing

}

/*********************************************************************
 * @fn      simpleBLECentralSearchDevice
 *
 * @brief   start to search device
 *
 * @param   none
 *
 * @return  none
 */
static void simpleBLECentralSearchDevice( void )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    if ( !simpleBLEScanning )
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;

#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nDiscovering...");
#endif

      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST );
    }
    else
    {
      GAPCentralRole_CancelDiscovery();
    }
  }
}
/*********************************************************************
 * @fn      simpleBLECentralSelectDevice
 *
 * @brief   Select a device with max rssi value
 *
 * @return  none
 */
static void simpleBLECentralSelectDevice()
{
  if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
  {
    simpleBLEScanIdx = 0;
    simpleBLEScanSelectedIdx = 0;
    uint8 minRssi = 120;
    uint8 rssi;
    while ( simpleBLEScanIdx < simpleBLEScanRes )
    {
#if defined ( UART_DEBUG_MODE )
      SerialPrintValue( "\r\nDevice", simpleBLEScanIdx + 1, 10);
      SerialPrintString((uint8*) bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ));
      SerialPrintValue("\r\nRssi", simpleBLEDevList[simpleBLEScanIdx].rssi, 10);
#endif

      rssi = simpleBLEDevList[simpleBLEScanIdx].rssi;
      if ( rssi < minRssi )
      {
        simpleBLEScanSelectedIdx = simpleBLEScanIdx;
        minRssi = rssi;
      }
      // Increment index of current result (with wraparound)
      simpleBLEScanIdx++;
    }


#if defined ( UART_DEBUG_MODE )
    SerialPrintValue("\r\nSelect Device", simpleBLEScanSelectedIdx + 1, 10);
    SerialPrintString(" to connect ");
    SerialPrintString((uint8*) bdAddr2Str( simpleBLEDevList[simpleBLEScanSelectedIdx].addr ));
    SerialPrintValue("\r\nRssi:", simpleBLEDevList[simpleBLEScanSelectedIdx].rssi, 10);
#endif

    osal_set_event( simpleBLETaskId, CONNECT_EVT );
  }
}

/*********************************************************************
 * @fn      simpleBLECentralConnectDevice
 *
 * @brief   Connect Device be selected
 *
 * @return  none
 */
static void simpleBLECentralConnectDevice()
{
  uint8 addrType;
  uint8 *peerAddr;
  if ( simpleBLEState == BLE_STATE_IDLE )
  {
    // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[simpleBLEScanSelectedIdx].addr;
        addrType = simpleBLEDevList[simpleBLEScanSelectedIdx].addrType;

        simpleBLEState = BLE_STATE_CONNECTING;

        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );


#if defined ( UART_DEBUG_MODE )
        SerialPrintString("\r\nConnecting:");
        SerialPrintString((uint8*)bdAddr2Str( peerAddr));//SerialPrintString("\r\n");
#endif
      }
  }

}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }

  if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {

    if ( pMsg->method == ATT_WRITE_RSP ) 
    {
      // a succesful write
#if defined ( UART_DEBUG_MODE )
      uint8 temp = 
#endif
        workingReq.value[0];
      
#if defined ( UART_DEBUG_MODE )
      SerialPrintValue( "\r\nWrite sent:", temp, 10);
#endif
    }
    else
    {
#if defined ( UART_DEBUG_MODE )
      uint8 status = 
#endif
        pMsg->msg.errorRsp.errCode;

#if defined ( UART_DEBUG_MODE )
      SerialPrintValue( "\r\nWrite Error", status, 10);
#endif
    }

    simpleBLEProcedureInProgress = FALSE;

  }
  else if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_READ_RSP ) 
    {
      // After a successful read, display the read value
#if defined ( UART_DEBUG_MODE )
      uint8 valueRead = 
#endif
        pMsg->msg.readRsp.value[0];

#if defined ( UART_DEBUG_MODE )
      SerialPrintValue("\r\nRead rsp:", valueRead, 10);
#endif
    }
    else
    {
#if defined ( UART_DEBUG_MODE )
      uint8 status = 
#endif
        pMsg->msg.errorRsp.errCode;

#if defined ( UART_DEBUG_MODE )
      SerialPrintValue("\r\nRead Error", status, 10);
#endif
    }

    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }

}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
#if defined ( UART_DEBUG_MODE )
    SerialPrintValue("RSSI -dB:", (uint8) (-rssi), 10);//SerialPrintString("\r\n");
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
      {
#if defined ( UART_DEBUG_MODE )
        SerialPrintString("\r\nBLE Central: ");
        SerialPrintString((uint8*)bdAddr2Str( pEvent->initDone.devAddr ));
#endif
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        //if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          //if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
          //                           pEvent->deviceInfo.pEvtData,
          //                           pEvent->deviceInfo.dataLen ) )
          if ( simpleBLEFindDeviceName(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen))
          {
            
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, ~(pEvent->deviceInfo.rssi)+1);
          }
        }
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;

        //simpleBLEScanRes = pEvent->discCmpl.numDevs;

/*
        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
*/
#if defined ( UART_DEBUG_MODE )
        SerialPrintValue("\r\nDevices Found", simpleBLEScanRes,10);
#endif

        if ( simpleBLEScanRes > 0 )
        {
#if defined ( UART_DEBUG_MODE )
          SerialPrintString("\r\nPress Left To Select");
          SerialPrintString("\r\nPress Middle To Connect");
#endif
        }

        // initialize scan index to last device
        //simpleBLEScanIdx = simpleBLEScanRes;

        // begin to wait some time and then di
        osal_set_event( simpleBLETaskId, SELECT_EVT);

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;

          // If service discovery not performed initiate service discovery
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }

#if defined ( UART_DEBUG_MODE )
          SerialPrintString("\r\nConnected: ");
          SerialPrintString((uint8*) bdAddr2Str( pEvent->linkCmpl.devAddr ));//SerialPrintString("\r\n");
#endif
          

        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;

#if defined ( UART_DEBUG_MODE )
          SerialPrintString("\r\nConnect Failed");
          SerialPrintValue(" Reason:",  pEvent->gap.hdr.status,10);
#endif
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
        simpleBLEProcedureInProgress = FALSE;

#if defined ( UART_DEBUG_MODE )
        SerialPrintString("\r\nDisconnected");
        SerialPrintValue(" Reason:",  pEvent->linkTerminate.reason,10);
#endif
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType, int8 rssi)
{
  uint8 i;

  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }

    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    simpleBLEDevList[simpleBLEScanRes].rssi = rssi;
    

    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
#if defined ( UART_DEBUG_MODE )
    SerialPrintString("\r\nPairing started");
#endif
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nPairing success");
#endif
    }
    else
    {
#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nPairing fail");
#endif
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nBonding success");
#endif
    }
    else
    {
#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nBonding fail");
#endif
      
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
  uint32  passcode;

  // Create random passcode
  //LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  //passcode %= 1000000;
  
  // using temp code
  passcode = 123456;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
#if defined ( UART_DEBUG_MODE )
    uint8 str[7];
    SerialPrintString("\r\nPasscode:");
    SerialPrintString((uint8 *) _ltoa(passcode, str, 10));
#endif
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;

  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;

  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }

    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  &&
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;

        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      simpleBLECharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                       pMsg->msg.readByTypeRsp.dataList[1] );

#if defined ( UART_DEBUG_MODE )
      SerialPrintString("\r\nSimple Svc Found");
#endif
      simpleBLEProcedureInProgress = FALSE;
    }

    simpleBLEDiscState = BLE_DISC_STATE_IDLE;


  }
}

/*********************************************************************
 * @fn      simpleBLEFindDeviceName
 *
 * @brief   check scan device's name is equal with default device name
 *
 * @return  bool, TRUE for equal and FALSE for unequal
 */
static bool simpleBLEFindDeviceName( uint8* pData, uint8 dataLen)
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;

  pEnd = pData + dataLen - 1;

  while ( pData < pEnd )
  {
    // Get length of name
    adLen = *pData++;
    if ( adLen > 0)
    {
      adType = *pData;
      if ( adType == GAP_ADTYPE_LOCAL_NAME_COMPLETE)
      {
        pData++;
        adLen--;
        while ( adLen > 0)
        {
          if (defaultDeviceName[defaultDeviceNameLength - adLen] != *pData++)
          {
            return FALSE;
          }
          adLen--;
        }
        return TRUE;
      }
      else
      {
        pData += adLen;
      }
    }
  }

  return FALSE;

}
/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;

  pEnd = pData + dataLen - 1;

  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;

      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;

        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }

          // Go to next
          pData += 2;
          adLen -= 2;
        }

        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }

  // Match not found
  return FALSE;
}


/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}

/*********************************************************************
*********************************************************************/
