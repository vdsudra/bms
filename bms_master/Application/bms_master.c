#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/UART.h>
#include <ti/common/cc26xx/uartlog/UartLog.h>
#include "bcomdef.h"
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "osal_list.h"
#include <ti_drivers_config.h>
#include "ti_ble_config.h"
#include "ble_user_config.h"
#include "simple_gatt_profile.h"
#include "bms_master.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define BMS_EVT_KEY_CHANGE          0x01
#define BMS_EVT_SCAN_ENABLED        0x02
#define BMS_EVT_SCAN_DISABLED       0x03
#define BMS_EVT_ADV_REPORT          0x04
#define BMS_EVT_SVC_DISC            0x05
#define BMS_EVT_READ_RSSI           0x06
#define BMS_EVT_PAIR_STATE          0x07
#define BMS_EVT_PASSCODE_NEEDED     0x08
#define BMS_EVT_READ_RPA            0x09
#define BMS_EVT_INSUFFICIENT_MEM    0x0A
#define BMS_EVT_DISCOVERY           0xFF
#define BMS_EVT_SEND_RQT            0xFE
#define BMS_EVT_SENT_UART           0xFD


// Bms Master Task Events
#define BMS_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define BMS_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define BMS_ALL_EVENTS                        (BMS_ICALL_EVT           | \
                                              BMS_QUEUE_EVT)

#define CLK_SEC 100000
// Default connection interval when connecting to more then 8 connections and autoconnenct enabled
#define DEFAULT_MULTICON_INTERVAL            200 //250 ms (200 frames of 1.25ms)

// Default connection supervision timeout when connnecting to more then 8 connections and autoconnenct enabled
#define DEFAULT_MULTICON_LSTO                3200 // 32 secs

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10
#define DELAY_SEC 1000

// Task configuration
#define BMS_TASK_PRIORITY                     1

#ifndef BMS_TASK_STACK_SIZE
#define BMS_TASK_STACK_SIZE                   1024
#endif

#define BMS_LOOP_STACK_SIZE                   512

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define BMS_ADDR_STR_SIZE     15

// Spin if the expression is not true
#define BMSMASTER_ASSERT(expr) if (!(expr)) BmsMaster_spin();

// Timeout for the initiator to cancel connection if not successful
#define CONNECTION_TIMEOUT                   3000

// Auto connect chosen group
#define GROUP_NAME_LENGTH                    5

//Member defalult status when initalized
#define GROUP_MEMBER_INITIALIZED             0x00

//Member connected
#define GROUP_MEMBER_CONNECTED               0x01

//Default connection handle which is set when group member is created
#define GROUP_INITIALIZED_CONNECTION_HANDLE  0xFFFF

static UART_Handle uart2Handle = NULL;

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

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr; // event header
  uint8_t *pData;  // event data
} bmsEvt_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

// Connected device information
typedef struct
{
  uint16_t connHandle;        // Connection Handle
  uint8_t  addr[B_ADDR_LEN];  // Peer Device Address
  uint8_t  charHandle;        // Characteristic Handle
  Clock_Struct *pRssiClock;   // pointer to clock struct
} connRec_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint16_t connHandle;
  uint8_t  status;
} bmsPairStateData_t;

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
} bmsPasscodeData_t;

typedef struct  
{
	osal_list_elem elem;
	uint8_t  addr[B_ADDR_LEN];  // member's BDADDR
	uint8_t  addrType;          // member's Address Type
	uint16_t connHandle;        // member's connection handle
	uint8_t  status;            // bitwise status flag 
} groupListElem_t;


uint8_t reqData[5][12] = {"Unknown","Voltage", "Current", "Temperature"};

struct Bms_Uart_Data
{
     uint8_t flag;
     uint8_t data[20];
};

struct Bms_Uart_Data bms_uart_data[MAX_NUM_BLE_CONNS] = {0};


/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct bmsTask;
Task_Struct bmsLoop;

#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(bmsTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t bmsTaskStack[BMS_TASK_STACK_SIZE];
uint8_t bmsLoopStack[BMS_TASK_STACK_SIZE];

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Number of connected devices
static uint8_t numConn = 0;

// List of connections
static connRec_t connList[MAX_NUM_BLE_CONNS];

// Connection handle of current connection
static uint16_t bmsConnHandle = LINKDB_CONNHANDLE_INVALID;

// Accept or reject L2CAP connection parameter update request
static bool acceptParamUpdateReq = true;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Maximum PDU size (default = 27 octets)
static uint16_t bmsMaxPduSize;

// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

//AutoConnect Group list
static osal_list_list groupList;

//AutoConnect ADV data filter according to local name short
static uint8_t acGroup[5] = {
  0x04,
  GAP_ADTYPE_LOCAL_NAME_SHORT,
  'B',
  'M',
  'S'
 };

//Number of group members found
static uint8_t numGroupMembers = 0;

//Connection in progress to avoid double initiate
static groupListElem_t *memberInProg;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Bms_Uart2_init();
static void BmsMaster_init(void);
static void BmsMaster_taskFxn(uintptr_t a0, uintptr_t a1);

static uint8_t BmsMaster_isMember(uint8_t *advData , uint8_t *groupName , uint8_t len);
static void BmsMaster_autoConnect(void);

//static void BmsMaster_handleKeys(uint8_t keys);
static uint8_t BmsMaster_processStackMsg(ICall_Hdr *pMsg);
static void BmsMaster_processGapMsg(gapEventHdr_t *pMsg);
static void BmsMaster_processGATTMsg(gattMsgEvent_t *pMsg);
static void BmsMaster_processAppMsg(bmsEvt_t *pMsg);
static void BmsMaster_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void BmsMaster_startSvcDiscovery(void);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static bool BmsMaster_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen);
static void BmsMaster_addScanInfo(uint8_t *pAddr, uint8_t addrType);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t BmsMaster_addConnInfo(uint16_t connHandle, uint8_t *pAddr);
static uint8_t BmsMaster_removeConnInfo(uint16_t connHandle);
static uint8_t BmsMaster_getConnIndex(uint16_t connHandle);
#undef Display_DISABLE_ALL
#ifndef Display_DISABLE_ALL
static char* BmsMaster_getConnAddrStr(uint16_t connHandle);
#endif
static void BmsMaster_processPairState(uint8_t state,
                                           bmsPairStateData_t* pPairStateData);
static void BmsMaster_processPasscode(bmsPasscodeData_t *pData);

static void BmsMaster_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static status_t BmsMaster_StartRssi();
static status_t BmsMaster_CancelRssi(uint16_t connHandle);

static void BmsMaster_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                     uint8_t uiInputs, uint8_t uiOutputs,
                                     uint32_t numComparison);
static void BmsMaster_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status);

static void BmsMaster_clockHandler(UArg arg);

static status_t BmsMaster_enqueueMsg(uint8_t event, uint8_t status,
                                        uint8_t *pData);

static void BmsMaster_scanCb(uint32_t evt, void* msg, uintptr_t arg);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Bond Manager Callbacks
static gapBondCBs_t bondMgrCBs =
{
  BmsMaster_passcodeCb, // Passcode callback
  BmsMaster_pairStateCb // Pairing/Bonding state Callback
};

static void Bms_Uart2_init(){
    //Nothing for now
    UART_Params uart2Params;

    UART_Params_init(&uart2Params);

    uart2Params.writeDataMode  = UART_DATA_BINARY;
    uart2Params.readDataMode   = UART_DATA_BINARY;
    uart2Params.readReturnMode = UART_RETURN_FULL;
    uart2Params.readMode       = UART_MODE_CALLBACK;
    uart2Params.writeMode      = UART_MODE_CALLBACK;
    //uart2Params.readCallback   = uartReadCallback;
    //uart2Params.writeCallback  = uartWriteCallback;
    uart2Params.readEcho       = UART_ECHO_OFF;
    uart2Params.baudRate       = 115200;

    //uart2Handle = UART_open(BMS_UART2, &uart2Params);

    if (uart2Handle == NULL) {
      // UART_open() failed
      while (1){
          Log_error0("Failed to open uart2");
      }
    }
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn		BmsMaster_isMember
 *
 * @brief	Check if Advertiser is part of the group according to its Adv Data
 *
 * @param	advData   - pointer to adv data
 *          groupNmae - group name which need to be compared with
 *          len       - length of the group name
 *
 * @return  TRUE: part of the group
 *          FALSE: not part of the group
 */

static uint8_t BmsMaster_isMember(uint8_t *advData , uint8_t *groupName , uint8_t len)
{
  if (osal_memcmp((uint8_t *)advData, (uint8_t *)groupName, len))
  {
    return TRUE;
  }
  return FALSE;
}

/*********************************************************************
 * @fn		BmsMaster_autoConnect
 *
 * @brief	Check if Advertiser is part of the group according to its Adv Data
 *
 * @param   none
 *
 * @return  none
 */

static void BmsMaster_autoConnect(void)
{
  status_t status;
  //Log_info0("BmsMaster_autoConnect() : Connect to device in groupListElem_t");
  if (memberInProg == NULL)
  {
    if (numConn < MAX_NUM_BLE_CONNS)
    {
	  groupListElem_t *tempMember = (groupListElem_t *)osal_list_head(&groupList);
      //If group member is not connected
      if ((tempMember != NULL) && (!(tempMember->status & GROUP_MEMBER_CONNECTED)))
      {
        //Initiate a connection
        Log_info1("GapInit_connect for Device : %s",(uintptr_t)Util_convertBdAddr2Str(tempMember->addr));
        status = GapInit_connect(tempMember->addrType & MASK_ADDRTYPE_ID,tempMember->addr, DEFAULT_INIT_PHY, CONNECTION_TIMEOUT);
        if (status != SUCCESS)
        {
          //Couldn't create connection remove element from list and free the memory.
          osal_list_remove(&groupList, (osal_list_elem *)tempMember);
          ICall_free(tempMember);
        }
    	else
    	{
          //Save pointer to connection in progress until connection is established.
		  memberInProg = tempMember;
        }
	  }
    }
    else
    {
        Log_info0("AutoConnect turned off: Max connection reached.");
    }
  }
}

/*********************************************************************
 * @fn      BmsMaster_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void BmsMaster_spin(void)
{
  volatile uint8_t x;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      BmsMaster_createTask
 *
 * @brief   Task creation function for the Bms Master.
 *
 * @param   none
 *
 * @return  none
 */
void BmsMaster_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = bmsTaskStack;
  taskParams.stackSize = BMS_TASK_STACK_SIZE;
  taskParams.priority = BMS_TASK_PRIORITY;

  Task_construct(&bmsTask, BmsMaster_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      BmsMaster_Init
 *
 * @brief   Initialization function for the Bms Master App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void BmsMaster_init(void)
{
  uint8_t i;

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.

  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Initialize internal data
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
    connList[i].pRssiClock = NULL;
  }

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                   (void *)attDeviceName);

  //Set default values for Data Length Extension
  //Extended Data Length Feature is already enabled by default
  //in build_config.opt in stack project.
  {
    //Change initial values of RX/TX PDU and Time, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_RX_PDU_SIZE 251     //default is 251 octets(RX)
    #define APP_SUGGESTED_RX_TIME     17000   //default is 17000us(RX)
    #define APP_SUGGESTED_TX_PDU_SIZE 27      //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME     328     //default is 328us(TX)

    //This API is documented in hci.h
    //See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    //http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_EXT_SetMaxDataLenCmd(APP_SUGGESTED_TX_PDU_SIZE, APP_SUGGESTED_TX_TIME, APP_SUGGESTED_RX_PDU_SIZE, APP_SUGGESTED_RX_TIME);
  }

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set Bond Manager parameters
  setBondManagerParameters();

  // Start Bond Manager and register callback
  // This must be done before initialing the GAP layer
  VOID GAPBondMgr_Register(&bondMgrCBs);

  // Accept all parameter update requests
  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);

  // Register with GAP for HCI/Host messages (for RSSI)
  GAP_RegisterForMsgs(selfEntity);

  // Initialize GAP layer for Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_CENTRAL, selfEntity, addrMode, NULL);

    //Start discovery after 15 sec delay, BmsMaster_clockHandler
   Util_constructClock(&startDiscClock, BmsMaster_clockHandler, DELAY_SEC * 15, 0, false, BMS_EVT_DISCOVERY);

   //UART2 initialization   for actual data transfer to host
   //Bms_Uart2_init();
}

/*********************************************************************
 * @fn      BmsMaster_taskFxn
 *
 * @brief   Application task entry point for the Bms Master.
 *
 * @param   none
 *
 * @return  events not processed
 */

static void BmsLoop(uintptr_t a0, uintptr_t a1){
    uint8_t i;

    // Set Auto connect Group A mode
    BmsMaster_doAutoConnect(1);
    while(1){
        Log_info0("Scanning to get data..!");
        Log_info1("No of connected devices --> %d ", numConn);
        for(i=0; i < MAX_NUM_BLE_CONNS; i++){
            if(connList[i].connHandle != LINKDB_CONNHANDLE_INVALID && !(connList[i].charHandle)){
                BmsMaster_doSelectConn(i);
            }
            if(connList[i].connHandle != LINKDB_CONNHANDLE_INVALID && (connList[i].charHandle)){
                BmsMaster_enqueueMsg(BMS_EVT_SEND_RQT, i, NULL);
                }
        } // for connected
        Task_sleep(CLK_SEC * 10);
        //Send data to Uart
        BmsMaster_enqueueMsg(BMS_EVT_SENT_UART, 0, NULL);
        Task_sleep(CLK_SEC * 10);
        } // While infinite
    }
static void BmsMaster_taskFxn(uintptr_t a0, uintptr_t a1)
{
    // Initialize application
    BmsMaster_init();
    Util_startClock(&startDiscClock);
    Task_Params bmsLoopParams;
    // Configure task
    Task_Params_init(&bmsLoopParams);
    bmsLoopParams.stack = bmsLoopStack;
    bmsLoopParams.stackSize = BMS_LOOP_STACK_SIZE;
    bmsLoopParams.priority = BMS_TASK_PRIORITY;
    Task_construct(&bmsLoop, BmsLoop, &bmsLoopParams, NULL);
    Log_info0("*********************************************");
    Log_info0("*             Starting BMS Master           *");
    Log_info0("*********************************************");
   // Application main loop
    for (;;)
    {
    uint32_t events;

    events = Event_pend(syncEvent, Event_Id_NONE, BMS_ALL_EVENTS, ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = BmsMaster_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message
      if (events & BMS_QUEUE_EVT)
      {
        bmsEvt_t *pMsg;
        while (pMsg = (bmsEvt_t *)Util_dequeueMsg(appMsgQueue))
        {
          // Process message
          BmsMaster_processAppMsg(pMsg);

          // Free the space from the message
          ICall_free(pMsg);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      BmsMaster_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t BmsMaster_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      BmsMaster_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      BmsMaster_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          BmsMaster_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;

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
                  if (pMyMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {
                      Log_info0("PHY Change failure, peer does not support this");
                  }
                  else
                  {
                      Log_info1("PHY Update Status: 0x%02x", pMyMsg->cmdStatus);
                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {
                    Log_error2("Unknown Cmd Status: 0x%04x::0x%02x", pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
                }
              break;
            }
          }
          break;

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC
            = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {
                Log_error1("%s: PHY change failure", (uintptr_t)BmsMaster_getConnAddrStr(pPUC->connHandle));
            }
            else
            {
                Log_info2("%s: PHY updated to %s", (uintptr_t)BmsMaster_getConnAddrStr(pPUC->connHandle),
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
                                (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? (uintptr_t)"1 Mbps" :
                                (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? (uintptr_t)"2 Mbps" :
                                (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? (uintptr_t)"CODED" : (uintptr_t)"Unexpected PHY Value");
            }
          }

          break;
        }

        default:
          break;
      }

      break;
    }

    case L2CAP_SIGNAL_EVENT:
      // place holder for L2CAP Connection Parameter Reply
      break;

    default:
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      BmsMaster_processAppMsg
 *
 * @brief   Scanner application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void BmsMaster_processAppMsg(bmsEvt_t *pMsg)
{
  bool safeToDealloc = TRUE;
  switch (pMsg->hdr.event)
  {
    case BMS_EVT_KEY_CHANGE:
      break;

    case BMS_EVT_ADV_REPORT:
    {
       //Log_info0("SC_EVT_ADV_REPORT");
       GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);
      //Auto connect is enabled
      if (autoConnect) 
      {
        if (numGroupMembers == MAX_NUM_BLE_CONNS)
        {
          GapScan_disable();
          break;
		}
        //Check if advertiser is part of the group
        if (BmsMaster_isMember(pAdvRpt->pData , acGroup, GROUP_NAME_LENGTH))
	    {
     	  groupListElem_t *tempMember;
     	  //Traverse list to search if advertiser already in list.
  	      for (tempMember = (groupListElem_t *)osal_list_head(&groupList); tempMember != NULL; tempMember = (groupListElem_t *)osal_list_next((osal_list_elem *)tempMember)) 
  	      {
            if (osal_memcmp((uint8_t *)tempMember->addr ,(uint8_t *)pAdvRpt->addr,B_ADDR_LEN))
            {
              break;
            }
          }
          //If tempMemer is NULL this meams advertiser not in list.
          if (tempMember == NULL)
          {
            groupListElem_t *groupMember = (groupListElem_t *)ICall_malloc(sizeof(groupListElem_t));
            if (groupMember != NULL)
            {
              //Copy member's details into Member's list.
              osal_memcpy((uint8_t *)groupMember->addr , (uint8_t *)pAdvRpt->addr,B_ADDR_LEN);
              groupMember->addrType = pAdvRpt->addrType;
              groupMember->status = GROUP_MEMBER_INITIALIZED;
              groupMember->connHandle = GROUP_INITIALIZED_CONNECTION_HANDLE;
              //Add group member into list.
              osal_list_putHead(&groupList,(osal_list_elem *)groupMember);
			  numGroupMembers++;
            }
            else
            {
                Log_error0("AutoConnect: Allocation failed!");
              break;
            }
          }
        }
      }
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      //if (BmsMaster_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
          if (BmsMaster_findSvcUuid(0xEDE0,
                                    pAdvRpt->pData, pAdvRpt->dataLen))
      {
        BmsMaster_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);
        Log_info1("Discovered: %s",
                       Util_convertBdAddr2Str(pAdvRpt->addr));
      }`
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      Log_info1("Discovered: %s", (uintptr_t) Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      // Free report payload data
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      break;
    }

    case BMS_EVT_SCAN_ENABLED:
      Log_info0("BMS_EVT_SCAN_ENABLED");
      Log_info0("Discovering...");
      break;

    case BMS_EVT_SCAN_DISABLED:
    {
      Log_info0("BMS_EVT_SCAN_DISABLED");

      if (autoConnect)
      {
 	    Log_info1("AutoConnect: Number of members in the group %d",numGroupMembers);
 	    BmsMaster_autoConnect();
	  }
      else
      {
	      uint8_t numReport;
	      uint8_t i;
	      static uint8_t* pAddrs = NULL;
	      uint8_t* pAddrTemp;
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
	      numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
	      GapScan_Evt_AdvRpt_t advRpt;

	      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

	      Log_info1("%d devices discovered", numReport);

	      // Allocate buffer to display addresses
	      if (pAddrs != NULL)
	      {
	        // A scan has been done previously, release the previously allocated buffer
	        ICall_free(pAddrs);
	      }
	      pAddrs = ICall_malloc(numReport * BMS_ADDR_STR_SIZE);
	      if (pAddrs == NULL)
	      {
	        numReport = 0;
	      }

	      if (pAddrs != NULL)
	      {
	        pAddrTemp = pAddrs;
	        for (i = 0; i < numReport; i++, pAddrTemp += BMS_ADDR_STR_SIZE)
	        {
	  #if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
	          // Get the address from the list, convert it to string, and
	          // copy the string to the address buffer
	          memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
	                 BMS_ADDR_STR_SIZE);
	  #else // !DEFAULT_DEV_DISC_BY_SVC_UUID
	          // Get the address from the report, convert it to string, and
	          // copy the string to the address buffer
	          GapScan_getAdvReport(i, &advRpt);
	          memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
	                 BMS_ADDR_STR_SIZE);
	  #endif // DEFAULT_DEV_DISC_BY_SVC_UUID
        }
	        // Note: pAddrs is not freed since it will be used by the two button menu
	        // to display the discovered address.
	        // This implies that at least the last discovered addresses
	        // will be maintained until a new scan is done.
	      }
	      break;
	    }
    }
    case BMS_EVT_SVC_DISC:
        Log_info0("BMS_EVT_SVC_DISC");
        BmsMaster_startSvcDiscovery();
      break;

    case BMS_EVT_READ_RSSI:
    {

      uint8_t connIndex = pMsg->hdr.state;
      uint16_t connHandle = connList[connIndex].connHandle;

      // If link is still valid
      if (connHandle != LINKDB_CONNHANDLE_INVALID)
      {
        // Restart timer
        Util_startClock(connList[connIndex].pRssiClock);

        // Read RSSI
        VOID HCI_ReadRssiCmd(connHandle);
      }

      break;
    }

    // Pairing event
    case BMS_EVT_PAIR_STATE:
    {
        Log_info0("BMS_EVT_PAIR_STATE");
      BmsMaster_processPairState(pMsg->hdr.state,
                                     (bmsPairStateData_t*) (pMsg->pData));
      break;
    }

    // Passcode event
    case BMS_EVT_PASSCODE_NEEDED:
    {
        Log_info0("BMS_EVT_PASSCODE_NEEDED");
      BmsMaster_processPasscode((bmsPasscodeData_t *)(pMsg->pData));
      break;
    }

    case BMS_EVT_READ_RPA:
    {
      uint8_t* pRpaNew;

      // Read the current RPA.
      pRpaNew = GAP_GetDevAddress(FALSE);

      if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
      {
        // If the RPA has changed, update the display
          Log_info1("RP Addr: %s", (uintptr_t)Util_convertBdAddr2Str(pRpaNew));
          memcpy(rpa, pRpaNew, B_ADDR_LEN);
      }
      break;
    }

    // Insufficient memory
    case BMS_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.
      Log_error0("Insufficient Memory");
      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }

    case BMS_EVT_DISCOVERY:
    {
        if(numConn < MAX_NUM_BLE_CONNS){
        Log_info0("Auto discovery & Connect : BMS_EVT_DISCOVERY");
        // If connected then delay scanning by 120 sec otherwise scan periodicaly
        if(numConn){
            Util_rescheduleClock(&startDiscClock,DELAY_SEC * 120);
            Log_info0("Setting time for auto discovery 120");
            Util_startClock(&startDiscClock);
        }
        else{
            Util_rescheduleClock(&startDiscClock,DELAY_SEC * 30);
            Log_info0("Setting time for auto discovery 30");
            Util_startClock(&startDiscClock);
        }

        BmsMaster_doDiscoverDevices(0);
        }
        else
        {
            Util_stopClock(&startDiscClock);
        }
      break;
    }

    case BMS_EVT_SEND_RQT:
    {
        Log_info1("BMS_EVT_SEND_RQT : Sending GATT Read request to [%d] device.",pMsg->hdr.state);
        attReadReq_t req;
        req.handle = connList[pMsg->hdr.state].charHandle;
        GATT_ReadCharValue(connList[pMsg->hdr.state].connHandle, &req, selfEntity);
    }
    break;

    case BMS_EVT_SENT_UART:
    {
        Log_info0("Request received to flush data on UART port");
        uint8_t i;
        for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
        {
            if(bms_uart_data[i].flag)
            {
                Log_info1("{%s}",(uintptr_t)bms_uart_data[i].data);
            }
        }
    }

    default:
      // Do nothing.
      break;
  }

  if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      BmsMaster_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void BmsMaster_processGapMsg(gapEventHdr_t *pMsg){

  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      Log_info0("GAP_DEVICE_INIT_DONE_EVENT");
      uint8_t temp8;
      uint16_t temp16;
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      // Setup scanning
      // For more information, see the GAP section in the User's Guide:
      // http://software-dl.ti.com/lprf/ble5stack-latest/

      // Register callback to process Scanner events
      GapScan_registerCb(BmsMaster_scanCb, NULL);

      // Set Scanner Event Mask
      GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                           GAP_EVT_ADV_REPORT);

      // Set Scan PHY parameters
      GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_PASSIVE,
                           DEFAULT_SCAN_INTERVAL, DEFAULT_SCAN_WINDOW);

      // Set Advertising report fields to keep
      temp16 = ADV_RPT_FIELDS;
      GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
      // Set Scanning Primary PHY
      temp8 = DEFAULT_SCAN_PHY;
      GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
      // Set LL Duplicate Filter
      temp8 = SCAN_FLT_DUP_ENABLE;
      GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

      // Set PDU type filter -
      // Only 'Connectable' and 'Complete' packets are desired.
      // It doesn't matter if received packets are
      // whether Scannable or Non-Scannable, whether Directed or Undirected,
      // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
      temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
      GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);

	  // Set initiating PHY parameters
      GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MIN,
						  INIT_PHYPARAM_MIN_CONN_INT);
	  GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MAX,
						  INIT_PHYPARAM_MAX_CONN_INT);

      bmsMaxPduSize = pPkt->dataPktLen;
      Log_info0("Initialized");
      Log_info1("Num Conns: %d", numConn);

      // Display device address
      Log_info2("%s Addr: %s",
                (addrMode <= ADDRMODE_RANDOM) ? (uintptr_t)"Dev" : (uintptr_t)"ID",
                        (uintptr_t)Util_convertBdAddr2Str(pPkt->devAddr));

      if (addrMode > ADDRMODE_RANDOM)
      {
        // Update the current RPA.
        memcpy(rpa, GAP_GetDevAddress(FALSE), B_ADDR_LEN);

        Log_info1("RP Addr: %s", (uintptr_t)Util_convertBdAddr2Str(rpa));

        // Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, BmsMaster_clockHandler,
                            READ_RPA_PERIOD, 0, true, BMS_EVT_READ_RPA);
      }
      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {

	  if (autoConnect)
      {
        if (memberInProg != NULL)
		{
          //Remove node from member's group and free its memory.
          osal_list_remove(&groupList, (osal_list_elem *)memberInProg);
          ICall_free(memberInProg);  
  		  numGroupMembers--;
          memberInProg = NULL;
	    }
        Log_info1("AutoConnect: Number of members in the group %d",numGroupMembers);
		//Keep on connecting to the remaining members in the list
		BmsMaster_autoConnect();
      }

      Log_info0("Conneting attempt cancelled");
      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      Log_info0("GAP_LINK_ESTABLISHED_EVENT");
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      uint8_t* pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      if (autoConnect)
      {
        if (memberInProg != NULL)
		{
  		  if (osal_memcmp((uint8_t *)pAddr, (uint8_t *)memberInProg->addr, B_ADDR_LEN));
  		  {
            //Move the connected member to the tail of the list.
            osal_list_remove(&groupList,(osal_list_elem *)memberInProg);
            osal_list_put(&groupList,(osal_list_elem *)memberInProg);
            //Set the connected bit.;
  		    memberInProg->status |= GROUP_MEMBER_CONNECTED;
            //Store the connection handle.
            memberInProg->connHandle = connHandle;
  		    memberInProg = NULL;
  		  }
  	    }
	  }
      uint8_t  connIndex;
      uint8_t* pStrAddr;
      uint8_t pairMode = 0;

      // Add this connection info to the list
      connIndex = BmsMaster_addConnInfo(connHandle, pAddr);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      connList[connIndex].charHandle = 0;

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Log_info1("Connected to %s", (uintptr_t)pStrAddr);
      Log_info1("Num Conns: %d", numConn);


      GAPBondMgr_GetParameter(GAPBOND_PAIRING_MODE, &pairMode);
      if ((autoConnect) && (pairMode != GAPBOND_PAIRING_MODE_INITIATE))
      {
		    BmsMaster_autoConnect();
      }
      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      Log_info0("GAP_LINK_TERMINATED_EVENT");
	  uint8_t connIndex;
      uint8_t* pStrAddr;
      uint16_t connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      if (autoConnect)
      {
        groupListElem_t *tempMember;
        //Traverse from tail to head because of the sorting which put the connected at the end of the list.
		for (tempMember = (groupListElem_t *)osal_list_tail(&groupList); tempMember != NULL; tempMember = (groupListElem_t *)osal_list_prev((osal_list_elem *)tempMember)) 
        {
          if (tempMember->connHandle == connHandle)
          {
            //Move disconnected member to the head of the list for next connection.
            osal_list_remove(&groupList,(osal_list_elem *)tempMember);
            osal_list_putHead(&groupList,(osal_list_elem *)tempMember);
            //Clear the connected flag.
            tempMember->status &= ~GROUP_MEMBER_CONNECTED;
            //Clear the connnection handle.
            tempMember->connHandle = GROUP_INITIALIZED_CONNECTION_HANDLE;   
          }
        }
      }
      // Cancel timers
      BmsMaster_CancelRssi(connHandle);
	  
	  // Mark this connection deleted in the connected device list.
      connIndex = BmsMaster_removeConnInfo(connHandle);

      if (autoConnect)
      {
	    BmsMaster_autoConnect();
      }
      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Log_info1("%s is disconnected",(uintptr_t)pStrAddr);
      Log_info1("Num Conns: %d", numConn);
      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      Log_info0("GAP_UPDATE_LINK_PARAM_REQ_EVENT");
      gapUpdateLinkParamReqReply_t rsp;
      gapUpdateLinkParamReq_t *pReq;

      pReq = &((gapUpdateLinkParamReqEvent_t *)pMsg)->req;

      rsp.connectionHandle = pReq->connectionHandle;
      rsp.signalIdentifier = pReq->signalIdentifier;

      if (acceptParamUpdateReq)
      {
        rsp.intervalMin = pReq->intervalMin;
        rsp.intervalMax = pReq->intervalMax;
        rsp.connLatency = pReq->connLatency;
        rsp.connTimeout = pReq->connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        // Reject the request.
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

	  if (autoConnect)
      {
        BmsMaster_autoConnect();
      }

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      Log_info0("GAP_LINK_PARAM_UPDATE_EVENT");
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
      // Get the address from the connection handle
      linkDBInfo_t linkInfo;

      if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
      {
        if(pPkt->status == SUCCESS)
        {
            Log_info2("Updated: %s, connTimeout:%d",(uintptr_t)Util_convertBdAddr2Str(linkInfo.addr),linkInfo.connTimeout*CONN_TIMEOUT_MS_CONVERSION);
        }
        else
        {
          // Display the address of the connection update failure
          Log_error2("Update Failed 0x%h: %s", pPkt->opcode, (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));
        }
      }
      
      if (autoConnect)
      {
        BmsMaster_autoConnect();
      }

      break;
    }

    default:
      break;
  }
}

/*********************************************************************
 * @fn      BmsMaster_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  none
 */
static void BmsMaster_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (linkDB_Up(pMsg->connHandle))
  {
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
      // No HCI buffer was available. App can try to retransmit the response
      // on the next connection event. Drop it for now.
        Log_info1("ATT Rsp dropped %d", pMsg->method);
    }
    else if ((pMsg->method == ATT_READ_RSP)   ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
          Log_error1("Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        Log_info1("Get %s",(uintptr_t)pMsg->msg.readRsp.pValue);
        uint8_t i;

        for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
          {
            if (connList[i].connHandle != LINKDB_CONNHANDLE_INVALID && pMsg->connHandle == connList[i].connHandle)
            {
                memcpy(bms_uart_data[i].data, pMsg->msg.readRsp.pValue, 20);
            }
          }
      }
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
          Log_error1("Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful write, display the value that was written and increment value
        Log_info1("set request for %s", (uintptr_t)reqData[charVal]);
      }
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
      // ATT request-response or indication-confirmation flow control is
      // violated. All subsequent ATT requests or indications will be dropped.
      // The app is informed in case it wants to drop the connection.

      // Display the opcode of the message that caused the violation.
      Log_info1("FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
      // MTU size updated
      Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      BmsMaster_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.

  // Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      BmsMaster_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BmsMaster_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      uint16_t connHandle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);
      int8 rssi = (int8)pMsg->pReturnParam[3];
      Log_info2("%s: RSSI %d dBm",(uintptr_t)BmsMaster_getConnAddrStr(connHandle), rssi);
      break;
    }
    default:
      break;
  }
}

/*********************************************************************
 * @fn      BmsMaster_StartRssi
 *
 * @brief   Start periodic RSSI reads on the current link.
 *
 * @return  SUCCESS: RSSI Read timer started
 *          bleIncorrectMode: Aready started
 *          bleNoResources: No resources
 */
static status_t BmsMaster_StartRssi(void)
{
  uint8_t connIndex = BmsMaster_getConnIndex(bmsConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // If already running
  if (connList[connIndex].pRssiClock != NULL)
  {
    return bleIncorrectMode;
  }

  // Create a clock object and start
  connList[connIndex].pRssiClock
    = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

  if (connList[connIndex].pRssiClock)
  {
    Util_constructClock(connList[connIndex].pRssiClock,
                        BmsMaster_clockHandler,
                        DEFAULT_RSSI_PERIOD, 0, true,
                        (connIndex << 8) | BMS_EVT_READ_RSSI);
  }
  else
  {
    return bleNoResources;
  }

  return SUCCESS;
}

/*********************************************************************
 * @fn      BmsMaster_CancelRssi
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connection handle
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: Has not started
 */
static status_t BmsMaster_CancelRssi(uint16_t connHandle)
{
  uint8_t connIndex = BmsMaster_getConnIndex(connHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // If already running
  if (connList[connIndex].pRssiClock == NULL)
  {
    return bleIncorrectMode;
  }

  // Stop timer
  Util_stopClock(connList[connIndex].pRssiClock);

  // Destroy the clock object
  Clock_destruct(connList[connIndex].pRssiClock);

  // Free clock struct
  ICall_free(connList[connIndex].pRssiClock);
  connList[connIndex].pRssiClock = NULL;

  return SUCCESS;
}

/*********************************************************************
 * @fn      BmsMaster_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void BmsMaster_processPairState(uint8_t state,
                                           bmsPairStateData_t* pPairData)
{
  uint8_t status = pPairData->status;
  uint8_t pairMode = 0;

  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
      Log_info0("Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      linkDBInfo_t linkInfo;
      Log_info0("Pairing success");

      if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
      {
        // If the peer was using private address, update with ID address
        if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
             linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
             !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))
        {
          // Update the address of the peer to the ID address
          Log_info1("Addr updated: %s", (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));

          // Update the connection list with the ID address
          uint8_t i = BmsMaster_getConnIndex(pPairData->connHandle);

          BMSMASTER_ASSERT(i < MAX_NUM_BLE_CONNS);
          memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
        }
      }
    }
    else
    {
        Log_info1("Pairing fail: %d", status);
    }

    GAPBondMgr_GetParameter(GAPBOND_PAIRING_MODE, &pairMode);

    if ((autoConnect) && (pairMode == GAPBOND_PAIRING_MODE_INITIATE))
    {
	  BmsMaster_autoConnect();
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_ENCRYPTED)
  {
    if (status == SUCCESS)
    {
        Log_info0("Encryption success");
    }
    else
    {
        Log_error1("Encryption failed: %d", status);
    }

    GAPBondMgr_GetParameter(GAPBOND_PAIRING_MODE, &pairMode);

    if ((autoConnect) && (pairMode == GAPBOND_PAIRING_MODE_INITIATE))
    {
      BmsMaster_autoConnect();
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (status == SUCCESS)
    {
        Log_info0("Bond save success");
    }
    else
    {
        Log_error1("Bond save failed: %d", status);
    }
  }
}

/*********************************************************************
 * @fn      BmsMaster_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void BmsMaster_processPasscode(bmsPasscodeData_t *pData)
{
  // Display passcode to user
  if (pData->uiOutputs != 0)
  {
      Log_info1("Passcode: %d", B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      BmsMaster_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void BmsMaster_startSvcDiscovery(void)
{
  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  discState = BLE_DISC_STATE_MTU;

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = bmsMaxPduSize - L2CAP_HDR_SIZE;

  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(bmsConnHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      BmsMaster_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void BmsMaster_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  if (discState == BLE_DISC_STATE_MTU)
  {
    Log_info0("BLE_DISC_STATE_MTU");
    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
        /*
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
*/
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(0xEDE0), HI_UINT16(0xEDE0)};
      discState = BLE_DISC_STATE_SVC;

      // Discovery simple service
      VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid, ATT_BT_UUID_SIZE, selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    Log_info0("BLE_DISC_STATE_SVC");
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        //req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        //req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[0] = LO_UINT16(0xEDE1);
        req.type.uuid[1] = HI_UINT16(0xEDE1);

        VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    Log_info0("BLE_DISC_STATE_CHAR : Characteristic found, store handle");
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
        uint8_t connIndex = BmsMaster_getConnIndex(bmsConnHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

 	  // Store the handle of the simpleprofile characteristic 1 value
      connList[connIndex].charHandle
        = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                       pMsg->msg.readByTypeRsp.pDataList[4]);
    }
    else{
        Log_info0("Characteristic not found..");
    }

    discState = BLE_DISC_STATE_IDLE;
  }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      BmsMaster_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool BmsMaster_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  if (dataLen > 0)
  {
    pEnd = pData + dataLen - 1;

    // While end of data not reached
    while (pData < pEnd)
    {
      // Get length of next AD item
      adLen = *pData++;
      if (adLen > 0)
      {
        adType = *pData;

        // If AD type is for 16-bit service UUID
        if ((adType == GAP_ADTYPE_16BIT_MORE) ||
            (adType == GAP_ADTYPE_16BIT_COMPLETE))
        {
          pData++;
          adLen--;

          // For each UUID in list
          while (adLen >= 2 && pData < pEnd)
          {
            // Check for match
            if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
            {
              // Match found
              return TRUE;
            }

            // Go to next
            pData += 2;
            adLen -= 2;
          }

          // Handle possible erroneous extra byte in UUID list
          if (adLen == 1)
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
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      BmsMaster_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void BmsMaster_addScanInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;

  // If result count not at max
  if (numScanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < numScanRes; i++)
    {
      if (memcmp(pAddr, scanList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
    scanList[numScanRes].addrType = addrType;

    // Increment scan result count
    numScanRes++;
  }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      BmsMaster_addConnInfo
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t BmsMaster_addConnInfo(uint16_t connHandle, uint8_t *pAddr)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;
      memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
      numConn++;
      uint8_t dummy[20] = "Initial Dummy Data";
      memcpy(bms_uart_data[i].data, dummy,20);
      bms_uart_data[i].flag = 1;
      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      BmsMaster_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t BmsMaster_removeConnInfo(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      // Found the entry to mark as deleted
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      bms_uart_data[i].flag = 0;
      numConn--;
      if(!Util_isActive(&startDiscClock)){
          Util_startClock(&startDiscClock);
      }
      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      BmsMaster_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t BmsMaster_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      BmsMaster_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* BmsMaster_getConnAddrStr(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}

/*********************************************************************
 * @fn      BmsMaster_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void BmsMaster_pairStateCb(uint16_t connHandle, uint8_t state,
                                      uint8_t status)
{
  bmsPairStateData_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(bmsPairStateData_t))))
  {
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(BmsMaster_enqueueMsg(BMS_EVT_PAIR_STATE, state, (uint8_t*) pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
* @fn      BmsMaster_passcodeCb
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void BmsMaster_passcodeCb(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
  bmsPasscodeData_t *pData = ICall_malloc(sizeof(bmsPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData)
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if (BmsMaster_enqueueMsg(BMS_EVT_PASSCODE_NEEDED, 0,(uint8_t *) pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      BmsMaster_clockHandler
 *
 * @brief   clock handler function
 *
 * @param   arg - argument from the clock initiator
 *
 * @return  none
 */

void BmsMaster_clockHandler(UArg arg)
{
  uint8_t evtId = (uint8_t) (arg & 0xFF);

  switch (evtId)
  {
    case BMS_EVT_READ_RSSI:
      BmsMaster_enqueueMsg(BMS_EVT_READ_RSSI, (uint8_t) (arg >> 8) , NULL);
      break;

    case BMS_EVT_READ_RPA:
      // Restart timer
      Util_startClock(&clkRpaRead);
      // Let the application handle the event
      BmsMaster_enqueueMsg(BMS_EVT_READ_RPA, 0, NULL);
      break;

    case BMS_EVT_DISCOVERY:
        Log_info0("BMS_EVT_DISCOVERY start...");
        BmsMaster_enqueueMsg(BMS_EVT_DISCOVERY, 0, NULL);
    break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      BmsMaster_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t BmsMaster_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  uint8_t success;
  bmsEvt_t *pMsg = ICall_malloc(sizeof(bmsEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      BmsMaster_scanCb
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void BmsMaster_scanCb(uint32_t evt, void* pMsg, uintptr_t arg)
{
  uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = BMS_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = BMS_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = BMS_EVT_SCAN_DISABLED;
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    event = BMS_EVT_INSUFFICIENT_MEM;
  }
  else
  {
    return;
  }

  if(BmsMaster_enqueueMsg(event, SUCCESS, pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }
}

/*********************************************************************
 * @fn      BmsMaster_doAutoConnect
 *
 * @brief   Enable/Disable AutoConnect.
 *
 * @param   index - 0 : Disable AutoConnect
 *                  1 : Enable Group A
 *                  2 : Enable Group B
 *
 * @return  always true
 */
bool BmsMaster_doAutoConnect(uint8_t index)
{
    if (index == 1)
    {
      if ((autoConnect) && (autoConnect != AUTOCONNECT_GROUP_A))
      {
        groupListElem_t *tempMember;
        //Traverse list to search if advertiser already in list.
        for (tempMember = (groupListElem_t *)osal_list_head(&groupList); tempMember != NULL; tempMember = (groupListElem_t *)osal_list_next((osal_list_elem *)tempMember)) 
        {
          osal_list_remove(&groupList,(osal_list_elem *)tempMember);
          ICall_free(tempMember);
        }
		numGroupMembers = 0;
      }	
      Log_info0("AutoConnect enabled: Group BMS");
      autoConnect = AUTOCONNECT_GROUP_A;
    }
    else if (index == 2)
    {
      if ((autoConnect) && (autoConnect != AUTOCONNECT_GROUP_B))
      {
        groupListElem_t *tempMember;
        //Traverse list to search if advertiser already in list.
        for (tempMember = (groupListElem_t *)osal_list_head(&groupList); tempMember != NULL; tempMember = (groupListElem_t *)osal_list_next((osal_list_elem *)tempMember)) 
        {
          osal_list_remove(&groupList,(osal_list_elem *)tempMember);
          ICall_free(tempMember);
        }
		numGroupMembers = 0;
      }
      Log_info0("AutoConnect enabled: Group BMS");
      autoConnect = AUTOCONNECT_GROUP_B;
    }
    else
    {
      autoConnect = AUTOCONNECT_DISABLE;
      groupListElem_t *tempMember;
      //Traverse list to search if advertiser already in list.
      for (tempMember = (groupListElem_t *)osal_list_head(&groupList); tempMember != NULL; tempMember = (groupListElem_t *)osal_list_next((osal_list_elem *)tempMember)) 
      {
        osal_list_remove(&groupList,(osal_list_elem *)tempMember);
        ICall_free(tempMember);
      }
	  numGroupMembers = 0;
	  Log_info0("AutoConnect disabled");
    }
    if ((autoConnect) && (MAX_NUM_BLE_CONNS > 8))
    {
	  //Disable accepting L2CAP param upadte request
      acceptParamUpdateReq = false;
	  //Disable all parameter update requests
	  GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_DENY_ALL);
	  //Set connection interval and supervision timeout
      GapInit_setPhyParam(INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED,INIT_PHYPARAM_CONN_INT_MAX,DEFAULT_MULTICON_INTERVAL);
      GapInit_setPhyParam(INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED,INIT_PHYPARAM_CONN_INT_MIN,DEFAULT_MULTICON_INTERVAL);
	  GapInit_setPhyParam(INIT_PHY_1M | INIT_PHY_2M | INIT_PHY_CODED,INIT_PHYPARAM_SUP_TIMEOUT,DEFAULT_MULTICON_LSTO);
    }
    
    return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doSetScanPhy
 *
 * @brief   Set PHYs for scanning.
 *
 * @param   index - 0: 1M PHY
 *                  1: CODED PHY (Long range)
 *
 * @return  always true
 */
bool BmsMaster_doSetScanPhy(uint8_t index)
{
  uint8_t temp8;

  if (index == 0)
  {
    temp8 = SCAN_PRIM_PHY_1M;
  }
  else
  {
    temp8 = SCAN_PRIM_PHY_CODED;
  }

  // Set scanning primary PHY
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doDiscoverDevices
 *
 * @brief   Enables scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doDiscoverDevices(uint8_t index)
{
  (void) index;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.
  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doStopDiscovering
 *
 * @brief   Stop on-going scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doStopDiscovering(uint8_t index)
{
  (void) index;

  GapScan_disable();

  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doEstablishLink
 *
 * @brief   Establish a link to a peer device
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doConnect(uint8_t index)
{
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                  scanList[index].addr, DEFAULT_INIT_PHY, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);

  GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                  advRpt.addr, DEFAULT_INIT_PHY, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  Log_info0("Connecting...");
  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doCancelConnecting(uint8_t index)
{
  (void) index;

  GapInit_cancelConnect();

  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doSelectConn(uint8_t index)
{
  Log_info1("BmsMaster_doSelectConn() start for to connect device %d for read request", index);
  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  BMSMASTER_ASSERT(index < MAX_NUM_BLE_CONNS);

  bmsConnHandle = connList[index].connHandle;

  if (connList[index].charHandle == 0)
  {
    // Initiate service discovery
    BmsMaster_enqueueMsg(BMS_EVT_SVC_DISC, 0, NULL);
  }

  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doGattRead
 *
 * @brief   GATT Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doGattRead(uint8_t index)
{
  attReadReq_t req;
  uint8_t connIndex = BmsMaster_getConnIndex(bmsConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  req.handle = connList[connIndex].charHandle;
  GATT_ReadCharValue(bmsConnHandle, &req, selfEntity);

  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doGattWrite
 *
 * @brief   GATT Write
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doGattWrite(uint8_t index)
{
  status_t status;
  uint8_t charVals[4] = { 1, 2, 3,0}; // Should be consistent with those in bmsMenuGattWrite

  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(bmsConnHandle, ATT_WRITE_REQ, 1, NULL);

  if ( req.pValue != NULL )
  {
    uint8_t connIndex = BmsMaster_getConnIndex(bmsConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    BMSMASTER_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle;
    req.len = 1;
    charVal = charVals[index];
    req.pValue[0] = charVal;
    req.sig = 0;
    req.cmd = 0;
    status = GATT_WriteCharValue(bmsConnHandle, &req, selfEntity);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }

  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doRssiRead
 *
 * @brief   Toggle RSSI Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doRssiRead(uint8_t index)
{
  status_t status;

  if (index)
  {
    if ((status = BmsMaster_StartRssi()) == SUCCESS)
    {
      //Do nothing
    }
  }
  else // BMS_ITEM_STOP_RSSI
  {
    if ((status = BmsMaster_CancelRssi(bmsConnHandle)) == SUCCESS)
    {
      //Do nothing
    }
  }

  return ((status == SUCCESS) ? true : false);
}

/*********************************************************************
 * @fn      BmsMaster_doConnUpdate
 *
 * @brief   Initiate Connection Update procedure
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doConnUpdate(uint8_t index)
{
  gapUpdateLinkParamReq_t params;

  (void) index;

  params.connectionHandle = bmsConnHandle;
  params.intervalMin = DEFAULT_UPDATE_MIN_CONN_INTERVAL;
  params.intervalMax = DEFAULT_UPDATE_MAX_CONN_INTERVAL;
  params.connLatency = DEFAULT_UPDATE_SLAVE_LATENCY;

  linkDBInfo_t linkInfo;
  if (linkDB_GetInfo(bmsConnHandle, &linkInfo) == SUCCESS)
  {
    if (linkInfo.connTimeout == DEFAULT_UPDATE_CONN_TIMEOUT)
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT + 200;
    }
    else
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT;
    }
    GAP_UpdateLinkParamReq(&params);

    Log_info1("Param update Request:connTimeout =%d", params.connTimeout*CONN_TIMEOUT_MS_CONVERSION);
  }
  else
  {
      Log_info1("update :%s, Unable to find link information",
                (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));
  }
  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doSetConnPhy
 *
 * @brief   Set Connection PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range)
 *                  4: 1M + 2M + CODED PHY
 *
 * @return  always true
 */
bool BmsMaster_doSetConnPhy(uint8_t index)
{
  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
  };

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX. For more information, see the LE 2M PHY section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  HCI_LE_SetPhyCmd(bmsConnHandle, 0, phy[index], phy[index], 0);
  return (true);
}

/*********************************************************************
 * @fn      BmsMaster_doDisconnect
 *
 * @brief   Disconnect the specified link
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool BmsMaster_doDisconnect(uint8_t index)
{
  (void) index;

  GAP_TerminateLinkReq(bmsConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
}

/*********************************************************************
*********************************************************************/
