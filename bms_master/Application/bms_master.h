
#ifndef BMS_MASTER_H
#define BMS_MASTER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan results.
// Note: this value cannot be greater than the number of items reserved in
// scMenuConnect (See simple_central_menu.c)
// This cannot exceed 27 (two-button menu's constraint)
#define DEFAULT_MAX_SCAN_RES                 8

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task creation function for the Simple Central.
 */
extern void BmsMaster_createTask(void);

/*
 * Functions for menu action
 */

/* Action for Menu: Set Scanning PHY */
bool BmsMaster_doSetScanPhy(uint8_t index);

/* Action for Menu: Enable Scanning */
bool BmsMaster_doDiscoverDevices(uint8_t index);
bool BmsMaster_doDiscoverDevices1(uint8_t index);

/* Action for Menu: Disable Scanning */
bool BmsMaster_doStopDiscovering(uint8_t index);

/* Action for Menu: AutoConnect */
bool BmsMaster_doAutoConnect(uint8_t index);

/* Action for Menu: Connect */
bool BmsMaster_doConnect(uint8_t index);

/* Action for Menu: Cancel Connecting */
bool BmsMaster_doCancelConnecting(uint8_t index);

/* Action for Menu: Select Connection */
bool BmsMaster_doSelectConn(uint8_t index);

/* Action for Menu: GATT Read */
bool BmsMaster_doGattRead(uint8_t index);

/* Action for Menu: GATT Write */
bool BmsMaster_doGattWrite(uint8_t index);

/* Action for Menu: Start/Stop RSSI Read */
bool BmsMaster_doRssiRead(uint8_t index);

/* Action for Menu: Initiate Connection Update Procedure */
bool BmsMaster_doConnUpdate(uint8_t index);

/* Action for Menu: Set Connection PHY */
bool BmsMaster_doSetConnPhy(uint8_t index);

/* Action for Menu: Disconnect */
bool BmsMaster_doDisconnect(uint8_t index);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLECENTRAL_H */
