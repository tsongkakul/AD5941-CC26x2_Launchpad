/*
 * custom_peripheral.h
 *
 *  Created on: Mar 31, 2020
 *      Author: parve
 */

#ifndef APPLICATION_CUSTOM_PERIPHERAL_H_
#define APPLICATION_CUSTOM_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <menu/two_btn_menu.h>
//#include "Application/SDtask.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Custom Peripheral.
 */
extern void CustomPeripheral_createTask(void);

/*
 * Functions for menu action
 */
/* Actions for Menu: Choose connection to work with */
bool CustomPeripheral_doSelectConn(uint8 index);

/* Action for Menu: AutoConnect */
bool CustomPeripheral_doAutoConnect(uint8_t index);

/* Actions for Menu: Set PHY - Select */
bool CustomPeripheral_doSetConnPhy(uint8 index);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CUSTOM_PERIPHERAL_H_ */
