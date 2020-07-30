/*
 * myGPIOs.h
 *
 *  Created on: Apr 2, 2020
 *      Author: parvez
 */

#ifndef APPLICATION_MYGPIOS_H_
#define APPLICATION_MYGPIOS_H_

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>


#define Board_AFE_RESET             PIN_UNASSIGNED
#define Board_AFE_READY             PIN_UNASSIGNED
#define Board_ACC_READY             IOID_12
#define Board_MAG_READY             PIN_UNASSIGNED
#define Board_MAX_CLK               IOID_16
#define Board_MAX_INT1B             IOID_4
#define Board_MAX_INT2B             IOID_5
#define Board_MAX_CSN               IOID_15
#define Board_SD_MOUNTED            PIN_UNASSIGNED
#define Board_PIN_RLED              IOID_6
#define Board_PIN_GLED              IOID_7



#endif /* APPLICATION_MYGPIOS_H_ */
