/*
 * HET2_Chem.h
 *
 *  Created on: May 28, 2020
 *      Author: tsong
 */

#ifndef DRIVERS_HET2_ECHEM_H_
#define DRIVERS_HET2_ECHEM_H_


#include <Drivers/AD5941/ad5940.h>
#include <Drivers/AD5941/HET2_CAPH.h>

#include <stdio.h>
#include <string.h>

#define MODE_IDLE           0
#define MODE_STREAM         1
#define MODE_SAVE           2
#define MODE_DUMP           3
#define MODE_LPSTREAM       4

#define ECHEM_CA            0
#define ECHEM_CV            1

typedef struct{
    uint8_t Device_Mode;
    uint8_t Echem_Mode;
    AppCHRONOAMPCfg_Type EChem_CFG;
}EChem_Device_Type;

int32_t AD5940PlatformCfg(void);

void AD5940AMPStructInit(void);

int32_t AMPShowResult(float *pData, uint32_t DataCount);

#endif /* DRIVERS_HET2_ECHEM_H_ */
