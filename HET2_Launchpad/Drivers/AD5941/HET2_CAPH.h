/*!
 *****************************************************************************
 @file:    ChronoAmperometric.h
 @author:  $Author: nxu2 $
 @brief:   ChronoAmperometric measurement header file.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#ifndef DRIVERS_AD5941_HET2_CAPH_H_
#define DRIVERS_AD5941_HET2_CAPH_H_

#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include <Drivers/AD5941/ChronoAmperometric.h>

#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV

AD5940Err AppCAPHGetCfg(void *pCfg);
AD5940Err AppCAPHInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppCAPHCtrl(int32_t AmpCtrl, void *pPara);
AD5940Err AppCAPHISR(void *pBuff);
float AppCAPHCalcVoltage(uint32_t ADCcode);
float AppCAPHCalcCurrent(uint32_t ADCcode);


#endif
