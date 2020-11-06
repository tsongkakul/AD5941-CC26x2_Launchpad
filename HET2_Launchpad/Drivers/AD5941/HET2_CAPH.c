/*!
 *****************************************************************************
 @file:    CAPH.c
 @author:  $Author: nxu2, modified by Tanner Songkakul $
 @brief:   Chrono-amperometric and differenntial analog measurement sequences.
 @version: $Revision: 766 $
 @date:    $Date: 2020-06-02 $
 -----------------------------------------------------------------------------

 Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.
 By using this software you agree to the terms of the associated
 Analog Devices Software License Agreement.
 
 *****************************************************************************/
#include "HET2_CAPH.h"

#define PRINTVALUES

/* This file contains auto generated source code that user defined */

/* 
 Application configuration structure. Specified by user from template.
 The variables are usable in this whole application.
 It includes basic configuration for sequencer generator and application related parameters
 */
AppCHRONOAMPCfg_Type AppCAPHCfg = { .bParaChanged = bFALSE, .SeqStartAddr =
                                                 0,
                                         .MaxSeqLen = 0,

                                         .SeqStartAddrCal = 0,
                                         .MaxSeqLenCal = 0, .FifoThresh = 1000,

                                         .SysClkFreq = 16000000.0,
                                         .WuptClkFreq = 32000.0, .AdcClkFreq =
                                                 16000000.0,
                                         .AmpODR = 1, .NumOfData = -1,
                                         .RcalVal = 10000.0, /* 10kOhm */

                                         .ExtRtiaVal = 0,
                                         .PwrMod = AFEPWR_LP,
                                         /* LPTIA Configure */
                                         .LptiaRtiaSel = LPTIARTIA_10K,
                                         .LpTiaRf = LPTIARF_1M, .LpTiaRl =
                                                 LPTIARLOAD_SHORT,
                                         .ReDoRtiaCal = bTRUE,
                                         .RtiaCalValue = 0,
                                         /*LPDAC Configure */
                                         .Vbias = 1100,
                                         .Vzero = 1100,

                                         /* Waveform Configuration */
                                         .pulseAmplitude = 500, /* Amplitude of step in mV */
                                         .pulseLength = 500, /* Length of transient in ms*/
                                         .EndSeq = bFALSE, /* Flag to indicate sequence has finished */

                                         /* ADC Configuration*/
                                         .ADCPgaGain = ADCPGA_1P5,
                                         .ADCSinc3Osr = ADCSINC3OSR_4,
                                         .ADCSinc2Osr = ADCSINC2OSR_44,
                                         .ADCRefVolt = 1.82, /* Measure voltage on ADCRefVolt pin and enter here*/
                                         .DataFifoSrc = DATATYPE_SINC2, /* Data type must be SINC2 for chrono-amperometric measurement*/
                                         .CHRONOAMPInited = bFALSE,
                                         .StopRequired = bFALSE, };


/* Local Functions */
static AD5940Err AppCHRONOAMPSeqCfgGen(void);
static AD5940Err AppCHRONOAMPTransientMeasureGen(void);
static AD5940Err AppCASeqMeasureGen(void);
static AD5940Err AppPHSeqMeasureGen(void);
static AD5940Err AppCHRONOAMPRtiaCal(void);
static AD5940Err AppCHRONOAMPRegModify(int32_t * const pData,uint32_t *pDataCount);
static AD5940Err AppCAPHProcess(int32_t * const pData, uint32_t *pDataCount);

AD5940Err AppCAPHGetCfg(void *pCfg)
{
    if (pCfg)
    {
        *(AppCHRONOAMPCfg_Type**) pCfg = &AppCAPHCfg;
        return AD5940ERR_OK;
    }
    return AD5940ERR_PARA;
}

AD5940Err AppCAPHInit(uint32_t *pBuffer, uint32_t BufferSize)
/**
 * @brief Initialize the amperometric test. Call this function every time before starting amperometric test.
 * @param pBuffer: the buffer for sequencer generator. Only need to provide it for the first time.
 * @param BufferSize: The buffer size start from pBuffer.
 * @return return error code.
 */
{
    AD5940Err error = AD5940ERR_OK;
    SEQCfg_Type seq_cfg;
    FIFOCfg_Type fifo_cfg;

    //printf("Attempting FIFO set up\n");

    AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
    /* Configure sequencer and stop it */
    seq_cfg.SeqMemSize = SEQMEMSIZE_2KB; /* 2kB SRAM is used for sequencer, others for data FIFO */
    seq_cfg.SeqBreakEn = bFALSE;
    seq_cfg.SeqIgnoreEn = bFALSE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable = bFALSE;
    seq_cfg.SeqWrTimer = 0;
    //printf("FIFO setup complete, attempting sequencer config\n");
    AD5940_SEQCfg(&seq_cfg);
    //printf("Sequencer config complete\n");

    /* Do RTIA calibration */
    if (((AppCAPHCfg.ReDoRtiaCal == bTRUE)
            || AppCAPHCfg.CHRONOAMPInited == bFALSE)
            && AppCAPHCfg.ExtRtia == bFALSE)
    {
        AppCHRONOAMPRtiaCal();
        AppCAPHCfg.ReDoRtiaCal = bFALSE;
    }
    printf("RTIA Calibration complete\n");
    fflush(stdout);

    /* Reconfigure FIFO */
    AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE); /* Disable FIFO firstly */
    fifo_cfg.FIFOEn = bTRUE;
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
    fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
    fifo_cfg.FIFOThresh = AppCAPHCfg.FifoThresh; /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
    AD5940_FIFOCfg(&fifo_cfg);
    printf("FIFO reconfig complete\n");

    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    /* Start sequence generator */
    /* Initialize sequencer generator */
    printf("Initializing sequencer generator\n");
    if ((AppCAPHCfg.CHRONOAMPInited == bFALSE)
            || (AppCAPHCfg.bParaChanged == bTRUE))
    {
        if (pBuffer == 0)
            return AD5940ERR_PARA;
        if (BufferSize == 0)
            return AD5940ERR_PARA;
        AD5940_SEQGenInit(pBuffer, BufferSize);

        /* Generate initialize sequence */
        error = AppCHRONOAMPSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
        //printf("App init seq complete\n");

        if (error != AD5940ERR_OK)
            return error;

        /* Generate CA measurement sequence */
        error = AppCASeqMeasureGen();
        if (error != AD5940ERR_OK)
            return error;

        /* Generate PH measurement sequence */
        error = AppPHSeqMeasureGen();
        if (error != AD5940ERR_OK)
            return error;

        /* Generate transient sequence
        error = AppCHRONOAMPTransientMeasureGen();
        if (error != AD5940ERR_OK)
            return error;
*/
        AppCAPHCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
    }

    /* Initialization sequencer  */
    AppCAPHCfg.InitSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppCAPHCfg.InitSeqInfo);
    seq_cfg.SeqEnable = bTRUE;
    AD5940_SEQCfg(&seq_cfg); /* Enable sequencer */
    AD5940_SEQMmrTrig(AppCAPHCfg.InitSeqInfo.SeqId);
    while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE)
        ;
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

    /* PH sequence stored as transient sequence in CA struct*/
    AppCAPHCfg.TransientSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppCAPHCfg.TransientSeqInfo);

    /* Measurement sequence  */
    AppCAPHCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppCAPHCfg.MeasureSeqInfo);

    seq_cfg.SeqEnable = bTRUE;
    AD5940_SEQCfg(&seq_cfg); /* Enable sequencer, and wait for trigger */
    AD5940_ClrMCUIntFlag(); /* Clear interrupt flag generated before */

    AD5940_AFEPwrBW(AppCAPHCfg.PwrMod, AFEBW_250KHZ);
    AppCAPHCfg.CHRONOAMPInited = bTRUE; /* CHRONOAMP application has been initialized. */
    AppCAPHCfg.bMeasureTransient = bFALSE;
    return AD5940ERR_OK;
}

AD5940Err AppCAPHCtrl(int32_t AmpCtrl, void *pPara)
{
    switch (AmpCtrl)
    {
    case CHRONOAMPCTRL_START:
    {
        WUPTCfg_Type wupt_cfg;
        SEQCfg_Type seq_cfg;

        AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
        if (AppCAPHCfg.CHRONOAMPInited == bFALSE)
            return AD5940ERR_APPERROR;

        /* Configure FIFO and Sequencer for normal Amperometric Measurement */
        AD5940_FIFOThrshSet(AppCAPHCfg.FifoThresh);
        seq_cfg.SeqMemSize = SEQMEMSIZE_2KB; /* 2kB SRAM is used for sequencer, others for data FIFO */
        seq_cfg.SeqBreakEn = bFALSE;
        seq_cfg.SeqIgnoreEn = bFALSE;
        seq_cfg.SeqCntCRCClr = bTRUE;
        seq_cfg.SeqEnable = bTRUE;
        seq_cfg.SeqWrTimer = 0;
        AD5940_SEQCfg(&seq_cfg);

        /* Configure Wakeup Timer*/
        wupt_cfg.WuptEn = bTRUE;
        wupt_cfg.WuptEndSeq = WUPTENDSEQ_A; //loop after pH
//        wupt_cfg.WuptEndSeq = WUPTENDSEQ_B; //loop after pH

        //Add chronoamperometric measurement sequence to wakeup timer
        wupt_cfg.WuptOrder[0] = SEQID_0;
        wupt_cfg.SeqxSleepTime[SEQID_0] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
        wupt_cfg.SeqxWakeupTime[SEQID_0] =
                (uint32_t) (AppCAPHCfg.WuptClkFreq * AppCAPHCfg.AmpODR)
                        - 2 - 1;

//        //Add pH measurement sequence to wakeup timer
//        wupt_cfg.WuptOrder[1] = SEQID_2;
//        wupt_cfg.SeqxSleepTime[SEQID_2] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
//        wupt_cfg.SeqxWakeupTime[SEQID_2] =
//                (uint32_t) (AppCAPHCfg.WuptClkFreq * AppCAPHCfg.AmpODR)
//                        - 2 - 1;

        AD5940_WUPTCfg(&wupt_cfg);

        AppCAPHCfg.FifoDataCount = 0; /* restart */
        break;
    }
    case CHRONOAMPCTRL_STOPNOW:
    {
        AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
        /* Start Wupt right now */
        AD5940_WUPTCtrl(bFALSE);
        /* There is chance this operation will fail because sequencer could put AFE back
         to hibernate mode just after waking up. Use STOPSYNC is better. */
        AD5940_WUPTCtrl(bFALSE);
        break;
    }
    case CHRONOAMPCTRL_STOPSYNC:
    {
        AppCAPHCfg.StopRequired = bTRUE;
        break;
    }
    case CHRONOAMPCTRL_SHUTDOWN:
    {
        AppCHRONOAMPCtrl(CHRONOAMPCTRL_STOPNOW, 0); /* Stop the measurment if it's running. */
        /* Turn off LPloop related blocks which are not controlled automatically by sleep operation */
        AFERefCfg_Type aferef_cfg;
        LPLoopCfg_Type lp_loop;
        memset(&aferef_cfg, 0, sizeof(aferef_cfg));
        AD5940_REFCfgS(&aferef_cfg);
        memset(&lp_loop, 0, sizeof(lp_loop));
        AD5940_LPLoopCfgS(&lp_loop);
        AD5940_EnterSleepS(); /* Enter Hibernate */
    }
    case CHRONOAMPCTRL_PULSETEST:
    {
        FIFOCfg_Type fifo_cfg;
        AD5940_WUPTCtrl(bFALSE);
        AppCAPHCfg.bMeasureTransient = bTRUE;
        /* Reconfigure FIFO for Pulse test*/
        AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE); /* Disable FIFO firstly */
        fifo_cfg.FIFOEn = bTRUE;
        fifo_cfg.FIFOMode = FIFOMODE_FIFO;
        fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
        fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
        fifo_cfg.FIFOThresh = 1000;
        AD5940_FIFOCfg(&fifo_cfg);

        /* Trigger sequence by MMR write */
        AD5940_SEQMmrTrig(AppCAPHCfg.TransientSeqInfo.SeqId);
    }
        break;
    default:
        break;
    }
    return AD5940ERR_OK;
}

AD5940Err AppCAPHISR(void *pBuff)
{
    extern uint32_t IntCount;
    uint32_t FifoCnt;


    if (AppCAPHCfg.CHRONOAMPInited == bFALSE)
        return AD5940ERR_APPERROR;
    AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
    if (AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
    {
        FifoCnt = AD5940_FIFOGetCnt();

            AD5940_FIFORd((uint32_t *) pBuff, FifoCnt);
            AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
            AppCHRONOAMPRegModify(pBuff, &FifoCnt); /* If there is need to do AFE re-configure, do it here when AFE is in active state */
            AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Unlock so sequencer can put AD5940 to sleep */
            AD5940_EnterSleepS();
            /* Process data */
            AppCAPHProcess((int32_t*) pBuff, &FifoCnt);
        return 0;
    }

    else return 0;
}

/* Calculate voltage */
float AppCAPHCalcVoltage(uint32_t ADCcode)
{
    float kFactor = 1.835 / 1.82;
    float fVolt = 0.0;
    int32_t tmp = 0;
    tmp = ADCcode - 32768;
    switch (AppCAPHCfg.ADCPgaGain)
    {
    case ADCPGA_1:
        fVolt = ((float) (tmp) / 32768) * (AppCAPHCfg.ADCRefVolt / 1)
                * kFactor;
        break;
    case ADCPGA_1P5:
        fVolt = ((float) (tmp) / 32768) * (AppCAPHCfg.ADCRefVolt / 1.5f)
                * kFactor;
        break;
    case ADCPGA_2:
        fVolt = ((float) (tmp) / 32768) * (AppCAPHCfg.ADCRefVolt / 2)
                * kFactor;
        break;
    case ADCPGA_4:
        fVolt = ((float) (tmp) / 32768) * (AppCAPHCfg.ADCRefVolt / 4)
                * kFactor;
        break;
    case ADCPGA_9:
        fVolt = ((float) (tmp) / 32768) * (AppCAPHCfg.ADCRefVolt / 9)
                * kFactor;
        break;
    }

#ifdef PRINTVALUES
    printf("Voltage = %f\n",fVolt);
#endif
    return fVolt;
}

float AppCAPHCalcCurrent(uint32_t ADCcode)
{
    float fCurrent, fVoltage = 0.0;
    fVoltage = AppCAPHCalcVoltage(ADCcode);
    //printf("ADC Code: %i\n", ADCcode);
    fCurrent = fVoltage / AppCAPHCfg.RtiaCalValue.Magnitude;
#ifdef PRINTVALUES
    printf("Current Calc: %f\n", -fCurrent * 1000000);
#endif
    return -fCurrent * 1000000;
}

/* Generate init sequence */
static AD5940Err AppCHRONOAMPSeqCfgGen(void) //SEQ1
{
    AD5940Err error = AD5940ERR_OK;
    uint32_t const *pSeqCmd;
    uint32_t SeqLen;

    AFERefCfg_Type aferef_cfg;
    LPLoopCfg_Type lp_loop;
    DSPCfg_Type dsp_cfg;
    HSLoopCfg_Type hs_loop;

    /* Start sequence generator here */
    AD5940_SEQGenCtrl(bTRUE);

    //AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */

    aferef_cfg.HpBandgapEn = bTRUE;
    aferef_cfg.Hp1V1BuffEn = bTRUE;
    aferef_cfg.Hp1V8BuffEn = bTRUE;
    aferef_cfg.Disc1V1Cap = bFALSE;
    aferef_cfg.Disc1V8Cap = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE;
    aferef_cfg.Hp1V8Ilimit = bFALSE;
    aferef_cfg.Lp1V1BuffEn = bTRUE;
    aferef_cfg.Lp1V8BuffEn = bTRUE;
    /* LP reference control - turn off them to save powr*/
    aferef_cfg.LpBandgapEn = bTRUE;
    aferef_cfg.LpRefBufEn = bTRUE;
    aferef_cfg.LpRefBoostEn = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);

    lp_loop.LpDacCfg.LpdacSel = LPDAC0;
    lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA
            | /*LPDACSW_VBIAS2PIN|*/LPDACSW_VZERO2LPTIA | LPDACSW_VZERO2PIN;
    lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
    lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
    lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lp_loop.LpDacCfg.DataRst = bFALSE;
    lp_loop.LpDacCfg.PowerEn = bTRUE;
    lp_loop.LpDacCfg.DacData6Bit = (uint32_t) ((AppCAPHCfg.Vzero - 200)
            / DAC6BITVOLT_1LSB);
    lp_loop.LpDacCfg.DacData12Bit = (int32_t) ((AppCAPHCfg.SensorBias)
            / DAC12BITVOLT_1LSB) + lp_loop.LpDacCfg.DacData6Bit * 64;
    if (lp_loop.LpDacCfg.DacData12Bit > lp_loop.LpDacCfg.DacData6Bit * 64)
        lp_loop.LpDacCfg.DacData12Bit--;

    lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
    lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
    lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
    lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lp_loop.LpAmpCfg.LpTiaRf = AppCAPHCfg.LpTiaRf;
    lp_loop.LpAmpCfg.LpTiaRload = AppCAPHCfg.LpTiaRl;
    if (AppCAPHCfg.ExtRtia == bTRUE)
    {
        lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
        lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(
                9)|LPTIASW(2)|LPTIASW(4)|LPTIASW(5)/*|LPTIASW(12)*/|LPTIASW(13);
    }
    else
    {
        lp_loop.LpAmpCfg.LpTiaRtia = AppCAPHCfg.LptiaRtiaSel;
        lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(
                5)|LPTIASW(2)|LPTIASW(4)/*|LPTIASW(12)*/|LPTIASW(13);
    }
    AD5940_LPLoopCfgS(&lp_loop);

    dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
    dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
    dsp_cfg.ADCBaseCfg.ADCPga = AppCAPHCfg.ADCPgaGain;

    memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));

    dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16; /* Don't care becase it's disabled */
    dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* Tell filter block clock rate of ADC*/
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppCAPHCfg.ADCSinc2Osr;
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppCAPHCfg.ADCSinc3Osr;
    dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
    dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.Sinc2NotchClkEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.Sinc3ClkEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.WGClkEnable = bFALSE;
    dsp_cfg.ADCFilterCfg.DFTClkEnable = bFALSE;

    memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
    AD5940_DSPCfgS(&dsp_cfg);

    hs_loop.SWMatCfg.Dswitch = 0;
    hs_loop.SWMatCfg.Pswitch = 0;
    hs_loop.SWMatCfg.Nswitch = 0;
    hs_loop.SWMatCfg.Tswitch = 0;
    AD5940_HSLoopCfgS(&hs_loop);
    /* Enable all of them. They are automatically turned off during hibernate mode to save power */
    AD5940_AFECtrlS(AFECTRL_HPREFPWR | AFECTRL_SINC2NOTCH, bTRUE);
    AD5940_SEQGpioCtrlS(0);

    /* Sequence end. */
    AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extral command to disable sequencer for initialization sequence because we only want it to run one time. */

    /* Stop here */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE); /* Stop seuqncer generator */
    if (error == AD5940ERR_OK)
    {
        AppCAPHCfg.InitSeqInfo.SeqId = SEQID_1;
        AppCAPHCfg.InitSeqInfo.SeqRamAddr = AppCAPHCfg.SeqStartAddr;
        AppCAPHCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
        AppCAPHCfg.InitSeqInfo.SeqLen = SeqLen;
        /* Write command to SRAM */
        AD5940_SEQCmdWrite(AppCAPHCfg.InitSeqInfo.SeqRamAddr, pSeqCmd,
                           SeqLen);
    }
    else
        return error; /* Error */
    return AD5940ERR_OK;
}

static AD5940Err AppCASeqMeasureGen(void) //SEQ0
{
    AD5940Err error = AD5940ERR_OK;
    uint32_t const *pSeqCmd;
    uint32_t SeqLen;

    uint32_t WaitClks;
    ClksCalInfo_Type clks_cal;

    ADCBaseCfg_Type adc_base;

    clks_cal.DataType = AppCAPHCfg.DataFifoSrc;
    clks_cal.DataCount = 1;
    clks_cal.ADCSinc2Osr = AppCAPHCfg.ADCSinc2Osr;
    clks_cal.ADCSinc3Osr = AppCAPHCfg.ADCSinc3Osr;
    clks_cal.ADCAvgNum = 0;
    clks_cal.RatioSys2AdcClk = AppCAPHCfg.SysClkFreq
            / AppCAPHCfg.AdcClkFreq;
    AD5940_ClksCalculate(&clks_cal, &WaitClks);

    /*Measurement sequence start */
    AD5940_SEQGenCtrl(bTRUE);
    // Set up mux to measure TIA output (Echem)

    adc_base.ADCMuxP = ADCMUXP_LPTIA0_P;
    adc_base.ADCMuxN = ADCMUXN_LPTIA0_N;
    adc_base.ADCPga = AppCAPHCfg.ADCPgaGain; //make this user configurable
    AD5940_ADCBaseCfgS(&adc_base);
    // wait for mux to settle
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250)); //check this duration

    // AFE conversion
    AD5940_SEQGpioCtrlS(AGPIO_Pin1); //For debug, toggle GPIO1
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250));
    AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); /* Start ADC convert and DFT */
    AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); /* wait for first data ready */
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bFALSE); /* Stop ADC */
    AD5940_SEQGpioCtrlS(0);

  //  AD5940_EnterSleepS();/* Goto hibernate */

    // Set up mux to measure PH output
        adc_base.ADCMuxP = ADCMUXP_AIN2;
        adc_base.ADCMuxN = ADCMUXN_AIN1;
        adc_base.ADCPga = AppCAPHCfg.ADCPgaGain; //make this user configurable
        AD5940_ADCBaseCfgS(&adc_base);
        // wait for mux to settle
        AD5940_SEQGenInsert(SEQ_WAIT(16 * 250)); //check this duration
        // AFE conversion
        //AD5940_SEQGpioCtrlS(AGPIO_Pin1); //for debug, toggle GPIO1
        AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE);
        AD5940_SEQGenInsert(SEQ_WAIT(16 * 250));
        AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); /* Start ADC convert and DFT */
        AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); /* wait for first data ready */
        AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bFALSE); /* Stop ADC */
        AD5940_SEQGpioCtrlS(0);

        AD5940_EnterSleepS();/* Goto hibernate */



    /* Sequence end. */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

    if (error == AD5940ERR_OK)
    {
        AppCAPHCfg.MeasureSeqInfo.SeqId = SEQID_0;
        AppCAPHCfg.MeasureSeqInfo.SeqRamAddr =
                AppCAPHCfg.InitSeqInfo.SeqRamAddr
                        + AppCAPHCfg.InitSeqInfo.SeqLen;
        AppCAPHCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
        AppCAPHCfg.MeasureSeqInfo.SeqLen = SeqLen;
        /* Write command to SRAM */
        AD5940_SEQCmdWrite(AppCAPHCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd,
                           SeqLen);
    }
    else
        return error; /* Error */
    return AD5940ERR_OK;
}

static AD5940Err AppPHSeqMeasureGen(void) //SEQ2
{
    AD5940Err error = AD5940ERR_OK;
    uint32_t const *pSeqCmd;
    uint32_t SeqLen;

    uint32_t WaitClks;
    ClksCalInfo_Type clks_cal;

    ADCBaseCfg_Type adc_base;

    clks_cal.DataType = AppCAPHCfg.DataFifoSrc;
    clks_cal.DataCount = 1;
    clks_cal.ADCSinc2Osr = AppCAPHCfg.ADCSinc2Osr;
    clks_cal.ADCSinc3Osr = AppCAPHCfg.ADCSinc3Osr;
    clks_cal.ADCAvgNum = 0;
    clks_cal.RatioSys2AdcClk = AppCAPHCfg.SysClkFreq
            / AppCAPHCfg.AdcClkFreq;
    AD5940_ClksCalculate(&clks_cal, &WaitClks);

    /*Measurement sequence start */
    AD5940_SEQGenCtrl(bTRUE);

    // Set up mux to measure PH output
    adc_base.ADCMuxP = ADCMUXP_AIN2;
    adc_base.ADCMuxN = ADCMUXN_AIN1;
    adc_base.ADCPga = AppCAPHCfg.ADCPgaGain; //make this user configurable
    AD5940_ADCBaseCfgS(&adc_base);
    // wait for mux to settle
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250)); //check this duration
    // AFE conversion
    //AD5940_SEQGpioCtrlS(AGPIO_Pin1); //for debug, toggle GPIO1
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250));
    AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); /* Start ADC convert and DFT */
    AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); /* wait for first data ready */
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bFALSE); /* Stop ADC */
    AD5940_SEQGpioCtrlS(0);

    AD5940_EnterSleepS();/* Goto hibernate */
    /* Sequence end. */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
//(TODO: create separate sequence instead of replacing transient)
    if (error == AD5940ERR_OK) // store pH sequence info as transient sequence in CA struct
       {
           AppCAPHCfg.TransientSeqInfo.SeqId = SEQID_2;
           AppCAPHCfg.TransientSeqInfo.SeqRamAddr =
                   AppCAPHCfg.MeasureSeqInfo.SeqRamAddr
                           + AppCAPHCfg.MeasureSeqInfo.SeqLen;
           AppCAPHCfg.TransientSeqInfo.pSeqCmd = pSeqCmd;
           AppCAPHCfg.TransientSeqInfo.SeqLen = SeqLen;
           /* Write command to SRAM */
           AD5940_SEQCmdWrite(AppCAPHCfg.TransientSeqInfo.SeqRamAddr, pSeqCmd,
                              SeqLen);
       }
    else
        return error; /* Error */
    return AD5940ERR_OK;
}

static AD5940Err AppCHRONOAMPRtiaCal(void)
{
    fImpPol_Type RtiaCalValue; /* Calibration result */
    LPRTIACal_Type lprtia_cal;
    AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

    lprtia_cal.LpAmpSel = LPAMP0;
    lprtia_cal.bPolarResult = bTRUE; /* Magnitude + Phase */
    lprtia_cal.AdcClkFreq = AppCAPHCfg.AdcClkFreq;
    lprtia_cal.SysClkFreq = AppCAPHCfg.SysClkFreq;
    lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22; /* Use SINC2 data as DFT data source */
    lprtia_cal.DftCfg.DftNum = DFTNUM_2048; /* Maximum DFT number */
    lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
    lprtia_cal.DftCfg.HanWinEn = bTRUE;
    lprtia_cal.fFreq = AppCAPHCfg.AdcClkFreq / 4 / 22 / 2048 * 3; /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
    lprtia_cal.fRcal = AppCAPHCfg.RcalVal;
    lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
    lprtia_cal.bWithCtia = bFALSE;
    lprtia_cal.LpTiaRtia = AppCAPHCfg.LptiaRtiaSel;
    AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
    AppCAPHCfg.RtiaCalValue = RtiaCalValue;
    return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppCHRONOAMPRegModify(int32_t * const pData,
                                       uint32_t *pDataCount)
{
    FIFOCfg_Type fifo_cfg;
    SEQCfg_Type seq_cfg;
    /* Reset dtat FIFO threshold for normal amp */
    if (AppCAPHCfg.EndSeq)
    {
        AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE); /* Disable FIFO firstly */
        fifo_cfg.FIFOEn = bTRUE;
        fifo_cfg.FIFOMode = FIFOMODE_FIFO;
        fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
        fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
        fifo_cfg.FIFOThresh = AppCAPHCfg.FifoThresh; /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
        AD5940_FIFOCfg(&fifo_cfg);

        seq_cfg.SeqEnable = bTRUE;
        AD5940_SEQCfg(&seq_cfg); /* Enable sequencer, and wait for trigger */
        AD5940_ClrMCUIntFlag(); /* Clear interrupt flag generated before */
    }
    if (AppCAPHCfg.NumOfData > 0)
    {
        AppCAPHCfg.FifoDataCount += *pDataCount / 4;
        if (AppCAPHCfg.FifoDataCount >= AppCAPHCfg.NumOfData)
        {
            AD5940_WUPTCtrl(bFALSE);
            return AD5940ERR_OK;
        }
    }
    if (AppCAPHCfg.StopRequired == bTRUE)
    {
        AD5940_WUPTCtrl(bFALSE);
        return AD5940ERR_OK;
    }
    return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppCAPHProcess(int32_t * const pData,
                                         uint32_t *pDataCount)
{
    uint32_t i, datacount;
    uint8_t keep_index = 0;
    datacount = *pDataCount;
    float *pOut = (float *) pData;
    for (i = 0; i < datacount; i++)
    {
        pData[i] &= 0xffff;
        // For each set of 3 measurements, CA is [0], pH is [2], [1] is garbage
        if(i%3 == 0){
            pOut[keep_index] = AppCAPHCalcCurrent(pData[i]);
            keep_index++;
        }
        else if(i%3==2){
            pOut[keep_index] = AppCAPHCalcVoltage(pData[i]);
            keep_index++;
        }

    }
    return 0;
}

static AD5940Err AppCHRONOAMPTransientMeasureGen(void) //SEQ2, not currently used
{
    AD5940Err error = AD5940ERR_OK;
    uint32_t const *pSeqCmd;
    uint32_t SeqLen;
    uint32_t VbiasCode, VzeroCode;
    uint32_t WaitClks;
    ClksCalInfo_Type clks_cal;

    if (AppCAPHCfg.DataFifoSrc != DATATYPE_SINC2)
        return AD5940ERR_ERROR; /* FIFO data must be SINC2 filter for measuring transient */
    /* Calculate LPDAC codes */
    VzeroCode = (uint32_t) ((AppCAPHCfg.Vzero - 200) / DAC6BITVOLT_1LSB);
    VbiasCode = (int32_t) ((AppCAPHCfg.pulseAmplitude
            + AppCAPHCfg.SensorBias) / DAC12BITVOLT_1LSB) + VzeroCode * 64;
    if (VbiasCode < (VzeroCode * 64))
        VbiasCode--;
    /* Truncate */
    if (VbiasCode > 4095)
        VbiasCode = 4095;
    if (VzeroCode > 63)
        VzeroCode = 63;

    clks_cal.DataType = AppCAPHCfg.DataFifoSrc;
    clks_cal.DataCount = AppCHRONOAMPCalcDataNum(AppCAPHCfg.pulseLength);
    clks_cal.ADCSinc2Osr = AppCAPHCfg.ADCSinc2Osr;
    clks_cal.ADCSinc3Osr = AppCAPHCfg.ADCSinc3Osr;
    clks_cal.ADCAvgNum = 0;
    clks_cal.RatioSys2AdcClk = AppCAPHCfg.SysClkFreq
            / AppCAPHCfg.AdcClkFreq;
    AD5940_ClksCalculate(&clks_cal, &WaitClks);

    AD5940_SEQGenCtrl(bTRUE);
    AD5940_SEQGpioCtrlS(AGPIO_Pin1);
    AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250));
    AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE); /* Start ADC conversion before applying step to capture peak */
    AD5940_WriteReg(REG_AFE_LPDACDAT0, VzeroCode << 12 | VbiasCode);
    AD5940_SEQGenInsert(SEQ_WAIT(WaitClks)); /* wait for first data ready */
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bFALSE); /* Stop ADC */
    AD5940_WriteReg(
            REG_AFE_LPDACDAT0,
            (uint32_t) ((AppCAPHCfg.Vzero - 200) / DAC6BITVOLT_1LSB) << 12
                    | (int32_t) ((AppCAPHCfg.SensorBias)
                            / DAC12BITVOLT_1LSB) + VzeroCode * 64);
    AD5940_SEQGpioCtrlS(0);
    /* Sequence end. */
    AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */
    AD5940_EnterSleepS();/* Goto hibernate */
    /* Sequence end. */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE); /* Stop seuqncer generator */

    if (error == AD5940ERR_OK)
    {
        AppCAPHCfg.TransientSeqInfo.SeqId = SEQID_2;
        AppCAPHCfg.TransientSeqInfo.SeqRamAddr =
                AppCAPHCfg.MeasureSeqInfo.SeqRamAddr
                        + AppCAPHCfg.MeasureSeqInfo.SeqLen;
        AppCAPHCfg.TransientSeqInfo.pSeqCmd = pSeqCmd;
        AppCAPHCfg.TransientSeqInfo.SeqLen = SeqLen;
        /* Write command to SRAM */
        AD5940_SEQCmdWrite(AppCAPHCfg.TransientSeqInfo.SeqRamAddr, pSeqCmd,
                           SeqLen);
    }
    else
        return error; /* Error */
    return AD5940ERR_OK;
}


