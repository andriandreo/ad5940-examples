/*!
 *****************************************************************************
 @file:    AD5940_ADCNotchFIFO.c
 @author:  andriandreo based on nxu2's `AD5940_ADCMeanFIFO.c`
 @brief:   Use FIFO to read SINC2+NOTCH result.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
/** @addtogroup AD5940_Standard_Examples
  * @{
    @defgroup ADC_MEAN_FIFO_Example
    @{
  */

#include "ad5940.h"
#include <stdio.h>

uint32_t ADCBuff[256];

static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_1P5;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

void AD5940_Main(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  StatCfg_Type stat_cfg;
  FIFOCfg_Type fifo_cfg;
  
  /* Use hardware reset */
  AD5940_HWReset();
  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();

  AD5940_PGA_Calibration();

  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_VSE0;
  adc_base.ADCMuxN = ADCMUXN_AIN1;
  adc_base.ADCPga = ADCPGA_1P5;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH-->StatisticBlock */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_2;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bFALSE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */ 
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
  fifo_cfg.FIFOThresh = 2;
  AD5940_FIFOCfg(&fifo_cfg);

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Enable FIFO threshold interrupt. */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_ClrMCUIntFlag(); /* Clear the MCU interrupt flag which will be set in ISR. */

  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);  
  while(1)
  {
    uint32_t FifoCnt;
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
      {
        FifoCnt = AD5940_FIFOGetCnt();
        AD5940_FIFORd((uint32_t *)ADCBuff, FifoCnt);
        AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
        printf("Get %ld data, ADC Code[0]:%ld, %.2f V\n",FifoCnt, ADCBuff[0]&0xffff, AD5940_ADCCode2Volt(ADCBuff[0]&0xffff,ADCPGA_1P5, 1.82));
        /*!!!!!NOTE!!!!!*/
        /* The mean result already removed 32768. So to calculate the voltage, assume mean result is n, use below equation.
          Voltage = n/32768*Vref
         */
      }
    }
  }
}

/**
 * @}
 * @}
 * */
