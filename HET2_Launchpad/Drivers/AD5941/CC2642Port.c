/*!
 *****************************************************************************
 @file:    ADICUP3029Port.c
 @author:  $Author: ADI $
 @brief:   The port for ADI's ADICUP3029 board.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/

#include <Drivers/AD5941/AD5940.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to 
                     provide a more efficent way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/


void AD5940_CsClr(void)
{
   //pADI_GPIO1->CLR = (1<<10);
    GPIO_write(AD_CS,0);
}

void AD5940_CsSet(void)
{
   //pADI_GPIO1->SET = (1<<10);
    GPIO_write(AD_CS,1);

}

void AD5940_RstSet(void)
{
   //pADI_GPIO2->SET = 1<<6; //p2.6-ADC3-A3
    GPIO_write(AD_RST,1);
}

void AD5940_RstClr(void)
{
    GPIO_write(AD_RST,0);
}


uint32_t AD5940_GetMCUIntFlag(void)
{
   return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
   ucInterrupted = 0;
   return 1;
}

void AD5940_Delay10us(uint32_t time){
    for(uint32_t i = 0;i<(time*10);i++){
        DELAY_us(1);
    }
}


/* Functions that used to initialize MCU platform */

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
   /*
  // Step1, initialize SPI peripheral and its GPIOs for CS/RST
  pADI_GPIO0->PE = 0xFFFF;
  pADI_GPIO1->PE = 0xFFFF;
  pADI_GPIO2->PE = 0xFFFF;
  pADI_GPIO2->OEN |= (1<<6); //P2.6-ADC3-A3-AD5940_Reset
  pADI_GPIO2->SET = 1<<6; //Pull high this pin.

 //Setup Pins P0.0-->SCLK P0.1-->MOSI P0.2-->MISO P1.10-->CS
  pADI_GPIO0->CFG = (1<<0)|(1<<2)|(1<<4)|(pADI_GPIO0->CFG&(~((3<<0)|(3<<2)|(3<<4))));
  pADI_GPIO1->CFG &=~(3<<14);  Configure P1.10 to GPIO function
  pADI_GPIO1->OEN |= (1<<10);  P1.10 Output Enable
  Set SPI Baudrate = PCLK/2x(iCLKDiv+1).
  pADI_SPI0->DIV = 0; Baudrate is 13MHz
  pADI_SPI0->CTL = BITM_SPI_CTL_CSRST|        // Configure SPI to reset after a bit shift error is detected
      BITM_SPI_CTL_MASEN|                   // Enable master mode
      BITM_SPI_CTL_CON|                     // Enable continous transfer mode
         BITM_SPI_CTL_OEN|                     // Select MISO pin to operate as normal -
            BITM_SPI_CTL_RXOF|                    // overwrite data in Rx FIFO during overflow states
               BITM_SPI_CTL_ZEN|                     // transmit 00 when no valid data in Tx FIFO
                  BITM_SPI_CTL_TIM|                     // initiate trasnfer with a write to SPITX
                     BITM_SPI_CTL_SPIEN;                  // Enable SPI. SCLK idles low/ data clocked on SCLK falling edge
  pADI_SPI0->CNT = 1;// Setup to transfer 1 bytes to slave
   Step2: initialize GPIO interrupt that connects to AD5940's interrupt output pin(Gp0, Gp3, Gp4, Gp6 or Gp7 )
  pADI_GPIO0->IEN |= 1<<15;// Configure P0.15 as an input

  pADI_XINT0->CFG0 = (0x1<<0)|(1<<3);//External IRQ0 enabled. Falling edge
  pADI_XINT0->CLR = BITM_XINT_CLR_IRQ0;
  NVIC_EnableIRQ(XINT_EVT0_IRQn);		  //Enable External Interrupt 0 source.
  */
  
   //set up spi and interrupt
  AD5940_CsSet();
  AD5940_RstSet();
  return 0;
}

/* MCU related external line interrupt service routine */
void Ext_Int0_Handler()
{
//clear interrupt
    ucInterrupted = 1;
  /* This example just set the flag and deal with interrupt in AD5940Main function. It's your choice to choose how to process interrupt. */
}

