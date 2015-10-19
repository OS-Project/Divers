/**
 * \file  i2cEEprom.c
 *
 * \brief Sample application for HSI2C.This application reads the contents
 *        of EEPROM and displays it on serial console.
 *
*/

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "hsi2c.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "soc_AM335x.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/* I2C address of CAT24C256 e2prom */
#define  I2C_SLAVE_ADDR         (0x50)

/* Higher byte address (i.e A8-A15) */
#define  E2PROM_ADDR_MSB         0x00

/* Lower byte address (i.e A0-A7) */
#define  E2PROM_ADDR_LSB         0x00

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
void I2CIsr(void);
static void SetupI2C(void);
void I2CAINTCConfigure(void);
void CleanUpInterrupts(void);
static void E2promRead(unsigned char *data);
static void SetupI2CReception(unsigned int dcount);
/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
volatile unsigned int tCount;
volatile unsigned int rCount;
volatile unsigned int speed;
volatile unsigned int flag = 1;
volatile unsigned int numOfBytes;
volatile unsigned char dataToSlave[2];
volatile unsigned char dataFromSlave[50];
/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/

int main(void)
{
    unsigned char dataRead[50];
    unsigned char temp;
    unsigned int i;

    UARTStdioInit();

    UARTPuts("StarterWare HSI2C EEPROM example application\r\n",-1);
    UARTPuts("Enter the bus frequency at which i2c has to communicate (in Hz):",-1);

    speed = UARTGetNum();

    UARTPuts("\r\n", -1);

    UARTPuts("Data read from EEPROM:\r\n",-1);

    /* Enable IRQ in CPSR */
    IntMasterIRQEnable();

    /* Configures AINTC to generate interrupt */
    I2CAINTCConfigure();

    /*
    ** Configures I2C to Master mode to generate start
    ** condition on I2C bus and to transmit data at a
    ** speed of 100khz
    */
    SetupI2C();

    /*
    ** Read data from a selected address of
    ** e2prom
    */
    E2promRead(dataRead);


    for(i = 0; i < 50; i++)
    {
         temp = dataRead[i] & 0x0f;

         if(temp < 10)
         {
              UARTPutc(temp + 0x30); 
         }
         else
         {
              UARTPutc(temp + 0x37);
         } 

          temp = ((dataRead[i] & 0xf0) >> 4);

         if(temp < 10)
         {
              UARTPutc(temp + 0x30); 
         }
         else
         {
              UARTPutc(temp + 0x37);
         }
         
         UARTPutc(',');
    }

    PRINT_RESULT_PASS();

    while(1);
}


 /* 
 ** Reads data from a specific address of e2prom
 */
static void E2promRead(unsigned char *data)
{
    unsigned int i;

    dataToSlave[0] = E2PROM_ADDR_MSB;
    dataToSlave[1] = E2PROM_ADDR_LSB;

    tCount = 0;
    rCount = 0;
    SetupI2CReception(50);

    for (i = 0; i < 50; i++ )
    {
         data[i] = dataFromSlave[i];
    }
}

static void SetupI2C(void)
{
    /* Enable the clock for I2C0 */
    I2C0ModuleClkConfig();

    I2CPinMuxSetup(0);

    /* Put i2c in reset/disabled state */
    I2CMasterDisable(SOC_I2C_0_REGS);

    /* Disable auto Idle functionality */
    I2CAutoIdleDisable(SOC_I2C_0_REGS);

    /* Configure i2c bus speed to 100khz */
    I2CMasterInitExpClk(SOC_I2C_0_REGS, 48000000, 12000000, speed);

    /* Set i2c slave address */
    I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, I2C_SLAVE_ADDR);

    /* Bring I2C out of reset */
    I2CMasterEnable(SOC_I2C_0_REGS);
}

/* Configures AINTC to generate interrupt */
void I2CAINTCConfigure(void)
{
    /* Intialize the ARM Interrupt Controller(AINTC) */
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(SYS_INT_I2C0INT, I2CIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(SYS_INT_I2C0INT);
}

/*
** Receives data over I2C bus 
*/
static void SetupI2CReception(unsigned int dcount)
{
    /* Data Count specifies the number of bytes to be transmitted */
    I2CSetDataCount(SOC_I2C_0_REGS, 0x02);

    numOfBytes = I2CDataCountGet(SOC_I2C_0_REGS);

    /* Clear status of all interrupts */
    CleanUpInterrupts();

    /* Configure I2C controller in Master Transmitter mode */
    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_TX);

    /* Transmit interrupt is enabled */
    I2CMasterIntEnableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_0_REGS);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while(tCount != numOfBytes);

    flag = 1;

    /* Wait untill I2C registers are ready to access */
    while(!(I2CMasterIntRawStatus(SOC_I2C_0_REGS) & (I2C_INT_ADRR_READY_ACESS)));

    /* Data Count specifies the number of bytes to be received */
    I2CSetDataCount(SOC_I2C_0_REGS, dcount);

    numOfBytes = I2CDataCountGet(SOC_I2C_0_REGS);

    CleanUpInterrupts();

    /* Configure I2C controller in Master Receiver mode */
    I2CMasterControl(SOC_I2C_0_REGS, I2C_CFG_MST_RX);

    /* Receive and Stop Condition Interrupts are enabled */
    I2CMasterIntEnableEx(SOC_I2C_0_REGS,  I2C_INT_RECV_READY |
                                          I2C_INT_STOP_CONDITION);

    /* Generate Start Condition over I2C bus */
    I2CMasterStart(SOC_I2C_0_REGS);

    while(I2CMasterBusBusy(SOC_I2C_0_REGS) == 0);

    while(flag);

    flag = 1;
}


/* Clear status of all interrupts */
void CleanUpInterrupts(void)
{
    I2CMasterIntEnableEx(SOC_I2C_0_REGS, 0x7FF);
    I2CMasterIntClearEx(SOC_I2C_0_REGS,  0x7FF);
    I2CMasterIntDisableEx(SOC_I2C_0_REGS, 0x7FF);
}

/*
** I2C Interrupt Service Routine. This function will read and write
** data through I2C bus. 
*/
void I2CIsr(void)
{
    unsigned int status = 0;

    /* Get only Enabled interrupt status */
    status = I2CMasterIntStatus(SOC_I2C_0_REGS);

    /* 
    ** Clear all enabled interrupt status except receive ready and
    ** transmit ready interrupt status 
    */
    I2CMasterIntClearEx(SOC_I2C_0_REGS,
	                    (status & ~(I2C_INT_RECV_READY | I2C_INT_TRANSMIT_READY)));
						
    if(status & I2C_INT_RECV_READY)
    {
         /* Receive data from data receive register */
        dataFromSlave[rCount++] = I2CMasterDataGet(SOC_I2C_0_REGS);

	/* Clear receive ready interrupt status */	
        I2CMasterIntClearEx(SOC_I2C_0_REGS,  I2C_INT_RECV_READY);
		
         if(rCount == numOfBytes)
         {
              /* Disable the receive ready interrupt */
              I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_RECV_READY);
              /* Generate a STOP */
              I2CMasterStop(SOC_I2C_0_REGS);
			  
         }
    }

    if (status & I2C_INT_TRANSMIT_READY)
    {
        /* Put data to data transmit register of i2c */
        I2CMasterDataPut(SOC_I2C_0_REGS, dataToSlave[tCount++]);

        /* Clear Transmit interrupt status */
	I2CMasterIntClearEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);		 
						
         if(tCount == numOfBytes)
         {
              /* Disable the transmit ready interrupt */
              I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY);
         }

    }
        
    if (status & I2C_INT_STOP_CONDITION)
    {
      	 /* Disable transmit data ready and receive data read interupt */
         I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY |
                                               I2C_INT_RECV_READY     |
					       I2C_INT_STOP_CONDITION);
         flag = 0;
    }
   
    if(status & I2C_INT_NO_ACK)
    {
         I2CMasterIntDisableEx(SOC_I2C_0_REGS, I2C_INT_TRANSMIT_READY  |
                                               I2C_INT_RECV_READY      |
                                               I2C_INT_NO_ACK          |
                                               I2C_INT_STOP_CONDITION);
         /* Generate a STOP */
         I2CMasterStop(SOC_I2C_0_REGS);

         flag = 0;
    }
}

