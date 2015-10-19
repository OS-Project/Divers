/**
 * \file  wdtReset.c
 *
 * \brief Sample application for WDT
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

#include "watchdog.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "uartStdio.h"
#include "beaglebone.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define INITIAL_COUNT_VALUE          (0xFFFE0000u)
#define RELOAD_COUNT_VALUE           (0xFFFE0000u)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void WatchdogTimerSetUp(void);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/


/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
int main(void)
{
    unsigned int triggerValue = 0;

    /* Set up the UART2 peripheral */
    UARTStdioInit();

    /* Enable the WDT clocks */
    WatchdogTimer1ModuleClkConfig();

    /* Reset the Watchdog Timer */
    WatchdogTimerReset(SOC_WDT_1_REGS);

    /* Disable the Watchdog timer */
    WatchdogTimerDisable(SOC_WDT_1_REGS);
                                               
    /* Perform the initial settings for the Watchdog Timer */
    WatchdogTimerSetUp();

    /* Send the message to UART console */
    UARTPuts("Program Reset!", -1);
    UARTPuts("Input any key at least once in every 4 seconds to avoid a further reset.\n\r", -1);

    /* Enable the Watchdog Timer */
    WatchdogTimerEnable(SOC_WDT_1_REGS);

    while(1)
    {
        /* Wait for an input through UART. If no input is given, 
        ** the WDT will timeout and reset will occur.
        */
        if(UARTGetc())
        {

            triggerValue += 1;

            /* Write into the trigger register. This will load the value from the 
            ** load register into the counter register and hence timer will start 
            ** from the initial count.
            */
            WatchdogTimerTriggerSet(SOC_WDT_1_REGS, triggerValue);
        }
    }
}

/*
** This function will perform the necessary initialization for 
** the Watchdog Timer.
*/
static void WatchdogTimerSetUp(void)
{
    /* Configure and enable the pre-scaler clock */
    WatchdogTimerPreScalerClkEnable(SOC_WDT_1_REGS, WDT_PRESCALER_CLK_DIV_1);

    /* Set the count value into the counter register */
    WatchdogTimerCounterSet(SOC_WDT_1_REGS, INITIAL_COUNT_VALUE);

    /* Set the reload value into the load register */
    WatchdogTimerReloadSet(SOC_WDT_1_REGS, RELOAD_COUNT_VALUE);
}

