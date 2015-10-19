/**
 * \file   demoGpio.c
 *
 * \brief  This file uses APIs of GPIO to turn ON and turn OFF user LED D1.
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

#include "soc_AM335x.h"
#include "evmskAM335x.h"
#include "interrupt.h"
#include "gpio_v2.h"
#include "delay.h"

/******************************************************************************
**              EXTERNAL VARIABLES
******************************************************************************/
extern volatile unsigned int IsTSPress;
extern unsigned int clickIdx;

/******************************************************************************
**              INTERNAL MACRO DEFINITIONS
******************************************************************************/

#define GPIO_INSTANCE_ADDRESS        (SOC_GPIO_1_REGS)
#define GPIO_INSTANCE_PIN_NUMBER     (4)

#define GPIO_WAKE_INSTANCE	     (SOC_GPIO_0_REGS)
#define GPIO_WAKE_PIN_NUM	     (30)
#define GPIO_LATENCY_PIN_NUM	     (2)

/******************************************************************************
**              INTERNAL FUNCTION DEFINITIONS
******************************************************************************/

void gpioLedInit()
{
    /* Enabling the GPIO module. */
    GPIOModuleEnable(GPIO_INSTANCE_ADDRESS);

    /* Perform a module reset of the GPIO module. */
    GPIOModuleReset(GPIO_INSTANCE_ADDRESS);

    /* Set the specified pin as an output pin. */
    GPIODirModeSet(GPIO_INSTANCE_ADDRESS,
                   GPIO_INSTANCE_PIN_NUMBER,
                   GPIO_DIR_OUTPUT);
}

void gpioLedBlink()
{
    IsTSPress = 0;

    while(!(IsTSPress |clickIdx))
    {
        /* Driving GPIO0[7] pin to logic HIGH. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_HIGH);

        delay(500);

        /* Driving GPIO0[7] pin to logic LOW. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_LOW);

        delay(500);
    }
}

void configWakeGpio()
{
	/* Enabling the GPIO module. */
	GPIOModuleEnable(GPIO_WAKE_INSTANCE);
	
	/* Perform a module reset of the GPIO module. */
    //GPIOModuleReset(GPIO_WAKE_INSTANCE);
	
	/* Set the specified pin as an Input pin. */
    GPIODirModeSet(GPIO_WAKE_INSTANCE,
                   GPIO_WAKE_PIN_NUM,
                   GPIO_DIR_INPUT);
				   
	GPIOIntTypeSet(GPIO_WAKE_INSTANCE,
					GPIO_WAKE_PIN_NUM,
					GPIO_INT_TYPE_BOTH_EDGE);
					
	HWREG(GPIO_WAKE_INSTANCE + 0x34) = 0x40000000;
	HWREG(GPIO_WAKE_INSTANCE + 0x38) = 0x40000000;
	
	HWREG(GPIO_WAKE_INSTANCE + 0x44) = 0x40000000;
	
}

void enableGpioWake()
{
	/*	Clearing immediately after wake up is not consistent.
	**	Seems event is logged after clearing	*/
	/*	Clear wake interrupt	*/
	HWREG(GPIO_WAKE_INSTANCE + 0x2C) = 0x40000000;
	HWREG(GPIO_WAKE_INSTANCE + 0x30) = 0x40000000;

	/*	Configure GPIO in smart-dile and wake capable mode	*/
	HWREG(GPIO_WAKE_INSTANCE + GPIO_SYSCONFIG) = 0x1C;
}

void disableGpioWake()
{
	/*	Configure GPIO in Force-idle mode	*/
	HWREG(GPIO_WAKE_INSTANCE + GPIO_SYSCONFIG) = 0;
}

void toggleLatGpio()
{
	GPIOPinWrite(SOC_GPIO_1_REGS, GPIO_LATENCY_PIN_NUM, 
			((~GPIOPinRead(SOC_GPIO_1_REGS, GPIO_LATENCY_PIN_NUM)) >> GPIO_LATENCY_PIN_NUM) & 0x1);
}


/******************************* End of file *********************************/

