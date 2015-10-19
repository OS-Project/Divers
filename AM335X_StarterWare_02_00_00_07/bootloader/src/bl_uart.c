/**
 * \file   uartBootLoader.c
 * \brief  This file contains the functions definitions related to module norReadWrite
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

#include <interrupt.h>
#include <hw_types.h>
#include "uart_irda_cir.h"
#include "uartStdio.h"
#include "bl.h"
#include "bl_platform.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
extern unsigned int entryPoint;

/******************************************************************************
**                      FUNCTION DECLARATIONS
*******************************************************************************/
int xmodemReceive(unsigned char *dest, int destsz);
void UARTGetInt(int *val);

/**
 * \brief   This function builds an integer from teh data received
 *
 * \return  The value received
 *
 * \note   The numbers that this function can recieve should lie in the 
 *          following range:
 *          [-2^(31)] to [2^(31) - 1] i.e.
 *          0x80000000 to  0x7FFFFFFF  
 *
 *            This logic is meant only for Little Endian processors only.
 */
void UARTGetInt(int *val)
{
    char* pCh = (char*)(val);
    int size = 4;

    do
    {
        *pCh = (char)UARTGetc();
        pCh++;
    } while(--size);

    return;
}

/**
*
* \brief  UARTCharGetTimeout
* \param   UART base address
* \param   UART RX timeout 
* \return 
**/
signed char UARTCharGetTimeout(unsigned int baseAdd, unsigned int timeOut)
{
    unsigned int lcrRegValue = 0;
    signed char retVal = 0;
    unsigned int uartTimeOut;

    baseAdd = SOC_UART_0_REGS;

    /* Switching to Register Operational Mode of operation. */
    lcrRegValue = UARTRegConfigModeEnable(baseAdd, UART_REG_OPERATIONAL_MODE);

    /* Waits indefinitely until a byte arrives in the RX FIFO(or RHR). */
    for(uartTimeOut=(timeOut);(0 == (HWREG(baseAdd + UART_LSR) & UART_LSR_RX_FIFO_E))&&uartTimeOut;uartTimeOut--)
    {
        ;
    }
	if(!uartTimeOut)
	{
		timeOut = 0;
		return 0;
	}

    retVal = ((signed char)HWREG(baseAdd + UART_RHR));

    /* Restoring the value of LCR. */
    HWREG(baseAdd + UART_LCR) = lcrRegValue;

    return retVal;
}

/********************************** End of file *****************************/
