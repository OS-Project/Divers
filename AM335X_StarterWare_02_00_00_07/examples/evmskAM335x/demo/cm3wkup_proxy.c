/**
 * \file     cm3wkup_proxy.c
 *
 * \brief    This file contains the function prototypes for inter-processor 
			 communication between A8 and M3.
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
 
 #include "hw_types.h"
 #include "cm3wkup_proxy.h"
 #include "hw_control_AM335x.h"
 #include "soc_AM335x.h"
  
/********************** MACROS ***************************/

#define PM_STATUS_SHIFT			16

/**************************************************************************
  API FUNCTION DEFINITIONS
***************************************************************************/


/**
 *  \brief   This function configures the deep sleep data in to IPC registers
 *
 * \param     pmDsDataVar	structure variable containing deep sleep data
 *
 * \return 	  None
 */
  
void configIPCRegs(deepSleepData pmDsDataVar)
{

	/*	Command ID	*/
	HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(1)) = pmDsDataVar.dsParams.short1;
		
	if((PM_CMD_RTC_MODE == pmDsDataVar.dsDataBits.cmdID) || 
			(PM_CMD_RTC_FAST_MODE == pmDsDataVar.dsDataBits.cmdID))
	{
		/*	RTC time out value	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(2)) = pmDsDataVar.dsParams.byte1;
	}
	else
	{
		/*	Resume address	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(0)) = pmDsDataVar.dsParams.word0;
		/*	deep sleep data	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(2)) = pmDsDataVar.dsParams.word1;
		/*	deep sleep data	*/
		HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(3)) = pmDsDataVar.dsParams.word2;	
	}
}


/**
 *  \brief   This function reads teh trace data from CM3
 *
 * \param     None
 *
 * \return 	  trace		trace data indicating the state of CM3
 */
 
unsigned int readCM3Trace(void)
{
	return HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(4));
}


/**
 *  \brief   This function returns the status of the last sent command
 *
 * \param     None
 *
 * \return 	  status 	status of the last sent command
 */
 
unsigned short readCmdStatus(void)
{
	return ((HWREG(SOC_CONTROL_REGS + CONTROL_IPC_MSG_REG(1))) >> PM_STATUS_SHIFT);
}

