/**
 * \file     demoPwrMgmnt.h
 *
 * \brief    This file contains the function prototypes for PM RTC only mode
 *			 application
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
 
#ifndef      __PM_DEMO_H__
#define      __PM_DEMO_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_prm_mpu.h"
#include "hw_prm_per.h"
#include "evmAM335x.h"


/*	Wake Source	*/
//#define WAKE_SOURCES_ALL			(0x1FFF)
#define WAKE_SOURCE_TSC				(0x0100)
#define WAKE_SOURCE_UART			(0x0010)
#define WAKE_SOURCE_TMR				(0x0008)
#define WAKE_SOURCE_GPIO			(0x0020)

/*****************************************************************************
**                   FUNCTION DECLARATIONS                                   
*****************************************************************************/

void initializeAINTC(void);
void CM3EventsClear(void);
void initM3Events(void);
void waitForM3Txevent(void);
void reInitPeripherals(void);
void CM3LoadAndRun(void);
void clearTouchEvent(void);

void generateMailboxInt(unsigned int baseAdd);
void initializeMailbox(unsigned int baseAdd);

extern void ActionDeepSleep0(void);
extern void ActionDeepSleep1(void);
extern void ActionDeepSleep2(void);
extern void ActionDeepSleep1Dev(void);
extern void ActionDeepSleepDLP(void);
extern void saveRestoreContext(unsigned int slpMode);
extern void ActionWakeOnTsc(void);
extern void ActionWakeOnTimer(void);
extern void ActionWakeOnUart(void);
extern void ActionWakeOnGpio(void);
extern void CM3IntRegister(void);
extern void MailBoxInit(void);
extern void configVddOpVoltage(void);
extern void enableWakeSource(unsigned int wakeSource);
extern void disableWakeSource(unsigned int wakeSource);
#ifdef __cplusplus
}
#endif

#endif

