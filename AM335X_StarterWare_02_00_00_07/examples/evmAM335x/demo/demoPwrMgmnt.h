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
//#define WAKE_SOURCES_ALL			 	  (0x1FFF)
#define WAKE_SOURCE_TSC					  (0x0100)
#define WAKE_SOURCE_UART				  (0x0010)
#define WAKE_SOURCE_TMR					  (0x0008)
#define WAKE_SOURCE_GPIO				  (0x0020)

#define WR_MEM_32(addr, data) 			  *(unsigned int*)(addr) = (unsigned int)(data)

#define MOSC_CLK                          (24) //MHz

//****************************************************
//PRCM module definitions
//****************************************************
#define PRCM_BASE_ADDR                    (0x44E00000)

#define CM_AUTOIDLE_DPLL_MPU              (PRCM_BASE_ADDR + 0x41C)
#define CM_IDLEST_DPLL_MPU                (PRCM_BASE_ADDR + 0x420)
#define CM_CLKSEL_DPLL_MPU                (PRCM_BASE_ADDR + 0x42C)
#define CM_AUTOIDLE_DPLL_DDR              (PRCM_BASE_ADDR + 0x430)
#define CM_IDLEST_DPLL_DDR                (PRCM_BASE_ADDR + 0x434)
#define CM_CLKSEL_DPLL_DDR                (PRCM_BASE_ADDR + 0x440)
#define CM_AUTOIDLE_DPLL_DISP             (PRCM_BASE_ADDR + 0x444)
#define CM_IDLEST_DPLL_DISP               (PRCM_BASE_ADDR + 0x448)
#define CM_CLKSEL_DPLL_DISP               (PRCM_BASE_ADDR + 0x454)
#define CM_AUTOIDLE_DPLL_CORE             (PRCM_BASE_ADDR + 0x458)
#define CM_IDLEST_DPLL_CORE               (PRCM_BASE_ADDR + 0x45C)
#define CM_CLKSEL_DPLL_CORE               (PRCM_BASE_ADDR + 0x468)
#define CM_AUTOIDLE_DPLL_PER              (PRCM_BASE_ADDR + 0x46C)
#define CM_IDLEST_DPLL_PER                (PRCM_BASE_ADDR + 0x470)
#define CM_CLKSEL_DPLL_PER                (PRCM_BASE_ADDR + 0x49C)

#define CM_DIV_M4_DPLL_CORE               (PRCM_BASE_ADDR + 0x480)
#define CM_DIV_M5_DPLL_CORE               (PRCM_BASE_ADDR + 0x484)
#define CM_CLKMODE_DPLL_MPU               (PRCM_BASE_ADDR + 0x488)
#define CM_CLKMODE_DPLL_PER               (PRCM_BASE_ADDR + 0x48C)
#define CM_CLKMODE_DPLL_CORE              (PRCM_BASE_ADDR + 0x490)
#define CM_CLKMODE_DPLL_DDR               (PRCM_BASE_ADDR + 0x494)
#define CM_CLKMODE_DPLL_DISP              (PRCM_BASE_ADDR + 0x498)

#define CM_DIV_M2_DPLL_DDR                (PRCM_BASE_ADDR + 0x4A0)
#define CM_DIV_M2_DPLL_DISP               (PRCM_BASE_ADDR + 0x4A4)
#define CM_DIV_M2_DPLL_MPU                (PRCM_BASE_ADDR + 0x4A8)
#define CM_DIV_M2_DPLL_PER                (PRCM_BASE_ADDR + 0x4AC)
#define CM_DIV_M6_DPLL_CORE               (PRCM_BASE_ADDR + 0x4D8)

#define CM_CLKOUT_CTRL                    (PRCM_BASE_ADDR + 0x700)

/*	PLL Settings for OPP50	*/

#define     PM_OPP50_MPU_M                275
#define     PM_OPP50_MPU_N                23
#define     PM_OPP50_MPU_M2               1

#define     PM_OPP50_CORE_M               50
#define     PM_OPP50_CORE_N               23
#define     PM_OPP50_CORE_M4              1
#define     PM_OPP50_CORE_M5              1
#define     PM_OPP50_CORE_M6              1

#define     PM_OPP50_PER_M                960
#define     PM_OPP50_PER_N                23
#define     PM_OPP50_PER_M2               10

/*	PLL Settings for OPP100	*/

#define     PM_OPP_SRTURBO_MPU_M          720
#define     PM_OPP_SRTURBO_MPU_N          23
#define     PM_OPP_SRTURBO_MPU_M2         1

#define     PM_OPP100_CORE_M              1000
#define     PM_OPP100_CORE_N              23
#define     PM_OPP100_CORE_M4             10
#define     PM_OPP100_CORE_M5             8
#define     PM_OPP100_CORE_M6             4

#define     PM_OPP100_PER_M               960
#define     PM_OPP100_PER_N               23
#define     PM_OPP100_PER_M2              5

#define     PM_OPP100_DISP_M              48
#define     PM_OPP100_DISP_N              23
#define     PM_OPP100_DISP_M2             1

/*	Voltage rails	*/

#define		PM_RAIL_MPU					  1
#define		PM_RAIL_CORE				  2

/*
	Vout = (SEL[6:0] × 12.5 mV + 562.5 mV) × Gain
	0.95 = (SEL[6:0] × 12.5 mV + 562.5 mV) × 1
	=> SEL[6:0] = 0011111b = 0x1F
*/
#define 	PM_PMIC_CONFIG_0_95V		0x1F
/*
	Vout = (SEL[6:0] × 12.5 mV + 562.5 mV) × Gain
	1.1 = (SEL[6:0] × 12.5 mV + 562.5 mV) × 1
	=> SEL[6:0] = 0101011b = 0x2B
*/
#define 	PM_PMIC_CONFIG_1_1V			0x2B
#define 	PM_PMIC_CONFIG_1_26V		0x38

#define 	PM_PER_PLL_SD_DIV			0x04000000
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

