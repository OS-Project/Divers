/**
 * \file    demoMain.c
 *
 * \brief   This file contains the main() and other functions.
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


#include "interrupt.h"
#include "demoMain.h"
#include "soc_AM335x.h"
#include "demoCfg.h"
#include "demoTimer.h"
#include "demoEnet.h"
#include "demoLedIf.h"
#include "demoRtc.h"
#include "cache.h"
#include "mmu.h"
#include "beaglebone.h"
#include "uartStdio.h"
#include "demoSdRw.h"
#include "demoSdFs.h"
#include "delay.h"
#include <string.h>
#include "mmcsd_proto.h"
#include "gpio_v2.h"
#include "pin_mux.h"

/****************************************************************************
**                   INTERNAL MACRO DEFINITIONS                                       
****************************************************************************/
#define NUM_OF_IMAGES                         (9u)
#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)
 
/****************************************************************************
**                   LOCAL FUNCTION PROTOTYPES                                
****************************************************************************/
static void EnetStatusCheckNUpdate(void);
static void PeripheralsSetUp(void);
static void ClickAction(void);
static void ContextReset(void);
static void dummyIsr(void);
static void MMUConfigAndEnable(void);

/****************************************************************************
**                   EXTERNAL VARIABLE DECLARATIONS                             
****************************************************************************/
extern unsigned int ipAddr;
extern volatile tBoolean bConnected;

/*******************************************************************************
**                     EXTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
extern void etharp_tmr(void);

/****************************************************************************
**                  GLOBAL VARIABLES DEFINITIONS                                         
****************************************************************************/
unsigned int clickIdx = 0;

#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, MMU_PAGETABLE_ALIGN_SIZE);
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=MMU_PAGETABLE_ALIGN_SIZE
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY];
#elif defined(gcc)
static volatile unsigned int pageTable[MMU_PAGETABLE_NUM_ENTRY] 
            __attribute__((aligned(MMU_PAGETABLE_ALIGN_SIZE)));
#else
#error "Unsupported Compiler. \r\n"
#endif

/****************************************************************************
**                      FUNCTION DEFINITIONS                                         
****************************************************************************/
/*
** Function to setup MMU. This function Maps three regions (1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes
    ** specified here are,
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                     MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                      MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}

/*
** Enable all the peripherals in use
*/
static void PeripheralsSetUp(void)
{
    DMTimer2ModuleClkConfig();
    DMTimer3ModuleClkConfig();
    DMTimer4ModuleClkConfig();
    RTCModuleClkConfig();
    CPSWPinMuxSetup();
    CPSWClkEnable();
    EDMAModuleClkConfig();
    GPIO1ModuleClkConfig();
    GPIO1Pin23PinMuxSetup();
    HSMMCSDPinMuxSetup();
    HSMMCSDModuleClkConfig();
}

/*
** Resets the state
*/
static void ContextReset(void)
{
    tmrFlag  = FALSE;
    LedOff();
    rtcSetFlag = FALSE;
    rtcSecUpdate = FALSE;
    sdCardAccessFlag = FALSE;
}

/*
** Take the actions on click.
*/
static void ClickAction(void)
{
    switch(clickIdx)
    {
        case CLICK_IDX_RTC:
            sdCardAccessFlag = FALSE;
            tmrFlag  = FALSE;
            tmrClick = FALSE;
            LedOff();
            RtcTimeCalSet(); 
        break;
        
        case CLICK_IDX_LED:
            sdCardAccessFlag = FALSE;
            Timer2Stop();
            LedOn();
            tmrClick = FALSE;
            rtcSetFlag = FALSE;
        break;

        case CLICK_IDX_TIMER:
            sdCardAccessFlag = FALSE;
            rtcSetFlag = FALSE;
            tmrClick = TRUE;
            tmrFlag  = FALSE;
            Timer2Start();
        break;
        
        case CLICK_IDX_SD:
            rtcSetFlag = FALSE;
            tmrClick = FALSE;
            tmrFlag  = FALSE;
            LedOff();

            if(TRUE == HSMMCSDCardPresentStat())
            {
                sdCardAccessFlag = TRUE;
            }
            else
            {
                sdCardAccessFlag = FALSE;
                UARTPuts("\n\rSD card not present. Please insert an", -1);
                UARTPuts(" SD card and try again! \n\r", -1);
            }
        break;

        default:
        break;
    }  
}

/*
** Main function. The application starts here.
*/
int main(void)
{
    MMUConfigAndEnable();    

    /* Enable Instruction Cache */
    CacheEnable(CACHE_ALL);

    PeripheralsSetUp();

    /* Initialize the ARM Interrupt Controller */
    IntAINTCInit();

    /* Register the ISRs */  
    Timer2IntRegister();
    Timer4IntRegister();
    EnetIntRegister();
    RtcIntRegister();
    HSMMCSDIntRegister();
    IntRegister(127, dummyIsr);

    IntMasterIRQEnable();

    /* Enable system interrupts */
    IntSystemEnable(SYS_INT_RTCINT);
    IntPrioritySet(SYS_INT_RTCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT2);
    IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT4);
    IntPrioritySet(SYS_INT_TINT4, 0, AINTC_HOSTINT_ROUTE_IRQ);	
    IntSystemEnable(SYS_INT_MMCSD0INT);
    IntPrioritySet(SYS_INT_MMCSD0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_EDMACOMPINT);
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(127);
    IntPrioritySet(127, 0, AINTC_HOSTINT_ROUTE_IRQ);

    RtcInit();
    UARTStdioInit();
    HSMMCSDContolInit();
    DelayTimerSetup();

    Timer2Config();
    Timer4Config();
    LedIfConfig();

    Timer2IntEnable();
    Timer4IntEnable();
    RtcSecIntEnable();
    	
    Timer4Start(); 

    /*
    ** Loop for ever. Necessary actions shall be taken
    ** after detecting the click.
    */
    while(1)
    {
         EnetStatusCheckNUpdate();

         /*
         ** Check if click is detected
         */
         if(clickIdx != 0)
         {
             /*
             ** Take the Action for click
             */
             ClickAction();

             clickIdx = 0;
         }
       
         /*
         ** Check if the Timer Expired
         */ 
         if(TRUE == tmrFlag)
         {
             /* Toggle the LED state */
             LedToggle();
             tmrFlag = FALSE;
         }
 
         /*
         ** Check if RTC Time is set
         */
         if(TRUE == rtcSetFlag)
         {
             if(TRUE == rtcSecUpdate)
             { 
                 rtcSecUpdate = FALSE;
                 RtcTimeCalDisplay();
             }
         } 
   
         /*
         ** Check for SD Card
         */
         if(TRUE == sdCardAccessFlag)
         {
             HSMMCSDCardAccessSetup();
         } 
		 
		 		 
         if(TRUE == tmr4Flag)
         {
            tmr4Flag = FALSE;
             /* Make sure that interrupts are disabled and no lwIP functions
                are executed while calling an lwIP exported API */
             IntMasterIRQDisable();
             etharp_tmr();
             IntMasterIRQEnable();
         }
    }
}

/*
** Check for any change in ethernet link status. If so, update
** ip address
*/
static void EnetStatusCheckNUpdate(void)
{
    unsigned int linkFlag = FALSE;
    static unsigned int prevEnState = 0;
    static unsigned int nxtEnState = 1;

    if(prevEnState != nxtEnState)
    {
        if(!EnetIfIsUp())
        {
            ContextReset();
            linkFlag = FALSE;
            EnetHttpServerInit();

            if(ipAddr)
            {
                linkFlag = TRUE; 
                prevEnState = 1;
            }
        }
        else
        {
            if(EnetLinkIsUp())
            {
                linkFlag = TRUE;
                nxtEnState = 1;
            }
            else
            {
                ContextReset();
                linkFlag = FALSE;
                prevEnState = 0;
                nxtEnState = 0;
            }
        }

        if((TRUE == linkFlag) && (ipAddr != 0))
        {
             prevEnState = 1;
             UARTPuts("\n\rAccess the home page using http://", -1);
             IpAddrDisplay();
             UARTPuts("/index.html \n\r", -1);
        }
        else
        {
            UARTPuts("\n\rNetwork Connection failed.\n\r", -1);
        }
    }
    else
    {
        if(EnetLinkIsUp())
        {  
            nxtEnState = 1;
        }
        else
        {
            nxtEnState = 0;
        }
    }
}

/*
** Dummy ISR to handle spurious interrupts
*/
static void dummyIsr(void)
{
    ; /* Perform nothing */
}

/****************************** End of file *********************************/
  
