/*
 * enet_echo.c
 *
 * Sample application for ethernet. Demonstrates a sample http server
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

#include "locator.h"
#include "echod.h"
#include "interrupt.h"
#include "lwiplib.h"
#include "lwipopts.h"
#include "uartStdio.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "beaglebone.h"
#include "cache.h"
#include "delay.h"
#include "mmu.h"
#include "cache.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/

#define LEN_IP_ADDR                        (4u)
#define ASCII_NUM_IDX                      (48u) 

#define START_ADDR_DDR                     (0x80000000)
#define START_ADDR_DEV                     (0x44000000)
#define START_ADDR_OCMC                    (0x40300000)
#define NUM_SECTIONS_DDR                   (512)
#define NUM_SECTIONS_DEV                   (960)
#define NUM_SECTIONS_OCMC                  (1)

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif


/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void CPSWCore0RxIsr(void);
static void CPSWCore0TxIsr(void);
static void AintcCPSWIntrSetUp(void);
static void IpAddrDisplay(unsigned int ipAddr);
static void MMUConfigAndEnable(void);

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
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
** The main function
*/
int main(void)
{
    unsigned int ipAddr;
    LWIP_IF lwipIfPort1, lwipIfPort2;

    MMUConfigAndEnable();

#ifdef LWIP_CACHE_ENABLED
    CacheEnable(CACHE_ALL);
#endif

    CPSWPinMuxSetup();
    CPSWClkEnable();

    /* Set up the UART peripheral for Standard Input/Output operation.*/
    UARTStdioInit();

    /* Chip configuration RGMII selection */
    EVMPortMIIModeSelect();

    /* Get the MAC address */
    EVMMACAddrGet(0, lwipIfPort1.macArray); 
    EVMMACAddrGet(1, lwipIfPort2.macArray); 

    AintcCPSWIntrSetUp();
    DelayTimerSetup();

    UARTPuts("StarterWare Ethernet Echo Application. \n\r\n\r" , -1);
   
    UARTPuts("Acquiring IP Address for Port 1... \n\r" , -1);

#if STATIC_IP_ADDRESS_PORT1

    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1; 
    lwipIfPort1.ipAddr = STATIC_IP_ADDRESS_PORT1; 
    lwipIfPort1.netMask = 0; 
    lwipIfPort1.gwAddr = 0; 
    lwipIfPort1.ipMode = IPADDR_USE_STATIC; 

    ipAddr = lwIPInit(&lwipIfPort1);

#else

    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1; 
    lwipIfPort1.ipAddr = 0; 
    lwipIfPort1.netMask = 0; 
    lwipIfPort1.gwAddr = 0; 
    lwipIfPort1.ipMode = IPADDR_USE_DHCP; 

    ipAddr = lwIPInit(&lwipIfPort1);

#endif
    if(ipAddr)
    {
        UARTPuts("\n\r\n\rPort 1 IP Address Assigned: ", -1);
        IpAddrDisplay(ipAddr);
    }
    else
    {
        UARTPuts("\n\r\n\rPort 1 IP Address Acquisition Failed.", -1);
    }

    /* Initialize the sample httpd server. */
    echo_init();
   
    /* Loop forever.  All the work is done in interrupt handlers. */
    while(1)
    {
        ; /* Perform nothing */
    }
}

/*
** Interrupt Handler for Core 0 Receive interrupt
*/
static void CPSWCore0RxIsr(void)
{
    lwIPRxIntHandler(0);
}

/*
** Interrupt Handler for Core 0 Transmit interrupt
*/
static void CPSWCore0TxIsr(void)
{
    lwIPTxIntHandler(0);
}

/*
** Displays the IP addrss on the UART Console
*/
static void IpAddrDisplay(unsigned int ipAddr) 
{
    unsigned char byte;
    int cnt;
 
    for(cnt = 0; cnt <= LEN_IP_ADDR - 1; cnt++) 
    {
        byte = (ipAddr >> (cnt * 8)) & 0xFF; 

        if(cnt)
        {        
            UARTPuts(".", -1);
        }
    
        UARTPutNum((int)byte);
    }
}

/*
** Set up the ARM Interrupt Controller for generating timer interrupt
*/
static void AintcCPSWIntrSetUp(void)
{
    /* Enable IRQ for ARM (in CPSR)*/
    IntMasterIRQEnable();

    IntAINTCInit();
    
    /* Register the Receive ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWRXINT0, CPSWCore0RxIsr);
  
    /* Register the Transmit ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWTXINT0, CPSWCore0TxIsr);
    
    /* Set the priority */
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
}

/***************************** End Of File ***********************************/
