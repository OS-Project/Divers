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
#include "demoRaster.h"
#include "demoTouch.h"
#include "demoMain.h"
#include "soc_AM335x.h"
#include "demoToneLoop.h"
#include "demoCfg.h"
#include "demoAic31.h"
#include "demoTimer.h"
#include "demoEnet.h"
#include "demoRtc.h"
#include "evmskAM335x.h"
#include "uartStdio.h"
#include "demoEcap.h"
#include "demoGpio.h"
#include "cache.h"
#include "mmu.h"

#include "demoPwrMgmnt.h"
#include "demoI2c.h"
#include "demoGrlib.h"
#include "grlib.h"
#include "bannerImage.h"
#include "baseImage.h"
#include "delay.h"
#include "clock.h"
#include "uart_irda_cir.h"
#include "demoDvfs.h"
#include "rtc.h"

#include "pin_mux.h"
#include <string.h>

/****************************************************************************
**                   INTERNAL MACRO DEFINITIONS                                       
****************************************************************************/
#define NUM_OF_IMAGES                         (15u)

#define START_ADDR_DDR             (0x80000000)
#define START_ADDR_DEV             (0x44000000)
#define START_ADDR_OCMC            (0x40300000)
#define NUM_SECTIONS_DDR           (512)
#define NUM_SECTIONS_DEV           (960)
#define NUM_SECTIONS_OCMC          (1)
 
/****************************************************************************
**                   LOCAL FUNCTION PROTOTYPES                                
****************************************************************************/
static void CoOrdAction(int x, int y);
static void PeripheralsSetUp(void);
static void ClickAction(void);
static void PrevContextClear(void);
static void ActionIdle(void);
static void ActionDefault(void);
static void ActionTimer(void);
static void ActionTimeSet(void);
static void ActionEnetInit(void);
static void ActionECAP(void);
static void ActionECAPdemo(void);
static void ActionI2C(void);
static void MMUConfigAndEnable(void);
static void dummyIsr(void);
static void uartIsr(void);
static void gpioIsr(void);
static void ActionMenu(void);
static void ActionMenuIntro(void);
static void ActionMenuWebDemo(void);
static void ActionMenuMcASP(void);
static void ActionMenuMMCSD(void);
static void ActionMenuUart(void);
static void ActionMenuRTC(void);
static void ActionMenuEthernet(void);
static void ActionMenuTimer(void);
static void ActionMenuECAP(void);
static void ActionMenuGPIO(void); 
static void ActionMenuI2C(void);
static void ActionMenuPM(void);
static void toggleColors(void);
static void ActionMenuDVFS(void);


/*******************************************************************************
**                     EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern unsigned char g_pucBuffer[2][GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED)];
extern unsigned char baseUnCompImage[];

extern tContext sContext[];

extern unsigned int ipAddr;
extern volatile tBoolean bConnected;
extern unsigned int iram_start;
extern unsigned int iram_size;
extern unsigned int relocstart;
extern unsigned int relocend;
extern unsigned int printtemp;
extern unsigned int ipAddr1;
extern unsigned int ipAddr2;
extern volatile unsigned int wakeSource;

/*******************************************************************************
**                     INTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
volatile unsigned int ledBlinkFlag;

/*******************************************************************************
**                     EXTERNAL FUNCTION DECLARATIONS
*******************************************************************************/
extern void etharp_tmr(void);
extern void romRestoreLocation(void);

/****************************************************************************
**                  GLOBAL VARIABLES DEFINITIONS                                         
****************************************************************************/
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/*
** Coordinates for each icon.
*/
int const xyDefault[4] =  {0, 0, 0, 0};

int const xyNext[4] = {XMIN_NEXT, XMAX_NEXT, YMIN_NEXT, YMAX_NEXT};

int const xyPrev[4] = {XMIN_PREV, XMAX_PREV, YMIN_PREV, YMAX_PREV};    
         
int const xyHome[4] = {XMIN_HOME, XMAX_HOME, YMIN_HOME, YMAX_HOME};

int const xyTimeSet[4] = {XMIN_RTC_STD, XMAX_RTC_STD,
                          YMIN_RTC_STD, YMAX_RTC_STD};
                          
int const xyIntro[4] =  { XMIN_INTRO, XMAX_INTRO, YMIN_INTRO, YMAX_INTRO};

int const xyWebDemo[4] = { XMIN_WEB_DEMO, XMAX_WEB_DEMO,
                           YMIN_WEB_DEMO, YMAX_WEB_DEMO};
                           
int const xyMcASP[4] = { XMIN_MCASP, XMAX_MCASP, YMIN_MCASP, YMAX_MCASP};

int const xyUart[4] =  { XMIN_UART, XMAX_UART, YMIN_UART, YMAX_UART}; 

int const xyRTC[4] = { XMIN_RTC, XMAX_RTC, YMIN_RTC, YMAX_RTC}; 

int const xyTimer[4] = { XMIN_TIMER, XMAX_TIMER, YMIN_TIMER, YMAX_TIMER};

int const xyEthernet[4] = { XMIN_ETHERNET, XMAX_ETHERNET, 
                            YMIN_ETHERNET, YMAX_ETHERNET }; 

int const xyMMCSD[4] = { XMIN_MMCSD, XMAX_MMCSD, 
                                   YMIN_MMCSD, YMAX_MMCSD }; 
								   
int const xyEcapDemo[4] = {XMIN_ECAP, XMAX_ECAP,
                       YMIN_ECAP, YMAX_ECAP};								   

int const xyEcapMenu[4] = {XMIN_ECAP_MENU, XMAX_ECAP_MENU,
						   YMIN_ECAP_MENU, YMAX_ECAP_MENU};	
						   
int const xyGpioMenu[4] = {XMIN_GPIO_MENU, XMAX_GPIO_MENU,
						   YMIN_GPIO_MENU, YMAX_GPIO_MENU};	
						   
int const xyI2CMenu[4] = {XMIN_I2C_MENU, XMAX_I2C_MENU,
						  YMIN_I2C_MENU, YMAX_MOUSE_LEFT};	

int const xyPMMenu[4] = {XMIN_PM_MENU, XMAX_PM_MENU,
						 YMIN_PM_MENU, YMAX_PM_MENU};	

int const xyPMds0Demo[4] = {XMIN_PM_DS0_DEMO, XMAX_PM_DS0_DEMO,
                           YMIN_PM_DS0_DEMO, YMAX_PM_DS0_DEMO};								 

int const xyPMds1Demo[4] = {XMIN_PM_DS1_DEMO, XMAX_PM_DS1_DEMO,
                           YMIN_PM_DS1_DEMO, YMAX_PM_DS1_DEMO};							   
						   
int const xyPMds2Demo[4] = {XMIN_PM_DS2_DEMO, XMAX_PM_DS2_DEMO,
                           YMIN_PM_DS2_DEMO, YMAX_PM_DS2_DEMO};		

int const xyPMds1DevDemo[4] = {XMIN_PM_DS1DEV_DEMO, XMAX_PM_DS1DEV_DEMO,
                           YMIN_PM_DS1DEV_DEMO, YMAX_PM_DS1DEV_DEMO};		

int const xyPMwksTsc[4] = {XMIN_PM_WAKE_TSC, XMAX_PM_WAKE_TSC,
                           YMIN_PM_WAKE_TSC, YMAX_PM_WAKE_TSC};							   						   						   
						   
int const xyPMwksTmr[4] = {XMIN_PM_WAKE_TMR, XMAX_PM_WAKE_TMR,
                           YMIN_PM_WAKE_TMR, YMAX_PM_WAKE_TMR};							   
						   
int const xyPMwksUart[4] = {XMIN_PM_WAKE_UART, XMAX_PM_WAKE_UART,
                           YMIN_PM_WAKE_UART, YMAX_PM_WAKE_UART};		

int const xyPMwksGpio[4] = {XMIN_PM_WAKE_GPIO, XMAX_PM_WAKE_GPIO,
                           YMIN_PM_WAKE_GPIO, YMAX_PM_WAKE_GPIO};						   

int const xyDVFSMenu[4] = {XMIN_DVFS_MENU, XMAX_DVFS_MENU,
                           YMIN_DVFS_MENU, YMAX_DVFS_MENU};


int const xyDVFSOpp50[4] = {XMIN_DVFS_OPP50, XMAX_DVFS_OPP50,
                            YMIN_DVFS_OPP50, YMAX_DVFS_OPP50};
int const xyDVFSOpp100[4] = {XMIN_DVFS_OPP100, XMAX_DVFS_OPP100,
                            YMIN_DVFS_OPP100, YMAX_DVFS_OPP100};
int const xyDVFSOpp120[4] = {XMIN_DVFS_OPP120, XMAX_DVFS_OPP120,
                            YMIN_DVFS_OPP120, YMAX_DVFS_OPP120};
int const xyDVFSSrTurbo[4] = {XMIN_DVFS_SRTURBO, XMAX_DVFS_SRTURBO,
                            YMIN_DVFS_SRTURBO, YMAX_DVFS_SRTURBO};

/*
** Specifications for each icon
*/
static TOUCHSPEC const touchSpecBanner[NUM_ICON_BANNER] =
                    {
                       {xyDefault, ActionIdle}
                    };

static TOUCHSPEC const touchSpecMenu[NUM_ICON_MENU] =
                    {
                       {xyIntro, ActionMenuIntro},
                       {xyWebDemo, ActionMenuWebDemo},
                       {xyMcASP, ActionMenuMcASP},
					   {xyMMCSD, ActionMenuMMCSD},
                       {xyUart, ActionMenuUart},
                       {xyRTC, ActionMenuRTC},
                       {xyTimer, ActionMenuTimer},
                       {xyEthernet, ActionMenuEthernet},                                   
                       {xyEcapMenu, ActionMenuECAP},                                   
					   {xyGpioMenu, ActionMenuGPIO},  
					   {xyI2CMenu, ActionMenuI2C},  
					   {xyPMMenu, ActionMenuPM},  
					   {xyDVFSMenu, ActionMenuDVFS},  
                    };
                    
static TOUCHSPEC const touchSpecIntro[NUM_ICON_INTRO] = 
                    {
                       {xyNext, ActionEnetInit},
                       {xyPrev, ActionMenu},
                       {xyHome, ActionMenu}
                    };

static TOUCHSPEC const touchSpecUart[NUM_ICON_UART] = 
                    {
                       {xyNext, ActionDefault},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu}
                    };
					
static TOUCHSPEC const touchSpecRtc[NUM_ICON_RTC] = 
                    {
                       {xyNext, ActionTimer},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu},
                       {xyTimeSet, ActionTimeSet}                       
                    };
static TOUCHSPEC const touchSpecDefault[NUM_ICON_DFLT] = 
                    {
                       {xyNext, ActionDefault},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu}
                    };
static TOUCHSPEC const touchSpecMcASP[NUM_ICON_MCASP] = 
                    {
                       {xyNext, ActionDefault},
                       {xyPrev, ActionEnetInit},
                       {xyHome, ActionMenu}
                    };
                    
static TOUCHSPEC const touchSpecTimer[NUM_ICON_MCASP] = 
                    {
                       {xyNext, ActionDefault},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu}            
                    };
static TOUCHSPEC const touchSpecEthernet[NUM_ICON_ENET] =
                    {
                       {xyNext, ActionECAP}, // can be changed to default
                       {xyPrev, ActionTimer},
                       {xyHome, ActionMenu}
                    };      

static TOUCHSPEC const touchSpecECAP[NUM_ICON_ECAP] =
                    {
                       {xyNext, ActionDefault},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu},
                       {xyEcapDemo, ActionECAPdemo},
                    };  					
					
static TOUCHSPEC const touchSpecGPIO[NUM_ICON_GPIO] =
                    {
                       {xyNext, ActionI2C},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu},
                    };  	

static TOUCHSPEC const touchSpecI2C[NUM_ICON_I2C] =
                    {
                       {xyNext, ActionDefault},
                       {xyPrev, ActionDefault},
                       {xyHome, ActionMenu},
                    };  						
					
static TOUCHSPEC const touchSpecPM[NUM_ICON_PM] =
                    {
                       {xyNext, ActionMenuDVFS},
                       {xyPrev, ActionI2C},
                       {xyHome, ActionMenu},
                       {xyPMds0Demo, ActionDeepSleep0},
                       {xyPMds1Demo, ActionDeepSleep1},
                       {xyPMds2Demo, ActionDeepSleep2},
                       {xyPMds1DevDemo, ActionDeepSleep1Dev},
                       {xyPMwksTsc, ActionWakeOnTsc},
                       {xyPMwksTmr, ActionWakeOnTimer},
                       {xyPMwksUart, ActionWakeOnUart},
                       {xyPMwksGpio, ActionWakeOnGpio},
                    };  

static TOUCHSPEC const touchSpecDVFS[NUM_ICON_DVFS] =
                    {
                       {xyNext, ActionMenu},
                       {xyPrev, ActionMenuPM},
                       {xyHome, ActionMenu},
                       {xyDVFSOpp50, ActionDVFSOpp50},
                       {xyDVFSOpp100, ActionDVFSOpp100},
                       {xyDVFSOpp120, ActionDVFSOpp120},
                       {xyDVFSSrTurbo, ActionDVFSSrTurbo},
                    };
							
                    
/*
** Context information.
** Image, number of icons in the image, specification.
** The next icon will come as the last specification for each image.
*/
IMAGECONTEXT contextInfo[NUM_OF_IMAGES] =             
                    {
                       {bannerImage, NUM_ICON_BANNER, 	touchSpecBanner},
                       {bannerImage, NUM_ICON_MENU, 	touchSpecMenu},
                       {bannerImage, NUM_ICON_INTRO, 	touchSpecIntro},
                       {bannerImage, NUM_ICON_CHOICE, 	touchSpecDefault},
                       {bannerImage, NUM_ICON_MCASP, 	touchSpecMcASP},
	               {bannerImage, NUM_ICON_MMCSD, 	touchSpecDefault},
                       {bannerImage, NUM_ICON_UART, 	touchSpecUart},
                       {bannerImage, NUM_ICON_RTC, 		touchSpecRtc},
                       {bannerImage, NUM_ICON_TMR, 		touchSpecTimer},                       
                       {bannerImage, NUM_ICON_ENET, 	touchSpecEthernet},                       
                       {bannerImage, NUM_ICON_ECAP, 	touchSpecECAP},                       
                       {bannerImage, NUM_ICON_GPIO, 	touchSpecGPIO},                       
                       {bannerImage, NUM_ICON_I2C, 		touchSpecI2C},                       
                       {bannerImage, NUM_ICON_PM, 		touchSpecPM},                       
                       {bannerImage, NUM_ICON_DVFS, 		touchSpecDVFS},                       
                    };

/*
** The variable which is used to keep track of the current slide.
*/
volatile unsigned int imageCount = 0;

/*
** The variable which is used to keep track of the current Frame buffer.
*/
volatile unsigned int frameBufIdx = 0;

/*
** The variable which indicates which peripheral demo to be shown.
*/
unsigned int clickIdx = 0;

/*
** The variable which indicates whether ethernet is initialized.
*/
unsigned int enetInitFlag = FALSE;

/*
** The variable which indicates whether I2C slide is active.
*/
unsigned int I2CDemoFlag = FALSE;

/*
** The variable which indicates whether an action is called as a result of prev
** button click.
*/
unsigned int prevAction = 0;

/****************************************************************************
**                      FUNCTION DEFINITIONS                                         
****************************************************************************/
/*
** Function to setup MMU. This function Maps three regions ( 1. DDR
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
    enableModuleClock(CLK_UART0);
	enableModuleClock(CLK_LCDC);
	enableModuleClock(CLK_TIMER2);
	enableModuleClock(CLK_TIMER3);
	enableModuleClock(CLK_TIMER4);
	//enableModuleClock(CLK_TIMER7);
    enableModuleClock(CLK_I2C0);	
	enableModuleClock(CLK_I2C1);
	enableModuleClock(CLK_MCASP1);
	//enableModuleClock(CLK_GPIO1);
	//enableModuleClock(CLK_GPIO0);
	enableModuleClock(CLK_ADC_TSC);
	enableModuleClock(CLK_TPTC2);
	enableModuleClock(CLK_TPTC1);
	enableModuleClock(CLK_TPTC0);
	enableModuleClock(CLK_TPCC);
	
	GPIO1ModuleClkConfig();
	GPIO0ModuleClkConfig();
	
    EDMAModuleClkConfig();	

    RTCModuleClkConfig();
    CPSWClkEnable();	

    LCDPinMuxSetup();
    CPSWPinMuxSetup();
	I2CPinMuxSetup(0);
    I2CPinMuxSetup(1);
    McASP1PinMuxSetup();
    TSCADCPinMuxSetUp();
    ECAPPinMuxSetup(2);

    GPIO_PMUX_OFFADDR_VALUE(0, 30,PAD_FS_RXE_PU_PUPDE(7));
	
    GPIO1Pin4PinMuxSetup();
}

/*
** Dummy ISR to handle spurious interrupts
*/
static void dummyIsr(void)
{
    ; /* Perform nothing */
}

/*
** Uart ISR to read the inputs
*/
static void uartIsr(void)
{
	volatile unsigned char rxByte;
    ; /* Perform nothing */
	rxByte = UARTCharGetNonBlocking(SOC_UART_0_REGS);
	UARTCharPutNonBlocking(SOC_UART_0_REGS, rxByte);
}


/*
** Uart ISR to read the inputs
*/
static void gpioIsr(void)
{
	/*	Clear wake interrupt	*/
	HWREG(SOC_GPIO_0_REGS + 0x2C) = 0x40000000;
	HWREG(SOC_GPIO_0_REGS + 0x30) = 0x40000000;
}

/*
** This function clears any previous context info
*/
static void PrevContextClear(void)
{
    /* Close the previous state */
    Timer2Stop();
    tmr2Flag  = FALSE;
    rtcSetFlag = FALSE;
	I2CDemoFlag = FALSE;
}

/*
** Take the actions on the touch.
** The coordinates are given by the parameters
*/
static void CoOrdAction(int x, int y)
{
    int const *coOrd;
    unsigned int cnt;
    unsigned int numIcon = contextInfo[imageCount].numIcon;
    TOUCHSPEC const *touchSpec;

    for(cnt = 0; cnt < numIcon; cnt++)
    {
        touchSpec = contextInfo[imageCount].touchSpec + cnt;
        coOrd = touchSpec->coOrd;

        /* Take action only for proper coordinates */
        if((x >= coOrd[0]) && (x <= coOrd[1]) &&
           ((y >= coOrd[2]) && (y <= coOrd[3])))
        {
            PrevContextClear();            

            if((1 == cnt) && (1 != imageCount))
            {
                prevAction = 1;
            }
            (touchSpec->action)();
            break;
        }
    }    
}

/*
** Take the actions on click. 
*/
static void ClickAction(void)
{
    TOUCHSPEC const *clickSpec;

    /*
    ** Get the spec. Assumed that the last touch spec only will give 
    ** action for the next image.
    */
    clickSpec = contextInfo[clickIdx - 1].touchSpec;

    PrevContextClear();

    (clickSpec->action)();
}

/*
** Main function. The application starts here.
*/
int main(void)
{
    int x;
    int y;    

    #ifdef __TMS470__
    /* Relocate the required section to internal RAM */
    memcpy((void *)(&relocstart), (const void *)(&iram_start),
           (unsigned int)(&iram_size));
    #elif defined(__IAR_SYSTEMS_ICC__)
    #pragma section = "CodeRelocOverlay"
    #pragma section = "DataRelocOverlay"
    #pragma section = "DataOverlayBlk"
    #pragma section = "CodeOverlayBlk"
    char* srcAddr = (__section_begin("CodeRelocOverlay"));
    char* endAddr = (__section_end("DataRelocOverlay"));

    memcpy((void *)(__section_begin("CodeRelocOverlay")),
           (const void *)(__section_begin("CodeOverlayBlk")),
           endAddr - srcAddr);

    #else
    memcpy((void *)&(relocstart), (const void *)&(iram_start),
           (unsigned int)(((&(relocend)) -
            (&(relocstart))) * (sizeof(unsigned int))));
    #endif

    MMUConfigAndEnable();

    /* Enable cache */
    CacheEnable(CACHE_ALL);

    PeripheralsSetUp();

    /* Initialize the ARM Interrupt Controller */
    IntAINTCInit();

    /* Register the ISRs */  
    Raster0IntRegister();
    Timer2IntRegister();
    Timer4IntRegister();
    EnetIntRegister();
    RtcIntRegister();
    TouchIntRegister();
    CM3IntRegister();
    IntRegister(127, dummyIsr); 
 
    IntMasterIRQEnable();

    /* Enable system interrupts */
    IntPrioritySet(SYS_INT_LCDCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_LCDCINT);
    IntPrioritySet(SYS_INT_RTCINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_RTCINT);
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
    IntPrioritySet(SYS_INT_TINT2, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT2);
    IntPrioritySet(SYS_INT_TINT3, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_TINT3);
    IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
    IntPrioritySet(SYS_INT_M3_TXEV, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntSystemEnable(SYS_INT_M3_TXEV);
    IntSystemEnable(127);
    IntPrioritySet(127, 0, AINTC_HOSTINT_ROUTE_IRQ);
	
	IntSystemEnable(SYS_INT_UART0INT);
    IntPrioritySet(SYS_INT_UART0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
	IntRegister(SYS_INT_UART0INT, uartIsr);
	
	/*	GPIO interrupts	*/
	IntSystemEnable(SYS_INT_GPIOINT0A);
    IntPrioritySet(SYS_INT_GPIOINT0A, 0, AINTC_HOSTINT_ROUTE_IRQ);
	IntRegister(SYS_INT_GPIOINT0A, gpioIsr);
	IntSystemEnable(SYS_INT_GPIOINT0B);
    IntPrioritySet(SYS_INT_GPIOINT0B, 0, AINTC_HOSTINT_ROUTE_IRQ);
	IntRegister(SYS_INT_GPIOINT0B, gpioIsr);
	
    CM3EventsClear();
    CM3LoadAndRun();
    waitForM3Txevent();    

    DelayTimerSetup();
    EcapInit();

    UARTStdioInit();
    Raster0Init();

    I2CIntRegister(I2C_0);
    I2CIntRegister(I2C_1);
    IntPrioritySet(SYS_INT_I2C0INT, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntPrioritySet(SYS_INT_I2C1INT, 0, AINTC_HOSTINT_ROUTE_IRQ );
    IntSystemEnable(SYS_INT_I2C0INT);
    IntSystemEnable(SYS_INT_I2C1INT);		
    I2CInit(I2C_0);
    I2CInit(I2C_1);
	
	IntSystemEnable(SYS_INT_TINT1_1MS);
    IntPrioritySet(SYS_INT_TINT1_1MS, 0, AINTC_HOSTINT_ROUTE_IRQ);
	IntRegister(SYS_INT_TINT1_1MS,clearTimerInt);	

    AudioCodecInit();
    ToneLoopInit();

    gpioLedInit();
	
    configVddOpVoltage();
    TouchInit();

    RtcInit();

	configWakeGpio();

	initializeTimer1();
    Timer2Config();
    Timer4Config();
    GrlibInit();
	initAccelerometer();
    EthernetInit();

    MailBoxInit();

    Raster0EOFIntEnable();
    Timer2IntEnable();
    Timer4IntEnable();
    RtcSecIntEnable();
    TouchIntEnable();


    imageCount = 0;
    prevAction = 0;
	
    frameBufIdx = 0;
	
    /* Extract banner image to Frame buffer */
    ImageArrExtract(bannerImage,  
                    (unsigned int*)(g_pucBuffer[!frameBufIdx]+PALETTE_OFFSET));

    CacheDataCleanBuff((unsigned int) &g_pucBuffer[0]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
    CacheDataCleanBuff((unsigned int) &g_pucBuffer[1]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));

    Raster0Start();
    EcapBkLightEnable();

    Timer4Start();
    while(FALSE == tmr4Flag);
    tmr4Flag = FALSE;
    Timer4Stop();
	
    /* Extract base image to uncomp buffer */
    ImageArrExtract(baseImage, (unsigned int*)baseUnCompImage);
  
    /* Copy base image to FB */
    memcpy((void *)((g_pucBuffer[frameBufIdx]+PALETTE_OFFSET)), 
           (const void *)baseUnCompImage, (LCD_SIZE+PALETTE_SIZE));

    CacheDataCleanBuff((unsigned int) &g_pucBuffer[0]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
    CacheDataCleanBuff((unsigned int) &g_pucBuffer[1]+PALETTE_OFFSET,
          GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
	
    /* Start playing the tone */
    AudioTxActivate();

    /*	Create menu page */
    imageCount = 1;
    updatePage(imageCount);
      
    mpuOpp = OppGet();
	
    TouchEnable();

    /*
    ** Loop for ever. Necessary actions shall be taken
    ** after detecting touch, based on the coordinates
    */
	TouchReleaseDetect();
    while(1)
    {
		// Enter sleep mode (DS0) by entering 't' in the serial terminal.
		// This can be used to automate sleep/wake cycle for testing
		//#ifdef POWER_MEASURE
			if('t' == UARTCharGetNonBlocking(SOC_UART_0_REGS))
			{
                            if(WAKE_SOURCE_UART != wakeSource)
                            {
                                ActionWakeOnUart();
			    }
                            ActionDeepSleep0();
                        }
		//#endif
	
         /*
         ** Check for touch and released on any icon
         */
         if (TRUE == TouchReleaseDetect())
         {
             TouchCoOrdGet(&x, &y);

             /*
             ** Validate the coordinates and take action 
             */
             CoOrdAction(x, y);
         }

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
         if(TRUE == tmr2Flag)
         {
             tmr2Flag = FALSE;
             toggleColors();
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
         ** Timer demo
         */
         if(TRUE == tmr4Flag)
	     {
             tmr4Flag = FALSE;
 
             /* Make sure that interrupts are disabled and no lwIP functions
                are executed while calling an lwIP exported API */
             IntMasterIRQDisable();
             etharp_tmr();
             IntMasterIRQEnable();
         }
		 
         /*
         ** I2C demo (Temperature Sensor, Accelerometer)
         */
		 if(TRUE == I2CDemoFlag)
	     {
			updateI2CDemo();
         }

         if(TRUE == ledBlinkFlag)
         {
             ledBlinkFlag = FALSE;

             gpioLedBlink();
         }
    }
}

/*
** Action to be taken when the demo is to be driven via Ethernet
*/
static void ActionEnetInit(void)
{
    char ipMsg[60] = {"http://"};
    unsigned int i_index, i_msg = 7, ipByte = 0;
    unsigned int Port1linkFlag = FALSE;	
    unsigned int Port2linkFlag = FALSE;	
 
    if(prevAction)
    {
        /* Go to prev image */
        imageCount--;
    }
    else
    {
        /* Go to next image */
        (0 == clickIdx) ? imageCount++ : (imageCount = clickIdx); 
    }
    prevAction = 0;
	
    GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm18b);
    GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
    GrStringDrawCentered(&sContext[frameBufIdx], "Checking Ethernet link and acquiring IP address...", -1, 228,
		        (42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    updatePage(imageCount);
   
    if(!EnetPort1IfIsUp())
    {
        Port1linkFlag = FALSE;
        EnetPort1HttpServerInit();

        if(ipAddr1)
        {
            Port1linkFlag = TRUE;
        }
    }
    else
    {
        if(EnetPort1LinkIsUp())
        {
            Port1linkFlag = TRUE;
        }
   
        else
        {
            Port1linkFlag = FALSE;
        }
    }

    if(!EnetPort2IfIsUp())
    {
        Port2linkFlag = FALSE;
        EnetPort2HttpServerInit();

        if(ipAddr2)
        {
            Port2linkFlag = TRUE;
        }
    }
    else
    {
        if(EnetPort2LinkIsUp())
        {
            Port2linkFlag = TRUE;
        }
        else
        {
            Port2linkFlag = FALSE;
        }
    }


 
    GrContextFontSet(&sContext[frameBufIdx], &g_sFontCm14b);
    GrContextForegroundSet(&sContext[frameBufIdx], ClrRed);
    if((TRUE == Port1linkFlag) && (ipAddr1 != 0))
    {
        UARTPuts("\n\r\n\rTo control via Port1 of Ethernet, use the IP address ", -1);
                IpAddrDisplay(ipAddr1);
         GrStringDrawCentered(&sContext[frameBufIdx], "To control via Port1 of Ethernet, type the address", -1, 222,
			     (42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
			
        for(i_index = 0; i_index < 4; i_index++)
        {
            ipByte = 0x000000FF & (ipAddr1 >> ((i_index)*8) );

            if(ipByte/100)
            {
                ipMsg[i_msg++] = (ipByte/100) + 48;
                ipByte = ipByte%100;
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte/10)
            {
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte)
            {
                ipMsg[i_msg++] = ipByte + 48;
            }

            ipMsg[i_msg++] = '.';
        }
        ipMsg[--i_msg] = '\0';

        strcat(ipMsg, "/index.html");
        GrStringDrawCentered(&sContext[frameBufIdx], ipMsg, -1, 135,
					(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);	
	GrStringDrawCentered(&sContext[frameBufIdx], " in the host web", -1, 280,
					(42 + (5*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
        GrStringDrawCentered(&sContext[frameBufIdx], "browser to access embedded demo page on the target.", -1, 215,
					(42 + (6*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);					
    }
    else
    {
        UARTPuts("\n\rNetwork Connection to Port1 failed.\n\r", -1);
		GrStringDrawCentered(&sContext[frameBufIdx], "Network Connection to Port1 failed !!!", -1, 240,
						(42 + (4*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    }

    if((TRUE == Port2linkFlag) && (ipAddr2 != 0))
    {
        UARTPuts("\n\rTo control via Port2 of Ethernet, use the IP address ", -1);
        IpAddrDisplay(ipAddr2);
  
        GrStringDrawCentered(&sContext[frameBufIdx], "To control via Port2 of Ethernet, type the address", -1, 222,
			     (42 + (7*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
	i_msg = 7;

        for(i_index = 0; i_index < 4; i_index++)
        {
            ipByte = 0x000000FF & (ipAddr2 >> ((i_index)*8) );

            if(ipByte/100)
            {
                ipMsg[i_msg++] = (ipByte/100) + 48;
                ipByte = ipByte%100;
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte/10)
            {
                ipMsg[i_msg++] = (ipByte/10) + 48;
                ipByte = ipByte%10;
                ipMsg[i_msg++] = ipByte + 48;
                ipByte = 0;
            }
            if(ipByte)
            {
                ipMsg[i_msg++] = ipByte + 48;
            }

            ipMsg[i_msg++] = '.';
        }
        ipMsg[--i_msg] = '\0';
   
        strcat(ipMsg, "/index.html");
        GrStringDrawCentered(&sContext[frameBufIdx], ipMsg, -1, 135,
					(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
	GrStringDrawCentered(&sContext[frameBufIdx], " in the host web", -1, 280,
					(42 + (8*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
	GrStringDrawCentered(&sContext[frameBufIdx], "browser to access embedded demo page on the target.", -1, 215,
					(42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    }
    else
    {
        UARTPuts("\n\rNetwork Connection to Port2 failed.\n\r", -1);
	GrStringDrawCentered(&sContext[frameBufIdx], "Network Connection to Port2 failed !!!", -1, 240,
			    (42 + (9*(3 + GrStringHeightGet(&sContext[frameBufIdx])))), 0);
    }


    updatePage(imageCount);
}


/*
** Action to be taken when the eCAP icon is clicked
*/
static void ActionECAP(void)
{
    if(prevAction)
    {
        /* Go to prev image */
        imageCount--;
    }
    else
    {
        /* Go to next image */
        (0 == clickIdx) ? imageCount++ : (imageCount = clickIdx); 
    }
    prevAction = 0;
	updatePage(imageCount);	

}

/*
** ECAP demo
*/
static void ActionECAPdemo(void)
{
	EcapBkLightVary();
}

/*
** Action when no touch is detected 
*/
static void ActionIdle(void)
{
    ; /* Perform nothing */
}


/*
** Action for Menu
*/
static void ActionMenu(void)
{
    imageCount = 1;
    prevAction = 0;
	updatePage(imageCount);
}

/*
** Action for menu introduction icon click
*/
static void ActionMenuIntro(void)
{
    if(prevAction)
    {
        /* Go to prev image */
        imageCount--;
    }
    else
    {
        (0 == clickIdx) ? imageCount++ : (imageCount = clickIdx); 
    }
    prevAction = 0;
	
	updatePage(imageCount);	 
	 
}

/*
** Action for menu webdemo icon click
*/
static void ActionMenuWebDemo(void) 
{ 
    clickIdx = CLICK_IDX_CHOICE;
	updatePage(clickIdx);
}

/*
** Action for menu McASP icon click
*/
static void ActionMenuMcASP(void) 
{
    clickIdx = CLICK_IDX_MCASP;
	updatePage(clickIdx);
}

/*
** Action for menu MMCSD icon click
*/
static void ActionMenuMMCSD(void) 
{
    clickIdx = CLICK_IDX_MMCSD;
	updatePage(clickIdx);
}

/*
** Action for menu Uart icon click
*/
static void ActionMenuUart(void) 
{
    clickIdx = CLICK_IDX_UART;
	updatePage(clickIdx);
}

/*
** Action for menu RTC icon click
*/
static void ActionMenuRTC(void) 
{
    clickIdx = CLICK_IDX_RTC;
	updatePage(clickIdx);
}

/*
** Action for menu timer icon click
*/
static void ActionMenuTimer(void) 
{
    clickIdx = CLICK_IDX_TIMER;
	updatePage(clickIdx);
}

/*
** Action for menu ethernet icon click
*/
static void ActionMenuEthernet(void) 
{
    clickIdx = CLICK_IDX_ETHERNET;
	updatePage(clickIdx);
}


/*
** Action for menu ethernet icon click
*/
static void ActionMenuECAP(void) 
{
    clickIdx = CLICK_IDX_ECAP;
	updatePage(clickIdx);
}

/*
** Action for menu GPIO icon click
*/
static void ActionMenuGPIO(void) 
{
    clickIdx = CLICK_IDX_GPIO;
	updatePage(clickIdx);
}

/*
** Action for menu I2C icon click
*/
static void ActionMenuI2C(void) 
{
    clickIdx = CLICK_IDX_I2C;
	I2CDemoFlag = TRUE;	
	printtemp = 1;
	updatePage(clickIdx);
}

/*
** Action for menu PM icon click
*/
static void ActionMenuPM(void) 
{
    clickIdx = CLICK_IDX_PM;
	updatePage(clickIdx);
}

/*
** Action for DVFS icon click
*/
static void ActionMenuDVFS(void)
{
    clickIdx = CLICK_IDX_DVFS;
    updatePage(clickIdx);
    imageCount = 14;
}



/*
** Default action 
*/
static void ActionDefault(void)
{
    if(prevAction)
    {
        /* Go to prev image */
        imageCount--;
    }
    else
    {
        (0 == clickIdx) ? imageCount++ : (imageCount = clickIdx); 
    }
    prevAction = 0;
	
	updatePage(imageCount);
}

/*
** Action for LED blink with varying period
*/
static void ActionTimer(void)
{
    /* Display the proper Image */ 
    if(prevAction)
    {
        /* Go to prev image */
        imageCount--;
    }
    else
    {
        (0 == clickIdx) ? imageCount++ : (imageCount = clickIdx); 
    }
    prevAction = 0; 
	updatePage(imageCount);	
  
    tmr2Flag  = FALSE;
    tmrStepVary = TRUE;

    Timer2Start();
}


/*
** Action for I2C demo
*/
static void ActionI2C(void)
{
	/* Display the proper Image */ 
    if(prevAction)
    {
        /* Go to prev image */
        imageCount--;
    }
    else
    {
        (0 == clickIdx) ? imageCount++ : (imageCount = clickIdx); 
    }
    prevAction = 0; 
	I2CDemoFlag = TRUE;
	printtemp = 1;
	updatePage(imageCount);	
}


/*
** Action for RTC Time Set. This is a blocking call.
*/
static void ActionTimeSet(void)
{
    RtcTimeCalSet(); 
}


/*
** Toggle colors on LCD
*/
static void toggleColors(void)
{
	static unsigned int colour[2] = {ClrBlue, ClrRed};
	static unsigned int index = 0;
    static tRectangle sRect1 = {220,210,248,238};
	
    GrContextForegroundSet(&sContext[!frameBufIdx], colour[index]);
    GrRectFill(&sContext[!frameBufIdx], &sRect1);

    CacheDataCleanBuff((unsigned int) &g_pucBuffer[0],
           GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
    CacheDataCleanBuff((unsigned int) &g_pucBuffer[1],
           GrOffScreen24BPPSize(LCD_WIDTH, LCD_HEIGHT, PIXEL_24_BPP_UNPACKED));
	
	index = !index;
}

/****************************** End of file *********************************/
  
