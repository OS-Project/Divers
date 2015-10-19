/**
 *  \file   demoPwrMgmnt.c
 *
 *  \brief  Functions used Power management implementation are defined here
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
 
#include <string.h>
#include "pmic.h"
#include "cm3wkup_proxy.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "clock.h"
#include "mailbox.h"
#include "demoPwrMgmnt.h"
#include "hw_prm_wkup.h"
#include "hw_cm_wkup.h"
#include "ecap.h"
#include "edma.h"
#include "edma_event.h"
#include "soc_AM335x.h"
#include "hw_tps65910.h"
#include "cm3image.h"
#include "raster.h"
#include "hsi2c.h"
#include "tsc_adc.h"
#include "gpio_v2.h"
#include "hw_cm_per.h"
#include "demoTouch.h"
#include "cpsw.h"
#include "mdio.h"
#include "mcasp.h"
#include "dmtimer.h"
#include "delay.h"
#include "evmAM335x.h"
#include "demoRaster.h"
#include "demoToneLoop.h"
#include "demoEcap.h"
#include "demoEnet.h"
#include "demoTimer.h"
#include "ehrpwm.h"
#include "demoGrlib.h"
#include "demoI2c.h"
#include "uart_irda_cir.h"
#include "demoGpio.h"
#include "cache.h"

/*****************************************************************************
**                LOCAL MACRO DEFINITIONS                                   
*****************************************************************************/
/*	CM3 memory start address and length	*/
#define CM3_UMEM_START_ADDR			(0x44D00000u)
#define CM3_UMEM_LENGTH				(16*1024)

#define CM3_DMEM_START_ADDR			(0x44D80000u)
#define CM3_DMEM_LENGTH				(8*1024)

/*	CM3 intrrupt number on A8	*/
#define	CM3_TXEV_EOI_A8_INT_NUM			(78)

/*	Mailbox0 base address	*/
#define MAILBOX_0_BASE_ADDR			(0x480C8000u)

/*	WFI intstruction	*/
#define wfi()					asm("	WFI");

#define FAIL					(0)
#define SUCCESS					(1)


/* Address of TPS65910A (PMIC - control)  over I2C0. */
#define  PMIC_CNTL_I2C_SLAVE_ADDR  		(0x2D)

/* Address of TPS65910A (PMIC - SR)  over I2C0. */
#define  PMIC_SR_I2C_SLAVE_ADDR     		(0x12)

#define  FORCE_IDLE				(0x0)

#define  SLEEP_MODE_DS0                         (0x01)
#define  SLEEP_MODE_DS1_DEV                     (0x02)
#define  SLEEP_MODE_DS1                         (0x06)
#define  SLEEP_MODE_DS2                         (0x08)

/******************************************************************************
**              EXTERNAL FUNCTIONS
******************************************************************************/
extern void configI2c(void);
extern void ddrPllOpp100Config(void);
extern void ddrPllOpp50Config(void);
extern void romRestoreLocation(void);
extern void romRestoreLocationDs2(void);

/******************************************************************************
**              FUNCTION PROTOTYPES
******************************************************************************/
void reduceVddOpVoltage(unsigned int);
void restoreVddOpVoltage(unsigned int);
void clearMailboxMsg(void);
void configModulesMode(void);
void syncCm3(void);
void enableUartWakeup(void);
void disableUartWakeup(void);


void mpuPllConfig(unsigned int oscClk,unsigned int N1,unsigned int M1,unsigned int M2);

#if 0
void corePllConfig(unsigned int oscClk,unsigned int N,unsigned int M,unsigned int M4,\
													unsigned int M5,unsigned int M6);
void perPllConfig(unsigned int oscClk,unsigned int N,unsigned int M,unsigned int M2);
void dispPllConfig(unsigned int oscClk,unsigned int N,unsigned int M,unsigned int M2);
void bypassDispPll();
#endif

static void CM3Isr(void);
static void PeripheralsContextSave(void);
static void PeripheralsContextRestore(void);
static void PeripheralsHalt(unsigned int slpMode);
static void PeripheralsResume(unsigned int slpMode);
static void PowerSaveModeEnter(deepSleepData dsData, unsigned int slpMode);
static void PowerDownConfig(void);
static void PowerUpConfig(void);
static void TouchScreenFIFOFlush(void);

/******************************************************************************
**              EXTERNAL VARIABLES
******************************************************************************/
extern unsigned int iram_start;
extern unsigned int iram_size;
extern unsigned int relocstart;
extern unsigned int relocend;
extern ModuleClock ModuleClockList[];
extern volatile unsigned int IsTSPress;
extern volatile unsigned int imageCount;
extern volatile unsigned int x_data[];
extern volatile unsigned int y_data[];


/******************************************************************************
**                       GLOBAL VARIABLE DEFINITIONS
******************************************************************************/
RASTERCONTEXT rasterContext;
MCASPCONTEXT mcaspContext;
CPSWCONTEXT cpswContext;
MDIOCONTEXT mdioContext;
DMTIMERCONTEXT dmtimerContext[4];
EDMACONTEXT edmaContext;
ECAPCONTEXT ecapContext;
I2CCONTEXT i2cContext;
GPIOCONTEXT gpioContext;
CTRLREGCONTEXT ctrlContext;

/*	Flag to indicate M3 event is received	*/
volatile unsigned int isM3IntReceived = 0;

/*	Hold Wake source	*/
volatile unsigned int wakeSource = WAKE_SOURCE_TSC;

/*	Flag to indicate whether sleep mode image should be displayed	*/
unsigned int isSleepModeSel = 0;

/*	DS1 data - don't cares are not defined	*/
deepSleepData ds1Data = {
	.dsDataBits.cmdID = PM_CMD_DS1_MODE,

	.dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
	.dsDataBits.moscState = PM_MOSC_STATE_OFF,
	.dsDataBits.deepSleepCount = 0, 

	.dsDataBits.pdMpuState = PM_MPU_POWERSTATE_RET,
	.dsDataBits.pdMpuRamRetState = PM_MPU_RAM_RETSTATE_OFF,
	.dsDataBits.pdMpul1RetState = 0,
	.dsDataBits.pdMpul2RetState = 0,
	
	.dsDataBits.pdPerState = PM_PER_POWERSTATE_ON,	
	.dsDataBits.pdPerIcssMemOnState = PM_PER_ICSS_RAM_ONSTATE_ON,
	.dsDataBits.pdPerMemOnState = PM_PER_MEM_ONSTATE_ON,
	.dsDataBits.pdPerOcmcOnState = PM_PER_OCMC_RAM_ONSTATE_ON,

	.dsDataBits.wakeSources = WAKE_SOURCE_TSC,

};

/*      DS1 Deviation data - don't cares are not defined  */
deepSleepData ds1DevData = {
        .dsDataBits.cmdID = PM_CMD_DS1_MODE,

        .dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
        .dsDataBits.moscState = PM_MOSC_STATE_ON,
        .dsDataBits.deepSleepCount = 0,

        .dsDataBits.pdMpuState = PM_MPU_POWERSTATE_RET,
        .dsDataBits.pdMpuRamRetState = PM_MPU_RAM_RETSTATE_OFF,
        .dsDataBits.pdMpul1RetState = 0,
        .dsDataBits.pdMpul2RetState = 0,

        .dsDataBits.pdPerState = PM_PER_POWERSTATE_ON,
        .dsDataBits.pdPerIcssMemOnState = PM_PER_ICSS_RAM_ONSTATE_ON,
        .dsDataBits.pdPerMemOnState = PM_PER_MEM_ONSTATE_ON,
        .dsDataBits.pdPerOcmcOnState = PM_PER_OCMC_RAM_ONSTATE_ON,

        .dsDataBits.wakeSources = WAKE_SOURCE_TSC,

};

/*	DS1 data - don't cares are not defined	*/
deepSleepData ds0Data = {
	.dsDataBits.cmdID = PM_CMD_DS0_MODE,

	.dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
	.dsDataBits.moscState = PM_MOSC_STATE_OFF,
	.dsDataBits.deepSleepCount = 0,

	.dsDataBits.pdMpuState = PM_MPU_POWERSTATE_RET,
	.dsDataBits.pdMpuRamRetState = PM_MPU_RAM_RETSTATE_OFF,
	.dsDataBits.pdMpul1RetState = 0,
	.dsDataBits.pdMpul2RetState = 0,
	
	.dsDataBits.pdPerState = PM_PER_POWERSTATE_RET,	
	.dsDataBits.pdPerIcssMemRetState = PM_PER_ICSS_RAM_RETSTATE_OFF,
	.dsDataBits.pdPerMemRetState = PM_PER_MEM_RETSTATE_OFF,
	.dsDataBits.pdPerOcmcRetState = PM_PER_OCMC_RAM_RETSTATE_RET,
	
	.dsDataBits.wakeSources = WAKE_SOURCE_TSC,
};

/*	DS2 data - don't cares are not defined	*/
deepSleepData ds2Data = {
	.dsDataBits.cmdID = PM_CMD_DS2_MODE,

	.dsDataBits.resumeAddr = (unsigned int)romRestoreLocation,
	.dsDataBits.moscState = PM_MOSC_STATE_OFF,
	.dsDataBits.deepSleepCount = 0, 

	.dsDataBits.wakeSources = WAKE_SOURCE_TSC,
};

/*      Command to reset CM3 state machine	*/
deepSleepData dsDataM3reset = {
	.dsDataBits.cmdID = 0xE
};

/*	This list is application specific. It indexes into the master module clock list.
	It lists the modules to be disabled while entering DS1	*/
unsigned int moduleDisableList[] = 
{
	CLK_MAILBOX0,
	CLK_GPIO1,
	CLK_GPIO0,
	CLK_LCDC,		
	//CLK_I2C0,
	CLK_I2C1,
	CLK_ADC_TSC,
	CLK_RTC,
	CLK_MPU_CLK,
	CLK_TIMER2,	
	CLK_TIMER3,	
	CLK_TIMER4,	
	CLK_TIMER7,	
	CLK_MCASP1,
	CLK_TPTC2,
	CLK_TPTC1,
	CLK_TPTC0,
	CLK_TPCC,	
	CLK_CPGMAC0,
	CLK_EPWMSS0,	
	CLK_UART0,
};

/*	List of modules to be enabled after wakeup	*/
unsigned int moduleEnableList[] = 
{
	CLK_UART0,
    	CLK_CPGMAC0,
	CLK_EPWMSS0,
	CLK_TIMER2,	
	CLK_TIMER3,	
	CLK_TIMER4,	
	CLK_TIMER7,
	CLK_TPCC,	
	CLK_TPTC0,	
	CLK_TPTC1,
	CLK_TPTC2,
	CLK_MCASP1,
	CLK_RTC,
	CLK_ADC_TSC,
	CLK_I2C1,
	CLK_I2C0,
	CLK_LCDC,
	CLK_GPIO1,
	CLK_GPIO0,
	CLK_MAILBOX0,
};

unsigned int moduleDisableListDev[] =
{
        CLK_MAILBOX0,
        CLK_GPIO1,
        CLK_GPIO0,
        //CLK_I2C0,
        CLK_I2C1,
        CLK_ADC_TSC,
        CLK_RTC,
        CLK_MPU_CLK,
        CLK_TIMER2,
        CLK_TIMER3,
        CLK_TIMER4,
        CLK_TIMER7,
        CLK_MCASP1,
        CLK_TPTC2,
        CLK_TPTC1,
        CLK_TPTC0,
        CLK_TPCC,
        CLK_CPGMAC0,
        CLK_UART0,
};

/*      List of modules to be enabled after wakeup      */
unsigned int moduleEnableListDev[] =
{
        CLK_UART0,
        CLK_CPGMAC0,
        CLK_TIMER2,
        CLK_TIMER3,
        CLK_TIMER4,
        CLK_TIMER7,
        CLK_TPCC,
        CLK_TPTC0,
        CLK_TPTC1,
        CLK_TPTC2,
        CLK_MCASP1,
        CLK_RTC,
        CLK_ADC_TSC,
        CLK_I2C1,
        CLK_I2C0,
        CLK_GPIO1,
        CLK_GPIO0,
        CLK_MAILBOX0,
};

/*	List of clock domains	*/
unsigned int clockDomainStatusList[] = 
{
	CLK_L4LS_CLK_DOMAIN,
	CLK_PER_L3_CLK_DOMAIN,
	//CLK_LCDC_CLK_DOMAIN,    
	//CLK_CPSW_125MHZ_CLK_DOMAIN,
	CLK_L4HS_CLK_DOMAIN,
	//CLK_24MHZ_CLK_DOMAIN,
	//CLK_WKUP_CLK_DOMAIN,
	//CLK_L3_CLK_DOMAIN,
	//CLK_L3S_CLK_DOMAIN,
};


/******************************************************************************
**              EXTERNAL FUNCTIONS
******************************************************************************/
extern void ddrPllOpp100Config(void);
extern void ddrPllOpp50Config(void);

/******************************************************************************
**              FUNCTION PROTOTYPES
******************************************************************************/
void clearMailboxMsg(void);
void configModulesMode(void);

/******************************************************************************
**              FUNCTION DEFINITIONS
******************************************************************************/
/*
** Register interrupt hand;er for CM3 Event
*/
void CM3IntRegister(void)
{
    /* Register the ISR in the Interrupt Vector Table.*/
    IntRegister(SYS_INT_M3_TXEV, CM3Isr);
}

/*
** Clear CM3 event and re-enable the event
*/
void CM3EventsClear(void)
{
    /*      Clear M3_TXEV event     */
    HWREG(SOC_CONTROL_REGS + CONTROL_M3_TXEV_EOI) |=
                                    CONTROL_M3_TXEV_EOI_M3_TXEV_EOI;

    /*      Re-arm M3_TXEV event    */
    HWREG(SOC_CONTROL_REGS + CONTROL_M3_TXEV_EOI) &=
                                 (~CONTROL_M3_TXEV_EOI_M3_TXEV_EOI);
}

/*
** Load CM3 image into its memory and release CM3 from reset
*/
void CM3LoadAndRun(void)
{
    /* Load CM3 SW */
    memcpy((void *)(CM3_UMEM_START_ADDR), (const void *)cm3image,
           sizeof(cm3image));

    /* Release CM3 from reset */
    HWREG(SOC_PRM_WKUP_REGS + PRM_WKUP_RM_WKUP_RSTCTRL) &=
          (~PRM_WKUP_RM_WKUP_RSTCTRL_WKUP_M3_LRST);
}

/*
** Wait for ACK from CM3
*/
void waitForM3Txevent(void)
{
    /* wait until CM3 TX Event is generated */
    while(!isM3IntReceived);

    while(PM_IN_PROGRESS == readCmdStatus())
    {
        CM3EventsClear();
    }

    switch(readCmdStatus())
    {
       case PM_CMD_PASS:
       case PM_WAIT4OK:
           break;
       case PM_CMD_FAIL:
       default:
           /* Command failed or invalid status */
           UARTPuts("\n\n ACK Failed \r\n", -1);
           while(1);
    }

    CM3EventsClear();

    /* Reset interrupt flag */
    isM3IntReceived = 0;
}

/*
** Select wake source
*/
void ActionWakeOnTsc()
{
	// update wake source
	wakeSource = WAKE_SOURCE_TSC;
	UARTPuts("\n\n Touch screen wake selected \r\n", -1);
		
	// update the display
	updatePage(imageCount);
}

/*
** Select wake source
*/
void ActionWakeOnTimer()
{
	// update wake source
	wakeSource = WAKE_SOURCE_TMR;
	UARTPuts("\n\n Timer wake selected \r\n", -1);
	
	// update the display
	updatePage(imageCount);	
}

/*
** Select wake source
*/
void ActionWakeOnUart()
{
	// update wake source
	wakeSource = WAKE_SOURCE_UART;
	UARTPuts("\n\n Uart wake selected \r\n", -1);
	
	// update the display
	updatePage(imageCount);		
}

/*
** Select wake source
*/
void ActionWakeOnGpio()
{
	// update wake source
	wakeSource = WAKE_SOURCE_GPIO;
	UARTPuts("\n\n GPIO wake selected \r\n", -1);
	
	// update the display
	updatePage(imageCount);	
}

/*
** Halts any currently active transactions
*/
static void PeripheralsHalt(unsigned int slpMode)
{
    /* Disable EDMA for the transfer */
    EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MCASP1_TX,
                        EDMA3_TRIG_MODE_EVENT);

    McASPTxReset(SOC_MCASP_1_CTRL_REGS);

    if(slpMode != SLEEP_MODE_DS1_DEV)
    {
        /* LCD back light OFF	*/		
        EcapBkLightDisable();
        /* Disable End-of-frame interrupt	*/
        Raster0EOFIntDisable();
    }
    else
    {
        EcapBkLightDim();
    }

    CPSWCPDMACmdIdleEnable(SOC_CPSW_CPDMA_REGS); 
}

/*
** Halts any currently active transactions
*/
static void PeripheralsResume(unsigned int slpMode)
{
    EDMAModuleClkConfig();

    AudioTxActivate();

    if(slpMode != SLEEP_MODE_DS1_DEV)
    {
        Raster0EOFIntEnable();
        EcapBkLightEnable(); 
    }
    else
    {
        EcapBkLightEnable();
    }

    /* Bring CPSW out of standby and restart DHCP */   
    CPSWWrControlRegReset(SOC_CPSW_WR_REGS);
    CPSWCPDMACmdIdleDisable(SOC_CPSW_CPDMA_REGS);
    EnetDHCPRestart();	
}


/*
** Initializes Mailbox
*/
void MailBoxInit(void)
{
    initializeMailbox(MAILBOX_0_BASE_ADDR);
}

/*
** Save the peripheral register context and suspends any transactions
*/
static void PeripheralsContextSave(void)
{
    /*	LCD back light OFF	*/		
    EcapBkLightDisable();
    // LCDC standby
	RasterIdleModeConfig(SOC_LCDC_0_REGS, LCDC_SYSCONFIG_IDLEMODE_FORCE << 
											LCDC_SYSCONFIG_IDLEMODE_SHIFT);
	RasterStandbyModeConfig(SOC_LCDC_0_REGS, LCDC_SYSCONFIG_STANDBYMODE_FORCE <<
											LCDC_SYSCONFIG_STANDBYMODE_SHIFT);
	
    RasterContextSave(SOC_LCDC_0_REGS, &rasterContext);
	
    EcapContextSave(SOC_ECAP_0_REGS, SOC_PWMSS0_REGS, &ecapContext);
	
    EDMA3ContextSave(SOC_EDMA30CC_0_REGS, &edmaContext);
    McASPContextSave(SOC_MCASP_1_CTRL_REGS, SOC_MCASP_1_FIFO_REGS,
                      &mcaspContext, McASP_CONTEXT_TX);
					  
    MDIOContextSave(SOC_CPSW_MDIO_REGS, &mdioContext);
    cpswContext.aleBase = SOC_CPSW_ALE_REGS;
    cpswContext.ssBase = SOC_CPSW_SS_REGS;
    cpswContext.port1Base = SOC_CPSW_PORT_1_REGS;
    cpswContext.port2Base = SOC_CPSW_PORT_2_REGS;
    cpswContext.cpdmaBase = SOC_CPSW_CPDMA_REGS;
    cpswContext.wrBase = SOC_CPSW_WR_REGS;
    cpswContext.sl1Base = SOC_CPSW_SLIVER_1_REGS;
    cpswContext.sl2Base = SOC_CPSW_SLIVER_2_REGS;
    cpswContext.cppiRamBase = SOC_CPSW_CPPI_RAM_REGS; 
    CPSWContextSave(&cpswContext);

    DMTimerContextSave(SOC_DMTIMER_2_REGS, &dmtimerContext[0]);
    DMTimerContextSave(SOC_DMTIMER_3_REGS, &dmtimerContext[1]);
    DMTimerContextSave(SOC_DMTIMER_4_REGS, &dmtimerContext[2]);
    DMTimerContextSave(SOC_DMTIMER_7_REGS, &dmtimerContext[3]);
  
    I2CContextSave(SOC_I2C_1_REGS, &i2cContext);
	
    gpioContextSave(SOC_GPIO_1_REGS, &gpioContext);

    /* Disable EDMA for the transfer */
    EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS, EDMA3_CHA_MCASP1_TX,
                         EDMA3_TRIG_MODE_EVENT);

    McASPTxReset(SOC_MCASP_1_CTRL_REGS);

    CPSWSSReset(SOC_CPSW_SS_REGS);
    CPSWSlReset(SOC_CPSW_SLIVER_1_REGS);
    CPSWSlReset(SOC_CPSW_SLIVER_2_REGS);
    CPSWWrReset(SOC_CPSW_WR_REGS);
    CPSWCPDMAReset(SOC_CPSW_CPDMA_REGS);

    /*	Disable End-of-frame interrupt	*/
    Raster0EOFIntDisable();
}

/*
** Restore the peripherals context and resumes operation
*/
static void PeripheralsContextRestore(void)
{
    RasterContextRestore(SOC_LCDC_0_REGS, &rasterContext);
    Raster0EOFIntEnable();
	// LCDC NON-Standby
	RasterStandbyModeConfig(SOC_LCDC_0_REGS, LCDC_SYSCONFIG_STANDBYMODE_SMART <<
											LCDC_SYSCONFIG_IDLEMODE_SHIFT);   		
	RasterIdleModeConfig(SOC_LCDC_0_REGS, LCDC_SYSCONFIG_IDLEMODE_SMART << 
											LCDC_SYSCONFIG_STANDBYMODE_SHIFT);
		
    EcapContextRestore(SOC_ECAP_0_REGS, SOC_PWMSS0_REGS, &ecapContext);
    /*	LCD back light ON	*/			
    EcapBkLightEnable();	
	
    gpioContextRestore(SOC_GPIO_1_REGS, &gpioContext);

    DMTimerContextRestore(SOC_DMTIMER_2_REGS, &dmtimerContext[0]);
    DMTimerContextRestore(SOC_DMTIMER_3_REGS, &dmtimerContext[1]);
    DMTimerContextRestore(SOC_DMTIMER_4_REGS, &dmtimerContext[2]);
    DMTimerContextRestore(SOC_DMTIMER_7_REGS, &dmtimerContext[3]);

    initializeMailbox(MAILBOX_0_BASE_ADDR);

    MDIOContextRestore(SOC_CPSW_MDIO_REGS, &mdioContext);

    CPSWContextRestore(&cpswContext);
	
    EDMAModuleClkConfig();
    EDMA3ContextRestore(SOC_EDMA30CC_0_REGS, &edmaContext);
    McASPContextRestore(SOC_MCASP_1_CTRL_REGS, SOC_MCASP_1_FIFO_REGS,
                        &mcaspContext, McASP_CONTEXT_TX);

    AudioTxActivate();

    EnetDHCPRestart(); 
	
    I2CContextRestore(SOC_I2C_1_REGS, &i2cContext);
}

/*
** Initialize the Mailbox
**	- Enable clock and reset mailbox
*/
void initializeMailbox(unsigned int baseAdd)
{
	/*	Enable Mailbox clock	*/
	enableModuleClock(CLK_MAILBOX0);
	
	/*	Reset Mailbox	*/
	MBresetMailbox(baseAdd);
	
	/*	Clear new message status	*/
	MBclrNewMsgStatus(baseAdd, MAILBOX_USER_CM3WKUP, MAILBOX_QUEUE_0);
	
	/*	Enable new message interrupt	*/
	MBenableNewMsgInt(baseAdd, MAILBOX_USER_CM3WKUP, MAILBOX_QUEUE_0);
	
	/*	Configure idle mode	*/
	MBconfigIdleMode(MAILBOX_0_BASE_ADDR, (MAILBOX_SYSCONFIG_SIDLEMODE_FORCEIDLE << MAILBOX_SYSCONFIG_SIDLEMODE_SHIFT));
}


/*
** Generate Mailbox interrupt to CM3 by writing a dummy vlaue to mailbox register
*/
void generateMailboxInt(unsigned int baseAdd)
{
	/*	Write to Mailbox register	*/
	MBsendMessage(baseAdd, MAILBOX_QUEUE_0, 0x12345678u);
}


/*
** CM3 ISR handler
*/
static void CM3Isr(void)
{
	isM3IntReceived = 1;
	CM3EventsClear();
}


/*
** Clear mail box messages
*/
void clearMailboxMsg(void)
{
	unsigned int tamp;
	/*	Read the message back	*/
	MBgetMessage(MAILBOX_0_BASE_ADDR, MAILBOX_QUEUE_0, &tamp); 
	/*	Clear new message status	*/
	MBclrNewMsgStatus(MAILBOX_0_BASE_ADDR, MAILBOX_USER_CM3WKUP, MAILBOX_QUEUE_0);
}


/*
** Reduce Vdd1 & Vdd2 operating voltage
*/
void reduceVddOpVoltage(unsigned int vdd)
{
	// Configure PMIC slave address
	I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);
	
	/*	Select VDD1 voltage	*/
	if (PM_RAIL_MPU == vdd)
		/*	Advisory 1.0.15:ARM Cortex-A8: OPP50 Operation on MPU Domain Not
			Supported. Voltage will be retained at 1.1v	*/
		setVdd1OpVoltage(PM_PMIC_CONFIG_1_1V);
	else
		/*	Select VDD2 voltage	*/
		setVdd2OpVoltage(PM_PMIC_CONFIG_0_95V);
}


/*
** Restore Vdd1 & Vdd2 operating voltage
*/
void restoreVddOpVoltage(unsigned int vdd)
{
	// Configure PMIC slave address
	I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);
	
	/*	Select VDD1 voltage	*/
	if(PM_RAIL_MPU == vdd)
		setVdd1OpVoltage(PM_PMIC_CONFIG_1_26V);
	
	else
		/*	Select VDD2 voltage	*/
		setVdd2OpVoltage(PM_PMIC_CONFIG_1_1V);
}


void configVddOpVoltage(void)
{
	// Configure PMIC slave address
	I2CMasterSlaveAddrSet(SOC_I2C_0_REGS, PMIC_CNTL_I2C_SLAVE_ADDR);
	
	/* Using Control Interface of PMIC for communication with I2C0. */
	selectI2CInstance(PMIC_DEVCTRL_REG_SR_CTL_I2C_SEL_CTL_I2C);
	
	/*	Configure vdd1- need to validate these parameters	*/
	configureVdd1(PMIC_VDD1_REG_VGAIN_SEL_X1, PMIC_VDD1_REG_ILMAX_1_5_A,
				PMIC_VDD1_REG_TSTEP_12_5, PMIC_VDD1_REG_ST_ON_HI_POW);
	
	/*	Select the source for VDD1 control	*/
	selectVdd1Source(PMIC_VDD1_OP_REG_CMD_OP);
	
	/*	Configure vdd2 - need to validate these parameters	*/
	configureVdd2(PMIC_VDD2_REG_VGAIN_SEL_X1, PMIC_VDD2_REG_ILMAX_1_5_A,
				PMIC_VDD2_REG_TSTEP_12_5, PMIC_VDD2_REG_ST_ON_HI_POW);
	
	/*	Select the source for VDD2 control	*/
	selectVdd2Source(PMIC_VDD2_OP_REG_CMD_OP);
}


void configModulesMode(void)
{
	volatile unsigned int delay = 0xFFFF;

	/*	Enable the modules	*/
	HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) = 
		(CM_PER_USB0_CLKCTRL_MODULEMODE_ENABLE << CM_PER_USB0_CLKCTRL_MODULEMODE_SHIFT);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) =
		(CM_PER_TPTC0_CLKCTRL_MODULEMODE_ENABLE << CM_PER_TPTC0_CLKCTRL_MODULEMODE_SHIFT);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) = 
		(CM_PER_TPTC1_CLKCTRL_MODULEMODE_ENABLE << CM_PER_TPTC1_CLKCTRL_MODULEMODE_SHIFT);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) =
		(CM_PER_TPTC2_CLKCTRL_MODULEMODE_ENABLE << CM_PER_TPTC2_CLKCTRL_MODULEMODE_SHIFT);

	HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) =
		(CM_PER_GPMC_CLKCTRL_MODULEMODE_ENABLE << CM_PER_GPMC_CLKCTRL_MODULEMODE_SHIFT);
		
	HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) =
		(CM_PER_MMC0_CLKCTRL_MODULEMODE_ENABLE << CM_PER_MMC0_CLKCTRL_MODULEMODE_SHIFT);

	
	delay = 0xFFFF;
	while(delay--);
	
	/*	Configure the modules Mode to idle state	*/
	HWREG(0x4a101208) = 0x2;	/*	GEMAC	*/	
	HWREG(0x47400010) = 0x08;	/*	USB Module	*/
	
	HWREG(0x49800010) = 0x08;	/*	TPTC0	*/
	HWREG(0x49900010) = 0x08;	/*	TPTC1	*/
	HWREG(0x49a00010) = 0x08;	/*	TPTC2	*/
	
	HWREG(0x50000010) = 0x10;	/*	GPMC	*/
	HWREG(0x48060110) &= (0xFFFFFFE7); /*	MMC0 (Force idle)	*/
	
	/*	Disable the modules	*/
	HWREG(SOC_CM_PER_REGS + CM_PER_USB0_CLKCTRL) = 
		(CM_PER_USB0_CLKCTRL_MODULEMODE_DISABLE << CM_PER_USB0_CLKCTRL_MODULEMODE_SHIFT);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_TPTC0_CLKCTRL) =
		(CM_PER_TPTC0_CLKCTRL_MODULEMODE_DISABLE << CM_PER_TPTC0_CLKCTRL_MODULEMODE_SHIFT);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_TPTC1_CLKCTRL) = 
		(CM_PER_TPTC1_CLKCTRL_MODULEMODE_DISABLE << CM_PER_TPTC1_CLKCTRL_MODULEMODE_SHIFT);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_TPTC2_CLKCTRL) =
		(CM_PER_TPTC2_CLKCTRL_MODULEMODE_DISABLE << CM_PER_TPTC2_CLKCTRL_MODULEMODE_SHIFT);

	HWREG(SOC_CM_PER_REGS + CM_PER_GPMC_CLKCTRL) =
		(CM_PER_GPMC_CLKCTRL_MODULEMODE_DISABLE << CM_PER_GPMC_CLKCTRL_MODULEMODE_SHIFT);
		
	HWREG(SOC_CM_PER_REGS + CM_PER_MMC0_CLKCTRL) =
		(CM_PER_MMC0_CLKCTRL_MODULEMODE_DISABLE << CM_PER_MMC0_CLKCTRL_MODULEMODE_SHIFT);

}


void enableWakeSource(unsigned int wakeSource)
{
	StepDisable();  /*	TS Step disable	*/
	switch(wakeSource)
	{
		case WAKE_SOURCE_TSC:
			/*	Enable touch screen wake	*/
			configTSWakeup();
			enableTSWakeup();
		break;
		
		case WAKE_SOURCE_UART:
			enableUartWakeup();
		break;
		
		case WAKE_SOURCE_TMR:
			UARTPuts("\t...system will wakeup after 15 Sec... ", -1);
			/*	Refer (19.2.3.8 Timer Counting Rate) of TRM 	*/
			setTimerCount(0xFFFC3FFF); /* 15 Sec*/
			//setTimerCount(0xFFF87FFF); /* 30 Sec*/
			//setTimerCount(0xffe1ffff);
		break;
		
		case WAKE_SOURCE_GPIO:
			enableGpioWake();
		break;
		
		default:
				;
	}
}

void disableWakeSource(unsigned int wakeSource)
{
	StepEnable();	/*	TS Step Enable	*/
	switch(wakeSource)
	{
		case WAKE_SOURCE_TSC:
			/*	Disable	touch screen wake event	*/
			disableTSWakeup();	
		break;
		
		case WAKE_SOURCE_UART:
			disableUartWakeup();
		break;	
		
		case WAKE_SOURCE_TMR:
			/*	Clear timer interrupt	*/
			clearTimerInt();
		break;
		
		case WAKE_SOURCE_GPIO:
			disableGpioWake();
		break;

		default:
				;
	}
}

/*
** Configurations for DS0
*/
static void PowerDownConfig(void)
{
    unsigned int idx;

    /* Disable GFX power domain */
    HWREG(0x44E01100) = 0x0;

    /* disable GFX module */
    HWREG(0x44E00904) = 0x0;
    HWREG(0x44E00908) = 0x0;
    HWREG(0x44E00910) = 0x0;
    HWREG(0x44E00914) = 0x0;

    /* Set GFX modules to SW_SLEEP */
    HWREG(0x44E0090C) = 0x1;
    HWREG(0x44E00900) = 0x1;
	
    /* disable Cust_efuse PD */
    HWREG(0x44E01200) = 0x0;

    /* disable cust_efuse module */
    HWREG(0x44E00A20) = 0x0;

    /* Set CUST_EFUSE modules to SW_SLEEP */
    HWREG(0x44E00A00) = 0x1;

    /* IO pad configuration for low power mode */
	// The pins which are not configured here are needed for the system to wake up
    for(idx = 0x800; idx< 0x950; idx += 4)
    { 
        HWREG(SOC_CONTROL_REGS + idx) = 0x0F;
    }

    if(wakeSource != WAKE_SOURCE_GPIO)
    {
        HWREG(SOC_CONTROL_REGS + 0x950) = 0x0F;
    }
    for(idx = 0x954; idx< 0x968; idx += 4)
    { 
        HWREG(SOC_CONTROL_REGS + idx) = 0x0F;
    }

    if(wakeSource != WAKE_SOURCE_UART)
    {
        for(idx = 0x968; idx< 0x97C; idx += 4)
        {
            HWREG(SOC_CONTROL_REGS + idx) = 0x0F;
        }
    }
    for(idx = 0x97C; idx< 0x9B8; idx += 4)
    { 
		if((idx != 0x988) && (idx != 0x98C))
		{
			HWREG(SOC_CONTROL_REGS + idx) = 0x0F;
		}
    }
    for(idx = 0xA08; idx< 0xB08; idx += 4)
    { 
        HWREG(SOC_CONTROL_REGS + idx) = 0x0F;
    }
       
    if(wakeSource != WAKE_SOURCE_TSC)
    {
        for(idx = 0xB10; idx< 0xB30; idx += 4)
        {
            HWREG(SOC_CONTROL_REGS + idx) = 0x0F;
        }
    }
}

/*
** Configurations to be done when waking up from DS0 mode
*/
static void PowerUpConfig(void)
{
    /* Set GFX modules to SW_WAKE */
    HWREG(0x44E00900) = 0x2;
    HWREG(0x44E0090C) = 0x2;

    /* Set CUST_EFUSE modules to SW_WAKE */
    HWREG(0x44E00A00) = 0x2;
}		    
 
/*
** Enter the desired power save mode
*/
void PowerSaveModeEnter(deepSleepData dsData, unsigned int slpMode)
{

    volatile unsigned int status = 0x00;
    unsigned int i = 0;

    toggleLatGpio();	

    if(slpMode & SLEEP_MODE_DS0)
    {
        /*	Save the control reg configurations */
        ControlRegContextSave(&ctrlContext);
    }

    if(slpMode & SLEEP_MODE_DS0)
    {
        PeripheralsContextSave();
    }
    else
    {
        PeripheralsHalt(slpMode);
    }

    /*  configure Modules mode  */
    configModulesMode();

    dsData.dsDataBits.wakeSources = wakeSource;

    /* Configure CMD_ID and other parameters which are to be
    ** communicated with CM3 */
    configIPCRegs(dsData);

    syncCm3();

    /*  Enable wake source      */
    enableWakeSource(dsData.dsDataBits.wakeSources);

    TouchScreenFIFOFlush();

    if((SLEEP_MODE_DS0 == slpMode) && (wakeSource != WAKE_SOURCE_TSC))
    {
                /*Config ADC in powerdown mode*/
		TSCADCSetADCPowerDown(SOC_ADC_TSC_0_REGS);
                TouchDisable();
    }

    if(SLEEP_MODE_DS1_DEV == slpMode)
    {
        /*  Disable clock   */
        while(SUCCESS != deviceClockDisable(moduleDisableListDev,
                (sizeof(moduleDisableListDev)/sizeof(moduleDisableListDev[0]))));
    }
    else
    {
        /*  Disable clock   */
        while(SUCCESS != deviceClockDisable(moduleDisableList,
                (sizeof(moduleDisableList)/sizeof(moduleDisableList[0]))));
    }
	
	// Disable unused modules
    if(slpMode & SLEEP_MODE_DS0)
    {
        HWREG(0x44E0004C) = 0; // SPI0
	while(0x00030000 != (HWREG(0x44E0004C) & 0x00030000));
    }
	
    if((slpMode & SLEEP_MODE_DS0) == 0)
    {
        /*  Check clock gate status */
        while(SUCCESS != deviceClockGateStatus(clockDomainStatusList,
              (sizeof(clockDomainStatusList)/sizeof(clockDomainStatusList[0]))));
    }

    if(SLEEP_MODE_DS1_DEV != slpMode)
    {	
		#if 0
		ddrPllOpp50Config();
		#endif
		
		// Configure other PLL's to OPP50
		mpuPllConfig(MOSC_CLK, PM_OPP50_MPU_N, PM_OPP50_MPU_M, PM_OPP50_MPU_M2);
		
		#if 0
		corePllConfig(MOSC_CLK, PM_OPP50_CORE_N, PM_OPP50_CORE_M, PM_OPP50_CORE_M4,\
                                               PM_OPP50_CORE_M5, PM_OPP50_CORE_M6);
		perPllConfig(MOSC_CLK, PM_OPP50_PER_N, PM_OPP50_PER_M, PM_OPP50_PER_M2);
		bypassDispPll();
		#endif
		
		enableModuleClock(CLK_I2C0);

		#if 0
		/*  Reduce VDD_CORE voltage  */
		reduceVddOpVoltage(PM_RAIL_CORE);
		#endif
		/*  Reduce VDD_MPU voltages   */
		reduceVddOpVoltage(PM_RAIL_MPU);
    }
    disableModuleClock(CLK_I2C0, TRUE);

    /*  Disable IRQ & FIQ       */
    IntMasterIRQDisable();
    IntMasterFIQDisable();

    if(SLEEP_MODE_DS0 == slpMode)
    {
        PowerDownConfig();
    }

    /* 
    **  Save A8 context 
    **  WFI
    **  Restore A8 context
    */

    saveRestoreContext(slpMode);

    /*  Enable IRQ & FIQ        */
    IntMasterFIQEnable();
    IntMasterIRQEnable();
  
    enableModuleClock(CLK_I2C0);
	
    if(SLEEP_MODE_DS1_DEV != slpMode)
    {
		/*  Restore Vdd1 operating voltage  */
		restoreVddOpVoltage(PM_RAIL_MPU);
		
		#if 0
		/*  Restore Vdd2 operating voltage  */
		restoreVddOpVoltage(PM_RAIL_CORE);		
		#endif

		// Configure all PLL's to OPP_SRTURBO
		mpuPllConfig(  MOSC_CLK, PM_OPP_SRTURBO_MPU_N, PM_OPP_SRTURBO_MPU_M, 
													PM_OPP_SRTURBO_MPU_M2);
		#if 0
		corePllConfig( MOSC_CLK, PM_OPP100_CORE_N, PM_OPP100_CORE_M, PM_OPP100_CORE_M4,
                                        PM_OPP100_CORE_M5, PM_OPP100_CORE_M6);
		perPllConfig(  MOSC_CLK, PM_OPP100_PER_N, PM_OPP100_PER_M, PM_OPP100_PER_M2);
		dispPllConfig( MOSC_CLK, PM_OPP100_DISP_N, PM_OPP100_DISP_M, PM_OPP100_DISP_M2);	

		/*  Configure DDR PLL for OPP100    */
		ddrPllOpp100Config();
		#endif
    }

    if(slpMode & SLEEP_MODE_DS0)
    {
        /* Restore the control reg configurations */
        ControlRegContextRestore(&ctrlContext);
    }
	
    if(SLEEP_MODE_DS1_DEV == slpMode)
    {
        /*  Device clock enable     */
        deviceClockEnable(moduleEnableListDev,
                   (sizeof(moduleEnableListDev)/sizeof(moduleEnableListDev[0])));
    }
    else
    {
        /*  Device clock enable     */
        deviceClockEnable(moduleEnableList,
                   (sizeof(moduleEnableList)/sizeof(moduleEnableList[0])));
    }	
	
    if(slpMode & SLEEP_MODE_DS0)
    {
        PeripheralsContextRestore();
    }
    else
    {
        PeripheralsResume(slpMode);
    }

    if(SLEEP_MODE_DS0 == slpMode)
    {
       PowerUpConfig();
    }

    // Restore other modules state 
    if(slpMode & SLEEP_MODE_DS0)
    {
        HWREG(0x44E0004C) = 2; // SPI0
        while(0x00000000 != (HWREG(0x44E0004C) & 0x00030000));
    }
	
    if((SLEEP_MODE_DS0 == slpMode) && (wakeSource != WAKE_SOURCE_TSC))
    {
         do
         {
              status = TSCADCSequencerFSMBusyStatus(SOC_ADC_TSC_0_REGS);
         }while(status);	

         TouchEnable();
       	 /* Enable ADC power */
	 TSCADCSetADCPowerUp(SOC_ADC_TSC_0_REGS);
    }

    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS,0xFFFF);

    /*  disable wake source     */
    disableWakeSource(dsData.dsDataBits.wakeSources);

    // Reset CM3 State Machine
    configIPCRegs(dsDataM3reset);	
    syncCm3();
	
    // Check the DS status
    if(0x1 == (readCmdStatus() & (0x1)))
        UARTPuts("\n\r DS attempt failed", -1);
    else
        UARTPuts("\n\r DS attempt passed", -1);	
	
    if((slpMode & SLEEP_MODE_DS2) || (slpMode & SLEEP_MODE_DS1))
    {
	    /*	delay to reduce the frequency of sleep/wake cycle for DS1 and DS2	*/
	    delay(500);
    }

    for(i = 0; i < 2;  i++)
    {
         x_data[i] = 0;
         y_data[i] = 0;
    }
    // Reset sleep trigger flag
    IsTSPress = 0;
}

/*
** Steps to be taken when DS2 is selected
*/
void ActionDeepSleep2(void)
{
    UARTPuts("\n\r DS2 is not supported in this release...", -1);
    //UARTPuts("\n\rEntering DS2 Power Saving Mode...", -1);
    //PowerSaveModeEnter(ds2Data, SLEEP_MODE_DS2);	
}

/*
** Steps to be taken when DS1 Deviation is selected
*/
void ActionDeepSleep1Dev(void)
{
    UARTPuts("\n\rEntering DS1 Deviation Power Saving Mode...", -1);
	
    // Configure Sleep mode slide
    isSleepModeSel = 1;
    updatePage(imageCount);	

    isSleepModeSel = 0;
	
    PowerSaveModeEnter(ds1DevData, SLEEP_MODE_DS1_DEV);
	
    // Configure Normal mode slide
    updatePage(imageCount);	
}

/*
** Steps to be taken when DS1 is selected
*/
void ActionDeepSleep1(void)
{
    UARTPuts("\n\rEntering DS1 Power Saving Mode...", -1);
    PowerSaveModeEnter(ds1Data, SLEEP_MODE_DS1);
}

/*
** Steps to be taken when DS0 is selected
*/
void ActionDeepSleep0(void)
{
    UARTPuts("\n\rEntering DS0 Power Saving Mode...", -1);
    PowerSaveModeEnter(ds0Data, SLEEP_MODE_DS0);
}

/* MPU and CM3 Sync	*/
void syncCm3(void)
{
    /*  Generate mailbox interrupt to CM3       */
    generateMailboxInt(MAILBOX_0_BASE_ADDR);

    /*  Wait for ACK from CM3   */
    waitForM3Txevent();

    /*  Clear the message in mailbox    */
    clearMailboxMsg();
}

/*	Enable Uart wakeup	*/
void enableUartWakeup(void)
{
    // enable rx interrupt
	UARTIntEnable(SOC_UART_0_REGS, (UART_IER_RHR_IT_ENABLE << UART_IER_RHR_IT_SHIFT) |
						(UART_IER_LINE_STS_IT_ENABLE < UART_IER_LINE_STS_IT_SHIFT));
						
	// Uart Wake up enable
	UARTWakeUpControl(SOC_UART_0_REGS, (UART_SYSC_ENAWAKEUP_ENABLE << 
										UART_SYSC_ENAWAKEUP_SHIFT));
	// Uart idle mode
	UARTIdleModeConfigure(SOC_UART_0_REGS, (UART_SYSC_IDLEMODE_WAKEUP << 
										UART_SYSC_IDLEMODE_SHIFT));
}

/*	Disable Uart wakeup	*/
void disableUartWakeup(void)
{
    // disable rx interrupt
	UARTIntDisable(SOC_UART_0_REGS, (UART_IER_RHR_IT_ENABLE << UART_IER_RHR_IT_SHIFT) |
						(UART_IER_LINE_STS_IT_ENABLE < UART_IER_LINE_STS_IT_SHIFT));
						
	// Uart Wake up disable
	UARTWakeUpControl(SOC_UART_0_REGS, (UART_SYSC_ENAWAKEUP_DISABLE << 
										UART_SYSC_ENAWAKEUP_SHIFT));
	// Uart idle mode
	UARTIdleModeConfigure(SOC_UART_0_REGS, (UART_SYSC_IDLEMODE_SMART << 
										UART_SYSC_IDLEMODE_SHIFT));
}

/*	Configure MPU PLL	*/
void mpuPllConfig(unsigned int oscClk, unsigned int N, unsigned int M, unsigned int M2)
{
	unsigned int clkmode;
	unsigned int clksel;
	unsigned int div_m2;

	clkmode=HWREG(CM_CLKMODE_DPLL_MPU);
	clksel= HWREG(CM_CLKSEL_DPLL_MPU);
	div_m2= HWREG(CM_DIV_M2_DPLL_MPU);

	/*	Put the DPLL in bypass mode	*/
	WR_MEM_32(CM_CLKMODE_DPLL_MPU,
		CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE);

	/*	Wait for bypass status	*/
	while(((HWREG(CM_IDLEST_DPLL_MPU) & CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS) != \
							CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS)); 

	/*	Set multiply and divide values	*/
	clksel = clksel & (~(CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT | \
						CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV));
	clksel = clksel | ((M << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT) | N);
	WR_MEM_32(CM_CLKSEL_DPLL_MPU,clksel);
	
	/*	Configure Post divider- M2	*/
	div_m2 = div_m2 & ~(CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV);
	div_m2 = div_m2 | M2;  
	WR_MEM_32(CM_DIV_M2_DPLL_MPU,div_m2);

	/*	Now lock the DPLL	*/
	clkmode = clkmode | CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_LOCK_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_MPU,clkmode);
	
	/*	Wait for lock	*/
	while(((HWREG(CM_IDLEST_DPLL_MPU) & CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK) != \
				CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK_DPLL_LOCKED));
}

/*	Configure CORE PLL	*/
void corePllConfig(unsigned int oscClk, unsigned int N, unsigned int M,
                   unsigned int M4, unsigned int M5, unsigned int M6)
{
	unsigned int clkmode;
	unsigned int clksel;
	unsigned int div_m4;
	unsigned int div_m5;
	unsigned int div_m6;

	clkmode=HWREG(CM_CLKMODE_DPLL_CORE);
	clksel= HWREG(CM_CLKSEL_DPLL_CORE);
	div_m4= HWREG(CM_DIV_M4_DPLL_CORE); 
	div_m5= HWREG(CM_DIV_M5_DPLL_CORE); 
	div_m6= HWREG(CM_DIV_M6_DPLL_CORE); 

	/*	Put DPLL in bypass mode	*/
	clkmode = (clkmode & ~(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN)) | \
						CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN_DPLL_MN_BYP_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_CORE,clkmode);
	
	/*	Wait for bypass status	*/
	while((HWREG(CM_IDLEST_DPLL_CORE) & CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS ) != \
									CM_WKUP_CM_IDLEST_DPLL_CORE_ST_MN_BYPASS); 

	/*	Set multiply and divide values	*/
	clksel = clksel & (~(CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT | \
							CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_DIV));
	clksel = clksel | ((M  << CM_WKUP_CM_CLKSEL_DPLL_CORE_DPLL_MULT_SHIFT) | N);
	WR_MEM_32(CM_CLKSEL_DPLL_CORE,clksel);
	
	/*	Configure Post divider- M4	*/
	div_m4= M4;
	WR_MEM_32(CM_DIV_M4_DPLL_CORE,div_m4);
	
	/*	Configure Post divider- M5	*/
	div_m5=  M5;
	WR_MEM_32(CM_DIV_M5_DPLL_CORE,div_m5);
	
	/*	Configure Post divider- M6	*/
	div_m6=  M6;
	WR_MEM_32(CM_DIV_M6_DPLL_CORE,div_m6);

	/*	Lock the PLL	*/
	clkmode =(clkmode & ~(CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN)) | \
							CM_WKUP_CM_CLKMODE_DPLL_CORE_DPLL_EN;
	WR_MEM_32(CM_CLKMODE_DPLL_CORE,clkmode);

	/*	Wait till the PLL is locked	*/
	while((HWREG(CM_IDLEST_DPLL_CORE) & CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK )!= \
							CM_WKUP_CM_IDLEST_DPLL_CORE_ST_DPLL_CLK_DPLL_LOCKED);
}

/*	Configure PER PLL	*/
void perPllConfig(unsigned int oscClk, unsigned int N,
                  unsigned int M, unsigned int M2)
{
	unsigned int clkmode;
	unsigned int clksel;
	unsigned int div_m2;

	clkmode=HWREG(CM_CLKMODE_DPLL_PER);
	clksel= HWREG(CM_CLKSEL_DPLL_PER);
	div_m2= HWREG(CM_DIV_M2_DPLL_PER);

	/*	Configure PER Pll in bypass mode	*/
	clkmode =(clkmode & ~(CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN)) | \
				CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_DPLL_MN_BYP_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_PER,clkmode);
	
	/*	Wait till the PLL enters bypas mode	*/
	while((HWREG(CM_IDLEST_DPLL_PER) & CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS )!= \
								CM_WKUP_CM_IDLEST_DPLL_PER_ST_MN_BYPASS);

		
	/*	Configure Multiplier and divider values	*/
	clksel = clksel & (~(CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_SD_DIV |
						CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT |
						CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_DIV));
	clksel = clksel | PM_PER_PLL_SD_DIV;
	clksel = clksel | ((M << CM_WKUP_CM_CLKSEL_DPLL_PERIPH_DPLL_MULT_SHIFT) | N);
	WR_MEM_32(CM_CLKSEL_DPLL_PER,clksel);
	
	/*	Configure M2 divider	*/
	div_m2= (~CM_WKUP_CM_DIV_M2_DPLL_PER_DPLL_CLKOUT_DIV) | M2;
	WR_MEM_32(CM_DIV_M2_DPLL_PER,div_m2);
	
	/*	Configre PLL in lock mode	*/
	clkmode =(clkmode & (~CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN)) | \
			CM_WKUP_CM_CLKMODE_DPLL_PER_DPLL_EN_DPLL_LOCK_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_PER,clkmode);

	/*	wait till the PLL is locked	*/
	while((HWREG(CM_IDLEST_DPLL_PER) & CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK )!= \
					CM_WKUP_CM_IDLEST_DPLL_PER_ST_DPLL_CLK_DPLL_LOCKED);

}

/*	Configure DISP PLL	*/
void dispPllConfig(unsigned int oscClk,unsigned int N,unsigned int M,unsigned int M2)
{
	unsigned int clkmode,clksel,div_m2;

	clkmode=HWREG(CM_CLKMODE_DPLL_DISP);
	clksel= HWREG(CM_CLKSEL_DPLL_DISP);
	div_m2= HWREG(CM_DIV_M2_DPLL_DISP);

	/*	Configure PLL in bypass mode	*/
	clkmode =(clkmode & (~CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN))| \
				CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_MN_BYP_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_DISP,clkmode);
	
	/*	Wait till the PLL enters bypass mode	*/
	while((HWREG(CM_IDLEST_DPLL_DISP) & CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS )!= \
										CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS);

	/*	Configure multiplier and divider	*/
	clksel = clksel & (~(CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT | \
										CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_DIV));
	clksel = clksel | ((M << CM_WKUP_CM_CLKSEL_DPLL_DISP_DPLL_MULT_SHIFT) | N);
	WR_MEM_32(CM_CLKSEL_DPLL_DISP,clksel);
	
	/*	Configre M2 divider	*/
	div_m2= (~CM_WKUP_CM_DIV_M2_DPLL_DISP_DPLL_CLKOUT_DIV) | M2;
	WR_MEM_32(CM_DIV_M2_DPLL_DISP,div_m2);
	
	/*	Configre PLL in Lock mode	*/
	clkmode =(clkmode& (~CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN))| \
							CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_LOCK_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_DISP,clkmode);

	/*	Wait till the PLL is locked	*/
	while((HWREG(CM_IDLEST_DPLL_DISP) & CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK )!= \
							CM_WKUP_CM_IDLEST_DPLL_DISP_ST_DPLL_CLK_DPLL_LOCKED);
}

/*	Configure display pll in bypass mode	*/
void bypassDispPll()
{
	unsigned int clkmode;

	/*	COnfigure PLL in bypass mode	*/
    clkmode=HWREG(CM_CLKMODE_DPLL_DISP);
    clkmode =(clkmode & (~CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN))| \
						CM_WKUP_CM_CLKMODE_DPLL_DISP_DPLL_EN_DPLL_MN_BYP_MODE;
	WR_MEM_32(CM_CLKMODE_DPLL_DISP,clkmode);
	
	/*	Wait till PLL enters bypass mode	*/
	while((HWREG(CM_IDLEST_DPLL_DISP) & CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS) != \
									CM_WKUP_CM_IDLEST_DPLL_DISP_ST_MN_BYPASS);
}


/*
** Flush Touch screen FIFO.
*/
static void TouchScreenFIFOFlush(void)
{
    volatile unsigned int wordsLeft = 0;
    unsigned int i;

    do
    {
         wordsLeft = TSCADCFIFOWordCountRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);

         for(i = 0; i < wordsLeft; i++)
         {
              TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);
         }

    }while(wordsLeft != 0x00);

    do
    {
         wordsLeft = TSCADCFIFOWordCountRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);

         for(i = 0; i < wordsLeft; i++)
         {
               TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);
         }

    }while(wordsLeft != 0x00);
}
