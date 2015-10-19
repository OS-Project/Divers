/**
 * \file   uartEcho_edma.c
 *
 * \brief  This is a sample application file which invokes some APIs
 *         from the EDMA3 device abstraction layer as well as UART/IrDA/CIR
 *         device abstraction layer to perform configuration and
 *         transfer of data between UART and DDR SDRAM by the
 *         use of EDMA3 controller.
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

#include "uart_irda_cir.h"
#include "soc_AM335x.h"
#include "edma_event.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "edma.h"

/****************************************************************************/
/*                      INTERNAL MACROS                                     */
/****************************************************************************/
/* Address of THR and RHR registers of UART. */
#define UART_THR_RHR_REG          (SOC_UART_0_REGS)

/* UART Module Input Frequency. */
#define UART_MODULE_INPUT_CLK     (48000000)

/* Baud Rate of UART for communication. */
#define BAUD_RATE_115200          (115200)
#define BAUD_RATE_128000          (128000)
#define BAUD_RATE_230400          (230400)
#define BAUD_RATE_460800          (460800)
#define BAUD_RATE_921600          (921600)

/* Transmit DMA Threshold Value. */
#define TX_DMA_THRESHOLD          (8)

/* Receive DMA Threshold Value. */
#define RX_DMA_THRESHOLD          (1)

/* Receiver Buffer Size. */
#define RX_BUFFER_SIZE            (8)

/* EDMA3 Event queue number. */
#define EVT_QUEUE_NUM             (0)

/* PaRAM Set number for Dummy Transfer. */
#define DUMMY_CH_NUM              (5)

/* Wrapper Definitions. */
#define UART_INSTANCE_BASE_ADD    (SOC_UART_0_REGS)
#define EDMA3_UART_TX_CHA_NUM     (EDMA3_CHA_UART0_TX)
#define EDMA3_UART_RX_CHA_NUM     (EDMA3_CHA_UART0_RX)
#define UART_INT_NUM              (SYS_INT_UART0INT)

/****************************************************************************/
/*                      LOCAL FUNCTION PROTOTYPES                           */
/****************************************************************************/

static void callback(unsigned int tccNum, unsigned int status);
static void UartEDMATxConfTransfer(unsigned int tccNum,
                                   unsigned int chNum,
                                   volatile unsigned char *txBuffer,
                                   unsigned int length);
static void UartEDMARxConfTransfer(unsigned int tccNum,
                                   unsigned int chNum,
                                   volatile unsigned char *rxBuffer,
                                   unsigned int length);

#ifdef UART_ENABLE_FIFO
static void UartFIFOConfigure(void);
#endif

static void TxDummyPaRAMConfEnable(void);
static void EDMA3INTCConfigure(void);
static void Edma3CompletionIsr(void);
static void UartBaudRateSet(void);
static void Edma3CCErrorIsr(void);
static void EDMA3Initialize(void);
static void UARTInitialize(void);

/****************************************************************************/
/*                      GLOBAL VARIABLES                                    */
/****************************************************************************/
static void (*cb_Fxn[EDMA3_NUM_TCC]) (unsigned int tcc, unsigned int status);

volatile unsigned char str80[] = "80* abcdefghABCDEFGHabcdefghABCDEFGHabcdefghABCDEFGHabcdefghABCDEFGHabcdefgh *\r\n";
volatile unsigned char str64[] = "64* abcdefghABCDEFGHabcdefghABCDEFGHabcdefghABCDEFGHabcdefgh *\r\n";
volatile unsigned char str56[] = "56* abcdefghABCDEFGHabcdefghABCDEFGHabcdefghABCDEFGH *\r\n";
volatile unsigned char welcome[] = "StarterWare AM335X UART DMA application.";
volatile unsigned char enter[] = "\r\nPlease Enter 08 bytes from keyboard.\r\n";
volatile unsigned int clBackFlag = 0;

volatile unsigned char rxBuffer[RX_BUFFER_SIZE];

/****************************************************************************/
/*                   LOCAL FUNCTION DEFINITIONS                             */
/****************************************************************************/

int main(void)
{
    /* Configuring the system clocks for EDMA. */
    EDMAModuleClkConfig();

    /* Configuring the system clocks for UART0 instance. */
    UART0ModuleClkConfig();

    /* Performing Pin Multiplexing for UART0 instance. */
    UARTPinMuxSetup(0);

    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Initializing the EDMA. */
    EDMA3Initialize();

    /* Initializing the UART0 instance for use. */
    UARTInitialize();

    /*
    ** Configuring the EDMA.
    */

    /* Request DMA Channel and TCC for UART Transmit*/
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_UART_TX_CHA_NUM, EDMA3_UART_TX_CHA_NUM,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for TX*/
    cb_Fxn[EDMA3_UART_TX_CHA_NUM] = &callback;

    /* Request DMA Channel and TCC for UART Receive */
    EDMA3RequestChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                        EDMA3_UART_RX_CHA_NUM, EDMA3_UART_RX_CHA_NUM,
                        EVT_QUEUE_NUM);

    /* Registering Callback Function for RX*/
    cb_Fxn[EDMA3_UART_RX_CHA_NUM] = &callback;

    /******************** Transmission of a string **************************/

    /* Configuring EDMA PaRAM sets to transmit 'welcome' message. */
    UartEDMATxConfTransfer(EDMA3_UART_TX_CHA_NUM,
                           EDMA3_UART_TX_CHA_NUM,
                           welcome,
                           sizeof(welcome) - 1);

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /******************** Transmission of a string **************************/

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring EDMA PaRAM sets to transmit 'enter' message. */
    UartEDMATxConfTransfer(EDMA3_UART_TX_CHA_NUM,
                           EDMA3_UART_TX_CHA_NUM,
                           enter,
                           sizeof(enter) - 1);
 
    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /********************* Receiving Data from User *************************/

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring the PaRAM set for reception. */
    UartEDMARxConfTransfer(EDMA3_UART_RX_CHA_NUM,
                           EDMA3_UART_RX_CHA_NUM,
                           rxBuffer,
                           sizeof(rxBuffer));
    
    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /******************* Echoing received bytes *****************************/

    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);

    /* Configuring the PaRAM set to transmit the bytes that were received. */
    UartEDMATxConfTransfer(EDMA3_UART_TX_CHA_NUM,
                           EDMA3_UART_TX_CHA_NUM,
                           rxBuffer,
                           sizeof(rxBuffer));

    /* Wait for return from callback */
    while(0 == clBackFlag);
    clBackFlag = 0;

    /******************* Freeing of allocated channels **********************/

    /* Free EDMA3 Channels for TX and RX */
    EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                     EDMA3_UART_TX_CHA_NUM, EDMA3_TRIG_MODE_EVENT,
                     EDMA3_UART_TX_CHA_NUM, EVT_QUEUE_NUM);

    EDMA3FreeChannel(SOC_EDMA30CC_0_REGS, EDMA3_CHANNEL_TYPE_DMA,
                     EDMA3_UART_RX_CHA_NUM, EDMA3_TRIG_MODE_EVENT,
                     EDMA3_UART_RX_CHA_NUM, EVT_QUEUE_NUM);


    while(1);
}

/*
** This function is used to set the PaRAM entries in EDMA3 for the Transmit
** channel of UART. The EDMA transfer is also started within this API.
** However, the actual transfer shall happen only when DMA feature is enabled
** in the UART.
*/

static void UartEDMATxConfTransfer(unsigned int tccNum,
                                   unsigned int chNum,
                                   volatile unsigned char *txBuffer,
                                   unsigned int length)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Fill the PaRAM Set with transfer specific information */
    paramSet.srcAddr = (unsigned int) txBuffer;
    paramSet.destAddr = (unsigned int)UART_THR_RHR_REG;

    /* A, B, and C Counts. */
    paramSet.aCnt = 1;
    if(length < TX_DMA_THRESHOLD)
    {
        paramSet.bCnt = 1;
        paramSet.cCnt = (unsigned short)length;
        paramSet.srcCIdx = 1;
        paramSet.srcBIdx = 1;
    }
#ifdef UART_ENABLE_FIFO
    else
    {
        paramSet.bCnt = (unsigned short)TX_DMA_THRESHOLD;
        paramSet.cCnt = (unsigned short)(length/TX_DMA_THRESHOLD);
        paramSet.srcBIdx = 1;
        paramSet.srcCIdx = (short)TX_DMA_THRESHOLD;
    }
#else
    else
    {
        paramSet.bCnt = (unsigned short)1;
        paramSet.cCnt = (unsigned short)length;
        paramSet.srcBIdx = 1;
        paramSet.srcCIdx = (short)1;
    }
#endif

    /* The destination indexes should not increment since it is a h/w register. */
    paramSet.destBIdx = 0;
    paramSet.destCIdx = 0;

    /* Linking this PaRAM set to a Dummy PaRAM set. */
    paramSet.linkAddr = (unsigned short)(EDMA3CC_OPT(DUMMY_CH_NUM));
    paramSet.bCntReload = 0;

    /* OPT PaRAM entries. */
    paramSet.opt = 0x00000000u;

    /* Source and Destination addressing modes are Incremental. */

    /* AB Synchronized Transfer. */
    paramSet.opt |= (1 << EDMA3CC_OPT_SYNCDIM_SHIFT);

    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* Enabling the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);

    /* Configuring the PaRAM set for Dummy Transfer. */
    TxDummyPaRAMConfEnable();

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, chNum, EDMA3_TRIG_MODE_EVENT);
}

/*
** This function is used to set the PaRAM entries in EDMA3 for the Receive
** Channel of UART. The EDMA transfer is also started within this wrapper
** function. However, the actual transfer shall happen only when the DMA
** feature is enabled in the UART.
*/

static void UartEDMARxConfTransfer(unsigned int tccNum,
                                   unsigned int chNum,
                                   volatile unsigned char *rxBuffer,
                                   unsigned int length)
{
    EDMA3CCPaRAMEntry paramSet;

    /* Fill the PaRAM Set with transfer specific information */
    paramSet.srcAddr = (unsigned int)UART_THR_RHR_REG;
    paramSet.destAddr = (unsigned int) rxBuffer;

    /* A, B and C Count values. */
    paramSet.aCnt = (unsigned short)RX_DMA_THRESHOLD;
    paramSet.bCnt = (unsigned short)length/RX_DMA_THRESHOLD;
    paramSet.cCnt = 1;
    paramSet.destBIdx = (short)RX_DMA_THRESHOLD;

    paramSet.srcBIdx = 0;
    paramSet.srcCIdx = 0;
    paramSet.destCIdx = 0;

    paramSet.linkAddr = 0xFFFF;
    paramSet.bCntReload = 0;

    /* OPT PaRAM Entries. */
    paramSet.opt = 0x00000000u;

    /* Source and Destination addressing modes are Incremental. */

    /* Setting the Transfer Complete Code(TCC). */
    paramSet.opt |= ((tccNum << EDMA3CC_OPT_TCC_SHIFT) & EDMA3CC_OPT_TCC);

    /* Enabling the Completion Interrupt. */
    paramSet.opt |= (1 << EDMA3CC_OPT_TCINTEN_SHIFT);

    /* Now write the PaRAM Set */
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, chNum, &paramSet);

    /* Enable EDMA Transfer */
    EDMA3EnableTransfer(SOC_EDMA30CC_0_REGS, chNum, EDMA3_TRIG_MODE_EVENT);
}

/*
** This configures the PaRAM set for the Dummy Transfer.
*/

static void TxDummyPaRAMConfEnable(void)
{
    EDMA3CCPaRAMEntry dummyPaRAMSet;

    EDMA3GetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);

    dummyPaRAMSet.aCnt = 1;
    dummyPaRAMSet.bCnt = 0;
    dummyPaRAMSet.cCnt = 0;
    dummyPaRAMSet.srcAddr = 0;
    dummyPaRAMSet.destAddr = 0;
    dummyPaRAMSet.srcBIdx = 0;
    dummyPaRAMSet.destBIdx = 0;
    dummyPaRAMSet.srcCIdx = 0;
    dummyPaRAMSet.destCIdx = 0; 
    dummyPaRAMSet.linkAddr = 0xFFFF;
    dummyPaRAMSet.bCntReload = 0;
    dummyPaRAMSet.opt = 0;
    
    EDMA3SetPaRAM(SOC_EDMA30CC_0_REGS, DUMMY_CH_NUM, &dummyPaRAMSet);
}

/*
** EDMA Completion Interrupt Service Routine(ISR).
*/

static void Edma3CompletionIsr(void)
{
    volatile unsigned int pendingIrqs;
    volatile unsigned int isIPR = 0;

    unsigned int indexl;
    unsigned int Cnt = 0;

    indexl = 1;

    isIPR  = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
    if(isIPR)
    {
        while ((Cnt < EDMA3CC_COMPL_HANDLER_RETRY_COUNT)&& (indexl != 0u))
        {
            indexl = 0u;
            pendingIrqs = EDMA3GetIntrStatus(SOC_EDMA30CC_0_REGS);
            while (pendingIrqs)
            {
                if((pendingIrqs & 1u) == TRUE)
                {
                    /**
                    * If the user has not given any callback function
                    * while requesting the TCC, its TCC specific bit
                    * in the IPR register will NOT be cleared.
                    */
                    /* Writing to ICR to clear the corresponding IPR bits. */
                    EDMA3ClrIntr(SOC_EDMA30CC_0_REGS, indexl);

                    (*cb_Fxn[indexl])(indexl, EDMA3_XFER_COMPLETE);
                }
                ++indexl;
                pendingIrqs >>= 1u;
            }
            Cnt++;
        }
    }
}

/*
** This function is used as a callback from EDMA3 Completion Handler.
** UART in DMA Mode is Disabled over here.
*/
static void callback(unsigned int tccNum, unsigned int status)
{
    /* Disabling DMA Mode of operation in UART. */
    UARTDMADisable(UART_INSTANCE_BASE_ADD);

    /* Disabling DMA transfer on the specified channel. */
    EDMA3DisableTransfer(SOC_EDMA30CC_0_REGS, tccNum, EDMA3_TRIG_MODE_EVENT);

    clBackFlag = 1;
}

/*
** EDMA Error Interrupt Service Routine(ISR).
*/

static void Edma3CCErrorIsr(void)
{
    volatile unsigned int pendingIrqs = 0;
    unsigned int evtqueNum = 0;
    unsigned int index = 1;
    unsigned int Cnt = 0;

    if((0 != EDMA3GetErrIntrStatus(SOC_EDMA30CC_0_REGS)) ||
       (0 != (EDMA3GetCCErrStatus(SOC_EDMA30CC_0_REGS))))
    {
        /* Loop for EDMA3CC_ERR_HANDLER_RETRY_COUNT number of time, breaks
           when no pending interrupt is found */
        while ((Cnt < EDMA3CC_ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;
            pendingIrqs = EDMA3GetErrIntrStatus(SOC_EDMA30CC_0_REGS);

            while (pendingIrqs)
            {
                /*Process all the pending interrupts*/
                if((pendingIrqs & 1u)==TRUE)
                {
                    /* Writing to EMCR to clear the corresponding EMR bits.
                       Also clearing any Secondary events in SER. */
                    EDMA3ClrMissEvt(SOC_EDMA30CC_0_REGS, index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;

            pendingIrqs = EDMA3GetCCErrStatus(SOC_EDMA30CC_0_REGS);
            if (pendingIrqs != 0u)
            {
                /* Process all the pending CC error interrupts. */
                /* Queue threshold error for different event queues. */
                for (evtqueNum = 0u; evtqueNum < EDMA3_0_NUM_EVTQUE; evtqueNum++)
                {
                    if((pendingIrqs & (1u << evtqueNum)) != 0u)
                    {
                        /* Clear the error interrupt. */
                        EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS, (1u << evtqueNum));
                    }
                }

                /* Transfer completion code error. */
                if ((pendingIrqs & (1 << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
                {
                    EDMA3ClrCCErr(SOC_EDMA30CC_0_REGS,
                                  (0x01u << EDMA3CC_CCERR_TCCERR_SHIFT));
                }
                ++index;
            }

            Cnt++;
        }
    }
}

/* 
** Powering up, initializing and registering interrupts for EDMA.
*/

static void EDMA3Initialize(void)
{
    /* Initialization of EDMA3 */
    EDMA3Init(SOC_EDMA30CC_0_REGS, EVT_QUEUE_NUM);

    /* Configuring the AINTC to receive EDMA3 interrupts. */ 
    EDMA3INTCConfigure();
}

/*
** This function initializes the UART instance for use.
*/

static void UARTInitialize(void)
{
    /* Performing a module reset. */
    UARTModuleReset(UART_INSTANCE_BASE_ADD);

#ifdef UART_ENABLE_FIFO
    /* Performing FIFO configurations. */
    UartFIFOConfigure();
#else
    /* Enabling DMA Mode 1. */
    UARTDMAEnable(UART_INSTANCE_BASE_ADD, UART_DMA_MODE_1_ENABLE);
#endif

#ifdef UART_ENABLE_FIFO
    /* Selecting the method of setting the Transmit DMA Threshold value. */
    UARTTxDMAThresholdControl(UART_INSTANCE_BASE_ADD, UART_TX_DMA_THRESHOLD_REG);

    /* Configuring the Transmit DMA Threshold value. */
    UARTTxDMAThresholdValConfig(UART_INSTANCE_BASE_ADD, TX_DMA_THRESHOLD);
#endif

    /* Performing Baud Rate settings. */
    UartBaudRateSet();
    
    /* Switching to Configuration Mode B. */
    UARTRegConfigModeEnable(UART_INSTANCE_BASE_ADD, UART_REG_CONFIG_MODE_B);

    /* Programming the Line Characteristics. */
    UARTLineCharacConfig(UART_INSTANCE_BASE_ADD,
                         (UART_FRAME_WORD_LENGTH_8 | UART_FRAME_NUM_STB_1), 
                         UART_PARITY_NONE);

    /* Disabling write access to Divisor Latches. */
    UARTDivisorLatchDisable(UART_INSTANCE_BASE_ADD);

    /* Disabling Break Control. */
    UARTBreakCtl(UART_INSTANCE_BASE_ADD, UART_BREAK_COND_DISABLE);

    /* Switching to UART16x operating mode. */
    UARTOperatingModeSelect(UART_INSTANCE_BASE_ADD, UART16x_OPER_MODE);
}


#ifdef UART_ENABLE_FIFO
/*
** A wrapper function performing FIFO configurations.
*/
static void UartFIFOConfigure(void)
{
    unsigned int fifoConfig = 0;

    /* Setting the TX and RX FIFO Trigger levels as 1. No DMA enabled. */
    fifoConfig = UART_FIFO_CONFIG(UART_TRIG_LVL_GRANULARITY_4,
                                  UART_TRIG_LVL_GRANULARITY_1,
                                  UART_FCR_TX_TRIG_LVL_56,
                                  RX_DMA_THRESHOLD,
                                  1,
                                  1,
                                  UART_DMA_EN_PATH_SCR,
                                  UART_DMA_MODE_1_ENABLE);

    /* Configuring the FIFO settings. */
    UARTFIFOConfig(UART_INSTANCE_BASE_ADD, fifoConfig);
}
#endif

/*
** A wrapper function performing Baud Rate settings.
*/

static void UartBaudRateSet(void)
{
    unsigned int divisorValue = 0;

    /* Computing the Divisor Value. */
    divisorValue = UARTDivisorValCompute(UART_MODULE_INPUT_CLK,
                                         BAUD_RATE_115200,
                                         UART16x_OPER_MODE,
                                         UART_MIR_OVERSAMPLING_RATE_42);

    /* Programming the Divisor Latches. */
    UARTDivisorLatchWrite(UART_INSTANCE_BASE_ADD, divisorValue);

}

/*
** This function configures the AINTC to receive EDMA3 interrupts.
*/

static void EDMA3INTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering EDMA3 Channel Controller 0 transfer completion interrupt.  */
    IntRegister(SYS_INT_EDMACOMPINT, Edma3CompletionIsr);

    /* Setting the priority for EDMA3CC0 completion interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMACOMPINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Registering EDMA3 Channel Controller 0 Error Interrupt. */
    IntRegister(SYS_INT_EDMAERRINT, Edma3CCErrorIsr);

    /* Setting the priority for EDMA3CC0 Error interrupt in AINTC. */
    IntPrioritySet(SYS_INT_EDMAERRINT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the EDMA3CC0 completion interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMACOMPINT);
    
    /* Enabling the EDMA3CC0 Error interrupt in AINTC. */
    IntSystemEnable(SYS_INT_EDMAERRINT);
}

/******************************** End of file *******************************/
