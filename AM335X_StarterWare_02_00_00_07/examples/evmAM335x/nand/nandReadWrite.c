/**
 *  \file   nandReadWrite.c
 *
 *  \brief  Sample application for NAND.
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


#include "hw_gpmc.h"
#include "hw_types.h"
#include "soc_AM335x.h"
#include "uartStdio.h"
#include "interrupt.h"
#include "gpmc.h"
#include "nandlib.h"
#include "nand_gpmc.h"
#include "nandDma.h"
#include "evmAM335x.h"


/*******************************************************************************
*                       INTERNAL MACRO DEFINITIONS
*******************************************************************************/

/* #define NAND_DATAINTEGRITY_TEST_WITH_FIXED_ADDR */

/*****************************************************************************/
/*
** Macros which defines attached device info like num of pages per block, 
** number of blocks, device ID and manufacturer ID.
**
*/
#define NAND_DATA_XFER_MODE                     (NAND_XFER_MODE_DMA)
#define NAND_BUSWIDTH                           (NAND_BUSWIDTH_8BIT)
#define NAND_CHIP_SELECT                        (NAND_CHIP_SELECT_0)
#define NAND_PAGE_SIZE_IN_BYTES                 (NAND_PAGESIZE_2048BYTES)
#define NAND_BLOCK_SIZE_IN_BYTES                (NAND_BLOCKSIZE_128KB)
#define NAND_NUMOF_BLK                          (2048)
#define NAND_MANUFATURER_MICRON_ID              (0x2C)
#define NAND_DEVICE_ID                          (0xDA)
#define MAX_VALUE_OF_CHAR                       (256)

/*****************************************************************************/
/*
** Macros which defines the read write size, buffer size and number of transfers
**
*/
#define NAND_DATA_BUFF_SIZE                     (NAND_PAGE_SIZE_IN_BYTES)
#define NAND_ECC_BUFF_SIZE                      ((NAND_PAGE_SIZE_IN_BYTES/NAND_BYTES_PER_TRNFS) \
                                                 * NAND_MAX_ECC_BYTES_PER_TRNFS)
/*****************************************************************************/
/*
** Macros which defines default block, page and num of pages for read/write.
**
*/
#define NAND_DEFAULT_START_PAGE                 (0)
#define NAND_DEFAULT_BLK                        (5)
#define NAND_DEFAULT_NMBR_OF_PAGES              (1)

/*****************************************************************************/
/*
** Macros which defines the data integrity status.
**
*/
#define NAND_DATA_INTEGRITY_PASS                (0)
#define NAND_DATA_INTEGRITY_FAIL                (1)

/*****************************************************************************/
/*
** Macros which defines the NAND timing info.
**
*/
#define NAND_CSWROFFTIME                        (30)
#define NAND_CSRDOFFTIME                        (31)
#define NAND_CSONTIME                           (0)

#define NAND_ADVONTIME                          (0)
#define NAND_ADVAADMUXONTIME                    (0)
#define NAND_ADVRDOFFTIME                       (31)
#define NAND_ADVWROFFTIME                       (31)
#define NAND_ADVAADMUXRDOFFTIME                 (0)
#define NAND_ADVAADMUXWROFFTIME                 (0)

#define NAND_WEOFFTIME                          (31)
#define NAND_WEONTIME                           (3)
#define NAND_OEAADMUXOFFTIME                    (31)
#define NAND_OEOFFTIME                          (31)
#define NAND_OEAADMUXONTIME                     (3)
#define NAND_OEONTIME                           (1)

#define NAND_RDCYCLETIME                        (31)
#define NAND_WRCYCLETIME                        (31)
#define NAND_RDACCESSTIME                       (28)
#define NAND_PAGEBURSTACCESSTIME                (0)

#define NAND_BUSTURNAROUND                      (0)
#define NAND_CYCLE2CYCLEDIFFCSEN                (0)
#define NAND_CYCLE2CYCLESAMECSEN                (1)
#define NAND_CYCLE2CYCLEDELAY                   (0)
#define NAND_WRDATAONADMUXBUS                   (15)
#define NAND_WRACCESSTIME                       (22)

/*****************************************************************************/
/*
** Macros which defines the chip select base address and cs region size.
**
*/
#define NAND_CS0_BASEADDR                       (0x10000000)
#define NAND_CS0_REGIONSIZE                     (GPMC_CS_SIZE_256MB)


/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/

#if defined(__IAR_SYSTEMS_ICC__)

#pragma data_alignment=4
volatile unsigned char txData[NAND_DATA_BUFF_SIZE];
#pragma data_alignment=4
volatile unsigned char rxData[NAND_DATA_BUFF_SIZE];

#elif defined(__TMS470__) || defined(_TMS320C6X)

#pragma DATA_ALIGN(txData, 4);
volatile unsigned char txData[NAND_DATA_BUFF_SIZE];
#pragma DATA_ALIGN(rxData, 4);
volatile unsigned char rxData[NAND_DATA_BUFF_SIZE];

#else

volatile unsigned char  __attribute__ ((aligned (4))) txData[NAND_DATA_BUFF_SIZE];
volatile unsigned char  __attribute__ ((aligned (4))) rxData[NAND_DATA_BUFF_SIZE];

#endif

unsigned char eccData[NAND_ECC_BUFF_SIZE];


/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/


/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  This function prints the device ID info of NAND.\n                  *
*                                                                             *
* \param  nandInfo        Pointer to structure which conatins controller and  *
*                         device information.                                 *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDDeviceIdInfoPrint(NandInfo_t *nandInfo)
{
    UARTPuts(" ****************** NAND DEVICE INFO ****************** \r\n",-1);
    UARTPuts("    MANUFACTURER ID    : ",-1);
    UARTPutHexNum((unsigned int)nandInfo->manId);
    UARTPuts("\r\n", -1);
    UARTPuts("    DEVICE ID          : ",-1);
    UARTPutHexNum((unsigned int)nandInfo->devId);
    UARTPuts("\r\n", -1);
    UARTPuts("    PAGESIZE           : ",-1);
    UARTPutNum((unsigned int)nandInfo->pageSize);
    UARTPuts(" Bytes\r\n", -1);
    UARTPuts("    BLK SIZE           : ",-1);
    UARTPutNum((unsigned int)nandInfo->blkSize);
    UARTPuts(" Bytes\r\n", -1);
    UARTPuts("    PAGES PER BLOCK    : ",-1);
    UARTPutNum((unsigned int)nandInfo->pagesPerBlk);
    UARTPuts("\r\n", -1);
    UARTPuts(" ****************************************************** \r\n",-1);
}

/******************************************************************************
*                                                                             *
* \brief  This function initializes the read, write and ecc buffers.\n        *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return none                                                                *
*                                                                             *
******************************************************************************/
static void NANDBuffersInit()
{
    unsigned int byteCnt;

    txData[0]  = 'S';
    txData[1]  = 'T';
    txData[2]  = 'A';
    txData[3]  = 'R';
    txData[4]  = 'T';
    txData[5]  = 'E';
    txData[6]  = 'R';
    txData[7]  = 'W';
    txData[8]  = 'A';
    txData[9]  = 'R';
    txData[10] = 'E';

    for(byteCnt = 11; byteCnt < NAND_PAGE_SIZE_IN_BYTES; byteCnt++)
    {
        txData[byteCnt]= ((byteCnt) % MAX_VALUE_OF_CHAR);
    }

    for(byteCnt = 0; byteCnt < NAND_PAGE_SIZE_IN_BYTES; byteCnt++)
    {
        rxData[byteCnt]= 0x00;
    }

    for(byteCnt = 0; byteCnt < NAND_ECC_BUFF_SIZE; byteCnt++)
    {
        eccData[byteCnt] = 0;
    }
}

/******************************************************************************
*                                                                             *
* \brief  This function checks for the data integrity of tx and rx buffers.\n *
*                                                                             *
* \param none                                                                 *
*                                                                             *
* \return Data inegrity status i.e.                                           *
*                                                                             *
*        NAND_DATA_INTEGRITY_PASS -- On pass                                  *
*        NAND_DATA_INTEGRITY_FAIL -- On fail                                  *
*                                                                             *
******************************************************************************/
static unsigned int NANDDataIntegrityCheck()
{
    unsigned int status = NAND_DATA_INTEGRITY_PASS;
    unsigned int byteCnt;

    for(byteCnt = 0; byteCnt < NAND_PAGE_SIZE_IN_BYTES; byteCnt++)
    {
        if(rxData[byteCnt] != txData[byteCnt])
        {
            status = NAND_DATA_INTEGRITY_FAIL;
            break;
        }
    }
    return (status);
}

/******************************************************************************
*                                                                             *
*                                                                             *
* \brief  Function to initalize the GPMC NAND timing and base addr info.      *
*                                                                             *
* \param  nandTimimgInfo : Pointer to structure containing                    *
*                          NAND timing info.                                  *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDTimingInfoInit(void *TimingInfo)
{

    GPMCNANDTimingInfo_t *nandTimingInfo;
    nandTimingInfo = (GPMCNANDTimingInfo_t * )TimingInfo;

    nandTimingInfo->CSWrOffTime               = NAND_CSWROFFTIME;
    nandTimingInfo->CSRdOffTime               = NAND_CSRDOFFTIME;
    nandTimingInfo->CSExtDelayFlag            = GPMC_CS_EXTRA_NODELAY;
    nandTimingInfo->CSOnTime                  = NAND_CSONTIME;

    nandTimingInfo->ADVAADMuxWrOffTime        = NAND_ADVAADMUXWROFFTIME;
    nandTimingInfo->ADVAADMuxRdOffTime        = NAND_ADVAADMUXRDOFFTIME;
    nandTimingInfo->ADVWrOffTime              = NAND_ADVWROFFTIME;
    nandTimingInfo->ADVRdOffTime              = NAND_ADVRDOFFTIME;
    nandTimingInfo->ADVExtDelayFlag           = GPMC_ADV_EXTRA_NODELAY;
    nandTimingInfo->ADVAADMuxOnTime           = NAND_ADVAADMUXONTIME;
    nandTimingInfo->ADVOnTime                 = NAND_ADVONTIME;

    nandTimingInfo->WEOffTime                 = NAND_WEOFFTIME;
    nandTimingInfo->WEExtDelayFlag            = GPMC_WE_EXTRA_NODELAY;
    nandTimingInfo->WEOnTime                  = NAND_WEONTIME;
    nandTimingInfo->OEAADMuxOffTime           = NAND_OEAADMUXOFFTIME;
    nandTimingInfo->OEOffTime                 = NAND_OEOFFTIME;
    nandTimingInfo->OEExtDelayFlag            = GPMC_OE_EXTRA_NODELAY;
    nandTimingInfo->OEAADMuxOnTime            = NAND_OEAADMUXONTIME;
    nandTimingInfo->OEOnTime                  = NAND_OEONTIME;

    nandTimingInfo->rdCycleTime               = NAND_RDCYCLETIME;
    nandTimingInfo->wrCycleTime               = NAND_WRCYCLETIME;
    nandTimingInfo->rdAccessTime              = NAND_RDACCESSTIME;
    nandTimingInfo->pageBurstAccessTime       = NAND_PAGEBURSTACCESSTIME;

    nandTimingInfo->cycle2CycleDelay          = NAND_CYCLE2CYCLEDELAY;
    nandTimingInfo->cycle2CycleDelaySameCSCfg = NAND_CYCLE2CYCLESAMECSEN;
    nandTimingInfo->cycle2CycleDelayDiffCSCfg = NAND_CYCLE2CYCLEDIFFCSEN;
    nandTimingInfo->busTAtime                 = NAND_BUSTURNAROUND;
}


/******************************************************************************
*                                                                             *
* \brief  Function to initialize the device and controller info.              *
*                                                                             *
* \param  nandInfo      : Pointer to structure containing controller and      *
*                         device information.                                 *
*                                                                             *
* \param  csNum         : Chip select where device is interfaced.             *
*                                                                             *
* \return none.                                                               *
*                                                                             *
******************************************************************************/
static void NANDInfoInit(NandInfo_t *nandInfo, unsigned int cs)
{
    NandCtrlInfo_t *hNandCtrlInfo = nandInfo->hNandCtrlInfo;
    NandDmaInfo_t  *hNandDmaInfo  = nandInfo->hNandDmaInfo;
    NandEccInfo_t  *hNandEccInfo  = nandInfo->hNandEccInfo;

    /* Init the NAND Device Info */
    nandInfo->opMode                        = NAND_DATA_XFER_MODE;
    nandInfo->eccType                       = NAND_ECC_ALGO_BCH_8BIT;

    nandInfo->chipSelectCnt                 = 1;
    nandInfo->dieCnt                        = 1;
    nandInfo->chipSelects[0]                = cs;
    nandInfo->busWidth                      = NAND_BUSWIDTH;
    nandInfo->pageSize                      = NAND_PAGE_SIZE_IN_BYTES;
    nandInfo->blkSize                       = NAND_BLOCK_SIZE_IN_BYTES;
    nandInfo->manId                         = NAND_MANUFATURER_MICRON_ID;
    nandInfo->devId                         = NAND_DEVICE_ID;
    nandInfo->dataRegAddr                   = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_DATA(GPMC_CHIP_SELECT_0));
    nandInfo->addrRegAddr                   = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_ADDRESS(GPMC_CHIP_SELECT_0));
    nandInfo->cmdRegAddr                    = (SOC_GPMC_0_REGS +
                                          GPMC_NAND_COMMAND(GPMC_CHIP_SELECT_0));
    /* Init the NAND Controller Info struct */
    hNandCtrlInfo->CtrlInit                 = GPMCNANDInit;
    hNandCtrlInfo->WaitPinStatusGet         = GPMCNANDWaitPinStatusGet;
    hNandCtrlInfo->currChipSelect           = cs;
    hNandCtrlInfo->baseAddr                 = SOC_GPMC_0_REGS;
    hNandCtrlInfo->eccSupported             = (NAND_ECC_ALGO_HAMMING_1BIT |
                                          NAND_ECC_ALGO_BCH_4BIT |
                                          NAND_ECC_ALGO_BCH_8BIT |
                                          NAND_ECC_ALGO_BCH_16BIT );

    hNandCtrlInfo->waitPin                  = GPMC_WAIT_PIN0;
    hNandCtrlInfo->waitPinPol               = GPMC_WAIT_PIN_POLARITY_LOW;
    hNandCtrlInfo->wpPinPol                 = GPMC_WP_PIN_LEVEL_HIGH;
    hNandCtrlInfo->chipSelectBaseAddr[0]    = NAND_CS0_BASEADDR;
    hNandCtrlInfo->chipSelectRegionSize[0]  = NAND_CS0_REGIONSIZE;
    NANDTimingInfoInit(hNandCtrlInfo->hNandTimingInfo);


    /* Init the NAND Ecc Info */
    hNandEccInfo->baseAddr                  = SOC_ELM_0_REGS;
    hNandEccInfo->ECCInit                   = GPMCNANDECCInit;
    hNandEccInfo->ECCEnable                 = GPMCNANDECCEnable;
    hNandEccInfo->ECCDisable                = GPMCNANDECCDisable;
    hNandEccInfo->ECCWriteSet               = GPMCNANDECCWriteSet;
    hNandEccInfo->ECCReadSet                = GPMCNANDECCReadSet;
    hNandEccInfo->ECCCalculate              = GPMCNANDECCCalculate;
    hNandEccInfo->ECCCheckAndCorrect        = GPMCNANDECCCheckAndCorrect;

    /* Init the NAND DMA info */
    hNandDmaInfo->DMAXfer                   = GPMCNANDDMAXfer;
    hNandDmaInfo->DMAInit                   = GPMCNANDEdmaInit;
    hNandDmaInfo->DMAXferSetup              = GPMCNANDXferSetup;
    hNandDmaInfo->DMAXferStatusGet          = GPMCNANDXferStatusGet;
}


/******************************************************************************
**                       GLOBAL FUNCTION DEFINITIONS
*******************************************************************************/

/******************************************************************************
*                                                                             *
* \brief  Main Function.\n                                                    *
*                                                                             *
******************************************************************************/
int main(void)
{
    int blkNum;
    int pageNum;
    int numOfPages;
    unsigned int retVal;
    unsigned int eraseBlkFlg;

    /* NAND structure allocations for this application */
    NandInfo_t              nandInfo;
    NandCtrlInfo_t          nandCtrlInfo;
    NandEccInfo_t           nandEccInfo;
    NandDmaInfo_t           nandDmaInfo;
    GPMCNANDTimingInfo_t    nandTimingInfo;


    /* Initializing the UART instance for serial communication.  */
    UARTStdioInit();

    UARTPuts("\r\n ************* StarterWare NAND Application ************"
             "\r\n\r\n", -1);

    /* Pin mux and clock setting */
    NANDPinMuxSetup();
    GPMCClkConfig();
    EDMAModuleClkConfig();
             
    /* Initialize the nandInfo struct */
    nandCtrlInfo.hNandTimingInfo = (void *) &nandTimingInfo;    
    nandInfo.hNandCtrlInfo = &nandCtrlInfo;
    nandInfo.hNandEccInfo = &nandEccInfo;
    nandInfo.hNandDmaInfo = &nandDmaInfo;
    NANDInfoInit(&nandInfo, NAND_CHIP_SELECT);

    /* Open the NAND device */
    retVal = NANDOpen(&nandInfo);
    if (retVal & NAND_STATUS_FAILED)
    {
        UARTPuts("\r\n*** ERROR : NAND Open Failed... ",-1);
        while(1);    
    }
    else if (retVal & NAND_STATUS_WAITTIMEOUT)
    {
        UARTPuts("\r\n*** ERROR : Device Is Not Ready...!!!\r\n", -1);
        while(1);
    }
    else if (retVal & NAND_STATUS_NOT_FOUND)
    {
        UARTPuts("\r\n*** ERROR : DEVICE MAY NOT BE ACCESSABLE OR NOT PRESENT."
                 "\r\n", -1);
        while(1);
    }
    else if(nandInfo.devId != NAND_DEVICE_ID)
    {
        /* Check if detected ID matches supplied ID */
        UARTPuts("\r\n*** ERROR : INVALID DEVICE ID.", -1);
        while(1);
    }
    else
    {
        /* Print The Device ID info */
        NANDDeviceIdInfoPrint(&nandInfo);
    }
    
#ifdef NAND_DATAINTEGRITY_TEST_WITH_FIXED_ADDR
    /* Do read/write for predefined address */
    pageNum = NAND_DEFAULT_START_PAGE;
    blkNum = NAND_DEFAULT_BLK;
    numOfPages = NAND_DEFAULT_NMBR_OF_PAGES;   
#else
    /* Take the read/write address from the user */
    UARTPuts("\r\n Please Enter The Block Number(0 - ", -1);
    UARTPutNum((unsigned int)(NAND_NUMOF_BLK - 1));
    UARTPuts(")\r\n", -1);
    blkNum = UARTGetNum();
    UARTPuts("\r\n Please Enter The Page Start Page Number(0 - ", -1);
    UARTPutNum((unsigned int)(nandInfo.pagesPerBlk - 1));
    UARTPuts(")\r\n", -1);
    pageNum = UARTGetNum();
    UARTPuts("\r\n Please Enter The Number Of Pages To Write\r\n", -1);
    numOfPages = UARTGetNum();
#endif
    eraseBlkFlg = 1;
    
    if( (pageNum < 0 ) || (pageNum > (nandInfo.pagesPerBlk - 1))
        || (blkNum < 0 || blkNum  > (NAND_NUMOF_BLK - 1)) || (numOfPages <= 0) )
    {
        UARTPuts("\r\n *** ERROR : Wrong Input(s) Entered...!!!\r\n", -1);
        while(1);
    }
    else if( ( blkNum * (nandInfo.pagesPerBlk )
              + pageNum + numOfPages ) >
              ( NAND_NUMOF_BLK * nandInfo.pagesPerBlk))
    {
        UARTPuts("\r\n *** ERROR : Requsted Page(s) For Read/Write Does Not"
                 " Exist...!!!\r\n", -1);
        while(1);
    }    
   
    while( numOfPages > 0 )
    {
        if( eraseBlkFlg )
        {
            retVal = NANDBadBlockCheck(&nandInfo, blkNum);
            if(retVal == NAND_BLOCK_BAD)
            {
                UARTPuts("\r\n Block Is Bad, Can't Continue ...!!! ",-1);
                while(1);
            }
            if(retVal == NAND_BLOCK_SPARE_AREA_READ_FAILED)
            {
                UARTPuts("\r\n Spare Area Read Failed While Checking ", -1);
                UARTPuts(" For Bad Block ", -1);
                while(1);
            }

            UARTPuts("\r\n Erasing The Block ", -1);
            UARTPutNum((unsigned int)blkNum);
            UARTPuts("                         :", -1);

            retVal = NANDBlockErase(&nandInfo, blkNum);
            if( retVal == NAND_STATUS_PASSED )
            {
                UARTPuts(" Succeeded.",-1);
            }
            else
            {
                UARTPuts(" Failed.", -1);
                UARTPuts("\r\n Marking The Block As Bad.\r\n Read/Write Test", -1);
                UARTPuts(" Will Be Continued On Next block. ", -1);
                NANDMarkBlockAsBad(&nandInfo, blkNum);
                blkNum++;
                pageNum = 0;
                eraseBlkFlg = 1;
                continue;
            }
            eraseBlkFlg = 0;
        }

        NANDBuffersInit();
        UARTPuts("\r\n Writing To Page ", -1);
        UARTPutNum((unsigned int)pageNum);
        UARTPuts(" Of Block ", -1);
        UARTPutNum((unsigned int)blkNum);
        UARTPuts("                :", -1);

        retVal = NANDPageWrite(&nandInfo, blkNum, pageNum, &txData[0],
                               &eccData[0]);
        if( (retVal & NAND_STATUS_WAITTIMEOUT) )
        {
            UARTPuts(" Failed.(Device Is Busy).", -1);
            while(1);
        }
        else if( (retVal & NAND_STATUS_DEVWRPROTECT) )
        {
            UARTPuts(" Failed.(Device Is Write Protected).", -1);
            while(1);
        }
        else if( (retVal & NAND_STATUS_READWRITE_DMA_FAIL) )
        {
            UARTPuts(" Failed.(EDMA Transfer Failed.).", -1);
            while(1);
        }
        else
        {
            UARTPuts(" Succeeded.", -1);
        }

        /* As eccData, is filled by NANDPageWrite fun, reinit the same */
        NANDBuffersInit();

        UARTPuts("\r\n Reading From Page ", -1);
        UARTPutNum((unsigned int)pageNum);
        UARTPuts(" Of Block ", -1);
        UARTPutNum((unsigned int)blkNum);
        UARTPuts("              :", -1);
        retVal= NANDPageRead(&nandInfo, blkNum, pageNum, &rxData[0],
                             &eccData[0]);

        if( (retVal & NAND_STATUS_READ_ECC_ERROR_CORRECTED) )
        {
            UARTPuts(" Succeeded With ECC Errors And Corrected.", -1);
        }
        else if( (retVal & NAND_STATUS_READ_ECC_UNCORRECTABLE_ERROR) )
        {
            UARTPuts(" Failed.(Uncorrectable ECC errors) ", -1);
            while(1);
        }
        else if( (retVal & NAND_STATUS_READWRITE_DMA_FAIL) )
        {
            UARTPuts(" Failed.(EDMA Transfer Failed.)", -1);
            while(1);
        }
        else
        {
            UARTPuts(" Succeeded.",-1);
        }

        UARTPuts("\r\n NAND Data Integrity Test                    :", -1);
        retVal = NANDDataIntegrityCheck();
        if(retVal == NAND_DATA_INTEGRITY_PASS)
        {
            UARTPuts(" Passed\r\n", -1);
        }
        else
        {
            UARTPuts(" Failed....!!!\r\n", -1);
        }

        pageNum++;
        numOfPages--;
        if( pageNum == ((nandInfo.pagesPerBlk) ) )
        {
            pageNum = 0;
            eraseBlkFlg = 1;
            blkNum++;
        }
    }

    UARTPuts("\r\n ****************************************************** ", -1);
    while(1);    
}


/******************************************************************************
**                              END OF FILE
*******************************************************************************/
