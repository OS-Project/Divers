/**
 * \file   mcspiFlash.c
 *
 * \brief  This application code will write a data segment of 1 page(256 byte)
 *         size on to flash. It will also read the same data from flash and
 *         will verify whether the data written to and read from flash are same
 *         or not.
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

/* Include the necessary header files */
#include "soc_AM335x.h"
#include "evmAM335x.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "hw_types.h"
#include "mcspi.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS                           
******************************************************************************/
#define FLASH_WRITE_IN_PROGRESS          (0x01u)
#define FLASH_SECTOR_ADD_HIGH            (0x3Fu)
#define FLASH_SECTOR_ADD_LOW             (0x00u)
#define FLASH_SECTOR_ADD_MID             (0x00u)
#define FLASH_READ_STAT_REG1             (0x05u)
#define FLASH_WRITE_ENABLE               (0x06u)
#define FLASH_SECTOR_ERASE               (0x20u)
#define FLASH_PAGE_PROGRAM               (0x02u)
#define FLASH_DUMMY_BYTE                 (0xFFu)
#define FLASH_DATA_READ                  (0x03u)
#define MCSPI_OUT_FREQ                   (24000000u)
#define MCSPI_IN_CLK                     (48000000u)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES                         
******************************************************************************/
static unsigned int IsWriteSuccess(void);
static void McSPI0AintcConfigure(void);									
static void FlashStatusGet(void);
static void ReadFromFlash(void);
static void McSPITransfer(void);
static void WriteToFlash(void);
static void WriteEnable(void);
static void SectorErase(void);
static void IsFlashBusy(void);
static void McSPISetUp(void);
static void VerifyData(void);
static void McSPIIsr(void);

/******************************************************************************
**                      GLOBAL VARIABLE DEFINITIONS                          
******************************************************************************/
volatile unsigned int flag = 1;
unsigned char rxBuffer[260];
unsigned char vrfyData[256];
unsigned int length = 0;
unsigned int chNum = 0;
unsigned char *p_rx;
unsigned char txBuffer[260];
unsigned char *p_tx;

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS                       
******************************************************************************/
int main(void)
{
    volatile unsigned int count = 0x0FFFu;
    unsigned int retVal = FALSE;
    unsigned char choice = 0;

    /* Enable the clocks for McSPI0 module.*/
    McSPI0ModuleClkConfig();

    /* Perform Pin-Muxing for SPI0 Instance */
    McSPIPinMuxSetup(0);

    /* Perform Pin-Muxing for CS0 of SPI0 Instance */
    McSPI0CSPinMuxSetup(chNum);

    /* Initialize the UART utility functions */
    UARTStdioInit();

    UARTPuts("Here the McSPI controller on the SoC communicates with", -1);
    UARTPuts(" the SPI Flash.\r\n\r\n", -1);

    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Map McSPI Interrupts to AINTC */
    McSPI0AintcConfigure();
					
    /* Do the necessary set up configurations for McSPI.*/
    McSPISetUp();

    /* Pass the write enable command to flash.*/
    WriteEnable();

    /* Wait until write enable command is successfully written to flash.*/
    while(FALSE == retVal)
    {
        retVal = IsWriteSuccess();
    }

    retVal = FALSE;

    UARTPuts("Do you want to erase a sector of the flash before ", -1);
    UARTPuts("writing to it ?.", -1);
    UARTPuts("\r\nInput y(Y)/n(N) to proceed.\r\n", -1);

    choice = UARTGetc();
    UARTPutc(choice);

    if(('Y' == choice) || ('y' == choice))
    {
        /* Erase a sector of flash.*/
        SectorErase();
    }

    /* Pass the write enable command to flash.*/
    WriteEnable();

    /* Wait until write enable command is successfully written to flash.*/
    while(FALSE == retVal)
    {
        retVal = IsWriteSuccess();
    }

    /* Write data of 1 page size to flash.*/
    WriteToFlash();

    while(count--);
    count = 0x0FFFu;

    /* Read data of 1 page size from flash.*/
    ReadFromFlash();

    while(count--);

    /* Verify the data written to and read from flash are same or not.*/
    VerifyData();

    while(1);
}

/* Interrupt mapping to AINTC and registering McSPI ISR */
static void McSPI0AintcConfigure(void)		
{
    /* Initialze ARM interrupt controller */												
    IntAINTCInit();

    /* Register McSPIIsr interrupt handler */
    IntRegister(SYS_INT_SPI0INT, McSPIIsr);

    /* Set Interrupt Priority */
    IntPrioritySet(SYS_INT_SPI0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable system interrupt in AINTC */
    IntSystemEnable(SYS_INT_SPI0INT);
}

/*
** This function will write data of 1 page size on to a page of a specific 
** sector.
*/
static void WriteToFlash(void)
{
    unsigned int index = 0;

    txBuffer[0] = FLASH_PAGE_PROGRAM;
    txBuffer[1] = FLASH_SECTOR_ADD_HIGH;
    txBuffer[2] = FLASH_SECTOR_ADD_MID;
    txBuffer[3] = FLASH_SECTOR_ADD_LOW;

    for(index = 0; index < 256; index++)
    {
        txBuffer[index + 4] = (unsigned char) index;
        vrfyData[index] = (unsigned char) index;
    }

    length = 260;
    
    McSPITransfer();
}

/*
** This function will verify the data written to and read from flash and print
** the appropriate message.
*/
static void VerifyData(void)
{
    unsigned int index = 0;

    for(index = 4; index < 260; index++)
    {
        if(rxBuffer[index] != vrfyData[index - 4])
        {
            UARTPuts("\r\n", -1);
            UARTPuts("VerifyData: Comparing the data written to and read", -1);
            UARTPuts(" from Flash.\r\nThe two data blocks are unequal.", -1);
            UARTPuts(" Mismatch found at index ", -1);
            UARTPutNum((int)index);
            UARTPuts("\r\n", -1);

            break;
        }
    }

    if(260 == index)
    {
        UARTPuts("\r\nThe data in the Flash and the one written ", -1);
        UARTPuts("to it are equal.\r\n", -1);

    }
}

/*
** This function will read data of 1 page size from a specific sector of flash.
*/
static void ReadFromFlash(void)
{
    unsigned int index = 0;

    txBuffer[0] = FLASH_DATA_READ;
    txBuffer[1] = FLASH_SECTOR_ADD_HIGH;
    txBuffer[2] = FLASH_SECTOR_ADD_MID;
    txBuffer[3] = FLASH_SECTOR_ADD_LOW;

    for(index = 4; index < 260; index++)
    {
        txBuffer[index] = 0;
    }    

    length = 260;

    McSPITransfer();
}

/*
** This function will check whether the write enable command is successfully 
** latched on to flash or not. 
*/
static unsigned int IsWriteSuccess(void)
{
    unsigned int retVal = FALSE;

    txBuffer[0] = FLASH_READ_STAT_REG1; 
    txBuffer[1] = FLASH_DUMMY_BYTE;

    length = 2;

    McSPITransfer();

    if(0x02 == rxBuffer[1])
    {
        retVal = TRUE;
    }
    
    return retVal;
}

/*
** This function will send the write enable command to the flash device.
*/
static void WriteEnable(void)
{
    txBuffer[0] = FLASH_WRITE_ENABLE;

    length = 1;

    McSPITransfer();
}

/*
** This function will activate/deactivate CS line and also enable Tx and Rx
** interrupts of McSPI peripheral.
*/
static void McSPITransfer(void)
{
    p_tx = txBuffer;
    p_rx = rxBuffer;

    /* SPIEN line is forced to low state.*/
    McSPICSAssert(SOC_SPI_0_REGS, chNum);

    /* Enable the Tx/Rx interrupts of McSPI.*/
    McSPIIntEnable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum) | 
                   MCSPI_INT_RX_FULL(chNum));

    /* Enable the McSPI channel for communication.*/
    McSPIChannelEnable(SOC_SPI_0_REGS, chNum);

    /* Wait until control returns back from McSPI ISR.*/
    while(flag);

    flag = 1;

    /* Force SPIEN line to the inactive state.*/
    McSPICSDeAssert(SOC_SPI_0_REGS, chNum);

    /* Disable the McSPI channel.*/
    McSPIChannelDisable(SOC_SPI_0_REGS, chNum);
}

/*
** This function will erase a sector of flash.
*/
static void SectorErase(void)
{
    txBuffer[0] = FLASH_SECTOR_ERASE;
    txBuffer[1] = FLASH_SECTOR_ADD_HIGH;
    txBuffer[2] = FLASH_SECTOR_ADD_MID;
    txBuffer[3] = FLASH_SECTOR_ADD_LOW;

    length = 4;

    McSPITransfer();

    IsFlashBusy();
}

/*
** This function will send the flash status register bits on to the receive
** buffer. 
*/
static void FlashStatusGet(void)
{
    txBuffer[0] = FLASH_READ_STAT_REG1;
    txBuffer[1] = 0xFF;

    length = 2;

    McSPITransfer();

}

/*
** This function will check whether write enable command is still in progress
**  or not.
*/
static void IsFlashBusy(void)
{
    do{
        FlashStatusGet();
      } while(rxBuffer[1] & FLASH_WRITE_IN_PROGRESS);

}

/*
** This function will call the necessary McSPI APIs which will configure the 
** McSPI controller. 
*/
static void McSPISetUp(void)
{

    /* Reset the McSPI instance.*/
    McSPIReset(SOC_SPI_0_REGS);

    /* Enable chip select pin.*/
    McSPICSEnable(SOC_SPI_0_REGS);

    /* Enable master mode of operation.*/
    McSPIMasterModeEnable(SOC_SPI_0_REGS);

    /* Perform the necessary configuration for master mode.*/
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_SINGLE_CH, 
                          MCSPI_TX_RX_MODE, MCSPI_DATA_LINE_COMM_MODE_1,
                          chNum);

    /* Configure the McSPI bus clock depending on clock mode. */
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, chNum, 
                   MCSPI_CLK_MODE_0);

    /* Configure the word length.*/
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(8), chNum);

    /* Set polarity of SPIEN to low.*/
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, chNum);

    /* Enable the transmitter FIFO of McSPI peripheral.*/
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_ENABLE, chNum);

    /* Enable the receiver FIFO of McSPI peripheral.*/
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_ENABLE, chNum);
}

/*
** McSPI Interrupt Service Routine. This function will clear the status of the
** Tx/Rx interrupts when generated. Will write the Tx data on transmit data
** register and also will put the received data from receive data register to
** a location in memory.
*/
static void McSPIIsr(void)
{
    unsigned int intCode = 0;

    intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);

    while(intCode)
    {
        if(MCSPI_INT_TX_EMPTY(chNum) == (intCode & MCSPI_INT_TX_EMPTY(chNum)))
        {
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));

            length--;

            McSPITransmitData(SOC_SPI_0_REGS,(unsigned int)(*p_tx++), chNum);

            if(!length)
            {
                McSPIIntDisable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));

                McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(chNum));
            }
        }

        if(MCSPI_INT_RX_FULL(chNum) == (intCode & MCSPI_INT_RX_FULL(chNum)))
        {
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_RX_FULL(chNum));

            *p_rx++ = (unsigned char) McSPIReceiveData(SOC_SPI_0_REGS, chNum);

            if(!(length))
            {
                McSPIIntDisable(SOC_SPI_0_REGS, MCSPI_INT_RX_FULL(chNum));

                flag = 0;
            }
        }

        intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);
    }
}
/********************************* End Of File ******************************/
