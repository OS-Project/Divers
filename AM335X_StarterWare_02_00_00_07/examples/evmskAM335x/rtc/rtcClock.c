/**
 * \file   rtcClock.c
 * 
 * \brief  This is a RTC application file. This application receives the 
 *         time and calendar information from the user and sets those values 
 *         in the RTC registers. It periodically reads those registers and
 *         displays their current values on the serial communication console
 *         of the host machine.
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

#include "evmskAM335x.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "uartStdio.h"
#include "cache.h"
#include "rtc.h"
#include "mmu.h"

/*****************************************************************************
**                 INTERNAL MACRO DEFINITIONS
*****************************************************************************/

#define MASK_HOUR               (0xFF000000u)
#define MASK_MINUTE             (0x00FF0000u)
#define MASK_SECOND             (0x0000FF00u)
#define MASK_MERIDIEM           (0x000000FFu)

#define HOUR_SHIFT              (24u)
#define MINUTE_SHIFT            (16u)
#define SECOND_SHIFT            (8u)

#define MASK_DAY                (0xFF000000u)
#define MASK_MONTH              (0x00FF0000u)
#define MASK_YEAR               (0x0000FF00u)
#define MASK_DOTW               (0x000000FFu)

#define DAY_SHIFT               (24u)
#define MONTH_SHIFT             (16u)
#define YEAR_SHIFT              (8u)

#define RTC_INST_BASE           (SOC_RTC_0_REGS)
#define RTC_INT_NUM             (SYS_INT_RTCINT)

/* Definitions related to MMU Configuration. */
#define START_ADDR_DDR          (0x80000000u)
#define START_ADDR_DEV          (0x44000000u)
#define START_ADDR_OCMC         (0x40300000u)
#define NUM_SECTIONS_DDR        (512u)
#define NUM_SECTIONS_DEV        (960u)
#define NUM_SECTIONS_OCMC       (1u)

/******************************************************************************
**                 INTERNAL FUNCTION PROTOTYPES
******************************************************************************/

static void CalendarResolve(unsigned int calendarValue);
static unsigned char ASCIIToInt(unsigned char byte);
static unsigned int IntToASCII(unsigned char byte);
static void TimeResolve(unsigned int timeValue);
static unsigned int UserCalendarInfoGet(void);
static unsigned int UserTimeInfoGet(void);
static void MMUConfigAndEnable(void);
static void RTCINTCConfigure(void);
static void RTCIsr(void);

/******************************************************************************
**                 INTERNAL VARIABLE DEFINITIONS
******************************************************************************/

/* Page tables start must be aligned in 16K boundary */
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

/******************************************************************************
**                 INTERNAL FUNCTION DEFINITIONS
******************************************************************************/

/*
** This function will setup the MMU. The function maps three regions -
** 1. DDR
** 2. OCMC RAM
** 3. Device memory
** The function also enables the MMU.
*/
static void MMUConfigAndEnable(void)
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
** Main function.
*/
int main(void)
{
    unsigned int userCalendar = 0;
    unsigned int userTime = 0;

    /* Configure and enable the MMU. */
    MMUConfigAndEnable();

    /* Enable all levels of Cache. */
    CacheEnable(CACHE_ALL);
    
    /* Configuring the UART STDIO instance. */
    UARTStdioInit();

    /* Performing the System Clock configuration for RTC. */
    RTCModuleClkConfig();
    
    /* Disabling Write Protection for RTC registers.*/
    RTCWriteProtectDisable(RTC_INST_BASE);

    /* Selecting Internal Clock source for RTC. */
    RTC32KClkSourceSelect(RTC_INST_BASE, RTC_INTERNAL_CLK_SRC_SELECT);

    /* Enabling RTC to receive the Clock inputs. */
    RTC32KClkClockControl(RTC_INST_BASE, RTC_32KCLK_ENABLE);

    /* Enable the RTC module. */
    RTCEnable(RTC_INST_BASE);

    UARTPuts("StarterWare AM335x RTC Application.\r\n", -2);

    /* Receiving Time related information from the user. */
    userTime = UserTimeInfoGet();

    /* Receiving Calendar related information from the user. */
    userCalendar = UserCalendarInfoGet();
    
    /* Programming calendar information in the Calendar registers. */
    RTCCalendarSet(RTC_INST_BASE, userCalendar);

    /* Programming the time information in the Time registers. */
    RTCTimeSet(RTC_INST_BASE, userTime);
    
    /* Set the 32KHz counter to run. */
    RTCRun(RTC_INST_BASE);

    UARTPuts("\r\n\r\n", -2);
    UARTPuts("Current Time And Date:\r\n", -1);

    /* Enabling IRQ in CPSR of ARM processor. */
    IntMasterIRQEnable();

    /* Configure the AINTC to receive RTC interrupts. */
    RTCINTCConfigure();

    /* Enabling RTC interrupts. Configuring RTC to interrupt every second.*/
    RTCIntTimerEnable(RTC_INST_BASE, RTC_INT_EVERY_SECOND);

    while(1);
}

/*
** This function receives time related information from the user.
*/
static unsigned int UserTimeInfoGet()
{
    unsigned char minute[2] = {0};
    unsigned char second[2] = {0};
    unsigned char hour[2] = {0};
    unsigned int hourTime = 0;
    unsigned int minTime = 0;
    unsigned int secTime = 0;
    unsigned int lIndex = 0;
    unsigned int time = 0;

    UARTPuts("\nEnter the time in 24 hour format.\r\n", -1);
    UARTPuts("Example (hh:mm:ss) 20:15:09\r\n", -1);

    UARTPuts("\r\nEnter Hours: \r\n", -2);

    /*
    ** Collecting the 'Hour' information. The values collected shall be in
    ** the ASCII form of the numbers entered.
    */
    do
    {
        hour[lIndex] = UARTGetc();
        UARTPutc(hour[lIndex]);
        lIndex++;
    }while((lIndex < 2) && (hour[lIndex - 1] != '\r'));

    UARTPuts("\r\nEnter Minutes:\r\n", -2);
    lIndex = 0;

    /*
    ** Collecting the 'Minute' information. The values collected shall be in
    ** the ASCII form of the numbers entered.
    */
    do
    {
        minute[lIndex] = UARTGetc();
        UARTPutc(minute[lIndex]);
        lIndex++;
    }while((lIndex < 2) && (minute[lIndex - 1] != '\r'));

    UARTPuts("\r\nEnter Seconds:\r\n", -1);
    lIndex = 0;

    /*
    ** Collecting the 'Second' information. The values collected shall be in
    ** the ASCII form of the numbers entered.
    */
    do
    {
        second[lIndex] = UARTGetc();
        UARTPutc(second[lIndex]);
        lIndex++;
    }while((lIndex < 2) && (second[lIndex - 1] != '\r'));

    /* Converting the ASCII value of 'Hours' to its equivalent decimal value. */
    if(hour[0] != '\r')
    {
        hourTime = (ASCIIToInt(hour[0]) << 0x04);
        if(hour[1] != '\r')
        {
            hourTime |= ASCIIToInt(hour[1]);
        }
        else
        {
            hourTime = hourTime >> 0x04;
        }
    }

    /* Converting the ASCII value of 'Minutes' to its equivalent decimal value. */
    if(minute[0] != '\r')
    {
        minTime = (ASCIIToInt(minute[0]) << 0x04);
        if(minute[1] != '\r')
        {
            minTime |= ASCIIToInt(minute[1]);
        }
        else
        {
            minTime = minTime >> 0x04;
        }
    }

    /* Converting the ASCII value of 'Seconds' to its equivalent decimal value. */
    if(second[0] != '\r')
    {
        secTime = (ASCIIToInt(second[0]) << 0x04);
        if(second[1] != '\r')
        {
            secTime |= ASCIIToInt(second[1]);
        }
        else
        {
            secTime = secTime >> 0x04;
        }
    }

    /*
    ** Consolidating the decimal values of Hours, Minutes and Seconds to
    ** obtain the Time.
    */
    time = (hourTime << HOUR_SHIFT);
    time |= (minTime << MINUTE_SHIFT);
    time |= (secTime << SECOND_SHIFT);
 
    return time;
}

/*
** This function receives calendar related information from the user.
*/
static unsigned int UserCalendarInfoGet()
{
    unsigned char monthArr[2] = {0};
    unsigned char yearArr[2] = {0};
    unsigned char dotwArr[2] = {0};
    unsigned char dayArr[2] = {0};
    unsigned int calendar = 0;
    unsigned int lIndex = 0;
    unsigned int month = 0;
    unsigned int year = 0;
    unsigned int dotw = 0;
    unsigned int day = 0;

    UARTPuts("\r\n\r\nEnter the calendar information.\r\n", -2);
    UARTPuts("Example (DD:MM:YY) 31:03:73\r\n", -2);
    
    UARTPuts("\r\nEnter the day of the month: \r\n", -2);
    
    /*
    ** Collecting the 'Day of the Month' information. The values collected
    ** shall be in the ASCII form of the numbers entered.
    */
    do
    {
        dayArr[lIndex] = UARTGetc();
        UARTPutc(dayArr[lIndex]);
        lIndex++;
    }while((lIndex < 2) && (dayArr[lIndex - 1] != '\r'));

    UARTPuts("\r\nEnter the month (Jan=01, Dec=12):\r\n", -2);
    lIndex = 0;

    /*
    ** Collecting the 'Month' information. The values collected
    ** shall be in the ASCII form of the numbers entered.
    */
    do
    {
        monthArr[lIndex] = UARTGetc();
        UARTPutc(monthArr[lIndex]);
        lIndex++;        
    }while((lIndex < 2) && (monthArr[lIndex - 1] != '\r'));

    UARTPuts("\r\nEnter the year (Ex: 2010=10, 1987=87:):\r\n", -1);
    lIndex = 0;

    /*
    ** Collecting the 'Year' information. The values collected
    ** shall be in the ASCII form of the numbers entered.
    */
    do
    {
        yearArr[lIndex] = UARTGetc();
        UARTPutc(yearArr[lIndex]);
        lIndex++;
    }while((lIndex < 2) && (yearArr[lIndex - 1] != '\r'));

    UARTPuts("\r\nEnter the Day of the Week (Ex:Sun=00, Sat=06):\r\n", -3);
    lIndex = 0;

    /*
    ** Collecting the 'Day of the Week' information. The values collected
    ** shall be in the ASCII form of the numbers entered.
    */
    do
    {
        dotwArr[lIndex] = UARTGetc();
        UARTPutc(dotwArr[lIndex]);
        lIndex++;
    }while((lIndex < 2) && (dotwArr[lIndex - 1] != '\r'));

    /*
    ** Converting the ASCII value of 'Day of the Month' to its equivalent
    ** decimal value.
    */
    if(dayArr[0] != '\r')
    {
        day = (ASCIIToInt(dayArr[0]) << 0x04);
        if(dayArr[1] != '\r')
        {
            day |= ASCIIToInt(dayArr[1]);
        }
        else
        {
            day = day  >> 0x04;
        }
    }

    /* Converting the ASCII value of 'Month' to its equivalent decimal value. */
    if(monthArr[0] != '\r')
    {
        month = (ASCIIToInt(monthArr[0]) << 0x04);
        if(monthArr[1] != '\r')
        {
            month |= ASCIIToInt(monthArr[1]);
        }
        else
        {
            month = month >> 0x04;
        }
    }

    /* Converting the ASCII value of 'Year' to its equivalent decimal value. */
    if(yearArr[0] != '\r')
    {
        year = (ASCIIToInt(yearArr[0]) << 0x04);
        if(yearArr[1] != '\r')
        {
            year |= ASCIIToInt(yearArr[1]);
        }
        else
        {
            year = year >> 0x04;
        }
    }

    /*
    ** Converting the ASCII value of 'Day of the Week' to its equivalent
    ** decimal value.
    */
    if(dotwArr[0] != '\r')
    {
        dotw = (ASCIIToInt(dotwArr[0]) << 0x04);
        if(dotwArr[1] != '\r')
        {
            dotw |= ASCIIToInt(dotwArr[1]);
        }
        else
        {
            dotw = dotw >> 0x04;
        }
    }

    /*
    ** Consolidating the decimal values of Day of the Month, Month, Year
    ** and Day of the Week to obtain the Date.
    */
    calendar =  day << DAY_SHIFT;     
    calendar |= month << MONTH_SHIFT;
    calendar |= year << YEAR_SHIFT;
    calendar |= dotw;

    return calendar;
}  

/*
** This function prints the current time read from the RTC registers.
*/
static void TimeResolve(unsigned int timeValue)
{
    unsigned char timeArray[3] = {0};              
    unsigned char bytePrint[2] = {0};
    unsigned int asciiTime = 0;
    unsigned int lIndex = 0;
    unsigned int count = 0;
   
    /*
    ** The variable 'timeValue' will have the time in the format
    ** <hh:mm:ss:meridiem>. Now, obtaining the individual fields
    ** to facilitate their display.
    */
    timeArray[0] = (unsigned char)((timeValue & MASK_HOUR) >> HOUR_SHIFT);
    timeArray[1] = (unsigned char)((timeValue & MASK_MINUTE) >> MINUTE_SHIFT);
    timeArray[2] = (unsigned char)((timeValue & MASK_SECOND) >> SECOND_SHIFT);

    /* Prints the time in the format: <hh:mm:ss>. */
    while(count < 3)
    {
        lIndex = 0;
        asciiTime = IntToASCII(timeArray[count]);
        bytePrint[0] = (unsigned char)((asciiTime & 0x0000FF00) >> 0x08);
        bytePrint[1] = (unsigned char)(asciiTime & 0x000000FF);
        while(lIndex < 2)
        {    
            UARTPutc(bytePrint[lIndex]);
            lIndex++;
        }
        count++;
        if(count != 3)
        {
            UARTPutc(':');
        }
        else
        {
            UARTPutc(' ');
        }
    }
}

/*
** This function prints the calendar information read from the RTC registers.
*/
static void CalendarResolve(unsigned int calendarValue)
{
    unsigned char calendarArray[3] = {0};
    unsigned char dotwString[4] = {0};
    unsigned char bytePrint[2] = {0};
    unsigned int asciiCalendar = 0;
    unsigned int dotwValue = 0;
    unsigned int lIndex = 0;
    unsigned int count = 0;

    /*
    ** The variable 'calendarValue' will have the date in the format
    ** <dd:mm:yy:day of the week>. Now, obtaining the individual fields
    ** to facilitate their display.
    */
    calendarArray[0] = (unsigned char)((calendarValue & MASK_DAY) >> DAY_SHIFT);
    calendarArray[1] = (unsigned char)((calendarValue & MASK_MONTH) >> MONTH_SHIFT);
    calendarArray[2] = (unsigned char)((calendarValue & MASK_YEAR) >> YEAR_SHIFT);

    dotwValue = (calendarValue & MASK_DOTW);

    switch(dotwValue)
    {
        case 0x00: 
             dotwString[0] = 'S';
             dotwString[1] = 'u';
             dotwString[2] = 'n';
             dotwString[3] = '\0';
        break;

        case 0x01: 
             dotwString[0] = 'M';
             dotwString[1] = 'o';
             dotwString[2] = 'n';
             dotwString[3] = '\0';
        break;

        case 0x02: 
             dotwString[0] = 'T';
             dotwString[1] = 'u';
             dotwString[2] = 'e';
             dotwString[3] = '\0';
        break;


        case 0x03: 
             dotwString[0] = 'W';
             dotwString[1] = 'e';
             dotwString[2] = 'd';
             dotwString[3] = '\0';
        break;

        case 0x04: 
             dotwString[0] = 'T';
             dotwString[1] = 'h';
             dotwString[2] = 'u';
             dotwString[3] = '\0';
        break;

        case 0x05: 
             dotwString[0] = 'F';
             dotwString[1] = 'r';
             dotwString[2] = 'i';
             dotwString[3] = '\0';
        break;

        case 0x06: 
             dotwString[0] = 'S';
             dotwString[1] = 'a';
             dotwString[2] = 't';
             dotwString[3] = '\0';

        default: 
        break;
    }
    
    /* Prints the date in the format: <dd:mm:yy:day of the week>. */
    while(count < 3)
    {
        lIndex = 0;
        asciiCalendar = IntToASCII(calendarArray[count]);
        bytePrint[0] = (unsigned char)((asciiCalendar & 0x0000FF00) >> 0x08);
        bytePrint[1] = (unsigned char)(asciiCalendar & 0x000000FF);
        while(lIndex < 2)
        {
            UARTPutc(bytePrint[lIndex]);
            lIndex++;
        }
        count++;
        if(count != 3)
        {
            UARTPutc('-');
        }
        else
        {
            UARTPutc(' ');
        }
    }  
    UARTPuts((char *)dotwString, -2);
}

/*
** This function converts the nibbles of an 8-bit number to their ASCII
** equivalent value. The 8-bit number is passed as a parameter to this
** function.
** For Example, if a number 19 is given to this function, this function
** returns 0x3139. Here, 0x31 is the ASCII value of 1 and 0x39 is the
** ASCII value of 9.
*/
static unsigned int IntToASCII(unsigned char byte)
{
    unsigned char highNibble = 0;
    unsigned char lowNibble = 0;
    unsigned int retVal = 0;

    lowNibble = (byte & 0x0F);
    highNibble = (byte & 0xF0) >> 0x04;

    retVal = (lowNibble + 0x30);
    retVal |= ((highNibble + 0x30) << 0x08);

    return retVal;
}

/*
** This function converts the ASCII value of a hexadecimal number to its
** corresponding hexadecimal value.
*/
static unsigned char ASCIIToInt(unsigned char byte)
{
    unsigned char retVal = 0;

    /* For numbers from 0x0 to 0x9.*/
    if((byte >= 0x30) && (byte <= 0x39))
    {
        retVal = byte - 0x30;
    }
    /* For alphabets from A to F.*/
    else if((byte >= 0x41) && (byte <= 0x46))
    {
        retVal = byte - 0x37;
    }
    
    return retVal;
}

/*
** This function configures the Interrupt Controller(INTC) to receive
** RTC interrupts.
*/
static void RTCINTCConfigure(void)
{
    /* Initializing the ARM Interrupt Controller. */
    IntAINTCInit();

    /* Registering the Interrupt Service Routine(ISR). */
    IntRegister(RTC_INT_NUM, RTCIsr);

    /* Setting the priority for the system interrupt in AINTC. */
    IntPrioritySet(RTC_INT_NUM, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC. */
    IntSystemEnable(RTC_INT_NUM);    
}

/*
** This is the Interrupt Service Routine(ISR) for RTC.
*/
static void RTCIsr(void)
{
    unsigned int calendarValue = 0;    
    unsigned int timeValue = 0;
        
    /* Read the current time from RTC time registers. */ 
    timeValue = RTCTimeGet(RTC_INST_BASE);

    /* Decode the time in 'timeValue' and display it on console.*/
    TimeResolve(timeValue);
 
    /* Read the current date from the RTC calendar registers. */
    calendarValue = RTCCalendarGet(RTC_INST_BASE);
    
    UARTPuts("   ", -2);

    /* Decode  the date in 'calendarValue' and display it on console.*/
    CalendarResolve(calendarValue);

    UARTPuts("\r", -2);    
}

/****************************** End of file **********************************/
