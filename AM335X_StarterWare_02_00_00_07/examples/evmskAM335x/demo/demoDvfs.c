/**
 *  \file   demoDvfs.c
 *
 *  \brief  Functions used to change the opp at which core operates.
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

#include "demoDvfs.h"
#include "demoMain.h"
#include "soc_AM335x.h"
#include "hw_cm_wkup.h"
#include "hw_types.h"
#include "pmic.h"
#include "uartStdio.h"
#include "demoGrlib.h"
#include "demoCfg.h"

unsigned int mpuOpp;

/*
** This function changes the voltage and
** and the freuency at which system operates
*/
void DynamicVoltFreqChange(unsigned int Opp)
{
    unsigned int currentOpp;
    unsigned int freq;

    /* Determine the OPP at which system is operating */
    currentOpp = OppGet();

    if(Opp >= currentOpp)
    {
        freq = FrequencyGet(Opp);

        if(freq)
        {
             VddVoltageConfigure(Opp);
             MPUConfigure(freq);
        }
        else
        {
             ;
        }
    }        
    else
    {
        freq = FrequencyGet(Opp);

        if(freq)
        {
             MPUConfigure(freq);
             VddVoltageConfigure(Opp);
        }
        else
        {
             ;
        }
    }
}

/* 
**This function configures the vdd1 voltage
**for different OPPs.
*/
void VddVoltageConfigure(unsigned int Opp)
{

    switch(Opp)
    {
        case OPP_50:
               /*
               ** Set vdd1 voltage to 1.1V. The voltage is
               ** configure by writing to SEL(0:6) bitfiled
               ** of VDD1_OP_REG. Minimum voltage level for
               ** Cortex A8 power terminals is same as the 
               ** minimum voltage level for OPP100.
               ** Vout = (SEL(0:6) * 12.5mv + 562.5mv)
               */
               setVdd1OpVoltage(PMIC_VOLT_SEL_11V);
               break;

        case OPP_100:
               /* 
               **set vdd1 voltage to 1.1v .This
               **voltage is configure by writing 
               **0x2b to SEL(0:6) bitfiled of VDD1_OP_REG.
               **
               ** Vout = (SEL(0:6) * 12.5mv + 562.5mv)
               */
               setVdd1OpVoltage(PMIC_VOLT_SEL_11V);
               break;

        case OPP_120:
               /* 
               **set vdd1 voltage to 1.2v .This
               **voltage is configure by writing 
               **0x33 to SEL(0:6) bitfiled of VDD1_OP_REG.
               **
               ** Vout = (SEL(0:6) * 12.5mv + 562.5mv)
               */
               setVdd1OpVoltage(PMIC_VOLT_SEL_12V);
               break;

        case SR_TURBO:
               /* 
               **set vdd1 voltage to 1.26v .This
               **voltage is configure by writing 
               **0x38 to SEL(0:6) bitfiled of VDD1_OP_REG.
               **
               ** Vout = (SEL(0:6) * 12.5mv + 562.5mv)
               */
               setVdd1OpVoltage(PMIC_VOLT_SEL_126V);
               break;

        default:
               break;
    }
}

/* 
**At different OPPs system will run at different
**frequencies.This function determines at which
**frequency system should run for a particular
**OPP.
*/
unsigned int FrequencyGet(unsigned int Opp)
{
    unsigned int freq;

    switch(Opp)
    {
        case OPP_50:
                   freq = 275;
                   break;

        case OPP_100:
                    freq = 500;
                    break;

        case OPP_120:
                    freq = 600;
                    break;

        case SR_TURBO:
                    freq = 720;
                    break;

        default:
                freq = 0;
                break;
    }

    return freq;
}

/*
**Clock for cortex-A8 is fed from MPUPLL.
**This function configures the MPUPLL to
**run the cortex-A8 at desired frequency.
*/
void MPUConfigure(unsigned int freq)
{
    volatile unsigned int regVal = 0;

    /* Put the PLL in bypass mode */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) &
                ~CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN_DPLL_MN_BYP_MODE;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) = regVal;

    /* Wait for DPLL to go in to bypass mode */
    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU) & 
                CM_WKUP_CM_IDLEST_DPLL_MPU_ST_MN_BYPASS));

    /* Clear the MULT and DIV field of DPLL_MPU register */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU) &= 
                      ~(CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT | 
                        CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV);

    /* Set the multiplier and divider values for the PLL */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKSEL_DPLL_MPU) |=
         ((freq << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_MULT_SHIFT) |
         (MPUPLL_N << CM_WKUP_CM_CLKSEL_DPLL_MPU_DPLL_DIV_SHIFT));

    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU);

    regVal = regVal & ~CM_WKUP_CM_DIV_M2_DPLL_MPU_DPLL_CLKOUT_DIV;

    regVal = regVal | MPUPLL_M2;

    /* Set the CLKOUT2 divider */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_DIV_M2_DPLL_MPU) = regVal;

    /* Now LOCK the PLL by enabling it */
    regVal = HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) &
                ~CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    regVal |= CM_WKUP_CM_CLKMODE_DPLL_MPU_DPLL_EN;

    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_CLKMODE_DPLL_MPU) = regVal;

    while(!(HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_IDLEST_DPLL_MPU) & 
                             CM_WKUP_CM_IDLEST_DPLL_MPU_ST_DPLL_CLK));
}

/*
**This function determines OPP at
**which system is operating.
*/
unsigned int OppGet(void)
{
    unsigned char vdd1;
    unsigned int Opp;

    vdd1 = Vdd1OpVoltageGet();
    
    switch(vdd1)
    {
        case PMIC_VOLT_SEL_095V:
                                Opp = OPP_50;
                                break;

        case PMIC_VOLT_SEL_11V:
                               Opp = OPP_100;
                               break;

        case PMIC_VOLT_SEL_12V:
                               Opp = OPP_120;
                               break;

        case PMIC_VOLT_SEL_126V:
                                Opp = SR_TURBO;
                                break;

        default:
                Opp = OPP_100;
                break;
    }

    return Opp;
}

/*
** Steps to be taken when OPP50 is selected
*/
void ActionDVFSOpp50(void)
{
    DynamicVoltFreqChange(OPP_50);
  
    mpuOpp = OPP_50;

    UARTPuts("\r\nOPP50 selected (vdd_mpu at 0.95V, 275MHz) \r\n", -1);
    updatePage(CLICK_IDX_DVFS);
}

/*
** Steps to be taken when OPP100 is selected
*/
void ActionDVFSOpp100(void)
{
    DynamicVoltFreqChange(OPP_100);
    mpuOpp = OPP_100;

    UARTPuts("\r\nOPP100 selected (vdd_mpu at 1.10V, 500MHz) \r\n", -1);
    updatePage(CLICK_IDX_DVFS);
}

/*
** Steps to be taken when OPP120 is selected
*/
void ActionDVFSOpp120(void)
{
    DynamicVoltFreqChange(OPP_120);
    mpuOpp = OPP_120;

    UARTPuts("\r\nOPP120 selected (vdd_mpu at 1.20V, 600MHz) \r\n", -1);
    updatePage(CLICK_IDX_DVFS);
}

/*
** Steps to be taken when SR Turbo is selected
*/
void ActionDVFSSrTurbo(void)
{
    DynamicVoltFreqChange(SR_TURBO);
    mpuOpp = SR_TURBO;

    UARTPuts("\r\nSR TURBO selected (vdd_mpu at 1.26V, 720MHz) \r\n", -1);
    updatePage(CLICK_IDX_DVFS);
}

