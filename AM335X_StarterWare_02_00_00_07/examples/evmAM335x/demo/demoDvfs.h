/**
 *  \file   demoDvfs.h
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


#ifndef _DVFS_H_
#define _DVFS_H_

/**************************************************************************
**                       Macro Definitions
**************************************************************************/

/* Types of Opp */
#define     OPP_50       1
#define     OPP_100      2
#define     OPP_120      3
#define     SR_TURBO     4

/* MACROS to Configure MPU divider */
#define     MPUPLL_N     23
#define     MPUPLL_M2    1

/* MACROS to configure SEL bit filed in VDD1_OP_REG  of PMIC*/
#define     PMIC_VOLT_SEL_095V      0x1f 
#define     PMIC_VOLT_SEL_11V       0x2b 
#define     PMIC_VOLT_SEL_12V       0x33 
#define     PMIC_VOLT_SEL_126V      0x38 


/******************************************************************************
**                      EXTERNAL VARIABLE DECLARATIONS
*******************************************************************************/
extern unsigned int mpuOpp;

/******************************************************************************
**                      EXTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
extern void DynamicVoltFreqChange(unsigned int Opp);
extern void VddVoltageConfigure(unsigned int Opp);
extern unsigned int FrequencyGet(unsigned int Opp);
extern void MPUConfigure(unsigned int freq);
extern unsigned int OppGet(void);
extern void ActionDVFSOpp50(void);
extern void ActionDVFSOpp100(void);
extern void ActionDVFSOpp120(void);
extern void ActionDVFSSrTurbo(void);

#endif
