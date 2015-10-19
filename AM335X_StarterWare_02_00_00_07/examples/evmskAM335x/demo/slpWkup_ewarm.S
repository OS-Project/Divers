;******************************************************************************
;
; slpWkup_ewarm.S - This file contains sub-routines to save the processor
; context before a deep sleep mode is invoked and also to restore the context
; after waking up.
;
;******************************************************************************
;
; Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
;
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;
;    Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
;
;    Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the
;    distribution.
;
;    Neither the name of Texas Instruments Incorporated nor the names of
;    its contributors may be used to endorse or promote products derived
;    from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;******************************************************************************

#include "pm33xx.h"

    PUBLIC saveRestoreContext
    PUBLIC romRestoreLocation
    EXTERN CacheDataCleanInvalidateAll
    EXTERN CacheDisable

MODE_SYS DEFINE 0x1F
MODE_FIQ DEFINE 0x11
MODE_IRQ DEFINE 0x12
MODE_SVC DEFINE 0x13
MODE_ABT DEFINE 0x17
MODE_UND DEFINE 0x1B
MODE_SEC DEFINE 0x16
I_F_BIT DEFINE 0xC0
NUM_ARM_MODES DEFINE 7
CACHE_DCACHE DEFINE 2

    SECTION IRAM_CODE:CODE:NOROOT(2)

;**************************** Code section ************************************
    ; This code is assembled for ARM instructions
    CODE32

; This function saves/restores the context
saveRestoreContext:
        LDR     r1, _slpmode_                      ; Save the sleep Mode
        STR     r0, [r1, #0]

        MRS     r3, cpsr
        STMFD   sp!, {r3 - r12, lr}               ; Save the Registers

        ; Save required CP15 registers. If more are used, can be saved here
        MRC     p15, #0, r0, c12, c0, #0            ; Vector base register
        MRC     p15, #0, r1, c1, c0, #0             ; Control register
        MRC     p15, #0, r2, c1, c0, #1             ; Aux control register
        MRC     p15, #0, r3, c2, c0, #0             ; TTB0 register
        MRC     p15, #0, r4, c3, c0, #0            ; Domain Access Control  
        STMFD   sp!, {r0 - r4}

        LDR     r1, _slpmode_                      ; Get the sleep mode
        LDR     r5, [r1, #0]
        CMP     r5, #0x02                          ; Check if DS1 Deviation
        BEQ     WaitForInterrupt

        LDR     r0, _cnxtstack_                   ; Save the stack pointer
        STR     sp, [r0], #4                      ; for the current mode    

        MSR     cpsr_c, #MODE_FIQ|I_F_BIT
        STR     sp, [r0], #4                    

        MSR     cpsr_c, #MODE_SVC|I_F_BIT
        STR     sp, [r0], #4                    

        MSR     cpsr_c, #MODE_ABT|I_F_BIT
        STR     sp, [r0], #4                    

        MSR     cpsr_c, #MODE_IRQ|I_F_BIT
        STR     sp, [r0], #4                    

        MSR     cpsr_c, #MODE_UND|I_F_BIT
        STR     sp, [r0], #4                    

        MSR     cpsr_c, #MODE_SYS|I_F_BIT
        STR     sp, [r0], #4  

        ; Make sure that the Cache is Cleaned before putting DDR
        ; to self-refresh.
        BL      CacheDataCleanInvalidateAll
        
        MOV     r0, #CACHE_DCACHE
        BL      CacheDisable
        
        ; EMIF CONTEXT SAVE
        LDR   r0, _emifcontext_
        LDR   r1, _EMIF_BASE

        MOV   r2, #0
emif_cont_save:
        LDR   r3, [r1, r2]
        STR   r3, [r0, r2]
        ADDS  r2, r2 ,#4

        CMP   r2, #240
        BNE   emif_cont_save
        ; EMIF Context Save End        

		;for DDR3, hold DDR_RESET high via control module
		LDR		r2, _DDRIO_CTRL
		LDR		r1,[r2]
		mov		r3,#1
		mov		r3,r3,lsl #31
		ORR		r1,r1,r3	;set ddr3_rst_def_val
		STR		r1,[r2]

        ; Putting DDR into self refresh mode
        LDR   r1, _EMIF_PMCTL
        LDR   r2, [r1]
        ORR   r2, r2, #0x20		;after 32 clks of idle, go to SR
        STR   r2, [r1, #0]
        STR   r2, [r1, #4]

        LDR   r2, _DDR_START
        LDR   r3, [r2, #0]			;read from DDR to make PMCTL write take affect
        LDR   r3, [r1, #0]			;put pm_ctrl reg in r3
        ORR	  r3,r3,#0x200			;set pm_ctrl to self refresh
        STR   r3, [r1, #0]
        STR   r3, [r1, #4]			;shdw register

        MOV     r0, #0x1000
wait_sr:							;wait for SR to complete
        SUBS    r0, r0 ,#1
        BNE     wait_sr

	;Control DDR_CKE via control module
		LDR		r2, _ddr_cke_ctrl_addr
		MOV		r1,#0
		STR		r1,[r2]

        ;Disable the VTT regulator
        LDR   r0, _GPIO_PRCM
        LDR   r2, [r0, #0x08]
        ORR   r2, r2, #0x02
        STR   r2, [r0, #0x08]

        LDR   r0, _GPIO_PRCM
        LDR   r2, [r0, #0x08]
        ORR   r2, r2, #(0x1<<18)
        STR   r2, [r0, #0x08]

        LDR   r0, _GPIO_ADDR
        LDR   r2, [r0, #0x190]
        ORR   r2, r2, #(0x1 << 7)
        STR   r2, [r0, #0x190]

        LDR   r0, _GPIO_PRCM
        LDR   r2, [r0, #0x08]
        MOV   r2, #0x0
        STR   r2, [r0, #0x08]		


		; Weak pull down for macro DATA0
		ldr	r1, ddr_data0_ioctrl
		ldr	r2, susp_io_pull_data
		str	r2, [r1]

		; Weak pull down for macro DATA1
		ldr	r1, ddr_data1_ioctrl
		ldr	r2, susp_io_pull_data
		str	r2, [r1]

		; Weak pull down for macro CMD0
		ldr	r1, ddr_cmd0_ioctrl
		ldr	r2, susp_io_pull_cmd01
		str	r2, [r1]

		; Weak pull down for macro CMD1
		ldr	r1, ddr_cmd1_ioctrl
		ldr	r2, susp_io_pull_cmd01
		str	r2, [r1]

		; Weak pull down for macro CMD2 (exception: keep DDR_RESET pullup)
		ldr	r1, ddr_cmd2_ioctrl
		ldr	r2, susp_io_pull_cmd2
		str	r2, [r1]

		;/* Disable VTP*/
		ldr	r1, ddr_vtp_ctrl            
		ldr	r2, susp_vtp_ctrl_val
		str	r2, [r1]

		;/* put IO in mDDR (CMOS) mode */
		ldr	r0, _DDRIO_CTRL
		ldr	r1, [r0]
		mov	r2, #(0x1<<28)
		orr	r3, r2, r1
		str	r3, [r0]

		; Disable EMIF clk
		LDR   r0, _EMIF_CLKCTL
		ldr	  r2, [r0]
		bic	  r2, r2, #(3 << 0)
		str	  r2, [r0]

wait_emif_disable:
		ldr	r2, [r0]
		ldr	r3, module_disabled_val
		cmp	r2, r3
		bne	wait_emif_disable
		
		;/* Enable SRAM LDO ret mode */
		ldr	r0, phys_sram_ldo_addr;
		ldr	r1, [r0]
		orr	r1, r1, #1
		str	r1, [r0]

	; Put PLL's in LP bypass Mode
        ; Configure DISP PLL in Bypass mode
        LDR    r0, _SOC_CM_WKUP_REGS
        LDR    r1, [r0, #0x98]
        BIC    r1, r1, #0x7		
        ORR    r1, r1, #0x400	
	ORR    r1, r1, #0x5
	STR    r1, [r0, #0x98]

	; Configure MPU PLL in Bypass mode
        LDR    r0, _SOC_CM_WKUP_REGS
        LDR    r1, [r0, #0x88]
        BIC    r1, r1, #0x7		
        ORR    r1, r1, #0x400	
        ORR    r1, r1, #0x5
	STR    r1, [r0, #0x88]
		
	; Configure PER PLL in Bypass mode
        LDR    r0, _SOC_CM_WKUP_REGS
        LDR    r1, [r0, #0x8C]
        BIC    r1, r1, #0x7		
        ORR    r1, r1, #0x400	
	ORR    r1, r1, #0x5
	STR    r1, [r0, #0x8C]

	; Configure DDR PLL in Bypass mode
        LDR    r0, _SOC_CM_WKUP_REGS
        LDR    r1, [r0, #0x94]
        BIC    r1, r1, #0x7		
        ORR    r1, r1, #0x400	
        ORR    r1, r1, #0x5
	STR    r1, [r0, #0x94]		

	; Configure CORE PLL in Bypass mode
        LDR    r0, _SOC_CM_WKUP_REGS
        LDR    r1, [r0, #0x90]
        BIC    r1, r1, #0x7		
        ORR    r1, r1, #0x400	
	ORR    r1, r1, #0x5
	STR    r1, [r0, #0x90]
		
loop_disp_bypass:
        LDR    r1, [r0, #0x48]
        ANDS   r1, r1, #0x1
        BNE    loop_disp_bypass
		
loop_mpu_bypass:
        LDR    r1, [r0, #0x20]
        ANDS   r1, r1, #0x1
        BNE    loop_mpu_bypass
		
loop_per_bypass:
        LDR    r1, [r0, #0x70]
        ANDS   r1, r1, #0x1
        BNE    loop_per_bypass
		
loop_ddr_bypass:
        LDR    r1, [r0, #0x34]
        ANDS   r1, r1, #0x1
        BNE    loop_ddr_bypass
		
loop_core_bypass:
        LDR    r1, [r0, #0x5C]
        ANDS   r1, r1, #0x1
        BNE    loop_core_bypass
 	
WaitForInterrupt:
        DSB
  
        WFI                                       ; Wait for interrupt
        
romRestoreLocation:
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

		
        ; Configure DDR PLL in Locked mode
        LDR     r0, _SOC_CM_WKUP_REGS

        ; CORE PLL Relock
        LDR     r2, [r0, #0x90]
	BIC     r2, r2, #0x7	
	BIC     r2, r2, #0x400	
        ORR     r2, r2, #0x07
        STR     r2, [r0, #0x90]		

        ; DDR PLL Relock
        LDR     r2, [r0, #0x94]
	BIC     r2, r2, #0x7	
	BIC     r2, r2, #0x400		
        ORR     r2, r2, #0x07
        STR     r2, [r0, #0x94]
		
	; CORE PER Relock
        LDR     r2, [r0, #0x8C]
	BIC     r2, r2, #0x7	
	BIC     r2, r2, #0x400	
        ORR     r2, r2, #0x07
        STR     r2, [r0, #0x8C]		
		
	; CORE MPU Relock
        LDR     r2, [r0, #0x88]
	BIC     r2, r2, #0x7	
	BIC     r2, r2, #0x400	
        ORR     r2, r2, #0x07
        STR     r2, [r0, #0x88]		        
		
	; CORE DISP Relock
        LDR     r2, [r0, #0x98]
	BIC     r2, r2, #0x7	
	BIC     r2, r2, #0x400	
        ORR     r2, r2, #0x07
        STR     r2, [r0, #0x98]		
		
loop_core_pll_relock:
        LDR     r1, [r0, #0x5C]
        ANDS    r1, r1, #0x01
        BEQ     loop_core_pll_relock

loop_ddr_pll_relock:
        LDR     r1, [r0, #0x34]
        ANDS    r1, r1, #0x01
        BEQ     loop_ddr_pll_relock
		
loop_ddr_per_relock:
        LDR     r1, [r0, #0x70]
        ANDS    r1, r1, #0x01
        BEQ     loop_ddr_per_relock

loop_mpu_pll_relock:
        LDR     r1, [r0, #0x20]
        ANDS    r1, r1, #0x01
        BEQ     loop_mpu_pll_relock

loop_disp_pll_relock:
        LDR     r1, [r0, #0x48]
        ANDS    r1, r1, #0x01
        BEQ     loop_disp_pll_relock

        LDR     r1, _slpmode_                      ; Get the sleep mode
        LDR     r5, [r1, #0]
        CMP     r5, #0x02                          ; Check if DS1_Dev 
        BEQ     CoreContextRestore

		; /* Disable SRAM LDO ret mode */
		ldr	r0, phys_sram_ldo_addr;
		ldr	r1, [r0]
		bic	r1, r1, #1
		str	r1, [r0]

		; Enable EMIF
		LDR   r0, _EMIF_CLKCTL
		ldr	  r2, [r0]
		orr	  r2, r2, #2
		str	  r2, [r0]

wait_emif_enable:
		ldr	r2, [r0]
		ldr	r3, module_enabled_val
		cmp	r2, r3
		bne	wait_emif_enable

        ;/* Take out IO of mDDR mode */
		ldr	r0, _DDRIO_CTRL
		ldr	r1, [r0]
		bic	r1, r1, #28
		str	r1, [r0]

config_vtp:
		;/* enable VTP*/
		ldr	r0, ddr_vtp_ctrl
		ldr	r1, resume_vtp_ctrl_val
		str	r1, [r0]

	ldr	r0, vtp0_addr
	ldr	r1, [r0]
	mov	r2, #0x0	; clear the register
	str	r2, [r0]
	mov	r2, #0x6	; write the filter value
	str	r2, [r0]

	ldr	r1, [r0]
	ldr	r2, vtp_enable	; set the enable bit
	orr	r2, r2, r1
	str	r2, [r0]

	ldr	r1, [r0]	; toggle the CLRZ bit
	bic	r1, r1, #1
	str	r1, [r0]

	ldr	r1, [r0]
	orr	r1, r1, #1
	str	r1, [r0]

poll_vtp_ready:
	ldr	r1, [r0]	; poll for VTP ready
	tst	r1, #(1 << 5)
	beq	poll_vtp_ready

		;Disable the pull for DATA0
		ldr	r1, ddr_data0_ioctrl
		ldr	r2, resume_io_pull_data
		str	r2, [r1]

		;Disable the pull for DATA1
		ldr	r1, ddr_data1_ioctrl
		ldr	r2, resume_io_pull_data
		str	r2, [r1]

		;Disable the pull for CMD0
		ldr	r1, ddr_cmd0_ioctrl
		ldr	r2, resume_io_pull_cmd
		str	r2, [r1]

		;Disable the pull for CMD1
		ldr	r1, ddr_cmd1_ioctrl
		ldr	r2, resume_io_pull_cmd
		str	r2, [r1]

		;Disable the pull for CMD2
		ldr	r1, ddr_cmd2_ioctrl
		ldr	r2, resume_io_pull_cmd
		str	r2, [r1]

		MOV     r0, #0x100
wait_sdram_config3:
        SUBS    r0, r0 ,#1
        BNE     wait_sdram_config3


	;Enable VTT regulator	
        LDR   r0, _GPIO_PRCM
        LDR   r2, [r0, #0x08]
        ORR   r2, r2, #0x02
        STR   r2, [r0, #0x08]

        LDR   r0, _GPIO_PRCM
        LDR   r2, [r0, #0x08]
        ORR   r2, r2, #(0x1 << 18)
        STR   r2, [r0, #0x08]

        LDR   r0, _GPIO_ADDR
        LDR   r2, [r0, #0x194]
        ORR   r2, r2, #(0x1 << 7)
        STR   r2, [r0, #0x194]	

        ; EMIF Context Restore
        LDR    r0, _emifcontext_
        LDR    r1, _EMIF_BASE
        MOV    r2, #0
        
        LDR    r3,[r0,r2]
        STR    r3,[r1, r2]
        ADDS   r2, r2 ,#4
        
        LDR    r3,[r0,r2]
        STR    r3,[r1, r2]
        ADDS   r2, r2 ,#4
        
        ADDS   r2, r2 ,#4 ; skip SDRAM config register        
        
emif_cont_restore:
		LDR    r3,[r0,r2]
        STR    r3,[r1, r2]
        ADDS   r2, r2 ,#4

        CMP    r2, #240
        BNE    emif_cont_restore
	; EMIF context Restore End
		;probably don't need this delay
        MOV     r0, #0x1000
wait_sdram_config2:
        SUBS    r0, r0 ,#1
        BNE     wait_sdram_config2        

		;release control of DDR_CKE via control module
		LDR		r2, _ddr_cke_ctrl_addr
		MOV		r1,#1
		STR		r1,[r2]

		;release control of DDR_RESET via control module
		LDR		r2, _DDRIO_CTRL
		LDR		r1,[r2]
		BIC		r1,r1,#(0x1<<31)	;set ddr3_rst_def_val
		STR		r1,[r2]

CoreContextRestore:
        LDR     r0, _cnxtstack_

        MSR     cpsr_c, #MODE_FIQ|I_F_BIT
        LDR     sp, [r0, #4]

        MSR     cpsr_c, #MODE_SVC|I_F_BIT
        LDR     sp, [r0, #8]

        MSR     cpsr_c, #MODE_ABT|I_F_BIT
        LDR     sp, [r0, #12]

        MSR     cpsr_c, #MODE_IRQ|I_F_BIT
        LDR     sp, [r0, #16]

        MSR     cpsr_c, #MODE_UND|I_F_BIT
        LDR     sp, [r0, #20]

        MSR     cpsr_c, #MODE_SYS|I_F_BIT
        LDR     sp, [r0, #24]

        LDR     sp, [r0, #0]                      ; Load the stack pointer
                                                  ; for the mode before sleep
        ; Restore all the saved CPSR registers
        LDMFD   sp!, {r0 - r4}
        MCR     p15, #0, r0, c12, c0, #0          ; Vector base register
        MCR     p15, #0, r2, c1, c0, #1           ; Aux control register
        MCR     p15, #0, r3, c2, c0, #0           ; TTB0 register
        MCR     p15, #0, r4, c3, c0, #0           ; Domain Access Control
        MCR     p15, #0, r1, c1, c0, #0           ; Control register

        LDMFD   sp!, {r3 - r12, lr}
        MSR     cpsr_cf, r3                       ; Update CPSR

        DSB

        BX    lr

		
_slpmode_:
    DCD _slpmode
		
_cnxtstack_:
    DCD _cnxtstack
	
_emifcontext_:
    DCD _emifcontext	

_EMIF_PMCTL:
    DCD  0x4C000038
_DDR_START:
    DCD  0x80000000
_GPIO_ADDR:
    DCD  0x44E07000
_GPIO_PRCM:
    DCD  0x44E00400
_EMIF_CLKCTL:
    DCD  0x44E00028
_EMIF_BASE:
    DCD  0x4C000000	
_SOC_CM_WKUP_REGS:
    DCD  0x44E00400

_ddr_cke_ctrl_addr:
    DCD  0x44E1131C
	
_DDRIO_CTRL:
    DCD  0x44E10E04
_DDRPHY_CTRL:
    DCD  0x4C0000E4	
	
ddr_vtp_ctrl:
	DCD	0x44E10E0C
;for DDR2
;susp_vtp_ctrl_val:
;	DCD	0x10117
;for DDR3
susp_vtp_ctrl_val:
	DCD	0x0
resume_vtp_ctrl_val:
	DCD   0x47

module_disabled_val:
	DCD	0x30000	

module_enabled_val:
	DCD	0x0002

ddr_cmd0_ioctrl:
	DCD	0x44E11404
ddr_cmd1_ioctrl:
	DCD	0x44E11408
ddr_cmd2_ioctrl:
	DCD	0x44E1140C
ddr_data0_ioctrl:
	DCD	0x44E11440
ddr_data1_ioctrl:
	DCD	0x44E11444
susp_io_pull_data:
	DCD	0x3FF00003
susp_io_pull_cmd01:
	DCD   0xFFE0018B
susp_io_pull_cmd2:
	DCD   0xFFA0098B

resume_io_pull_data:
	DCD	0x18B
resume_io_pull_cmd:
	DCD   0x18B

;################################
; /* DDR related definitions */
vtp0_addr:
	DCD	VTP0_CTRL_REG
vtp_enable:
	DCD	VTP_CTRL_ENABLE
	
ddr_phy_base:
	DCD	DDR_PHY_BASE_ADDR
ddr2_ratio_val:
	DCD	DDR2_RATIO
data0_rd_dqs_slave_ratio0_val:
	DCD	DDR2_RD_DQS
data0_rd_dqs_slave_ratio1_val:
	DCD	DDR2_RD_DQS
data0_wr_dqs_slave_ratio0_val:
	DCD	DDR2_WR_DQS
data0_wr_dqs_slave_ratio1_val:
	DCD	DDR2_WR_DQS
data0_wr_lvl_init_ratio0_val:
	DCD	DDR2_PHY_WRLVL
data0_wr_lvl_init_ratio1_val:
	DCD	DDR2_PHY_WRLVL
data0_gate_lvl_init_ratio0_val:
	DCD	DDR2_PHY_GATELVL
data0_gate_lvl_init_ratio1_val:
	DCD	DDR2_PHY_GATELVL
data0_wr_lvl_slave_ratio0_val:
	DCD	DDR2_PHY_FIFO_WE
data0_wr_lvl_slave_ratio1_val:
	DCD	DDR2_PHY_FIFO_WE
data0_wr_data_slave_ratio0_val:
	DCD	DDR2_PHY_WR_DATA
data0_wr_data_slave_ratio1_val:
	DCD	DDR2_PHY_WR_DATA
data0_dll_lock_diff_val:
	DCD	PHY_DLL_LOCK_DIFF

data0_rank0_delay0_val:
	DCD	PHY_RANK0_DELAY
data1_rank0_delay1_val:
	DCD	PHY_RANK0_DELAY

control_base:
	DCD	AM33XX_CTRL_BASE
ddr_io_ctrl_addr:
	DCD	DDR_IO_CTRL
ddr_ioctrl_val:
	DCD	0x18B
ddr_cmd_offset:
	DCD	0x1404
ddr_data_offset:
	DCD	0x1440

ddr_cke_addr:
	DCD	DDR_CKE_CTRL
emif_rd_lat_val:
	DCD	EMIF_READ_LATENCY
emif_timing1_val:
	DCD	EMIF_TIM1
emif_timing2_val:
	DCD	EMIF_TIM2
emif_timing3_val:
	DCD	EMIF_TIM3
emif_sdcfg_val:
	DCD	EMIF_SDCFG
emif_sdcfg_2_val:
	DCD	EMIF_SDCFG_2	
emif_ref_ctrl_const_val:
	DCD	0x4650
emif_ref_ctrl_val:
	DCD	EMIF_SDREF	

phys_sram_ldo_addr:
	DCD	0x44E00F1C	


;******************************* Data segment *********************************
    SECTION IRAM_DATA:DATA:NOROOT(2)
_slpmode:
        DS32 (1)
_cnxtstack:
        DS32 NUM_ARM_MODES
_emifcontext:
        DS32 (60)

    END

