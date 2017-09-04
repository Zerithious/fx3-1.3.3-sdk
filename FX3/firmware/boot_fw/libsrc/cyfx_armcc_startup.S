; Cypress FX3 2-Stage Bootloader Startup code (cyfx_armcc_startup.S)
; ## =====================================================
; ##
; ##  Copyright Cypress Semiconductor Corporation, 2010-2014,
; ##  All Rights Reserved
; ##  UNPUBLISHED, LICENSED SOFTWARE.
; ##
; ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
; ##  WHICH IS THE PROPERTY OF CYPRESS.
; ##
; ##  Use of this file is governed
; ##  by the license agreement included in the file
; ##
; ##     <install>/license/license.txt
; ##
; ##  where <install> is the Cypress software
; ##  installation root directory path.
; ##
; ## =======================================================

    PRESERVE8

ITCM_SETUP              EQU     0x00000015  ; ITCM_BASE = 0x00000000, size = 16K, enable
DTCM_SETUP              EQU     0x10000011  ; DTCM_BASE = 0x10000000, size = 8K Enable
VIC_ADDRESS             EQU     0xFFFFFF00  ; VIC ADDRESS register.
VIC_ADDRESS_OFFSET      EQU     (VIC_ADDRESS - 0x20)
ICACHE_EN_BIT           EQU     (1<<12)         ; Instruction Cache Enable Bit
INT_SELECT              EQU     0xFFFFF00C      ; INT_SELECT register, used to select FIQ
INT_CLEAR               EQU     0xFFFFF014      ; INT_CLEAR register, used to disable interrupts.
ITCM_BASE               EQU     0x00000000

;************************************************************************
; Stack size definitions
;************************************************************************
FX3_STACK_BASE      EQU 0x10000000 ; DTCM base address
SVC_STACK_LEN       EQU 0x0200     ; 512 
FIQ_STACK_LEN       EQU 0x0100     ; 256
IRQ_STACK_LEN       EQU 0x0400     ; 1 KB
ABT_STACK_LEN       EQU 0x0080     ; 128
UIM_STACK_LEN       EQU 0x0080     ; 128
SYS_STACK_LEN       EQU 0x1800     ; 6 KB

;The imported labels
    IMPORT  __main     ; Entry point to user code.
    
;The exported labels
    EXPORT  __user_initial_stackheap
    EXPORT  Reset_Handler
  
    CODE32
    AREA    RESET,CODE,READONLY
    ENTRY
;*******************************************************************
;interrupt vectors
;*******************************************************************
Reset_Handler
    LDR  PC, ResetVEC
    LDR  PC, UndefinedVEC
    LDR  PC, SWI_VEC
    LDR  PC, PrefetchVEC
    LDR  PC, DataAbortVEC
    LDR  PC, ReservedVEC
    LDR  PC, [PC, #VIC_ADDRESS_OFFSET]
    LDR  PC, FIQ_VEC

ResetVEC      DCD  ResetInit
UndefinedVEC  DCD  UndefinedISR
SWI_VEC       DCD  SwiISR
PrefetchVEC   DCD  PrefetchISR
DataAbortVEC  DCD  AbortISR
ReservedVEC   DCD  ReservedISR
FIQ_VEC       DCD  FiqISR

SwiISR
UndefinedISR
ReservedISR
PrefetchISR
AbortISR
FiqISR
    B   AbortISR

ResetInit
    ; Invalidate TLBs
    MOV  r0,  #0
    MCR  p15, 0, r0, c8, c7, 0   ; Invalidate both caches
    MCR  p15, 0, r0, c7, c7, 0
    MRC  p15, 0, r0, c1, c0, 0   ; Read Control Reg
    BIC  r0, r0, #1              ; Disable MMU 

    ; Relocate all exception vectors to address 0.
    BIC  r0, r0, #(1<<13)        

    ; Enable the Instruction cache if required.
    IF ICACHE_ON != 0
    ORR  r0, r0, #ICACHE_EN_BIT  ; Enable instruction cache
    ENDIF

    MCR  p15, 0, r0, c1, c0, 0   ; Write Control Reg

    ; Enable TCMs
    ldr  r0,=DTCM_SETUP
    MCR  p15, 0, r0, c9, c1, 0   ; Write DTCM
    ldr  r0,=ITCM_SETUP
    MCR  p15, 0, r0, c9, c1, 1   ; Write ITCM

SVC_MODE        EQU 0x13
SYS_MODE        EQU 0x1F
IRQ_MODE        EQU 0x12
FIQ_MODE        EQU 0x11
ABT_MODE        EQU 0x17
UIM_MODE        EQU 0x1B
I_Bit           EQU 0x80   ;when I bit is set, IRQ is disabled
F_Bit           EQU 0x40   ;when F bit is set, FIQ is disabled

; Initialize the stacks
    LDR  R1, =FX3_STACK_BASE    ; Stack base address = DTCM
    SUB  R1, R1, #8             ; Prevent overflow at the top of the DTCM

    MSR  CPSR_c, #SVC_MODE      ; Supervisor (SVC) mode stack
    LDR  R2, =SVC_STACK_LEN
    ADD  R1, R1, R2
    BIC  R1, R1, #7
    MOV  SP, R1

    MSR  CPSR_c, #IRQ_MODE      ; Interrupt (IRQ) mode stack
    LDR  R2, =IRQ_STACK_LEN
    ADD  R1, R1, R2
    BIC  R1, R1, #7
    MOV  SP, R1

    MSR  CPSR_c, #FIQ_MODE      ; Fast Interrupt (FIQ) mode stack
    LDR  R2, =FIQ_STACK_LEN
    ADD  R1, R1, R2
    BIC  R1, R1, #7
    MOV  SP, R1

    MSR  CPSR_c, #ABT_MODE:OR:I_Bit:OR:F_Bit
    LDR  R2, =ABT_STACK_LEN
    ADD  R1, R1, R2
    BIC  R1, R1, #7
    MOV  SP, R1

    MSR  CPSR_c, #UIM_MODE:OR:I_Bit:OR:F_Bit
    LDR  R2, =UIM_STACK_LEN
    ADD  R1, R1, R2
    BIC  R1, R1, #7
    MOV  SP, R1

    MSR  CPSR_c, #SYS_MODE      ; System (SYS) mode stack
    LDR  R2, =SYS_STACK_LEN
    ADD  R1, R1, R2
    BIC  R1, R1, #7
    MOV  SP, R1

    LDR     r8, =ITCM_BASE
    LDR     r9, =Reset_Handler
    LDMIA   r9!, {r0-r7}
    STMIA   r8!, {r0-r7}
    LDMIA   r9!, {r0-r7}
    STMIA   r8!, {r0-r7}

    LDMIA   r9!, {r0-r1}
    STMIA   r8!, {r0-r1}

    LDR     r0, =INT_SELECT
    MOV     r1, #0
    STR     r1, [r0]            ; Clear the INT_SELECT register to disable all FIQ selections.
    LDR     r0, =INT_CLEAR
    MOV     r1, #0xFFFFFFFF
    STR     r1, [r0]            ; Disable all interrupts on the device.

    B       __main              ; __main initializes the BSS and then jumps to the C main.

;************************************************************************
;*  Function : void user_initial_stackheap(void)                        *
;************************************************************************
__user_initial_stackheap    
    LDR r0,=heap_bottom
    BX  lr

;************************************************************************
;*  Function : void jump(uint32_t *address)                             *
;************************************************************************
    EXPORT jump
jump
    BX  R0

;************************************************************************
; Heap area (grow upwards from end of data)
;************************************************************************    
    AREA    Heap, DATA, NOINIT
heap_bottom SPACE   1

    END

;;[]
