;/*****************************************************************************/
;/* OSasm.s: low-level OS commands, written in assembly                       */
;/* derived from uCOS-II                                                      */
;/*****************************************************************************/
;Jonathan Valvano, OS Lab2/3/4/5, 1/12/20
;Students will implement these functions as part of EE445M/EE380L.12 Lab

        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt            ; currently running thread
		EXTERN sdload

        EXPORT  StartOS
        EXPORT  ContextSwitch
        EXPORT  PendSV_Handler
        EXPORT  SVC_Handler
		EXPORT	StartFromCheckpoint

NVIC_INT_CTRL   EQU     0xE000ED04                              ; Interrupt control state register.
NVIC_SYSPRI14   EQU     0xE000ED22                              ; PendSV priority register (position 14).
NVIC_SYSPRI15   EQU     0xE000ED23                              ; Systick priority register (position 15).
NVIC_LEVEL14    EQU           0xCF                              ; Systick priority value (second lowest).
NVIC_LEVEL15    EQU           0xFF                              ; PendSV priority value (lowest).
NVIC_PENDSVSET  EQU     0x10000000                              ; Value to trigger PendSV exception.
LR_INTERUPT_RET  EQU     0xFFFFFFF9                            ; Value to trigger PendSV exception.



StartOS
    LDR R0, =NVIC_SYSPRI15; Adjusting Systick priority
	LDR R1, [R0]
	MOV R2, #NVIC_LEVEL14
	ORR R1, R1, R2
	STR R1, [R0]
	
	LDR R0, =NVIC_SYSPRI14; Adjusting PendSV priority
	LDR R1, [R0]
	MOV R2, #NVIC_LEVEL15
	ORR R1, R1, R2
	STR R1, [R0]
	
	LDR R0, =RunPt; R0 = pointer to the pointer of the current TCB block
    LDR R1, [R0]
    LDR SP, [R1]
    POP {R4-R11}
    POP {R0-R3}; Not in an interrupt context so have to do this and next line manually
    POP {R12}
    ADD SP, SP, #4; Dont want R14 so add 4 to get to where the PC is stored
    POP {LR}; Pop the PC to into the LR for the later branch
    ADD SP, SP, #4; Dont need thumb bit so just add 4 to get to the top
    CPSIE I     
    BX      LR                 ; start first thread

OSStartHang
    B       OSStartHang        ; Should never get here
	
	
StartFromCheckpoint
    LDR R0, =NVIC_SYSPRI15; Adjusting Systick priority
	LDR R1, [R0]
	MOV R2, #NVIC_LEVEL14
	ORR R1, R1, R2
	STR R1, [R0]
	
	LDR R0, =NVIC_SYSPRI14; Adjusting PendSV priority
	LDR R1, [R0]
	MOV R2, #NVIC_LEVEL15
	ORR R1, R1, R2
	STR R1, [R0]
	
	PUSH {LR}
	BL ContextSwitch
	POP {LR}
	CPSIE I     
    BX      LR                 ; start first thread
	

;********************************************************************************************************
;                               PERFORM A CONTEXT SWITCH (From task level)
;                                           void ContextSwitch(void)
;
; Note(s) : 1) ContextSwitch() is called when OS wants to perform a task context switch.  This function
;              triggers the PendSV exception which is where the real work is done.
;********************************************************************************************************

ContextSwitch
    LDR R0, =NVIC_INT_CTRL
	MOV R1, #NVIC_PENDSVSET
    PUSH {LR}
    STR R1, [R0]
    POP {LR}
    BX      LR
    

;********************************************************************************************************
;                                         HANDLE PendSV EXCEPTION
;                                     void OS_CPU_PendSVHandler(void)
;
; Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
;              context switches with Cortex-M.  This is because the Cortex-M3 auto-saves half of the
;              processor context on any exception, and restores same on return from exception.  So only
;              saving of R4-R11 is required and fixing up the stack pointers.  Using the PendSV exception
;              this way means that context saving and restoring is identical whether it is initiated from
;              a thread or occurs due to an interrupt or exception.
;
;           2) Pseudo-code is:
;              a) Get the process SP, if 0 then skip (goto d) the saving part (first context switch);
;              b) Save remaining regs r4-r11 on process stack;
;              c) Save the process SP in its TCB, OSTCBCur->OSTCBStkPtr = SP;
;              d) Call OSTaskSwHook();
;              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
;              f) Get current ready thread TCB, OSTCBCur = OSTCBHighRdy;
;              g) Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr;
;              h) Restore R4-R11 from new process stack;
;              i) Perform exception return which will restore remaining context.
;
;           3) On entry into PendSV handler:
;              a) The following have been saved on the process stack (by processor):
;                 xPSR, PC, LR, R12, R0-R3
;              b) Processor mode is switched to Handler mode (from Thread mode)
;              c) Stack is Main stack (switched from Process stack)
;              d) OSTCBCur      points to the OS_TCB of the task to suspend
;                 OSTCBHighRdy  points to the OS_TCB of the task to resume
;
;           4) Since PendSV is set to lowest priority in the system (by OSStartHighRdy() above), we
;              know that it will only be run when no other exception or interrupt is active, and
;              therefore safe to assume that context being switched out was using the process stack (PSP).
;********************************************************************************************************
		IMPORT    Heap_Free
		IMPORT	  Snapshot_Heap


PendSV_Handler
    CPSID I
	LDR R0, =sdload
	LDR R1, [R0]
	CMP R1, #1
	BNE contextswitch
	MOV R1, #0
	STR R1, [R0]
	LDR R0, =RunPt; R0 = pointer to the pointer of the current TCB block
    LDR R1, [R0]
    LDR SP, [R1]
    POP {R4-R11}
    ;POP {R0-R3}; Not in an interrupt context so have to do this and next line manually
    ;POP {R12}
    ;ADD SP, SP, #4; Dont want R14 so add 4 to get to where the PC is stored
    ;POP {LR}; Pop the PC to into the LR for the later branch
    ;ADD SP, SP, #4; Dont need thumb bit so just add 4 to get to the top
	B leave
contextswitch
    PUSH {R4-R11}; These are the registers not automatically saved on interrupt
    LDR R0, =RunPt; R0 = pointer to the pointer of the current TCB block
    LDR R1, [R0]
	MOV R3, R1
    STR SP, [R1]; thread_sp is the first thing in the TCB
sleeping
    LDR R1, [R1, #4]; Loading pointer to the next TCB
	LDRB R2, [R1, #16] ;load sleep counter
	CMP R2, #0
	BNE sleeping ;branch back if sleepcount is not zero
	LDRB R2, [R3, #20] ;load killed value
	CMP R2, #0
	BEQ switch ;if killed is 0 then not killed go to switch
	PUSH {R0-R3, LR} ;save runpt address and LR
	LDR R0, [R3, #12] ;load stack top
	BL Heap_Free ;reclaim stack space
	MOV R0, R3 ;move thread pointer to r0
	BL Heap_Free ;reclaim tcb space
	POP {R0-R3, LR} ;restore runpt address and LR
switch
	PUSH {R0-R3, LR} ;save runpt address and LR
	BL Snapshot_Heap
	POP {R0-R3, LR} ;restore runpt address and LR
    STR R1, [R0]
    LDR SP, [R1]
    POP {R4-R11}; Pop the registers for this new thread
leave
    CPSIE I    
    BX      LR                 ; Exception return will restore remaining context   
    

;********************************************************************************************************
;                                         HANDLE SVC EXCEPTION
;                                     void OS_CPU_SVCHandler(void)
;
; Note(s) : SVC is a software-triggered exception to make OS kernel calls from user land. 
;           The function ID to call is encoded in the instruction itself, the location of which can be
;           found relative to the return address saved on the stack on exception entry.
;           Function-call paramters in R0..R3 are also auto-saved on stack on exception entry.
;********************************************************************************************************

        IMPORT    OS_Id
        IMPORT    OS_Kill
        IMPORT    OS_Sleep
        IMPORT    OS_Time
        IMPORT    OS_AddThread

SVC_Handler
; put your Lab 5 code here


    BX      LR                   ; Return from exception



    ALIGN
    END