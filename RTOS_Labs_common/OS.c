// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer0A.h"
#include "../inc/Timer1A.h"
#include "../inc/Timer2A.h"
#include "../inc/Timer3A.h"
#include "../inc/Timer4A.h"
#include "../inc/WTimer0A.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Lab2_RTOSkernel/TCB.h"


// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};

#define PD3  (*((volatile uint32_t *)0x40007020))

extern int StartOS(void);
extern int ContextSwitch(void);
extern int StartFromCheckpoint(void);

extern Sema4Type Snapshot;

#define HEAPSIZE 2000

extern int32_t heap[HEAPSIZE];

int32_t heapSnapshot[HEAPSIZE];
TCB_t *SavedRunPt;


// Initialize Variables
#define MAXTHREADS  10        // maximum number of threads

void(*SW1Task)(void);
void(*SW2Task)(void);

uint8_t num_threads = 0;
TCB_t *RunPt = NULL;
TCB_t *head = NULL;
TCB_t *tail = NULL;

mailbox_t data_mailbox;
fifo_t data_fifo;

int timerCount = 1;

uint32_t us_time = 0;
uint32_t system_ms_time = 0;


/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/
void SysTick_Handler(void) {
	//PD3 ^= 0x08;
	
	//sleep counter checks
	TCB_t* node = head;
	do {
		if (node->sleep_state > 0){
			node->sleep_state--;
		}
		node = node->next_TCB;
	} while(node != head);
	
	//ContextSwitch 
	ContextSwitch();
	//PD3 ^= 0x08;
} // end SysTick_Handler

unsigned long OS_LockScheduler(void){
  // lab 4 might need this for disk formating
  return 0;// replace with solution
}
void OS_UnLockScheduler(unsigned long previous){
  // lab 4 might need this for disk formating
}


void SysTick_Init(unsigned long period){
  STCTRL = 0; // disable SysTick during setup
  STCURRENT = 0; // any write to current clears it
  //SYSPRI3 = (SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = period - 1; // reload value
  STCTRL = 0x00000007; // enable, core clock and interrupt arm
}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void OS_Init(void){
  // put Lab 2 (and beyond) solution here
	PLL_Init(Bus80MHz);
	Heap_Init();
	DisableInterrupts();
}; 

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  // put Lab 2 (and beyond) solution here
	long status = StartCritical();
	semaPt->Value = value;
	EndCritical(status);
}; 

// ******** Save_Heap ***********
int Save_Heap(){
	if(eFile_Create("heap.bin")) {
		//return 1;
	}
	if (eFile_WOpen("heap.bin")){
		return 1;
	}
	for(int i = 0; i < 2000; i++){
    int32_t data = heapSnapshot[i];
    //write byte by byte
    if(eFile_Write(data & 0xFF)){ //low byte		
    // error in writing	
      return 1;
    }
    if(eFile_Write((data >> 8) & 0xFF)){ //second byte
    // error in writing	
      return 1;
    }
    if(eFile_Write((data >> 16) & 0xFF)){	//third byte
    // error in writing	
      return 1;
    }
    if(eFile_Write((data >> 24) & 0xFF)){	//high byte
    // error in writing	
      return 1;
    }
	}
	if(eFile_WClose()){			
		ST7735_Message(0, 1, "close error", 999999);
		return 1;
	}	
	
	ST7735_Message(1,1, "heap success", 999999);

	return 0;
}

// ******** Load_Heap ************
int Load_Heap(){
	if(eFile_ROpen("heap.bin")){
		return 1;
	}
	
	int starti;
	int32_t size;
	for(int i = 0; i < 2000; i++){
		//read 2000 words from the file byte by byte
    int32_t data = 0;
    for(int j = 0; j < 4; j++){
      char byte;
      if(eFile_ReadNext(&byte)){
				eFile_RClose();
        return 1;
      }
      data |= byte << (j * 8);
    }
		heap[i] = data;
	}
	
  if(eFile_RClose()){
    return 1;
  }
	
	return 0;
}

// ******** Save_RunPt ***********
int Save_RunPt(){
	if(eFile_Create("runpt.bin")){
		//something
	}
	
	if(eFile_WOpen("runpt.bin")){
		return 1;
	}
	
	//runpt
	int32_t data = (int32_t) SavedRunPt;
    //write byte by byte
	if(eFile_Write(data & 0xFF)){ //low byte		
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 8) & 0xFF)){ //second byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 16) & 0xFF)){	//third byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 24) & 0xFF)){	//high byte
	// error in writing	
		return 1;
	}
	
	//head
	data = (int32_t) head;
    //write byte by byte
	if(eFile_Write(data & 0xFF)){ //low byte		
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 8) & 0xFF)){ //second byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 16) & 0xFF)){	//third byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 24) & 0xFF)){	//high byte
	// error in writing	
		return 1;
	}
	
	//tail
	data = (int32_t) tail;
    //write byte by byte
	if(eFile_Write(data & 0xFF)){ //low byte		
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 8) & 0xFF)){ //second byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 16) & 0xFF)){	//third byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 24) & 0xFF)){	//high byte
	// error in writing	
		return 1;
	}
	
	//tail
	data = system_ms_time;
    //write byte by byte
	if(eFile_Write(data & 0xFF)){ //low byte		
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 8) & 0xFF)){ //second byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 16) & 0xFF)){	//third byte
	// error in writing	
		return 1;
	}
	if(eFile_Write((data >> 24) & 0xFF)){	//high byte
	// error in writing	
		return 1;
	}
	
	if(eFile_WClose()){			
		ST7735_Message(0, 1, "close error", 999999);
		return 1;
	}
	ST7735_Message(1, 2, "runpt success", 999999);
	return 0;
}

// ******** Save_CheckpointFlag ***********
int Save_CheckpointFlag(int8_t i){
	if(eFile_Create("control.bin")){
		//something
	}
	
	if(eFile_WOpen("control.bin")){
		ST7735_Message(0, 2, "control fail", 999999);
		return 1;
	}
	
	int8_t data = i;
    //write byte by byte
	if(eFile_Write(data)){ //low byte		
	// error in writing	
		return 1;
	}
	
	if(eFile_WClose()){			
		ST7735_Message(0, 1, "close error", 999999);
		return 1;
	}
	ST7735_Message(1, 3, "control success", 999999);
	return 0;
}

int log_variable(uint32_t value, char *filename){
	if (eFile_Create(filename)){
		// don't return
	}
	
	if (eFile_WOpen(filename)){
		return 1; // write open error
	}
	
	char temp[20];
	sprintf(temp, "%d", value);
	for (int i = 0; i < 20; i++){
		if (temp[i] == 0) {
			break;
		}
		eFile_Write(temp[i]);
	}
	
	if (eFile_WClose()){
		return 1; // write close error
	}
	return 0;
}

//************** Load_Runpt ******************
int Load_RunPt(){
	if(eFile_ROpen("runpt.bin")){
		return 1;
	}
	
	//runpt
	int32_t data = 0;
	for(int j = 0; j < 4; j++){
		char byte;
		if(eFile_ReadNext(&byte)){
			eFile_RClose();
			return 1;
		}
		data |= byte << (j * 8);
	}
	RunPt = (TCB_t*) data;
	
	//head
	data = 0;
	for(int j = 0; j < 4; j++){
		char byte;
		if(eFile_ReadNext(&byte)){
			eFile_RClose();
			return 1;
		}
		data |= byte << (j * 8);
	}
	head = (TCB_t*) data;
	RunPt = head;
	
	//tail
	data = 0;
	for(int j = 0; j < 4; j++){
		char byte;
		if(eFile_ReadNext(&byte)){
			eFile_RClose();
			return 1;
		}
		data |= byte << (j * 8);
	}
	tail = (TCB_t*) data;
	
	//time
	data = 0;
	for(int j = 0; j < 4; j++){
		char byte;
		if(eFile_ReadNext(&byte)){
			eFile_RClose();
			return 1;
		}
		data |= byte << (j * 8);
	}
	system_ms_time = (uint32_t) data;
	
  if(eFile_RClose()){
    return 1;
  }
	
	return 0;
}


// ******** Snapshot_Heap ***********
int Snapshot_Heap(){
	if(Snapshot.Value <= 0){
		return 1;
	}
	Snapshot.Value -= 1;
	for(int i = 0; i < 2000; i++){
		heapSnapshot[i] = heap[i];
	}
	SavedRunPt = RunPt;
	OS_Signal(&Snapshot);
	return 0;
}


void OS_Checkpoint_Launch(uint32_t theTimeSlice){
	SysTick_Init(theTimeSlice);
	//Timer4A_Init(&time_up_us, 80000000/1000000, 4); //1 us timer
	//OS_ClearMsTime(); // don't want to clear the time after retrieving from checkpoint.
	OS_checkpointingTime();
  StartFromCheckpoint(); // start on the first task
}
	
// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	while (semaPt->Value <= 0){
		EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	semaPt->Value -= 1;
	EnableInterrupts();
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status;
	status = StartCritical();
	semaPt->Value += 1;
	EndCritical(status);
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	DisableInterrupts();
	while (semaPt->Value == 0){
		EnableInterrupts();
		OS_Suspend();
		DisableInterrupts();
	}
	semaPt->Value = 0;
	EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  // put Lab 2 (and beyond) solution here
	long status;
	status = StartCritical();
	semaPt->Value = 1;
	EndCritical(status);
}; 

void SetInitialStack(stack_t *stack){
	stack->buffer[STACKSIZE-1] = 0x01000000; // thumb bit
	stack->buffer[STACKSIZE-3] = 0x14141414; // R14
	stack->buffer[STACKSIZE-4] = 0x12121212; // R12
	 stack->buffer[STACKSIZE-5] = 0x03030303; // R3
	 stack->buffer[STACKSIZE-6] = 0x02020202; // R2
	 stack->buffer[STACKSIZE-7] = 0x01010101; // R1
	 stack->buffer[STACKSIZE-8] = 0x00000000; // R0
	 stack->buffer[STACKSIZE-9] = 0x11111111; // R11
	 stack->buffer[STACKSIZE-10] = 0x10101010; // R10
	 stack->buffer[STACKSIZE-11] = 0x09090909; // R9
	 stack->buffer[STACKSIZE-12] = 0x08080808; // R8
	 stack->buffer[STACKSIZE-13] = 0x07070707; // R7
	 stack->buffer[STACKSIZE-14] = 0x06060606; // R6
	 stack->buffer[STACKSIZE-15] = 0x05050505; // R5
  stack->buffer[STACKSIZE-16] = 0x04040404; // R4
}


//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), 
   uint32_t stackSize, uint32_t priority){

	if(num_threads > MAXTHREADS){
		return 0;
	}

  int32_t status = StartCritical();

  TCB_t *thread = (TCB_t*) Heap_Malloc(sizeof(TCB_t));
	if(thread == 0){
		EndCritical(status);
		return 0;
	}
	
	stack_t *stack_pointer = (stack_t*) Heap_Malloc(sizeof(stack_t));
	if(stack_pointer == 0){
		Heap_Free(thread);
		EndCritical(status);
		return 0;
	}
	
	if(num_threads == 0){
		RunPt = thread;
		head = thread;
		head->next_TCB = thread;
	}

  tail->next_TCB = thread; //previous last TCB's next is new TCB
  thread->next_TCB = head; //new TCB's next is head
  thread->prev_TCB = tail; //new TCB's previous is tail
  head->prev_TCB = thread; //head's previous is new TCB
	tail = thread; //new TCB is now the tail

	thread->stack_top = (int32_t*) stack_pointer;
	thread->thread_sp = &stack_pointer->buffer[STACKSIZE-16]; // thread stack pointer
	SetInitialStack(stack_pointer);
	stack_pointer->buffer[STACKSIZE-2] = (int32_t)(task); //PC
	thread->id = num_threads;
	thread->sleep_state = 0;
	thread->priority = priority;
	thread->killed = 0;
	num_threads++;
	EndCritical(status);
	return 1;
  }
//}

//******** OS_AddProcess *************** 
// add a process with foregound thread to the scheduler
// Inputs: pointer to a void/void entry point
//         pointer to process text (code) segment
//         pointer to process data segment
//         number of bytes allocated for its stack
//         priority (0 is highest)
// Outputs: 1 if successful, 0 if this process can not be added
// This function will be needed for Lab 5
// In Labs 2-4, this function can be ignored
int OS_AddProcess(void(*entry)(void), void *text, void *data, 
  unsigned long stackSize, unsigned long priority){
  // put Lab 5 solution here

     
  return 0; // replace this line with Lab 5 solution
}


//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  return RunPt->id; // replace this line with solution
};


//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   uint32_t period, uint32_t priority){
  // put Lab 2 (and beyond) solution here
		 long status = StartCritical();
		 if(timerCount > 2){
			 return 0;
		 }
		 switch(timerCount){
//			 case 0:
//				 Timer0A_Init(task, period, priority); break;
			 case 1:
				 Timer1A_Init(task, period, priority); break;
			 case 2:
				 Timer2A_Init(task, period, priority); break;
		 }
		 timerCount++;
		 EndCritical(status);
  return 1; // replace this line with solution
};


/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
void GPIOPortF_Handler(void){
	if ((GPIO_PORTF_RIS_R & 0x01)) {
		GPIO_PORTF_ICR_R |= 0x01;			// acknowledge flag
		(*SW2Task)();
	}
	
	else if ((GPIO_PORTF_RIS_R & 0x10)) {
		GPIO_PORTF_ICR_R |= 0x10;			// acknowledge flag
		(*SW1Task)();
	}
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  uint8_t priByte = (uint8_t) (priority & 0xFF);
	uint32_t priReg = priByte << 5;
	
	SYSCTL_RCGCGPIO_R     |= 0x00000020;      // activate clock for Port F
  while((SYSCTL_PRGPIO_R & 0x20)==0){};     // allow time for clock to stabilize
	SW1Task = task;
    
  GPIO_PORTF_LOCK_R     = 0x4C4F434B;       // unlock GPIO Port F
  GPIO_PORTF_CR_R       = 0x1F;             // allow changes to PF4-0
  
	//***Potentially dangerous
	GPIO_PORTF_AMSEL_R    = 0x00;             // disable analog on PF
  GPIO_PORTF_PCTL_R 		&= ~0x000FFFFF; // configure PF4 as GPIO
	GPIO_PORTF_AFSEL_R 		&= ~0x1F;  //     disable alt funct on PF4,0
  
	GPIO_PORTF_DIR_R 			&= ~0x10;    // make PF4 in (built-in button)
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
	GPIO_PORTF_DEN_R      |= 0x10;             // enable digital I/O on PF4
		
	GPIO_PORTF_IS_R    	 	&= ~0x10; 						// PF4 edge-sensitive
  GPIO_PORTF_IBE_R   	 	&= ~0x10; 						// PF4 is not both edges
  GPIO_PORTF_IEV_R   	 	&= ~0x10; 						// PF4 falling edge event (Neg logic)
  GPIO_PORTF_ICR_R   	  |= 0x10;  						// clear flag
  GPIO_PORTF_IM_R    	 	|= 0x10;  						// arm interrupt on PF4
		
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|priReg; // GPIOF priority 5
  NVIC_EN0_R  = 0x40000000;													 // enable interrupt 4 in NVIC
  return 0; // replace this line with solution
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  // put Lab 2 (and beyond) solution here
  uint8_t priByte = (uint8_t) (priority & 0xFF);
	uint32_t priReg = priByte << 5;
	
	SYSCTL_RCGCGPIO_R     |= 0x00000020;      // activate clock for Port F
  while((SYSCTL_PRGPIO_R & 0x20)==0){};     // allow time for clock to stabilize
	SW2Task = task;
    
  GPIO_PORTF_LOCK_R     = 0x4C4F434B;       // unlock GPIO Port F
  GPIO_PORTF_CR_R       = 0x1F;             // allow changes to PF4-0
  
	//***Potentially dangerous
	GPIO_PORTF_AMSEL_R    = 0x00;             // disable analog on PF
  GPIO_PORTF_PCTL_R 		&= ~0x000FFFFF; // configure PF4 as GPIO
	GPIO_PORTF_AFSEL_R 		&= ~0x1F;  //     disable alt funct on PF4,0
  
	GPIO_PORTF_DIR_R 			&= ~0x01;    // (c) make PF0 in (built-in button)
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF0
	GPIO_PORTF_DEN_R      |= 0x01;             // enable digital I/O on PF0
		
	GPIO_PORTF_IS_R    	 	&= ~0x01; 						// PF0 is edge-sensitive
  GPIO_PORTF_IBE_R   	 	&= ~0x01; 						// PF0 is not both edges
  GPIO_PORTF_IEV_R   	 	&= ~0x01; 						// PF0 falling edge event (Neg logic)
  GPIO_PORTF_ICR_R   	  |= 0x01;  						// clear flag
  GPIO_PORTF_IM_R    	 	|= 0x01;  						// arm interrupt on PF0
		
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|priReg; // GPIOF priority 5
  NVIC_EN0_R  = 0x40000000;													 // enable interrupt 4 in NVIC  
  return 0; // replace this line with solution
};


// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  // put Lab 2 (and beyond) solution here
  RunPt->sleep_state = sleepTime;
	ContextSwitch();	//check this
};  

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  // put Lab 2 (and beyond) solution here
 //do not have to free anything now because it is statically allocated
	//remove from linked list
	RunPt->prev_TCB->next_TCB = RunPt->next_TCB;
	RunPt->next_TCB->prev_TCB = RunPt->prev_TCB;
	
	//Adjust head and tail
	if (RunPt == tail){
		tail = tail -> prev_TCB;
	}
	if (RunPt == head){
		head = head -> next_TCB;
	}
	
	//reclaim space
	RunPt->killed = 1;
	num_threads--;
	
	ContextSwitch();//check
  EnableInterrupts();   // end of atomic section 
  for(;;){};        // can not return
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  ContextSwitch();
};
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(uint32_t size){
	data_fifo.putI = 0;
	data_fifo.getI = 0; 
	data_fifo.room_left.Value = 1;
	data_fifo.data_available.Value = 0; 
	data_fifo.mutex.Value = 1; //1 means free and 0 means taken
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  if( (data_fifo.room_left.Value == 0) || (data_fifo.mutex.Value == 0)){ //FIFO full or cannot get mutex
		return 0;
	}
	
	//Grab the mutex
	data_fifo.mutex.Value = 0;
	
	data_fifo.buffer[data_fifo.putI & (FIFO_SIZE-1)] = data;
	data_fifo.putI++;
	
	if( ((data_fifo.putI) & (FIFO_SIZE-1)) == data_fifo.getI){
		data_fifo.room_left.Value = 0;
	}
	OS_Signal(&data_fifo.data_available);
	OS_bSignal(&data_fifo.mutex);
  return 1;
};  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  OS_Wait(&data_fifo.data_available);
	OS_bWait(&data_fifo.mutex); //Wait for the mutex
	
	//Reaching here means that data and mutex were available
	//data_fifo.mutex.Value = 0; //Grab mutex
	uint32_t retVal;
	retVal = data_fifo.buffer[data_fifo.getI & (FIFO_SIZE-1)];
  data_fifo.getI++;
	
	if( ((data_fifo.putI) & (FIFO_SIZE-1)) != data_fifo.getI){
		//data_fifo.room_left.Value = 1;
		OS_Signal(&data_fifo.room_left);
	}
//	if(data_fifo.getI == data_fifo.putI){
//		data_fifo.data_available.Value = 0;
//	}
	
	data_fifo.mutex.Value = 1; //Give mutex back
  return retVal;
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
   
  return 0; // replace this line with solution
};


// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
	long status = StartCritical();
	data_mailbox.box_free.Value = 1;
	data_mailbox.data_valid.Value = 0;
	EndCritical(status);
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
	OS_bWait(&data_mailbox.box_free);
	data_mailbox.data = data;
	OS_bSignal(&data_mailbox.data_valid);
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  int32_t retVal;
	OS_bWait(&data_mailbox.data_valid);
	retVal = data_mailbox.data;
	OS_bSignal(&data_mailbox.box_free);
  return retVal;
};

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
uint32_t OS_Time(void){
  return STCURRENT;
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
	//****Assumes that will not wraparound twice in between start and stop being defined in external code
  if (stop > start){ //wraparound case in countdown timer
		return start + (NVIC_ST_RELOAD_R - stop);
	} else{
		return start - stop;
	}
};

void time_upd_ms(){
	int32_t status = StartCritical();
	system_ms_time++;
	//PD3 ^= 0x08;
	EndCritical(status);
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), and start a periodic interrupt
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
  Timer3A_Init(time_upd_ms, 80000000/1000, 4);  //1 ms resolution
	system_ms_time = 0;
};

void OS_checkpointingTime(void){
	Timer3A_Init(time_upd_ms, 80000000/1000, 4);  //1 ms resolution
}

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
  return system_ms_time;
};

void time_up_us(){
	int32_t status = StartCritical();
	us_time++;
	EndCritical(status);
}

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
	SysTick_Init(theTimeSlice);
	//Timer4A_Init(&time_up_us, 80000000/1000000, 4); //1 us timer
	OS_ClearMsTime();
  StartOS(); // start on the first task
};

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice=0;                // 0=UART, 1=stream to file (Lab 4)

int fputc (int ch, FILE *f) { 
  if(StreamToDevice==1){  // Lab 4
    if(eFile_Write(ch)){          // close file on error
       OS_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }
  
  // default UART output
  UART_OutChar(ch);
  return ch; 
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(const char *name){  // Lab 4
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToDevice = 1;
  return 0;
}

int OS_EndRedirectToFile(void){  // Lab 4
  StreamToDevice = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int OS_RedirectToUART(void){
  StreamToDevice = 0;
  return 0;
}

int OS_RedirectToST7735(void){
  
  return 1;
}