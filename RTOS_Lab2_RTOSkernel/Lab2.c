// Lab2.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Lab 2
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3, Testmain4, Testmain5, TestmainCS and realmain

// Jonathan W. Valvano 1/29/20, valvano@mail.utexas.edu
// EE445M/EE380L.12
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for use by OS profile
// PF1 is preemptive thread switch
// PF2 is periodic task (DAS samples PE3)
// PC4 is PF4 button touch (SW1 task)

// IR distance sensors
// J5/A3/PE3 analog channel 0  <- connect an IR distance sensor to J5 to get a realistic analog signal on PE3
// J6/A2/PE2 analog channel 1  <- connect an IR distance sensor to J6 to get a realistic analog signal on PE2
// J7/A1/PE1 analog channel 2
// J8/A0/PE0 analog channel 3  

// Button inputs
// PF4 is SW1 button input

// Analog inputs
// PE3 Ain0 sampled at 2kHz, sequencer 3, by DAS, using software start in ISR
// PE2 Ain1 sampled at 250Hz, sequencer 0, by Producer, timer tigger

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/PLL.h"
#include "../inc/LPF.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/IRDistance.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/heap.h"
#include "../RTOS_Labs_common/ST7735.h"


//Dump Array
#define DUMPSIZE 1000
uint32_t dump_array[DUMPSIZE];
uint32_t dump_array_put = 0;

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

uint32_t NumCreated;   // number of foreground threads created
uint32_t PIDWork;      // current number of PID calculations finished
uint32_t FilterWork;   // number of digital filter calculations finished
uint32_t NumSamples;   // incremented every ADC sample, in Producer
#define FS 400              // producer/consumer sampling
#define RUNLENGTH (20*FS)   // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units
int32_t x[64],y[64];           // input and output arrays for FFT

//---------------------User debugging-----------------------
uint32_t DataLost;     // data sent by Producer, but not received by Consumer
extern int32_t MaxJitter;             // largest time jitter between interrupts in usec
extern uint32_t const JitterSize;
extern uint32_t JitterHistogram[];

#define HEAPSIZE 2000
extern int32_t heap[HEAPSIZE];
extern int32_t heapSnapshot[HEAPSIZE];
volatile int32_t sdload;

#define PD0  (*((volatile uint32_t *)0x40007004))
#define PD1  (*((volatile uint32_t *)0x40007008))
#define PD2  (*((volatile uint32_t *)0x40007010))
#define PD3  (*((volatile uint32_t *)0x40007020))
	
#define PF1   (*((volatile uint32_t *)0x40025008)) // RED LED
#define PF2   (*((volatile uint32_t *)0x40025010)) // BLUE LED
#define PF3   (*((volatile uint32_t *)0x40025020)) // GREEN LED


void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_RCGCGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;        // make PD3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;     // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;        // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;    // disable analog functionality on PD
		
	SYSCTL_RCGCGPIO_R     |= 0x00000020;      // activate clock for Port F
  while((SYSCTL_PRGPIO_R & 0x20)==0){};     // allow time for clock to stabilize
    
  GPIO_PORTF_LOCK_R     = 0x4C4F434B;       // unlock GPIO Port F
  GPIO_PORTF_CR_R       = 0x1F;             // allow changes to PF4-0
  
	GPIO_PORTF_AMSEL_R    = 0x00;             // disable analog on PF
  GPIO_PORTF_PCTL_R     = 0x00000000;       // PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R      = 0x0E;             // PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R    = 0x00;             // disable alt funct on PF7-0
  GPIO_PORTF_PDR_R      = 0x11;             // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R      = 0x1F;             // enable digital I/O on PF4-0

}

volatile uint32_t Count1;   // number of times thread1 loops
volatile uint32_t Count2;   // number of times thread2 loops
volatile uint32_t Count3;   // number of times thread3 loops
volatile uint32_t Count4;   // number of times thread4 loops
volatile uint32_t Count5;   // number of times thread5 loops


//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 2 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
void Thread1b(void){
  Count1 = 0;          
  for(;;){
    //PD0 ^= 0x01;       // heartbeat
    Count1++;
  }
}
void Thread2b(void){
  Count2 = 0;          
  for(;;){
    //PD1 ^= 0x02;       // heartbeat
    Count2++;
  }
}
void Thread3b(void){
  Count3 = 0;          
  for(;;){
    //PD2 ^= 0x04;       // heartbeat
    Count3++;
  }
}

int Testmain2(void){  // Testmain2
  OS_Init();          // initialize, disable interrupts
	PortD_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,0); 
  NumCreated += OS_AddThread(&Thread2b,128,0); 
  NumCreated += OS_AddThread(&Thread3b,128,0); 
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than Testmain1
 
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}


int TestmainFile(){
	PLL_Init(Bus80MHz);
	Heap_Init();
	NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,0); 
  NumCreated += OS_AddThread(&Thread2b,128,0); 
  NumCreated += OS_AddThread(&Thread3b,128,0); 
	
	ST7735_Message(0, 1, "heap test start", 1);
	Save_Heap();
	ST7735_Message(0, 1, "write success", 4);
	
	Load_Heap();
	
	for(int i = 0; i < 2000; i++){
		//read 2000 words from the file byte by byte
    if(heapSnapshot[i] != heap[i]){
			ST7735_Message(0, 1, "read fail", 4);
			return 1;
		}
		if(heapSnapshot[i] < 0){
			i += -heapSnapshot[i] + 1;
		}
	}
	ST7735_Message(0, 3, "read success", 4);
	return 0;
}

// -----------------------THREADS-----------------------------
void array(){
	uint8_t numbers[100];
	int loop = 1;
	for(int i = 0; i < 100; i++){
		numbers[i] = loop*i;
		ST7735_Message(0, 1, "Array Index", i);
		OS_Sleep(1000);
		if(i == 99){
			loop++;
			i = 0;
		}
	}
	OS_Kill();
}

void blink(){
	while(1){
		PF1 ^= 0x02;
		OS_Sleep(500);
	}
}

void time(){
	while(1){
		ST7735_Message(0, 2, "Sys Time (s): ", OS_MsTime()/1000);
		OS_Sleep(1000);
	}
}


Sema4Type Snapshot;
void Checkpoint(){
	while(1){
		OS_Wait(&Snapshot);
		PD0 ^= 0x01;
		int heapret = Save_Heap();
		PD0 ^= 0x01;
		
		PD1 ^= 0x02;
		int runptret = Save_RunPt();
		PD1 ^= 0x02;
		
		PD2 ^= 0x04;
		int count3ret = log_variable(Count3, "count3.txt");
		PD2 ^= 0x04;
		
		PD3 ^= 0x08;
		Save_CheckpointFlag(heapret&runptret);
		PD3 ^= 0x08;
		
		OS_Signal(&Snapshot);
		OS_Sleep(1000);
	}
}

int create_threads(){
	NumCreated = 0 ;
	NumCreated += OS_AddThread(&array,128,0); 
	NumCreated += OS_AddThread(&blink,128,0); 
	NumCreated += OS_AddThread(&Thread3b,128,0); 
	NumCreated += OS_AddThread(&time,128,0); 
	NumCreated += OS_AddThread(&Checkpoint,128,0); 
	return 0;
}

volatile int sw1 = 0;
volatile int sw2 = 0;

void sw1Set(){
	sw1 = 1;
}

void sw2Set(){
	sw2 = 1;
}

int CheckpointingOS(){
	PLL_Init(Bus80MHz);
	Heap_Init();
	OS_AddSW1Task(&sw1Set, 3);
	OS_AddSW2Task(&sw2Set, 3);
	OS_InitSemaphore(&Snapshot,1);
	
	if(eFile_ROpen("control.bin")){
		//read error load normally
		create_threads();
		eFile_RClose();
		OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
	} 
	else {
		char byte;
		if(eFile_ReadNext(&byte)){
			ST7735_Message(0, 1, "read error at ", 999999);
			eFile_RClose();
			return 1;
		}
		if(eFile_RClose()){
			ST7735_Message(0, 1, "read close error", 999999);
			return 1;
		}
		int8_t saved = (int8_t) byte;
		ST7735_Message(0, 1, "Press SW1 to load from SD", 999999);
		ST7735_Message(0, 2, "Press SW2 to clean start", 999999);
		while(sw1 == 0 && sw2 == 0){}
		ST7735_FillScreen(ST7735_BLACK);
		int8_t restore = sw1 == 1? 1 : 0;
	  sdload = !saved && restore;
		if(sdload){
			ST7735_Message(0, 0, "Checkpoint Recovered", 999999);
			Load_Heap();
			Load_RunPt();
			OS_Checkpoint_Launch(TIME_2MS);
		} 
		else {
			sdload = 0;
			create_threads();
			OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
		}
	}	return 0;
}

//*******************Trampoline for selecting main to execute**********
int main(void) { 			// main 
	PortD_Init();       // profile user threads
	ST7735_InitR(INITR_REDTAB); // LCD initialization
	OS_AddPeriodicThread(&disk_timerproc,TIME_1MS,0);   // time out routines for disk
	eFile_Init();
	eFile_Mount();
	CheckpointingOS();
}