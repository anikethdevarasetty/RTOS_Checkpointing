// filename *************************heap.c ************************
// Implements memory heap for dynamic memory allocation.
// Follows standard malloc/calloc/realloc/free interface
// for allocating/unallocating memory.

// Jacob Egner 2008-07-31
// modified 8/31/08 Jonathan Valvano for style
// modified 12/16/11 Jonathan Valvano for 32-bit machine
// modified August 10, 2014 for C99 syntax

/* This example accompanies the book
   "Embedded Systems: Real Time Operating Systems for ARM Cortex M Microcontrollers",
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains

 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */


#include <stdint.h>
#include "../RTOS_Labs_common/heap.h"
#include <string.h>
#include "OS.h"

Sema4Type HeapSema; 

#define HEAPSIZE 2000

int32_t heap[HEAPSIZE];


//******** Heap_Init *************** 
// Initialize the Heap
// input: none
// output: always 0
// notes: Initializes/resets the heap to a clean state where no memory
//  is allocated.
int32_t Heap_Init(void){
	OS_InitSemaphore(&HeapSema, 1);
	for(int i = 0; i < 2000; i++){
		heap[i] = 0;
	}
	*(heap) = -1*(int32_t)(HEAPSIZE-2);
	*(heap + HEAPSIZE - 1) = -1 * (int32_t)(HEAPSIZE - 2);
  return 0;   // replace
}

int32_t blockSpace(int32_t* block){
  if(*block > 0){
    return *block;
  }
  return -*block;
}

int useBlock(int32_t* block, int32_t size){
	if (blockSpace(block) - size - 2 > 0){ // there is space for another block below
		int32_t wholeBlockSpace = blockSpace(block);
		
		int32_t* firstEnd = block + size + 1;
		int32_t* secondStart = firstEnd + 1;
		int32_t* secondEnd = block + wholeBlockSpace + 1;
		
		
		*block = size;
		*firstEnd = size;
		*secondStart = -1 * (wholeBlockSpace- size - 2);
		*secondEnd = -1 * (wholeBlockSpace - size - 2);
	} 
	else {
		int32_t* blockEnd = block + blockSpace(block) + 1;
		if(*block > 0 || *block != *blockEnd){ //made a booboo with the heap somewhere
			OS_Signal(&HeapSema);
			return 1;
		}
		*block = -1*blockSpace(block);
		*blockEnd = -1*blockSpace(block);
	}
	OS_Signal(&HeapSema);
	return 0;
}

//******** Heap_Malloc *************** 
// Allocate memory, data not initialized
// input: 
//   desiredBytes: desired number of bytes to allocate
// output: void* pointing to the allocated memory or will return NULL
//   if there isn't sufficient space to satisfy allocation request
void* Heap_Malloc(int32_t desiredBytes){
	OS_Wait(&HeapSema);
	int32_t desiredWords = (desiredBytes + sizeof(int32_t) - 1) / sizeof(int32_t);
	
	if (desiredWords <= 0){
		OS_Signal(&HeapSema);
		return 0;
	}

	int32_t* block = heap;
	while (block >= heap && block <= (heap + HEAPSIZE)){ //in range of heap
		if (*block < 0 && blockSpace(block) >= desiredWords){ //unused and enough space
			if(useBlock(block, desiredWords)){
				OS_Signal(&HeapSema);
				return 0;
			}
			OS_Signal(&HeapSema);
			return block+1; //account for metadata;
		}
		block = block + blockSpace(block) + 2; //next block is at start + space + 2(metadata)
	}
	OS_Signal(&HeapSema);
  return 0;   // NULL
}


//******** Heap_Calloc *************** 
// Allocate memory, data are initialized to 0
// input:
//   desiredBytes: desired number of bytes to allocate
// output: void* pointing to the allocated memory block or will return NULL
//   if there isn't sufficient space to satisfy allocation request
//notes: the allocated memory block will be zeroed out
void* Heap_Calloc(int32_t desiredBytes){  

	int32_t* block = Heap_Malloc(desiredBytes);
	OS_Wait(&HeapSema);
	if(block == 0){
		OS_Signal(&HeapSema);
		return 0;
	}
	
	for(int i = 0; i < blockSpace(block-1); i++){
		block[i] = 0;
	}
	OS_Signal(&HeapSema);
  return block;
}


//******** Heap_Realloc *************** 
// Reallocate buffer to a new size
//input: 
//  oldBlock: pointer to a block
//  desiredBytes: a desired number of bytes for a new block
// output: void* pointing to the new block or will return NULL
//   if there is any reason the reallocation can't be completed
// notes: the given block may be unallocated and its contents
//   are copied to a new block if growing/shrinking not possible
void* Heap_Realloc(void* oldBlock, int32_t desiredBytes){

	int32_t* oldBlockpt = (int32_t*) (oldBlock);
	oldBlockpt = oldBlockpt - 1;
	
	if(oldBlockpt < heap || oldBlockpt > heap+HEAPSIZE || *oldBlockpt < 0){ //old block not used or in range
		return 0;
	}
	
	int32_t* newBlock = Heap_Malloc(desiredBytes);
	if(newBlock == 0){
		return 0;
	}
	
	//words to copy is block with less size
	//int32_t copySize = blockSpace(oldBlockpt) < blockSpace(newBlock) ? blockSpace(oldBlockpt) : blockSpace(newBlock);
	
	OS_Wait(&HeapSema);
	int32_t copySize;
	if(blockSpace(oldBlockpt) < blockSpace(newBlock-1)){
		copySize = blockSpace(oldBlockpt);
	} else {
		copySize = blockSpace(newBlock-1);
	}
	
	for(int i = 0; i < copySize; i++){
		newBlock[i] = (oldBlockpt+1)[i];
	}
	OS_Signal(&HeapSema);
		
	if(Heap_Free(oldBlock)){
		return 0;
	}
	return newBlock;   // NULL
}


//******** Heap_Free *************** 
// return a block to the heap
// input: pointer to memory to unallocate
// output: 0 if everything is ok, non-zero in case of error (e.g. invalid pointer
//     or trying to unallocate memory that has already been unallocated
int32_t Heap_Free(void* pointer){
	OS_Wait(&HeapSema);

	int32_t* block = (int32_t*) pointer - 1; // -1 to get metadata
	
	if(block < heap || block > heap+HEAPSIZE || *block < 0){ //out of range or unused
		OS_Signal(&HeapSema);
		return 1;
	}
	
	int32_t* endBlock = block + blockSpace(block) + 1;
	if(endBlock < heap || endBlock > heap+HEAPSIZE || *endBlock < 0 || *block != *endBlock){ //out of range or unused
		OS_Signal(&HeapSema);
		return 1;
	}
	
	//make negative to mark as unused
	*block = -1*(*block);
	*endBlock = -1*(*endBlock);
	
	//merge up?
	int32_t* prevBlock =  block - blockSpace(block-1) - 2; //this start - previous block size - 2
	if(prevBlock >= heap && prevBlock < heap+HEAPSIZE && *prevBlock < 0){ //in range and unused
		*prevBlock = -1 * (blockSpace(prevBlock) + blockSpace(block) + 2);
		*endBlock = *prevBlock;
		block = prevBlock;
	}
	
	//merge below?
	int32_t* nextBlock = endBlock + 1;
	if(nextBlock >= heap && nextBlock < heap+HEAPSIZE){ // in range and unused
		if(*(endBlock + 1) < 0){
			*block = -1 * (blockSpace(block) + blockSpace(nextBlock) + 2); //two block spaces + 2 words for metadata
			int32_t* nextBlockEnd = nextBlock + blockSpace(nextBlock) + 1;
			*nextBlockEnd = *block;
		}
	}
	
	OS_Signal(&HeapSema);
	
  return 0;
}


//******** Heap_Stats *************** 
// return the current status of the heap
// input: reference to a heap_stats_t that returns the current usage of the heap
// output: 0 in case of success, non-zeror in case of error (e.g. corrupted heap)
int32_t Heap_Stats(heap_stats_t *stats){
  int32_t* block = heap;
	
	stats->size = HEAPSIZE;
	stats->used = 0;
	stats->free = 0;
	
	while(block < heap + HEAPSIZE){
		if(*block < 0){ //unused
			stats->free += blockSpace(block);
		} else {
			stats->used += blockSpace(block);
		}
		block = block + blockSpace(block) + 2;
	}
  return 0;   // replace
}