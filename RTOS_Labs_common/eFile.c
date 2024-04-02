// filename ************** eFile.c *****************************
// High-level routines to implement a solid-state disk 
// Students implement these functions in Lab 4
// Jonathan W. Valvano 1/12/20
#include <stdint.h>
#include <string.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include <stdio.h>

static uint8_t init_flag = 0; //1 means file system initialized; 0 o.w.
static int32_t dir_block = -1; //Tells which memory block holds the directory
static int32_t curr_file = -1;

Directory_u directory; //Holds the read directory in RAM
Block_u read_block; //Holds the currently read block in RAM
uint32_t read_num; //The number of the currently read block in RAM
uint16_t read_idx; //Index of byte in current file that we have read up to

uint8_t find_unused_block(uint32_t* block_num);
uint8_t find_starting_block(const char n[], uint32_t* ret_num);

Sema4Type File_Sema;

//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
int eFile_Init(void){ // initialize file system
	eDisk_Init(0);
	if(!init_flag){
		init_flag = 1;
		return 0;
	} else{
		return 1;
	}
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void){ // erase disk, add format
	
	for(uint32_t i = 0; i < MAX_BLOCKS; i++){
			if(eDisk_ReadBlock(read_block.whole_block, i)){
				return 1; //Any trouble reading disk
			}
			read_num = 0;
			memset(&read_block, 0, BLOCK_SIZE);  // Sets whole block to 0
			eDisk_WriteBlock(read_block.whole_block, i);
	}
  return 0;
}

//---------- eFile_Mount-----------------
// Mount the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure
int eFile_Mount(void){ // initialize file system
	dir_block = 0;
	OS_InitSemaphore(&File_Sema, 1);
  return 0;
}


//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create(const char n[]){  // create new file, make it empty 
	OS_Wait(&File_Sema);
	//Find an unused block to allocate the file
	uint32_t found_block;
	if(find_unused_block(&found_block)){
		OS_Signal(&File_Sema);
		return 1; //Couldnt find unused block or trouble reading disk
	}
	
	//Need to find a unused directory entry
	if(eDisk_ReadBlock(directory.info, dir_block)){ //Read the directory into RAM
		OS_Signal(&File_Sema);
		return 1;
	}
	
	int16_t unused_entry = -1;
	for(uint16_t i = 0; i < MAX_FILES; i++){ //Cycle through all the entries
		if(!directory.table[i].in_use){ 
			unused_entry = i; //Found an unused one
			break;
		}
	}
	if(unused_entry < 0){ //Couldnt find an unused entry
		OS_Signal(&File_Sema);
		return 1;
	}
	//printf("%d\n", unused_entry);
	//Update the directory and write directory back to disk
	strcpy(directory.table[unused_entry].name, n);
	directory.table[unused_entry].in_use = 1;
	directory.table[unused_entry].starting_block = found_block;
	if(eDisk_WriteBlock(directory.info, dir_block)){
		OS_Signal(&File_Sema);
		return 1;
	}
	
	//Read Write Test
	if(eDisk_ReadBlock(directory.info, dir_block)){ 
		OS_Signal(&File_Sema);
		return 1;
	}
	
	//Read in the starting block of the new file and update the metadata
	if(eDisk_ReadBlock(read_block.whole_block, found_block)){ //Read the found unused block
		OS_Signal(&File_Sema);
		return 1;
	}
	read_num = found_block;
	read_block.structured.in_use = 1;
	read_block.structured.last_write_idx = HEADER_SIZE; //Want to start writing after the header
	read_block.structured.next_block = 0;
	if(eDisk_WriteBlock(read_block.whole_block, found_block)){ //Write back to disk
		OS_Signal(&File_Sema);
		return 1;
	}
	OS_Signal(&File_Sema);
  return 0;
}


//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen( const char n[]){      // open a file for writing 
	OS_Wait(&File_Sema);
	uint32_t start_block;
	printf("in wopen\n");
	if(find_starting_block(n, &start_block)){
		printf("not found\n");
		OS_Signal(&File_Sema);
		return 1; //Some sort of error occurred
	}
	
	printf("%d\n", start_block);
	
	//Cycle through all blocks in file till reach the last one
	//On exit from loop, the last block is in RAM
	uint32_t next_block = start_block;
	while(next_block != 0){
		if(eDisk_ReadBlock(read_block.whole_block, next_block)){ //Load the block into RAM
			printf("block load error\n");
			OS_Signal(&File_Sema);
			return 1;
		}
		read_num = next_block;
		next_block = read_block.structured.next_block;
	}
	
	OS_Signal(&File_Sema);
  return 0;  
}

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write( const char data){
		OS_Wait(&File_Sema);
		//Assumes the last block of the file is loaded in RAM
		if(read_block.structured.last_write_idx > BLOCK_SIZE){
			
			//Need to get a new block
			uint32_t new_tail;
			if(find_unused_block(&new_tail)){
				OS_Signal(&File_Sema);
				return 1; //Some sort of error
			}
			
			read_block.structured.next_block = new_tail;
			if(eDisk_WriteBlock(read_block.whole_block, read_num)){ //Write back to disk
				OS_Signal(&File_Sema);
				return 1;
			}
			
			//Read in the new tail block & initialize some information
			if(eDisk_ReadBlock(read_block.whole_block, new_tail)){
				OS_Signal(&File_Sema);
				return 1;
			}
			read_num = new_tail;
			read_block.structured.in_use = 1;
			read_block.structured.last_write_idx = HEADER_SIZE;
			read_block.structured.next_block = 0;
			if(eDisk_WriteBlock(read_block.whole_block, new_tail)){ //Write this temporarily to disk so no one else grabs the block
				OS_Signal(&File_Sema);
				return 1;
			}
		}
		
		read_block.whole_block[read_block.structured.last_write_idx] = data;
		read_block.structured.last_write_idx++;
		OS_Signal(&File_Sema);
    return 0;
}


//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void){
		OS_Wait(&File_Sema);
		if(eDisk_WriteBlock(read_block.whole_block, read_num)){ //Write back to disk
					OS_Signal(&File_Sema);
					return 1;
		}
		OS_Signal(&File_Sema);
		return 0;
}


//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is an ASCII string up to seven characters
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( const char n[]){      // open a file for reading 
	OS_Wait(&File_Sema);
	uint32_t start_block;
	if(find_starting_block(n, &start_block)){
		OS_Signal(&File_Sema);
		return 1; //Some sort of error occurred
	}
	
	//Read into RAM the first block of the file
	if(eDisk_ReadBlock(read_block.whole_block, start_block)){
		OS_Signal(&File_Sema);
		return 1;
	}
	read_num = start_block;
	
	read_idx = HEADER_SIZE; //Start reading at first byte after header
	OS_Signal(&File_Sema);
  return 0;
}
 

//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt){       // get next byte 
	OS_Wait(&File_Sema);
	if(read_idx > BLOCK_SIZE){ //Check if past end of block
		//Need to read in next block
		if(read_block.structured.next_block == 0){
			OS_Signal(&File_Sema);
			return 1; //Reached EOF
		}
		
		if(eDisk_ReadBlock(read_block.whole_block, read_block.structured.next_block)){
			OS_Signal(&File_Sema);
			return 1;
		}
		read_num = read_block.structured.next_block;
		read_idx = HEADER_SIZE; //Start reading at first byte after header
	}
	
	*pt = read_block.whole_block[read_idx];
	read_idx++;
	OS_Signal(&File_Sema);
  return 0; 
}
    
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void){ // close the file for reading
  return 0;   // ???- Shouldnt really have to do anything
}


//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a ASCII string
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( const char n[]){  // remove this file 
  OS_Wait(&File_Sema);
	uint32_t start_block;
	if(find_starting_block(n, &start_block)){
		OS_Signal(&File_Sema);
		return 1; //Some sort of error occurred
	}
	
	//Cycle through all the blocks and clear them
	uint32_t next_block = start_block;
	while(next_block != 0){
		if(eDisk_ReadBlock(read_block.whole_block, start_block)){
			OS_Signal(&File_Sema);
			return 1;
		}
		read_num = next_block;
		next_block = read_block.structured.next_block; //Grab the number of the next block before I clear the block
		memset(&read_block, 0, BLOCK_SIZE);  // Clears the block
		if(eDisk_WriteBlock(read_block.whole_block, read_num)){ //Write the cleared block back
			OS_Signal(&File_Sema);
			return 1;
		}
	}
	
	//Also need to update the directory
	if(eDisk_ReadBlock(directory.info, dir_block)){ //Read in the directory
		OS_Signal(&File_Sema);
		return 1;
	}
	for(uint16_t i = 0; i < MAX_FILES; i++){
		if(!strcmp(directory.table[i].name, n) && directory.table[i].in_use){ //strcmp return 0 if no difference in strings
			directory.table[i].in_use = 0;
			strcpy(directory.table[i].name, "");
			directory.table[i].starting_block = 0;
			OS_Signal(&File_Sema);
			break;
		}
	}
	if(eDisk_WriteBlock(directory.info, dir_block)){ //Write the directory back
		OS_Signal(&File_Sema);
		return 1;
	}
	
	OS_Signal(&File_Sema);
	return 0;
}                             


//---------- eFile_DOpen-----------------
// Open a (sub)directory, read into RAM
// Input: directory name is an ASCII string up to seven characters
//        (empty/NULL for root directory)
// Output: 0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_DOpen( const char name[]){ // open directory
	OS_Wait(&File_Sema);
  if(eDisk_ReadBlock(directory.info, dir_block)){ //Read in the directory
		OS_Signal(&File_Sema);
		return 1;
	}
	curr_file = 0;
	OS_Signal(&File_Sema);
  return 0;   // replace
}
  
//---------- eFile_DirNext-----------------
// Retreive directory entry from open directory
// Input: none
// Output: return file name and size by reference
//         0 if successful and 1 on failure (e.g., end of directory)
int eFile_DirNext( char *name[], unsigned long *size){  // get next entry 
	OS_Wait(&File_Sema);
	
	uint8_t num_files = get_num_files();
	
	if(num_files == 0){
		OS_Signal(&File_Sema);
		return 1;
	}

	if(curr_file > num_files){
		OS_Signal(&File_Sema);
		return 1;
	}
  
	*name = directory.table[curr_file].name;
	
	//size is BLOCK_SIZE * number of blocks
	unsigned long count = 0;
	
	uint32_t start_block;
	if(find_starting_block(*name, &start_block)){
		OS_Signal(&File_Sema);
		return 1; //Some sort of error occurred
	}
	
	//Cycle through all the blocks
	uint32_t next_block = start_block;
	while(next_block != 0){
		if(eDisk_ReadBlock(read_block.whole_block, next_block)){
			OS_Signal(&File_Sema);
			return 1;
		}
		read_num = next_block;
		next_block = read_block.structured.next_block; //Grab the number of the next block before I clear the block
		count++;
	}
	
	*size = count;
	
	curr_file++;
	OS_Signal(&File_Sema);
  return 0;   // replace
}

//---------- eFile_DClose-----------------
// Close the directory
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_DClose(void){ // close the directory
	if(curr_file == -1 || dir_block == -1){
		return -1;
	}
  curr_file = 0;
  return 0;   // replace
}


//---------- eFile_Unmount-----------------
// Unmount and deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently mounted)
int eFile_Unmount(void){ 
  if(dir_block != 0){
		return 1; //Not currently mounted
	} else{
		dir_block = -1;
		return 0;
	}
}

//Finds the first unused block of memory
//Returns by value 0 on success and 1 on failure
//Returns by pointer the number of the first found unused block
uint8_t find_unused_block(uint32_t* block_num){
	Block_u temp_block;
	for(uint32_t i = 1; i < MAX_BLOCKS; i++){
		if(eDisk_ReadBlock(temp_block.whole_block, i)){
			return 1; //Trouble reading disk
		}
		if(!temp_block.structured.in_use){ //Found an unused block
			*block_num = i;
			break;
		}
	}
	return 0;
}

uint8_t get_num_files(){
	uint8_t count = 0;
	for(uint16_t i = 0; i < MAX_FILES; i++){
		if(directory.table[i].in_use){ //strcmp return 0 if no difference in strings			
			count++;
		}
	}
	return count;
}



//Finds the starting block of the requested file name
//Returns by value 0 on success and 1 on failure
//Returns by pointer the number of the starting block of the file
uint8_t find_starting_block(const char n[], uint32_t* ret_num){
	if(eDisk_ReadBlock(directory.info, dir_block)){ //Read in the directory
		return 1;
	}
	
	uint32_t start_block = MAX_BLOCKS + 1; //Assume out of bounds
	for(uint16_t i = 0; i < MAX_FILES; i++){
		//printf("%s\n", directory.table[1].name);
		if(!strcmp(directory.table[i].name, n) && directory.table[i].in_use){ //strcmp return 0 if no difference in strings
			start_block = directory.table[i].starting_block; //Found the associated entry and req. start block
			*ret_num = start_block;
			break;
		}
	}
	if(start_block > MAX_BLOCKS){
		return 1; //Didnt find file entry associated with requested file name
	}
	
	return 0;
}

