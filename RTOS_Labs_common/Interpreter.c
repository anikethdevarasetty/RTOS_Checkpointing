// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/ADCSWTrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include <stdbool.h>

extern uint32_t system_ms_time;
extern int32_t ADCdata;
extern uint32_t NumCreated;   // number of foreground threads created
extern uint32_t PIDWork;      // current number of PID calculations finished
extern uint32_t FilterWork;   // number of digital filter calculations finished
extern uint32_t NumSamples;   // incremented every ADC sample, in Producer
extern int32_t MaxJitter;             // largest time jitter between interrupts in usec
extern uint32_t DataLost;     // data sent by Producer, but not received by Consumer


// Print jitter histogram
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
  // write this for Lab 3 (the latest)
	
}

#define MAX_CMD_SIZE 25     // max size for a parser command

static char uart_buffer[MAX_CMD_SIZE + 2] = {0};
static uint32_t uart_buffer_index = 0;
static char cmd_buffer[MAX_CMD_SIZE + 2] = {0};
static char param1_buffer[MAX_CMD_SIZE + 2] = {0};
static char param2_buffer[MAX_CMD_SIZE + 2] = {0};


bool str_equals(char* a, char* b) {
    while (*a != '\0' && *b != '\0') {
        if (*a != *b) {
            return false;
        }
        ++a;
        ++b;
    }
    if (*a != *b) {
        return false;
    }
    return true;
}

uint32_t str_to_uint(char* str) {
    uint32_t n = 0;
    uint32_t count = 0;
    while(*str != 0) {
        ++count;
        ++str;
    }
    --str;
    for(uint32_t i = 0, j = 1; i < count; ++i, --str) {
        if (*str >= '0' && *str <= '9') {
            n += (*str - '0') * j;
            j *= 10;
        }
    }
    return n;
}

// *********** Command line interpreter (shell) ************
void Interpreter(void){ 
  // write this  
   UART_OutString(
                "RTOS Interpreter Help:\n\r"
                "  usage: [help, h] cmd param\n\r"
                "  cmd: \n\r"
                "    lcd_top string integer\n\r"
                "    lcd_bottom string integer\n\r"
                "    adc_in integer\n\r"
                "    time\n\r"
            );

  char character;
    while (1) {
        
				UART_OutString("Enter command...\n\r");			
				do {
           
						// get a character from the input
            character = UART_InChar();
            // delete an input character
            if ((character == DEL || character == BS) && uart_buffer_index) {
                --uart_buffer_index;
                UART_OutChar(DEL);
            }
            // add a character to buffer
            else if (uart_buffer_index < MAX_CMD_SIZE) {
                uart_buffer[uart_buffer_index++] = character;
                UART_OutChar(character);
            }
        } while (character != CR);
        // null terminate the command
        uart_buffer[--uart_buffer_index] = '\0';
        uart_buffer_index = 0;
        UART_OutString("\n\r");

        // parse the command
        uint32_t i = 0;
        uint32_t cmd_index = 0;
        uint32_t param_index1 = 0;
        uint32_t param_index2 = 0;
        // overlook leading whitespace
        // 0x09, 0x0A, 0x0C, 0x0D, 0x20
        while ((uart_buffer[i] == 0x09 || uart_buffer[i] == 0x0A || uart_buffer[i] == 0x0C || uart_buffer[i] == 0x0D || uart_buffer[i] == 0x20) && uart_buffer[i] != '\0') {
            ++i;
        }
        // fill cmd
        while (uart_buffer[i] != 0x09 && uart_buffer[i] != 0x0A && uart_buffer[i] != 0x0C && uart_buffer[i] != 0x0D && uart_buffer[i] != 0x20 && uart_buffer[i] != '\0') {
            cmd_buffer[cmd_index++] = uart_buffer[i++];
        }
        // null terminate cmd
        cmd_buffer[cmd_index] = '\0';
        // overlook whitespace between cmd and param
        while ((uart_buffer[i] == 0x09 || uart_buffer[i] == 0x0A || uart_buffer[i] == 0x0C || uart_buffer[i] == 0x0D || uart_buffer[i] == 0x20) && uart_buffer[i] != '\0') {
            ++i;
        }
        // fill param
        while (uart_buffer[i] != 0x09 && uart_buffer[i] != 0x0A && uart_buffer[i] != 0x0C && uart_buffer[i] != 0x0D && uart_buffer[i] != 0x20 && uart_buffer[i] != '\0') {
            param1_buffer[param_index1++] = uart_buffer[i++];
        }
        // null terminate param
        param1_buffer[param_index1] = '\0';

        // overlook whitespace between cmd and param
        while ((uart_buffer[i] == 0x09 || uart_buffer[i] == 0x0A || uart_buffer[i] == 0x0C || uart_buffer[i] == 0x0D || uart_buffer[i] == 0x20) && uart_buffer[i] != '\0') {
            ++i;
        }
        // fill param
        while (uart_buffer[i] != 0x09 && uart_buffer[i] != 0x0A && uart_buffer[i] != 0x0C && uart_buffer[i] != 0x0D && uart_buffer[i] != 0x20 && uart_buffer[i] != '\0') {
            param2_buffer[param_index2++] = uart_buffer[i++];
        }
        // null terminate param
        param2_buffer[param_index2] = '\0';

        // execute cmd accordingly
        // lcd_top
        if (str_equals(cmd_buffer, "lcd_top")) {
            int line = str_to_uint(param2_buffer);
						ST7735_Message(0, line, param1_buffer, line);
        }
        // lcd_bottom
        else if (str_equals(cmd_buffer, "lcd_bottom")) {
          int line = str_to_uint(param2_buffer);
          ST7735_Message(1, line, param1_buffer, line);
        }
        // adc_in
        else if (str_equals(cmd_buffer, "adc_in")) {
          //ADC_Init(str_to_uint(param1_buffer));
          UART_OutSDec(ADCdata);
					UART_OutChar('\n');
        }
        // system time
        else if (str_equals(cmd_buffer, "time")) {
          UART_OutSDec(system_ms_time);
					UART_OutChar('\n');
        }
				else if(str_equals(cmd_buffer, "print")){
					UART_OutString("NumSamples\t\tNumCreated\t\tMaxJitter\t\tDataLost\t\tFilterWork\t\tPIDwork\n");
					UART_OutSDec(NumSamples);
					UART_OutString("\t\t");
					UART_OutSDec(NumCreated);
					UART_OutString("\t\t");
					UART_OutSDec(MaxJitter);
					UART_OutString("\t\t");
					UART_OutSDec(DataLost);
					UART_OutString("\t\t");
					UART_OutSDec(FilterWork);
					UART_OutString("\t\t");
					UART_OutSDec(PIDWork);
					UART_OutString("\t\t");
					UART_OutString("\n\n");
				}
        else if (str_equals(cmd_buffer, "help") || str_equals(cmd_buffer, "h")) {
            UART_OutString(
                "RTOS Interpreter Help:\n\r"
                "  usage: [help, h] cmd param\n\r"
                "  cmd: \n\r"	
                "    lcd_top string integer\n\r"
                "    lcd_bottom string integer\n\r"
                "    adc_in \n\r"
                "    time\n\r"
            );
        }
        // invalid command
        else {
            UART_OutString("Invalid command...\n\r");
        }
}
}


