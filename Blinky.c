/*------------------------------------------------------------------------------
 * Copyright (c) 2004-2020 Arm Limited (or its affiliates). All
 * rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   3.Neither the name of Arm nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *------------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher for MCB1700
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "Blinky.h"
#include "LPC17xx.h"                    // Device header
#include "Board_LED.h"                  // ::Board Support:LED
#include "Board_ADC.h"                  // ::Board Support:A/D Converter
#include "Board_Joystick.h"             // ::Board Support:Joystick


static char text[10];
static char joystick_val[10];

// variable to trace in LogicAnalyzer (should not read to often)
static volatile uint16_t AD_dbg;

uint16_t ADC_last;                      // Last converted value

/*------------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
int main (void) {
  int32_t  res;
  uint32_t AD_sum   = 0U;
  uint32_t AD_cnt   = 0U;
  uint32_t AD_value = 0U;
  uint32_t AD_print = 0U;

  LED_Initialize();                      // LED Initialization
  ADC_Initialize();                      // ADC Initialization
	Joystick_Initialize();                 // Joystick Initialization

	LPC_SC->PCONP		|= (1 << 15); 				 // Powering Up Joystick
	
	/* P1.20, P1.23..26 is GPIO (Joystick) */
	LPC_PINCON->PINSEL3 &= ~((3<< 8) | (3<< 14) | (3<< 16) | (3<< 18) | (3<< 20));
	
	/* P1.20, P1.23..26 ids input */
	LPC_GPIO1->FIODIR 	&= ~((1<<20) | (1<<23) | (1<<24) | (1<<25) | (1<<26));
	
	
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock/100U);  // Generate interrupt each 10 ms
							 
	printf("Joystick Initial Position: %d\r\n", Joystick_GetState()); // Printing initial position of joystick
	
  while (1) {                            // Loop forever
    
		//
		/*
    // AD converter input
    res = ADC_GetValue();
    if (res != -1) {                     // If conversion has finished
      ADC_last = (uint16_t)res;
      
      AD_sum += ADC_last;                // Add AD value to sum
      if (++AD_cnt == 16U) {             // average over 16 values
        AD_cnt = 0U;
        AD_value = AD_sum >> 4;          // average devided by 16
        AD_sum = 0U;
      }
    }

    if (AD_value != AD_print) {
      AD_print = AD_value;               // Get unscaled value for printout
      AD_dbg   = (uint16_t)AD_value;

      sprintf(text, "0x%04X", AD_value); // format text for print out
    }
*/
    // Print message with AD value every second 
    if (clock_1s) {
      clock_1s = 0;


     // printf("AD value: %s\r\n", text);
			
			//sprintf(joystick_val, "%d", Joystick_GetState());
			//printf("Current Position of Joystick: %s\r\n", joystick_val);
			
			switch(Joystick_GetState())
			{
				case 0:
					printf("Current Position of Joystick: No Direction and No Select\n");
				break;
				
				case 8:
					printf("Current Position of Joystick: Up\n");
				break;
				
				case 2:
					printf("Current Position of Joystick: Right\n");
				break;
				
				case 16:
					printf("Current Position of Joystick: Down\n");
				break;
				
				case 1:
					printf("Current Position of Joystick: Left\n");
				break;
				
				case 4:
					printf("Current Position of Joystick: Select\n");
				break;
				
				case 10:
					printf("Current Position of Joystick: Up-Right\n");
				break;
				
				case 9:
					printf("Current Position of Joystick: Up-Left\n");
				break;
				
				case 17:
					printf("Current Position of Joystick: Down-Left\n");
				break;
				
				case 18:
					printf("Current Position of Joystick: Down-Right\n");
				break;
				
				case 12:
					printf("Current Position of Joystick: Select-Up\n");
				break;
				
				case 6:
					printf("Current Position of Joystick: Select-Right\n");
				break;
				
				case 20:
					printf("Current Position of Joystick: Select-Down\n");
				break;
				
				case 5:
					printf("Current Position of Joystick: Select-Left\n");
				break;
				
				case 14:
					printf("Current Position of Joystick: Select-Up-Right\n");
				break;
				
				case 13:
					printf("Current Position of Joystick: Select-Up-Left\n");
				break;
				
				case 21:
					printf("Current Position of Joystick: Select-Down-Left\n");
				break;
				
				case 22:
					printf("Current Position of Joystick: Select-Down-Right\n");
				break;
				
				default:
					printf("Physically Impossible combination, try using non-opposing positions\n");
				
			}
			
    }
  }
}
