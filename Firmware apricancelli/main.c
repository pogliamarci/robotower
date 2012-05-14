/*
 * Gate opener output reader firmware for STM32F100RB,
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "libusart.h"
#include "libperiferiche.h"

#define BUFFER_SIZE 30
#define FACTORY_NUMBER 3
#define TRUE 1
#define FALSE 0

void delay(uint32_t cnt) 
{
	while (cnt) 
	{
		cnt--;
	}
}

int main(void) 
{
	int i;
	int cicla = 1;
	
	int num_factory = FACTORY_NUMBER;
	int num_tower = 1;
	int destroyed[] = {FALSE,FALSE,FALSE};
	int destroyedT = 0;
	
	char command_buffer[BUFFER_SIZE];
	char status_buffer[BUFFER_SIZE];
	/* initialize buffers */
	for(i=0; i<BUFFER_SIZE; i++)
	{
		command_buffer[i] = '\0';
		status_buffer[i] = '\0';
	}
	
	/* My welcome message */
	 //TODO cancellare lo schifo per sonar
	print_string_usart1("Firmware per leggere le uscite di un apricancelli. Davide Tateo, 2012\n\r");
	
	/* Main application loop */
	while(cicla)
	{
		/* control if a factory has been destroyed */		
		for(i = 0; i < FACTORY_NUMBER; i++)
		{
			if( (GPIOA->IDR & (1 << (i+1)) == 0) && !destroyed[i] )
			{
				print_string_usart1("Factory Destroyed\n\r"); //TODO: togliere schifo per screen
				if(num_factory>0)
				{
					destroyed[i] = TRUE;
					num_factory--;
				}
			}
		}
		
		/* control if the tower has been destroyed */		
		if(((GPIOA->IDR & (1 << 4)) == 0) && !destroyedT)
		{
			print_string_usart1("Tower Destroyed\n\r"); //TODO: togliere schifo per screen
			if(num_tower>0)
			{
				destroyedT = TRUE;
				num_tower--;
			}
		}
		
		/* input processing */
		if(arrived_char_usart1())
		{
			read_string_usart1(command_buffer,'\r', BUFFER_SIZE); //TODO: togliere schifo per screen
			
			if(strcmp(command_buffer, "status") == 0)
			{
				sprintf(status_buffer, "#F=%d; #D=%d\n\r", num_factory, num_tower); //TODO: togliere schifo per screen
				print_string_usart1(status_buffer);
			}
			else if(strcmp(command_buffer, "reset") == 0)
			{
				num_factory = 3;
				num_tower = 1;
				for(i=0; i < FACTORY_NUMBER; i++) destroyed[i] = FALSE;
				destroyedT = FALSE;
			}
			else if(strcmp(command_buffer, "stop") == 0)
			{
				cicla = FALSE;
				print_string_usart1("Exit the main loop...\n\r"); //TODO: togliere schifo per screen
			}
			else
			{
				print_string_usart1("Command not found\n\r"); //TODO: togliere schifo per screen
			}
		}
		
	}
	
	return 0;
	
}
