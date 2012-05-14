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
 
#include "libusart.h" 
#include "stm32f10x.h"

int arrived_char_usart1(void)
{
	return (USART1->SR & USART_SR_RXNE);
}

int send_char_usart1(void)
{
	return USART1->SR & USART_SR_TC;
}

char read_char_usart1(void)
{
	char c;
	/* Wait for a character. */
	while (!arrived_char_usart1());
	/* Read received character. */
	c = USART1->DR;
	return c;
}

void read_string_usart1(char* buffer, char terminator, int buffer_lenght)
{
	int i;
	char c = (terminator == '\0') ? ' ' : '\0';
	for(i=0; i < buffer_lenght && c != terminator; i++)
	{
		c = read_char_usart1();
		buffer[i] = c;
	}
	buffer[i-1] = '\0';
}

void print_char_usart1(char c)
{
	USART1->DR = c;
	while (!send_char_usart1());
}

void print_string_usart1(char* stringa)
{
		int i;
		char c=' ';			
		for(i=0; c!='\0'; i++)
		{
			c=stringa[i];
			print_char_usart1(c);
		}
}
