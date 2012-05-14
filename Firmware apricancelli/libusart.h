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
 
int arrived_char_usart1(void);
int send_char_usart1(void);
char read_char_usart1(void);
void read_string_usart1(char* buffer, char terminator, int buffer_lenght);
void print_char_usart1(char c);
void print_string_usart1(char* stringa);

