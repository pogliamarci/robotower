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
  
void SystemInit(void);
void clock_init(void);
void gpioA_init(void);
void gpioB_init(void);
void gpioC_init(void);
void usart1_init(void);
