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
 
#include "stm32f10x.h"
#include "libperiferiche.h"
 
void SystemInit(void) 
{
	/* Initialize clock*/
	clock_init();
	/* Initialize GPIO */
	gpioA_init();
	gpioB_init();
	gpioC_init();
	/* Initialize USART */
	usart1_init();
}
 
void clock_init(void) 
{
	/* Activate HSE. */
	RCC->CR |= RCC_CR_HSEON;
	/* Wait until HSE is stable. */
	while (!(RCC->CR & RCC_CR_HSERDY));
	/* PLL setup. */
	RCC->CFGR |= RCC_CFGR_PLLMULL3 | RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1;
	/* Activate PLL. */
	RCC->CR   |= RCC_CR_PLLON;
	/* Wait until PLL is stable. */
	while (!(RCC->CR & RCC_CR_PLLRDY));
	/* AHB, APB1 and APB2 prescaler setup. */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1;
	/* FLASH wait states setup. */
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
	/* Set PLL as system clock */
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	/* Wait until PLL is the system clock. */
	while (!(RCC->CFGR & RCC_CFGR_SWS_PLL ));
}

void gpioA_init(void)
{
	/* Enable GPIOA clock */
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	/* Configure PA9 as alternate function push-pull output (USART_TX) */
	/* Configure PA10 as alternate function floating input (USART1_RX) */
	GPIOA->CRH = 0x888884B8;
	/* Configure PA4, PA3, PA2, PA1 as pull-up inputs */
	GPIOA->CRL = (0x08 << (1 * 4)) | (0x08 << (2 * 4)) | (0x08 << (3 * 4)) | (0x08 << (4 * 4));
	GPIOA->ODR = (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1);
}

void gpioB_init(void)
{
	/* Activate GPIOB clock. */
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	/* Configure PB2, PB1 as pull-down inputs */
	GPIOB->CRL = (0x08 << (2 * 4)) | (0x08 << (1 * 4));
	GPIOB->ODR = (0 << 2) || (0 << 1);
}

void gpioC_init(void)
{
	/* Activate GPIOC clock. */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	/* Configure PC8 and PC9 as push-pull outputs */
	GPIOC->CRH = (0x03 << (1 * 4)) | (0x03 << (0 * 4));
}


void usart1_init(void)
{
	/* Activate USART1, GPIOA and AFIO clock. */
	RCC->APB2ENR |= (RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);
	/* USART1 configuration registers reset. */
	USART1->CR1 = 0;
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	/* Turn on USART1. */
	USART1->CR1 |= USART_CR1_UE;
	/* Configure word length to 8 bit. */
	USART1->CR1 &= ~USART_CR1_M;
	/* Configure stop bits to 1 stop bit. */
	USART1->CR2 &= ~USART_CR2_STOP;
	/* Configure baud rate register for 115200 bps. */
	USART1->BRR = (208);
	/* Enable USART1 TX. */
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
}
