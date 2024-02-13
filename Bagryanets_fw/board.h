/*
 * board.h
 *
 *  Created on: Feb 6, 2024
 *      Author: KONSTANTIN
 */

#ifndef BOARD_H_
#define BOARD_H_

#define USE_SYSTICK_DELAY 1
#define HSE_FREQ_HZ 12000000

// RGB led
#define LED_B	PC6
#define LED_R	PC7
#define LED_G	PC8
// IR led
#define LED_IR 	PA8

// UART shell parameters
#define UART_PARAMS		USART1, PA9, PA10, AF1, 9600
#define UART_DMA_TX		DMA1_Channel2, (uint32_t)&USART1->TDR
#define UART_DMA_RX		DMA1_Channel3, (uint32_t)&USART1->RDR

#endif /* BOARD_H_ */
