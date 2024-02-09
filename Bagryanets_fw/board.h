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

//
#define LED_B	PA5

// UART shell parameters
#define UART_PARAMS			USART1, PA9, PA10, AF1, 9600
#define UART_DMA_TX		DMA1_Channel2, (uint32_t)&USART1->TDR
#define UART_DMA_RX		DMA1_Channel3, (uint32_t)&USART1->RDR

#endif /* BOARD_H_ */
