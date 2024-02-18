/*
 * board.h
 *
 *  Created on: Feb 6, 2024
 *      Author: KONSTANTIN
 */

#ifndef BOARD_H_
#define BOARD_H_

//#define USE_SYSTICK_DELAY 1
#define HSE_FREQ_HZ 12000000

//ADC channels
#define ADC_IR_1 	PA1
#define ADC_IR_2 	PA2
#define ADC_BAT 	PB0

#define ADC_DMA_RX	DMA1_Channel1, (uint32_t)&ADC1->DR

// RGB led
#define LED_G	PC6
#define LED_R	PC7
#define LED_B	PC8
// IR led
#define LED_IR 	PA8

// UART shell parameters
#define UART_PARAMS		USART1, PA9, PA10, AF1, 115200
#define UART_DMA_TX		DMA1_Channel2, (uint32_t)&USART1->TDR
#define UART_DMA_RX		DMA1_Channel3, (uint32_t)&USART1->RDR

#endif /* BOARD_H_ */
