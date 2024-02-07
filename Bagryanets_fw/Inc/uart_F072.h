/*
 * uart_ezh.h
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#include <cctype>
//
#include <lib_F072.h>
//
#include <gpio_F072.h>
#include <rcc_F072.h>

#define TX_BUFFER_SIZE 		(1024UL)
#define RX_BUFFER_SIZE 		(1024UL)
//Remade but not today, same for all UARTs
#define DMA_TX_REQUEST 		0b0010
#define DMA_RX_REQUEST 		0b0010

//UartBase_t - implementation of base UART functions
/////////////////////////////////////////////////////////////////////
/*
* Need IRQ Handler wrapper for correct operation with character match
* Wrapper example
*
UartBase_t UartName(UART_PARAMS);
extern "C"
void UARTx_IRQHandler_IRQHandler(){ -
	UartName.UartIrqHandler();
}
*/

class UartBase_t {
protected:
	USART_TypeDef* Usart;
	GPIO_TypeDef* GpioTx;
	uint8_t PinTx;
	GPIO_TypeDef* GpioRx;
	uint8_t PinRx;
public:
	UartBase_t(USART_TypeDef* _USART,
	GPIO_TypeDef* _GPIO_TX, uint8_t _PIN_TX,
	GPIO_TypeDef* _GPIO_RX, uint8_t _PIN_RX) {
		Usart = _USART;
		GpioTx = _GPIO_TX;
		PinTx = _PIN_TX;
		GpioRx = _GPIO_RX;
		PinRx = _PIN_RX;
	}
	void Init(uint32_t _Bod);
	void Enable() { Usart->CR1 |= USART_CR1_UE; }
	void Disable() { Usart->CR1 &= ~USART_CR1_UE; }
	//
	void EnableCharMatch(char CharForMatch, uint32_t prio = 0);
	void UartIrqHandler();
	void DisableCharMatch(void) { Usart->CR1 &= USART_CR1_CMIE; }
	void TxByte(uint8_t data);
	uint8_t RxByte(uint8_t* fl = NULL, uint32_t timeout = 0xFFFF);
	//
}; //UartBase_t end

//UartDma_t - enables capability to transmit and receive data through DMA
/////////////////////////////////////////////////////////////////////
/*
* Need IRQ Handler wrapper for correct operation
* Wrapper example
*
UartDma_t UartName(UART_PARAMS, DMA_UART_TX_CHANNEL, DMA_UART_RX_CHANNEL);
extern "C" {
void DMAx_Channelx_IRQHandler(){ -
	UartName.DmaIrqHandler();
}
void DMAx_Channelx_IRQHandler(){ -
	UartName.DmaIrqHandler();
} }
*/

class UartDma_t : public UartBase_t {
protected:
	uint8_t TxBuffer[TX_BUFFER_SIZE];
	uint32_t TxBufferStartPtr;
	uint32_t TxBufferEndPtr;
	uint8_t RxBuffer[RX_BUFFER_SIZE];
	uint32_t RxBufferStartPtr;
	uint32_t GetRxBufferEndPtr();
	DMA_Channel_TypeDef* DmaTxChannel;
	DMA_Channel_TypeDef* DmaRxChannel;
public:
	UartDma_t(USART_TypeDef* _USART,
	GPIO_TypeDef* _GPIO_TX, uint8_t _PIN_TX,
	GPIO_TypeDef* _GPIO_RX, uint8_t _PIN_RX,
	DMA_Channel_TypeDef* _DmaTxChannel, DMA_Channel_TypeDef* _DmaRxChannel):
		UartBase_t(_USART, _GPIO_TX, _PIN_TX, _GPIO_RX, _PIN_RX) {
		DmaTxChannel = _DmaTxChannel;
		DmaRxChannel = _DmaRxChannel;
		TxBufferStartPtr = 0;
		RxBufferStartPtr = 0;
		TxBufferEndPtr = 0;
	}
	void InitDmaTx(uint32_t prio = 0);
	void InitDmaRx();
	uint8_t StartDmaTxIfNotYet();
	void StartDmaRx();
	void StopDmaRx(void) { DmaRxChannel -> CCR &= ~DMA_CCR_EN; }
	uint32_t GetNumberOfBytesInRxBuffer();
	uint32_t GetNumberOfBytesInTxBuffer();
	uint32_t CheckDmaStatus(DMA_Channel_TypeDef* DmaChannel); //0 - disable
	//
	uint8_t WriteToBuffer(uint8_t data);
	uint8_t ReadFromBuffer();
	void DmaIrqHandler(); //Only TX IRQ implemented
}; //UartDma_t end

//typedef enum {
//} DmaIrqRetv_t;

//UartCli_t - provides simple command line interface
/////////////////////////////////////////////////////////////////////

#define COMMAND_BUFFER_SIZE (128)
#define ARG_BUFFER_SIZE (128)

class UartCli_t {
private:
	UartDma_t* Channel;
public:
	UartCli_t(UartDma_t* _Channel) {
		Channel = _Channel;
	}
	char CommandBuffer[COMMAND_BUFFER_SIZE];
	char ArgBuffer[COMMAND_BUFFER_SIZE];
	uint8_t EchoEnabled = 1;
	//Methods
	void Clear() { CommandBuffer[0] = '\0'; ArgBuffer[0] = '\0';}
	void Echo();
	void PrintBinaryString(uint32_t binary);
	void SimplePrint(const char* text);
	void Printf(const char* text, ...);
	char* Read();
	char* ReadLine();
	void ReadCommand(); //Read command with argument if exist, put in buffer
	//
	void PrintBusFrequencies();
};

/////////////////////////////////////////////////////////////////////

constexpr uint32_t ReturnChannelNumberDma(DMA_Channel_TypeDef* ch){
	if(ch == DMA1_Channel1)
		return 1;
	else if(ch == DMA1_Channel2)
		return 2;
	else if(ch == DMA1_Channel3)
		return 3;
	else if(ch == DMA1_Channel4)
		return 4;
	else if(ch == DMA1_Channel5)
		return 5;
	else if(ch == DMA1_Channel6)
		return 6;
	else if(ch == DMA1_Channel7)
		return 7;
	else
		ASSERT_SIMPLE(0); //Bad DMA channel name
} //ReturnChNum_DMA end

constexpr IRQn_Type ReturnIrqVectorDma(DMA_Channel_TypeDef* ch){
	if(ch == DMA1_Channel1)
		return DMA1_Channel1_IRQn;
	else if(ch == DMA1_Channel2)
		return DMA1_Channel2_IRQn;
	else if(ch == DMA1_Channel3)
		return DMA1_Channel3_IRQn;
	else if(ch == DMA1_Channel4)
		return DMA1_Channel4_IRQn;
	else if(ch == DMA1_Channel5)
		return DMA1_Channel5_IRQn;
	else if(ch == DMA1_Channel6)
		return DMA1_Channel6_IRQn;
	else if(ch == DMA1_Channel7)
		return DMA1_Channel7_IRQn;
	else
		ASSERT_SIMPLE(0); //Bad DMA channel name
} //ReturnIRQNum_DMA end

constexpr IRQn_Type ReturnIrqVectorUsart(USART_TypeDef* ch){
	if(ch == USART1)
		return USART1_IRQn;
	else if(ch == USART2)
		return USART2_IRQn;
	else if(ch == USART3)
		return USART3_IRQn;
	else if(ch == UART4)
		return UART4_IRQn;
	else if(ch == UART5)
		return UART5_IRQn;
	else
		ASSERT_SIMPLE(0); //Bad USART name
} //ReturnIRQNum_USART end

#endif /* INC_UART_H_ */
