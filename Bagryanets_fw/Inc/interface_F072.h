/*
 * uart_ezh.h
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#ifndef INC_INTERFACE_H_
#define INC_INTERFACE_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
//#include <stdlib.h>
#include <cstring>
#include <cctype>
//
#include <lib_F072.h>
//
#include <gpio_F072.h>
#include <rcc_F072.h>

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

class Uart_t {
protected:
	USART_TypeDef* Usart;
public:
	void Init(USART_TypeDef* _Usart, GPIO_TypeDef* GpioTx, uint8_t PinTx,
			GPIO_TypeDef* GpioRx, uint8_t PinRx,
			AltFunction_t Af, uint32_t Bod);
	void Enable() { Usart->CR1 |= USART_CR1_UE; }
	void Disable() { Usart->CR1 &= ~USART_CR1_UE; }
	void EnableCharMatch(char CharForMatch, uint32_t prio = 0);
	void DisableCharMatch(void) { Usart->CR1 &= USART_CR1_CMIE; }
	void EnableDmaRequest();
	void TxByte(uint8_t data);
	uint8_t RxByte(uint8_t* fl = NULL, uint32_t timeout = 0xFFFF);
	uint8_t IrqHandler();
}; //Uart_t end

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

//DMA buffers sizes
#define TX_BUFFER_SIZE 		(256UL)
#define RX_BUFFER_SIZE 		(256UL)

// Memory to peripheral DMA TX channel
class DmaTx_t{
protected:
	uint8_t Buffer[TX_BUFFER_SIZE];
	uint32_t BufferStartPtr;
	uint32_t BufferEndPtr;
	DMA_Channel_TypeDef* Channel;
public:
	void Init(DMA_Channel_TypeDef* _Channel, uint32_t PeriphRegAdr, uint32_t prio = 0);
	uint8_t Start();
	uint32_t GetNumberOfBytesInBuffer();
	uint32_t CheckStatus(); //0 - disable
	uint8_t WriteToBuffer(uint8_t data);
	uint8_t IrqHandler();
}; //DmaTx_t end

// Peripheral to memory DMA RX channel
class DmaRx_t {
protected:
	uint8_t Buffer[RX_BUFFER_SIZE];
	uint32_t BufferStartPtr;
	uint32_t GetBufferEndPtr();
	DMA_Channel_TypeDef* Channel;
public:
	void Init(DMA_Channel_TypeDef* _Channel, uint32_t PeriphRegAdr, uint32_t prio = 0);
	void Start();
	void Stop();
	uint32_t GetNumberOfBytesInBuffer();
	uint32_t CheckStatus(); //0 - disable
	uint8_t ReadFromBuffer();
//	uint8_t IrqHandler();
}; //DmaRx_t end

//Cli_t - Simple command line interface
/////////////////////////////////////////////////////////////////////

#define COMMAND_BUFFER_SIZE (128UL)
#define ARG_BUFFER_SIZE (10UL)

class Cli_t {
protected:
	DmaTx_t* TxChannel;
	DmaRx_t* RxChannel;
public:
	Cli_t(DmaTx_t* _TxChannel, DmaRx_t* _RxChannel) {
		TxChannel = _TxChannel;
		RxChannel = _RxChannel;
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
	if (ch == DMA1_Channel1)
		return DMA1_Channel1_IRQn;
	else if ((ch == DMA1_Channel2) or (ch == DMA1_Channel3))
		return DMA1_Channel2_3_IRQn;
	else if ((ch == DMA1_Channel4) or (ch == DMA1_Channel5) or (ch == DMA1_Channel6) or (ch == DMA1_Channel7))
		return DMA1_Channel4_5_6_7_IRQn;

	else
		ASSERT_SIMPLE(0); //Bad DMA channel name
} //ReturnIRQNum_DMA end

constexpr IRQn_Type ReturnIrqVectorUsart(USART_TypeDef* ch){
	if (ch == USART1)
		return USART1_IRQn;
	else if (ch == USART2)
		return USART2_IRQn;
	else if ((ch == USART3) or (ch == USART4))
		return USART3_4_IRQn;
	else
		ASSERT_SIMPLE(0); //Bad USART name
} //ReturnIRQNum_USART end

#endif /* INC_INTERFACE_H_ */
