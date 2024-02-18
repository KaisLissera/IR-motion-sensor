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
	void UpdateBaudrate(uint32_t Bod);
	inline void Enable() { Usart->CR1 |= USART_CR1_UE; }
	inline void Disable() { Usart->CR1 &= ~USART_CR1_UE; }
	void EnableCharMatch(char CharForMatch, uint32_t prio = 0);
	inline void DisableCharMatch(void) { Usart->CR1 &= USART_CR1_CMIE; }
	void EnableDmaRequest();
	void TxByte(uint8_t data);
	uint8_t RxByte(uint8_t* fl = NULL, uint32_t timeout = 0xFFFF);
	uint8_t IrqHandler(); // Use if character match enabled
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

typedef enum {
	lowChPrio 		= 0b00,
	mediumChPrio 	= 0b01,
	highChPrio 		= 0b10,
	veryhighChPrio 	= 0b11
} DmaChPrio_t;

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
	void Init(DMA_Channel_TypeDef* _Channel, uint32_t PeriphRegAdr,
			uint8_t DmaIrqPrio = 0, DmaChPrio_t ChPrio = lowChPrio);
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
	void Init(DMA_Channel_TypeDef* _Channel, uint32_t PeriphRegAdr,
			uint8_t DmaIrqPrio = 0, DmaChPrio_t ChPrio = lowChPrio);
	void Start();
	inline void Stop();
	uint32_t GetNumberOfBytesInBuffer();
	uint32_t CheckStatus(); //0 - disable
	uint8_t ReadFromBuffer();
//	uint8_t IrqHandler();
}; //DmaRx_t end

/////////////////////////////////////////////////////////////////////
// Memory to peripheral DMA TX channel
/*
template <typename T>
class DmaTxTest_t{
protected:
	T Buffer[TX_BUFFER_SIZE];
	uint32_t BufferStartPtr;
	uint32_t BufferEndPtr;
	DMA_Channel_TypeDef* Channel;
public:
	void Init(DMA_Channel_TypeDef* _Channel, uint32_t PeriphRegAdr, uint8_t DataSize = 8,
			uint8_t DmaIrqPrio = 15, DmaChPrio_t ChPrio = lowChPrio) {
		Channel = _Channel;
		BufferStartPtr = 0;
		BufferEndPtr = 0;

		rcc::EnableClkDMA();
		Channel -> CCR =  DMA_CCR_MINC | DMA_CCR_CIRC; // Memory increment, circular mode
		Channel -> CCR |= ChPrio << DMA_CCR_PL_Pos; // DMA channel priority
		Channel -> CCR |= DMA_CCR_DIR_Msk; // 1 - Read from memory
		// Memory and peripheral sizes
		if(DataSize == 8)
			Channel -> CCR |= (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos); // 8 bit
		else if(DataSize == 8)
			Channel -> CCR |= (0b01 << DMA_CCR_MSIZE_Pos) | (0b01 << DMA_CCR_PSIZE_Pos); // 16 bit
		else
			Channel -> CCR |= (0b10 << DMA_CCR_MSIZE_Pos) | (0b10 << DMA_CCR_PSIZE_Pos); // 16 bit
		nvic::SetupIrq(ReturnIrqVectorDma(Channel), DmaIrqPrio);
		Channel -> CCR |= DMA_CCR_TCIE;
		Channel -> CPAR = PeriphRegAdr; //Peripheral register
	}
	uint8_t Start() {
		if(CheckStatus() != 0)
			return retvBusy; //Nothing changes if DMA already running

		if(BufferStartPtr == BufferEndPtr)
			return retvEmpty; //Nothing changes if Buffer Empty

		uint32_t NumberOfBytesReady;
		if (BufferEndPtr > BufferStartPtr)
			NumberOfBytesReady = GetNumberOfBytesInBuffer();
		else
			NumberOfBytesReady = TX_BUFFER_SIZE - BufferStartPtr;
		Channel -> CNDTR = NumberOfBytesReady;
		Channel -> CMAR = (uint32_t)&Buffer[BufferStartPtr];
		BufferStartPtr = (BufferStartPtr + NumberOfBytesReady) % TX_BUFFER_SIZE;
		Channel -> CCR |= DMA_CCR_EN;

		return retvOk;
	}
	uint32_t GetNumberOfBytesInBuffer() {
		if (BufferEndPtr >= BufferStartPtr)
			return BufferEndPtr - BufferStartPtr;
		else
			return TX_BUFFER_SIZE - BufferStartPtr + BufferEndPtr;
	}
	uint32_t CheckStatus() {
		uint32_t temp = Channel -> CCR;
		return temp & DMA_CCR_EN;
	}
	uint8_t WriteToBuffer(T data) {
		uint32_t EndPtrTemp = (BufferEndPtr + 1) % TX_BUFFER_SIZE;
		if (EndPtrTemp == BufferStartPtr)
			return retvOutOfMemory;
		Buffer[BufferEndPtr] = data;
		BufferEndPtr = EndPtrTemp;
		return retvOk;
	}
	uint8_t IrqHandler() {
		uint8_t ChNum = ReturnChannelNumberDma(Channel);
		if(DMA1->ISR & DMA_ISR_TCIF1 << (ChNum - 1)){
			DMA1->IFCR = DMA_IFCR_CTCIF1 << (ChNum - 1);
			Channel -> CCR &= ~DMA_CCR_EN;
			if(BufferStartPtr != BufferEndPtr) {
				Start();
			}
			return retvOk;
		} else
			return retvFail;
	}
}; //DmaTxTest_t end*/

//Cli_t - Simple command line interface
/////////////////////////////////////////////////////////////////////

#define COMMAND_BUFFER_SIZE (128UL)
#define ARG_BUFFER_SIZE (10UL)

class Cli_t {
protected:
	DmaTx_t* TxChannel;
	DmaRx_t* RxChannel;
	void PutBinary(uint32_t binary);
	void PutString(const char* text);
public:
	Cli_t(DmaTx_t* _TxChannel, DmaRx_t* _RxChannel) {
		TxChannel = _TxChannel;
		RxChannel = _RxChannel;
	}
	char CommandBuffer[COMMAND_BUFFER_SIZE];
	char ArgBuffer[ARG_BUFFER_SIZE];
	uint8_t EchoEnabled = 1;
	//Methods
	void Clear() { CommandBuffer[0] = '\0'; ArgBuffer[0] = '\0';}
	void Echo();
	void Printf(const char* text, ...);
	char* Read();
	char* ReadLine();
	void ReadCommand(); //Read command with argument if exist, put in buffer
	//
	void PrintBusFrequencies();
};

#endif /* INC_INTERFACE_H_ */
