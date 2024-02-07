/*
 * uart_ezh.cpp
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#include <uart_F072.h>

//UartBase_t
/////////////////////////////////////////////////////////////////////

void UartBase_t::Init(uint32_t bod) {
	rcc::EnableClkUART(Usart);
	ezhgpio::SetupPin(GpioTx, PinTx, PullUp, AlternateFunction, AF7);
	ezhgpio::SetupPin(GpioRx, PinRx, NoPullUpDown, AlternateFunction, AF7);

	uint32_t ApbClock = rcc::GetCurrentAPBClock();
	Usart -> BRR = (uint32_t)ApbClock/bod;
	Usart -> CR3 |= USART_CR3_OVRDIS; //Disable overrun
	Usart -> CR1 |= USART_CR1_TE | USART_CR1_RE ; //UART TX RX enable
	Usart -> CR1 |= USART_CR1_UE; //UART enable
} //UartBase_t::Init

//Simple UART byte transmit, wait until fully transmitted
void UartBase_t::TxByte(uint8_t data) {
	while ((Usart -> ISR & USART_ISR_TXE) == 0) {}
	Usart -> TDR = data;
} //UartBase_t::Txbyte

//Simple UART byte receive, wait until receive
uint8_t UartBase_t::RxByte(uint8_t* fl, uint32_t timeout) {
	while ((Usart -> ISR & USART_ISR_RXNE) == 0) {
		timeout--;
		if(timeout == 0) {
			*fl = retvTimeout;
			return 0; // Can't receive data
		}
	}
	// Data received
	uint8_t data = Usart -> RDR;
	*fl = retvOk;
	return data;
} //UartBase_t::Rxbyte

//USART Must be disabled, this need IRQ handler
void UartBase_t::EnableCharMatch(char CharForMatch, uint32_t prio) {
	Disable();
	Usart -> CR2 |= (uint8_t)CharForMatch << USART_CR2_ADD_Pos;
	Usart -> CR1 |= USART_CR1_CMIE;
	nvic::SetupIrq(ReturnIrqVectorUsart(Usart), prio);
	Enable();
} //UartBase_t::EnableCF

void UartBase_t::UartIrqHandler() {
	Usart -> ICR = USART_ICR_CMCF;
};

//UartDma_t
/////////////////////////////////////////////////////////////////////

void UartDma_t::InitDmaTx(uint32_t prio) {
	rcc::EnableClkDMA(DMA1);
	uint32_t num = ReturnChannelNumberDma(DmaTxChannel);

	DMA1_CSELR -> CSELR &= ~(0b1111UL << (num - 1)*4); //Clean
	DMA1_CSELR -> CSELR |= DMA_TX_REQUEST << (num - 1)*4; //Select request source
	DmaTxChannel -> CCR = (0b11 << DMA_CCR_PL_Pos) | (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_DIR;
	Usart -> CR3 |= USART_CR3_DMAT; //USART DMA request enable
	//
	nvic::SetupIrq(ReturnIrqVectorDma(DmaTxChannel), prio);
	DmaTxChannel -> CCR |= DMA_CCR_TCIE;
	DmaTxChannel -> CPAR = (uint32_t)&(Usart -> TDR); //Peripheral register
} //UartBase_t::InitDMA_Tx

void UartDma_t::InitDmaRx() {
	rcc::EnableClkDMA(DMA1);
	uint32_t num = ReturnChannelNumberDma(DmaRxChannel);
	DMA1_CSELR -> CSELR &= ~(0b1111UL << (num - 1)*4); //Clean
	DMA1_CSELR -> CSELR |= DMA_RX_REQUEST << (num - 1)*4; //Select request source
	DmaRxChannel -> CCR = (0b11 << DMA_CCR_PL_Pos) | (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC;
	Usart -> CR3 |= USART_CR3_DMAR; //USART DMA request enable
	//
	DmaRxChannel -> CPAR = (uint32_t)&(Usart -> RDR); //Peripheral register
} //UartBase_t::InitDMA_Rx

uint8_t UartDma_t::StartDmaTxIfNotYet() {
	if(CheckDmaStatus(DmaTxChannel) != 0)
		return retvBusy; //Nothing changes if DMA already running

	if(TxBufferStartPtr == TxBufferEndPtr)
		return retvEmpty; //Nothing changes if Buffer Empty

	uint32_t NumberOfBytesReadyToTx;
	if (TxBufferEndPtr > TxBufferStartPtr)
		NumberOfBytesReadyToTx = GetNumberOfBytesInTxBuffer();
	else
		NumberOfBytesReadyToTx = TX_BUFFER_SIZE - TxBufferStartPtr;
	DmaTxChannel -> CNDTR = NumberOfBytesReadyToTx;
	DmaTxChannel -> CMAR = (uint32_t)&TxBuffer[TxBufferStartPtr];
	TxBufferStartPtr = (TxBufferStartPtr + NumberOfBytesReadyToTx) % TX_BUFFER_SIZE;
	DmaTxChannel -> CCR |= DMA_CCR_EN;

	return retvOk;
} //UartBase_t::EnableTxDMA_Numbers

uint32_t UartDma_t::CheckDmaStatus(DMA_Channel_TypeDef* DmaChannel) {
	uint32_t temp = DmaChannel -> CCR;
	return temp & DMA_CCR_EN;
}

void UartDma_t::StartDmaRx() {
	DmaRxChannel -> CNDTR = RX_BUFFER_SIZE;
	DmaRxChannel -> CMAR = (uint32_t)&RxBuffer[0];
	DmaRxChannel -> CCR |= DMA_CCR_EN;
} //UartBase_t::EnableRxDMA()

uint32_t UartDma_t::GetRxBufferEndPtr() {
	return RX_BUFFER_SIZE - (DmaRxChannel -> CNDTR);
}

uint32_t UartDma_t::GetNumberOfBytesInTxBuffer() {
	if (TxBufferEndPtr >= TxBufferStartPtr)
		return TxBufferEndPtr - TxBufferStartPtr;
	else
		return TX_BUFFER_SIZE - TxBufferStartPtr + TxBufferEndPtr;
}

uint32_t UartDma_t::GetNumberOfBytesInRxBuffer() {
	uint32_t RxBufferEndPtr = GetRxBufferEndPtr();
	if (RxBufferEndPtr >= RxBufferStartPtr)
		return RxBufferEndPtr - RxBufferStartPtr;
	else
		return RX_BUFFER_SIZE - RxBufferStartPtr + RxBufferEndPtr;
}

uint8_t UartDma_t::WriteToBuffer(uint8_t data) {
	uint32_t EndPtrTemp = (TxBufferEndPtr + 1) % TX_BUFFER_SIZE;
	if (EndPtrTemp == TxBufferStartPtr)
		return retvOutOfMemory;
	TxBuffer[TxBufferEndPtr] = data;
	TxBufferEndPtr = EndPtrTemp;
	return retvOk;
}

uint8_t UartDma_t::ReadFromBuffer() {
	uint8_t temp = RxBuffer[RxBufferStartPtr];
	RxBufferStartPtr = (RxBufferStartPtr + 1) % RX_BUFFER_SIZE;
	return temp;
}

void UartDma_t::DmaIrqHandler() {
	DMA1 -> IFCR = DMA_IFCR_CTCIF7;
	DmaTxChannel -> CCR &= ~DMA_CCR_EN;
	if(TxBufferStartPtr != TxBufferEndPtr) {
		StartDmaTxIfNotYet();
	}
}

//UartCli_t
/////////////////////////////////////////////////////////////////////

void UartCli_t::SimplePrint(const char* text) {
	uint8_t length = strlen(text);
	for(uint32_t i = 0; i < length; i++) {
		Channel->WriteToBuffer((uint8_t)text[i]);
	}
}

void UartCli_t::PrintBinaryString(uint32_t number) {
	char BinaryString[35] = "0b";
	uint32_t mask = 1UL << 31;
	for(uint8_t i = 0; i < 32; i++) {
		if((number & mask))
			BinaryString[2 + i] = '1';
		else
			BinaryString[2 + i] = '0';
		mask = mask >> 1;
	}
	BinaryString[2 + 32] = '\0';
	SimplePrint(BinaryString);
//	return BinaryString;
}

void UartCli_t::Printf(const char* text, ...) {
	// String format processing
	va_list args;
	va_start(args, text); // Start string processing
	uint8_t length = strlen(text);
	for(uint32_t i = 0; i < length; i++) {
		if(text[i] == '%'){ // if argument found
			i++;
			char IntArg[10] = "";
			switch(text[i]) {
				case 'd': // integer
					sprintf(IntArg, "%d",va_arg(args,int));
					SimplePrint(IntArg);
					break;
				case 'u': // unsigned integer
					sprintf(IntArg, "%u",va_arg(args,unsigned int));
					SimplePrint(IntArg);
					break;
				case 's': // string
					SimplePrint(va_arg(args,char*));
					break;
				case 'c': // char
					Channel->WriteToBuffer(va_arg(args,int));
					break;
				case 'b': // print uint32 as binary
					PrintBinaryString(va_arg(args,uint32_t));
					break;
				default:
					Channel->WriteToBuffer((uint8_t)'%');
					Channel->WriteToBuffer((uint8_t)text[i]);
			}
		} else
			Channel->WriteToBuffer((uint8_t)text[i]);
	}
	va_end(args); // End format processing
	//
	Channel->StartDmaTxIfNotYet();
}

char* UartCli_t::Read() {
	uint32_t CmdBufferPtr = 0;
	char temp = (char)Channel->ReadFromBuffer();
	//Clear buffer beginning from useless symbols
	while((temp == '\n') || (temp == ' ')) {
		temp = (char)Channel->ReadFromBuffer();
	}
	//Get command from buffer
	while((temp != '\r') && (temp != '\n') && (temp != ' ')) {
		CommandBuffer[CmdBufferPtr] = std::tolower(temp); //Char normalization
		CmdBufferPtr++;
		temp = (char)Channel->ReadFromBuffer();
	}
	//Add terminators to strings
	CommandBuffer[CmdBufferPtr] = '\0';
	ArgBuffer[0] = '\0';
	//
	if(EchoEnabled)
		Echo();
	return CommandBuffer;
}

char* UartCli_t::ReadLine() {
	uint32_t CmdBufferPtr = 0;
	char temp = (char)Channel->ReadFromBuffer();
	//Clear buffer beginning from useless symbols
	while((temp == '\n') || (temp == ' ')) {
		temp = (char)Channel->ReadFromBuffer();
	}
	//Get command from buffer
	while((temp != '\r') && (temp != '\n')) {
		CommandBuffer[CmdBufferPtr] = std::tolower(temp); //Char normalization
		CmdBufferPtr++;
		temp = (char)Channel->ReadFromBuffer();
	}
	//Add terminators to strings
	CommandBuffer[CmdBufferPtr] = '\0';
	ArgBuffer[0] = '\0';
	//
	if(EchoEnabled)
		Echo();
	return CommandBuffer;
}

void UartCli_t::ReadCommand() {
	uint32_t CmdBufferPtr = 0;
	char temp = (char)Channel->ReadFromBuffer();
	//Clear buffer beginning from useless symbols
	while((temp == '\n') || (temp == ' ')) {
		temp = (char)Channel->ReadFromBuffer();
	}
	//Get command from buffer
	while((temp != '\r') && (temp != '\n') && (temp != ' ') && (Channel->GetNumberOfBytesInRxBuffer() != 0)) {
		CommandBuffer[CmdBufferPtr] = std::tolower(temp); //Char normalization
		CmdBufferPtr++;
		temp = (char)Channel->ReadFromBuffer();
	}
	//Get argument from buffer if exist
	uint32_t ArgumentBufferPtr = 0;
	if (temp == ' ') {
		temp = (char)Channel->ReadFromBuffer();
		while((temp != '\r') && (temp != '\n') && (Channel->GetNumberOfBytesInRxBuffer() != 0)) {
			ArgBuffer[ArgumentBufferPtr] = temp;
			ArgumentBufferPtr++;
			temp = (char)Channel->ReadFromBuffer();
		}
	}
	//Add terminators to strings
	ArgBuffer[ArgumentBufferPtr] = '\0';
	CommandBuffer[CmdBufferPtr] = '\0';
	//
	if(EchoEnabled)
		Echo();
}

void UartCli_t::Echo() {
	if(ArgBuffer[0] != '\0')
		Printf("[ECHO] %s %s\n\r",CommandBuffer, ArgBuffer);
	else
		Printf("[ECHO] %s\n\r", CommandBuffer);
}

void UartCli_t::PrintBusFrequencies() {
	Printf("Current System Clock %d Hz\r", rcc::GetCurrentSystemClock());
	Printf("Current AHB Clock %d Hz\r", rcc::GetCurrentAHBClock());
	Printf("Current APB1 Clock %d Hz\r", rcc::GetCurrentAPBClock());
}
