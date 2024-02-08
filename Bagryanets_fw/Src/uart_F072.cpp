/*
 * uart_ezh.cpp
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#include <uart_F072.h>

//Uart_t
/////////////////////////////////////////////////////////////////////

void Uart_t::Init(USART_TypeDef* _Usart, GPIO_TypeDef* GpioTx, uint8_t PinTx,
				GPIO_TypeDef* GpioRx, uint8_t PinRx,
				AltFunction_t Af, uint32_t Bod) {
	Usart = _Usart; // Set class parameter
	rcc::EnableClkUSART(Usart);
	gpio::SetupPin(GpioTx, PinTx, PullUp, AlternateFunction, Af);
	gpio::SetupPin(GpioRx, PinRx, PullAir, AlternateFunction, Af);

	// USART must be disabled before configuring
	Usart->CR1 &= ~(USART_CR1_M1_Msk | USART_CR1_M0_Msk); // M[1:0] = 00 - 8 data bits
	Usart->CR1 &= ~USART_CR1_OVER8; // 0 - oversampling 16, 1 - oversampling 8
	uint32_t ApbClock = rcc::GetCurrentAPBClock(); // Get current bus clock
	Usart->BRR = (uint32_t)ApbClock/Bod; // Setup baud rate
	Usart->CR3 |= USART_CR3_OVRDIS; //Disable overrun
	Usart->CR1 |= USART_CR1_TE | USART_CR1_RE ; //USART TX RX enable
	Usart->CR1 |= USART_CR1_UE; //USART enable
} //Uart_t constructor

//Simple UART byte transmit, wait until fully transmitted
void Uart_t::TxByte(uint8_t data) {
	while ((Usart->ISR & USART_ISR_TXE) == 0) {}
	Usart->TDR = data;
}

//Simple UART byte receive, wait until receive
uint8_t Uart_t::RxByte(uint8_t* fl, uint32_t timeout) {
	while ((Usart->ISR & USART_ISR_RXNE) == 0) {
		timeout--;
		if(timeout == 0) {
			*fl = retvTimeout;
			return 0; // Can't receive data
		}
	}
	// Data received
	uint8_t data = Usart->RDR;
	*fl = retvOk;
	return data;
}

//USART Must be disabled, this function need IRQ handler
void Uart_t::EnableCharMatch(char CharForMatch, uint32_t prio) {
	Usart->CR2 |= (uint8_t)CharForMatch << USART_CR2_ADD_Pos;
	Usart->CR1 |= USART_CR1_CMIE;
	nvic::SetupIrq(ReturnIrqVectorUsart(Usart), prio);
}

// If return value = retvOk - character match detected
uint8_t Uart_t::UartIrqHandler() {
	if (Usart->ISR & USART_ISR_CMF_Msk) {
		Usart->ICR = USART_ICR_CMCF;
		return retvOk;
	} else
		return retvFail;
};

void Uart_t::EnableDmaRequest() {
	Usart->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
}

//Dma_t
/////////////////////////////////////////////////////////////////////
// Init RX and TX DMA channels
void Dma_t::Init(DMA_Channel_TypeDef* _DmaTxChannel, DMA_Channel_TypeDef* _DmaRxChannel,
		uint32_t PeriphTxRegAdr, uint32_t PeriphRxRegAdr, uint32_t prio){
	// Setup class parameters
	DmaTxChannel = _DmaTxChannel;
	DmaRxChannel = _DmaRxChannel;
	TxBufferStartPtr = 0;
	RxBufferStartPtr = 0;
	TxBufferEndPtr = 0;

	rcc::EnableClkDMA(DMA1);
	// Init DMA TX
	DmaTxChannel -> CCR = (0b11 << DMA_CCR_PL_Pos) | (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_DIR;
	nvic::SetupIrq(ReturnIrqVectorDma(DmaTxChannel), prio);
	DmaTxChannel -> CCR |= DMA_CCR_TCIE;
	DmaTxChannel -> CPAR = PeriphTxRegAdr; //Peripheral register
	// Init DMA RX
	DmaRxChannel -> CCR = (0b11 << DMA_CCR_PL_Pos) | (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_CIRC;
	DmaRxChannel -> CPAR = PeriphRxRegAdr; //Peripheral register
}

uint8_t Dma_t::StartDmaTx() {
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
}

uint32_t Dma_t::CheckDmaStatus(DMA_Channel_TypeDef* DmaChannel) {
	uint32_t temp = DmaChannel -> CCR;
	return temp & DMA_CCR_EN;
}

void Dma_t::StartDmaRx() {
	DmaRxChannel -> CNDTR = RX_BUFFER_SIZE;
	DmaRxChannel -> CMAR = (uint32_t)&RxBuffer[0];
	DmaRxChannel -> CCR |= DMA_CCR_EN;
} //UartBase_t::EnableRxDMA()

uint32_t Dma_t::GetRxBufferEndPtr() {
	return RX_BUFFER_SIZE - (DmaRxChannel -> CNDTR);
}

uint32_t Dma_t::GetNumberOfBytesInTxBuffer() {
	if (TxBufferEndPtr >= TxBufferStartPtr)
		return TxBufferEndPtr - TxBufferStartPtr;
	else
		return TX_BUFFER_SIZE - TxBufferStartPtr + TxBufferEndPtr;
}

uint32_t Dma_t::GetNumberOfBytesInRxBuffer() {
	uint32_t RxBufferEndPtr = GetRxBufferEndPtr();
	if (RxBufferEndPtr >= RxBufferStartPtr)
		return RxBufferEndPtr - RxBufferStartPtr;
	else
		return RX_BUFFER_SIZE - RxBufferStartPtr + RxBufferEndPtr;
}

uint8_t Dma_t::WriteToBuffer(uint8_t data) {
	uint32_t EndPtrTemp = (TxBufferEndPtr + 1) % TX_BUFFER_SIZE;
	if (EndPtrTemp == TxBufferStartPtr)
		return retvOutOfMemory;
	TxBuffer[TxBufferEndPtr] = data;
	TxBufferEndPtr = EndPtrTemp;
	return retvOk;
}

uint8_t Dma_t::ReadFromBuffer() {
	uint8_t temp = RxBuffer[RxBufferStartPtr];
	RxBufferStartPtr = (RxBufferStartPtr + 1) % RX_BUFFER_SIZE;
	return temp;
}

uint8_t Dma_t::DmaIrqHandler() {
	uint8_t ChNum = ReturnChannelNumberDma(DmaTxChannel);
	if(DMA1->ISR & DMA_ISR_TCIF1 << (ChNum - 1)){
		DMA1->IFCR = DMA_IFCR_CTCIF1 << (ChNum - 1);
		DmaTxChannel -> CCR &= ~DMA_CCR_EN;
		if(TxBufferStartPtr != TxBufferEndPtr) {
			StartDmaTx();
		}
		return retvOk;
	} else
		return retvFail;
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
	Channel->StartDmaTx();
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
