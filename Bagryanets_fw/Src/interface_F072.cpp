/*
 * uart_ezh.cpp
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#include <interface_F072.h>

//Uart_t
/////////////////////////////////////////////////////////////////////

void Uart_t::Init(USART_TypeDef* _Usart, uint32_t CurrentUartClockHz, uint32_t Bod){
	Usart = _Usart; // Set class parameter

	// USART must be disabled before configuring
	Usart->CR1 &= ~USART_CR1_M_Msk; // 1 stop bit, 8 data bits
	Usart->BRR = (uint32_t)CurrentUartClockHz/Bod; // Setup baud rate
	Usart->CR1 |= USART_CR1_TE | USART_CR1_RE ; //USART TX RX enable
}

void Uart_t::UpdateBaudrate(uint32_t CurrentClockHz, uint32_t Bod){
	Disable();
	Usart->BRR = (uint32_t)CurrentClockHz/Bod; // Setup baud rate
	Enable();
}

//Simple UART byte transmit, wait until fully transmitted
uint8_t Uart_t::WriteChar(uint8_t data){
	while ((Usart->ISR & USART_ISR_TXE) == 0) {}
	Usart->TDR = data;
	return retvOk;
}

//Simple UART byte receive, wait until receive
uint8_t Uart_t::ReadChar(retv_t* retv){
	uint32_t timeout = 0xFFFF;
	while((Usart->ISR & USART_ISR_RXNE) == 0){
		timeout--;
		if(timeout == 0){
			*retv = retvTimeout;
			return 0; // Can't receive data
		}
	}
	// Data received
	uint8_t data = Usart->RDR;
	*retv = retvOk;
	return data;
}

uint32_t Uart_t::GetNumberOfBytesReady(){
	if(Usart->ISR & USART_ISR_RXNE)
		return 0;
	else
		return 1;
}

//USART Must be disabled, this function need IRQ handler
void Uart_t::EnableCharMatch(char CharForMatch, IRQn_Type vector, uint32_t prio) {
	Usart->CR2 |= (uint8_t)CharForMatch << USART_CR2_ADD_Pos;
	Usart->CR1 |= USART_CR1_CMIE;
	nvic::SetupIrq(vector, prio);
}

// If return value = retvOk - character match detected
uint8_t Uart_t::IrqHandler() {
	if (Usart->ISR & USART_ISR_CMF_Msk) {
		Usart->ICR = USART_ICR_CMCF;
		return retvOk;
	} else
		return retvFail;
};

//DmaTx_t
/////////////////////////////////////////////////////////////////////

// Initialize memory to peripheral DMA TX channel
void DmaTx_t::Init(uint32_t PeriphRegAdr, uint8_t DmaIrqPrio, DmaChPrio_t ChPrio){
	// Setup class parameters
	BufferStartPtr = 0;
	BufferEndPtr = 0;

	Dma->Channel -> CCR =  DMA_CCR_MINC; // Memory increment
	Dma->Channel -> CCR |= ChPrio << DMA_CCR_PL_Pos; // DMA channel priority
	Dma->Channel -> CCR |= DMA_CCR_DIR_Msk; // 1 - Read from memory
	// Memory and peripheral sizes
	Dma->Channel -> CCR |= (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos); // 8 bit
	nvic::SetupIrq(Dma->Irq, DmaIrqPrio);
	Dma->Channel -> CCR |= DMA_CCR_TCIE;
	Dma->Channel -> CPAR = PeriphRegAdr; //Peripheral register

}

uint8_t DmaTx_t::StartTransmission(){
	if(CheckStatus() != 0)
		return retvBusy; //Nothing changes if DMA already running

	if(BufferStartPtr == BufferEndPtr)
		return retvEmpty; //Nothing changes if Buffer Empty

	uint32_t NumberOfBytesReady;
	if (BufferEndPtr > BufferStartPtr)
		NumberOfBytesReady = GetNumberOfBytesInBuffer();
	else
		NumberOfBytesReady = BufferSize - BufferStartPtr;
	Dma->Channel -> CNDTR = NumberOfBytesReady;
	Dma->Channel -> CMAR = (uint32_t)&Buffer[BufferStartPtr];
	BufferStartPtr = (BufferStartPtr + NumberOfBytesReady) % BufferSize;
	Dma->Channel -> CCR |= DMA_CCR_EN;

	return retvOk;
}

uint32_t DmaTx_t::CheckStatus(){
	uint32_t temp = Dma->Channel -> CCR;
	return temp & DMA_CCR_EN;
}

uint32_t DmaTx_t::GetNumberOfBytesInBuffer(){
	if (BufferEndPtr >= BufferStartPtr)
		return BufferEndPtr - BufferStartPtr;
	else
		return BufferSize - BufferStartPtr + BufferEndPtr;
}

uint8_t DmaTx_t::WriteChar(uint8_t data){
	uint32_t EndPtrTemp = (BufferEndPtr + 1) % BufferSize;
	if (EndPtrTemp == BufferStartPtr)
		return retvOutOfMemory;

	Buffer[BufferEndPtr] = data;
	BufferEndPtr = EndPtrTemp;
	return retvOk;
}

uint8_t DmaTx_t::IrqHandler(){
	if(DMA1->ISR & DMA_ISR_TCIF1 << 4*(Dma->Number - 1)){
		DMA1->IFCR = DMA_IFCR_CTCIF1 << 4*(Dma->Number - 1);
		Dma->Channel -> CCR &= ~DMA_CCR_EN;
		StartTransmission();
		return retvOk;
	} else
		return retvFail;
}

//DmaRx_t
/////////////////////////////////////////////////////////////////////

// Initialize peripheral to memory DMA RX channel
void DmaRx_t::Init(uint32_t PeriphRegAdr, DmaChPrio_t ChPrio){
	// Setup class parameters
	BufferStartPtr = 0;

	// Initialize DMA RX
	Dma->Channel -> CCR =  DMA_CCR_MINC | DMA_CCR_CIRC; // Memory increment, circular mode
	Dma->Channel -> CCR |= ChPrio << DMA_CCR_PL_Pos; // DMA channel priority
	// Memory and peripheral sizes)
	Dma->Channel -> CCR |= (0b00 << DMA_CCR_MSIZE_Pos) | (0b00 << DMA_CCR_PSIZE_Pos); // 8 bit
	Dma->Channel -> CPAR = PeriphRegAdr; //Peripheral register
}

uint16_t DmaRx_t::GetBufferEndPtr(){
	return BufferSize - (Dma->Channel -> CNDTR);
}

void DmaRx_t::Start(){
	Dma->Channel -> CNDTR = BufferSize;
	Dma->Channel -> CMAR = (uint32_t)&Buffer[0];
	Dma->Channel -> CCR |= DMA_CCR_EN;
}

inline void DmaRx_t::Stop(){
	Dma->Channel -> CCR &= ~DMA_CCR_EN;
}

uint32_t DmaRx_t::GetNumberOfBytesReady(){
	uint32_t BufferEndPtr = GetBufferEndPtr();
	if (BufferEndPtr >= BufferStartPtr)
		return BufferEndPtr - BufferStartPtr;
	else
		return BufferSize - BufferStartPtr + BufferEndPtr;
}

uint32_t DmaRx_t::CheckStatus(){
	uint32_t temp = Dma->Channel -> CCR;
	return temp & DMA_CCR_EN;
}

uint8_t DmaRx_t::ReadChar(retv_t* retv){
	uint8_t temp = Buffer[BufferStartPtr];
	BufferStartPtr = (BufferStartPtr + 1) % BufferSize;
	return temp;
}

int32_t sstring::ToInt(const char* str1){
	uint32_t ptr = 0;
	uint32_t number = 0;
	while(str1[ptr] != '\0'){
		if((str1[ptr] >= '0') and (str1[ptr] <= '9')){
			number = number*10 + (str1[ptr] - '0');
			ptr++;
		}else
			return 0;
	}
	return number;
}

uint8_t sstring::Compare(const char* str1, const char* str2){
	uint32_t ptr = 0;
	while((str1[ptr] != '\0') or (str2[ptr] != '\0')){
		if(str1[ptr] != str2[ptr])
			return 0;
		ptr++;
	}
	return 1;
}

//Cli_t
/////////////////////////////////////////////////////////////////////

void Cli_t::PutString(const char* text){
	uint8_t length = strlen(text);
	for(uint32_t i = 0; i < length; i++)
		TxChannel->WriteChar((uint8_t)text[i]);
}

void Cli_t::PutBinary(uint32_t number){
	TxChannel->WriteChar((uint8_t)'0');
	TxChannel->WriteChar((uint8_t)'b');
	if(number == 0){
		TxChannel->WriteChar((uint8_t)'0');
		return;
	}

	uint32_t mask = 1UL << 31;
	while((number & mask) == 0)
		mask = mask >> 1;
	while(mask > 0){
		if((number & mask))
			TxChannel->WriteChar((uint8_t)'1');
		else
			TxChannel->WriteChar((uint8_t)'0');
		mask = mask >> 1;
	}
}

void Cli_t::PutUnsignedInt(uint32_t number){
	if(number == 0){
		TxChannel->WriteChar((uint8_t)'0');
		return;
	}

	char IntBuf[10] = "";
	uint32_t i = 0;
	while(number > 0){
		uint32_t temp = number % 10;
		number = (uint32_t)number/10;
		IntBuf[i] = '0' + temp;
		i++;
	}
	for(uint32_t k = 0; k < i; k++)
		TxChannel->WriteChar((uint8_t)IntBuf[i - k - 1]);
}

void Cli_t::PutUnsignedHex(uint32_t number){
	TxChannel->WriteChar((uint8_t)'0');
	TxChannel->WriteChar((uint8_t)'x');
	if(number == 0){
		TxChannel->WriteChar((uint8_t)'0');
		return;
	}

	char IntBuf[10] = "";
	uint32_t i = 0;
	while(number > 0){
		uint32_t temp = number & 0xF;
		number = number >> 4;
		if(temp < 10)
			IntBuf[i] = '0' + temp;
		else
			IntBuf[i] = 'A' + temp - 10;
		i++;
	}
	for(uint32_t k = 0; k < i; k++)
		TxChannel->WriteChar((uint8_t)IntBuf[i - k - 1]);
}

void Cli_t::PutInt(int32_t number) {
	if(number >= 0)
		PutUnsignedInt(number);
	else{
		TxChannel->WriteChar('-');
		PutUnsignedInt(-number);
	}
}

void Cli_t::Printf(const char* text, ...){
	// String format processing
	va_list args;
	va_start(args, text); // Start string processing
	uint8_t length = strlen(text);
	for(uint32_t i = 0; i < length; i++){
		if(text[i] == '%'){ // if argument found
			i++;
			switch(text[i]){
				case 'd': // integer
					PutInt(va_arg(args,int));
					break;
				case 'u': // unsigned integer
					PutUnsignedInt(va_arg(args,uint32_t));
					break;
				case 's': // string
					PutString(va_arg(args,char*));
					break;
				case 'c': // char
					TxChannel->WriteChar(va_arg(args,int));
					break;
				case 'b': // print uint32 as binary
					PutBinary(va_arg(args,uint32_t));
					break;
				case 'x': // print uint32 as hex
					PutUnsignedHex(va_arg(args,uint32_t));
					break;
				default:
					TxChannel->WriteChar((uint8_t)'%');
					TxChannel->WriteChar((uint8_t)text[i]);
			}
		} else
			TxChannel->WriteChar((uint8_t)text[i]);
	}
	va_end(args); // End format processing
	//
	TxChannel->StartTransmission();
}

char* Cli_t::Read(){
	char temp;
	uint8_t CmdBufferPtr = 0;

	if(RxChannel->GetNumberOfBytesReady() == 0)
		return NULL;
	//Clear buffer beginning from useless symbols
	while((RxChannel->GetNumberOfBytesReady() != 0)){
		temp = RxChannel->ReadChar(NULL);
		if((temp != '\r') or (temp != '\n'))
			break;
	}
	if(RxChannel->GetNumberOfBytesReady() == 0)
		return NULL;
	CommandBuffer[CmdBufferPtr] = temp;
	CmdBufferPtr++;
	//Get command from buffer
	while((RxChannel->GetNumberOfBytesReady() != 0)){
		temp = RxChannel->ReadChar(NULL);
		if((temp == '\r') or (temp == '\n') or (temp == ' '))
			break;
		CommandBuffer[CmdBufferPtr] = temp;
		CmdBufferPtr++;
	}
	//Add terminator to string
	CommandBuffer[CmdBufferPtr] = '\0';
	return CommandBuffer;
}

char* Cli_t::ReadLine(){
	char temp;
	uint8_t CmdBufferPtr = 0;

	if(RxChannel->GetNumberOfBytesReady() == 0)
		return NULL;
	//Clear buffer beginning from useless symbols
	while((RxChannel->GetNumberOfBytesReady() != 0)){
		temp = RxChannel->ReadChar(NULL);
		if((temp != '\r') or (temp != '\n'))
			break;
	}
	if(RxChannel->GetNumberOfBytesReady() == 0)
		return NULL;

	CommandBuffer[CmdBufferPtr] = temp;
	CmdBufferPtr++;
	//Get command from buffer
	while((RxChannel->GetNumberOfBytesReady() != 0)){
		temp = RxChannel->ReadChar(NULL);
		if((temp == '\r') or (temp == '\n'))
			break;
		CommandBuffer[CmdBufferPtr] = temp;
		CmdBufferPtr++;
	}
	//Add terminator to string
	CommandBuffer[CmdBufferPtr] = '\0';
	return CommandBuffer;
}
