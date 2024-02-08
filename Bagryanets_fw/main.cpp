/*
 * main.cpp
 *
 *  Created on: 2024.02.08
 *      Author: Kais Lissera
 */

#include <stm32f0xx.h>
//
#include <stdint.h>
#include <cstdio>
//
#include <lib_F072.h>
#include <rcc_F072.h>
#include <gpio_F072.h>
#include <uart_F072.h>
#include <tim_F072.h>

#include <board.h>

//Peripheral declaration
/////////////////////////////////////////////////////////////////////
Uart_t UartCmd;
Dma_t UartCmdDma;

//IRQ Handlers
/////////////////////////////////////////////////////////////////////
extern "C"{
	void USART1_IRQHandler(){
		UartCmd.UartIrqHandler(); // Interrupt on character match
	}
}
// CHECK!! In startup file interrupt vector marked as reserved
extern"C"{
	void DMA1_Channel2_3_IRQHandler(){
		UartCmdDma.DmaIrqHandler(); // Interrupt on DMA transfer complete
	}
}

//Main
/////////////////////////////////////////////////////////////////////
int main() {
	flash::SetFlashLatency(12);
	rcc::EnableHSE();
	rcc::SwitchSysClk(sysClkHse);

	gpio::SetupPin(LED_B, PullAir, GeneralOutput);
	gpio::ActivatePin(LED_B);

	UartCmd.Init(UART_PARAMS);
	UartCmd.EnableCharMatch('\r');
	UartCmd.EnableDmaRequest();
	UartCmd.Enable();

	UartCmdDma.Init(UART_DMA_PARAMS);
	UartCmdDma.StartDmaTx();
	UartCmdDma.StartDmaRx();

	while(1){
		while(UartCmdDma.GetNumberOfBytesInRxBuffer())
			UartCmdDma.WriteToBuffer(UartCmdDma.ReadFromBuffer());
		gpio::TogglePin(LED_B);
		DelayMs(250);
	}
}

