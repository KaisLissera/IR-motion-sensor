/*
 * main.cpp
 *
 *  Created on: 2024.02.08
 *      Author: Kais Lissera
 */

// rewrite DMA Init
// try to disable CIRC in DMA
// refactor Uart Shell

#include <stm32f0xx.h>
//
#include <stdint.h>
#include <cstdio>
//
#include <lib_F072.h>
#include <rcc_F072.h>
#include <gpio_F072.h>
#include <tim_F072.h>
#include <interface_F072.h>

#include <board.h>

//Peripheral declaration
/////////////////////////////////////////////////////////////////////
Uart_t UartCmd;
DmaTx_t UartCmdDmaTx;
DmaRx_t UartCmdDmaRx;
Cli_t UartCmdCli(&UartCmdDmaTx, &UartCmdDmaRx);

//IRQ Handlers
/////////////////////////////////////////////////////////////////////
extern "C"{
	void USART1_IRQHandler(){
		UartCmd.IrqHandler(); // Interrupt on character match
	}
}
// CHECK!! In startup file interrupt vector marked as reserved
extern"C"{
	void DMA1_Channel2_3_IRQHandler(){
		UartCmdDmaTx.IrqHandler(); // Interrupt on DMA transfer complete
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

	UartCmdDmaTx.Init(UART_DMA_TX);
	UartCmdDmaRx.Init(UART_DMA_RX);
	UartCmdDmaRx.Start();

	while(1){
		while(UartCmdDmaRx.GetNumberOfBytesInBuffer()){
			UartCmdDmaTx.WriteToBuffer(UartCmdDmaRx.ReadFromBuffer());
			UartCmdDmaTx.Start();
		}
		gpio::TogglePin(LED_B);
		DelayMs(250);
	}
}

