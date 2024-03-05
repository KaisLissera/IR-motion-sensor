/*
 * main.cpp
 *
 *  Created on: 2024.02.08
 *      Author: Kais Lissera
 */

#include <stm32f0xx.h>
//
#include <stdint.h>
//
#include <lib_F072.h>
#include <rcc_F072.h>
#include <gpio_F072.h>
#include <interface_F072.h>

#include <board.h>

#define DMA_IRQ_PRIORITY 0

//Peripheral declaration
/////////////////////////////////////////////////////////////////////
Uart_t UartCmd;
DmaTx_t UartCmdDmaTx;
DmaRx_t UartCmdDmaRx;
Cli_t UartCmdCli(&UartCmdDmaTx, &UartCmdDmaRx);

//IRQ Handlers
/////////////////////////////////////////////////////////////////////
extern"C"{
	void DMA1_Channel2_3_IRQHandler(){
//		UartCmdDmaTx.IrqHandler(); // Interrupt on DMA transfer complete
	}
}

extern"C"{
	void HardFault_Handler(){}
}

int main() {
//  Add RCC reset
//	flash::SetFlashLatency(0);
//	rcc::EnableHSI14();
//	rcc::SwitchSysClk(sysClkHsi);
	gpio::SetupPin(LED_G, PullAir, GeneralOutput);
//
	UartCmd.Init(UART_PARAMS);
	UartCmd.EnableDmaRequest(); // USART1 on DMA channels 2 and 3
	UartCmd.Enable();

	UartCmdDmaTx.Init(UART_DMA_TX, DMA_IRQ_PRIORITY);
	UartCmdDmaRx.Init(UART_DMA_RX);
	UartCmdDmaRx.Start();

	gpio::ActivatePin(LED_G);
	DelayMs(500);
	gpio::DeactivatePin(LED_G);
	DelayMs(500);
	// Add RCC reset

	__disable_irq();
	typedef void (*pFunction)(void); // Define pointer to void function type
	uint32_t jumpAddress = *(volatile uint32_t*)(0x8002000 + 4); // Get reset vector pointer
//	UartCmdCli.Printf("%x\n\r", jumpAddress);
	DelayMs(100);
	pFunction JumpToProgram = (pFunction)jumpAddress;
	__set_MSP(*(volatile uint32_t*)0x8002000); // Get main program stack pointer
	JumpToProgram();
}
