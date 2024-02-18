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
#include <tim_F072.h>
#include <interface_F072.h>
#include <adc_F072.h>

#include <board.h>

#include <FreeRTOS.h>
#include <task.h>

//RTOS tasks declarations and priorities
/////////////////////////////////////////////////////////////////////
#define BLINK_TASK_PRIORITY 1
#define CLI_TASK_PRIORITY 1

#define UART_IRQ_PRIORITY configSTM_MAX_SYSCALL_INTERRUPT_PRIORITY
#define DMA_IRQ_PRIORITY configSTM_MAX_SYSCALL_INTERRUPT_PRIORITY

static uint32_t uptime = 0;

TaskHandle_t BlinkBlueTaskHandle;
void BlinkBlueTask(void *pvParametrs);

TaskHandle_t CliTaskHandle;
void CliTask(void *pvParametrs);

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

//Main and RTOS tasks
/////////////////////////////////////////////////////////////////////

void BlinkBlueTask(void *pvParameters){
	while(1){
//		UartCmdCli.Printf("%d [SYS] Blink blue task\n\r", uptime);
		gpio::TogglePin(LED_B);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void CliTask(void *pvParameters){
	while(1){
		UartCmdCli.Printf("%d\n\r", ADC1->DR);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

int main() {
	flash::SetFlashLatency(12);
	rcc::EnableHSE();
	rcc::SwitchSysClk(sysClkHse);

	gpio::SetupPin(LED_B, PullAir, GeneralOutput);
	gpio::SetupPin(LED_R, PullAir, GeneralOutput);

	UartCmd.Init(UART_PARAMS);
	UartCmd.EnableCharMatch('\r', UART_IRQ_PRIORITY);
	UartCmd.EnableDmaRequest();
	UartCmd.Enable();

	UartCmdDmaTx.Init(UART_DMA_TX, DMA_IRQ_PRIORITY);
	UartCmdDmaRx.Init(UART_DMA_RX);
	UartCmdDmaRx.Start();


	uint8_t retv = rcc::EnableHSI14();
	if (retv == retvOk)
		UartCmdCli.Printf("HSI14 success\n\r");
	else
		UartCmdCli.Printf("HSI14 fail\n\r");
	adc::Init(hsi14, tim15Trgo, adcSample7_5Clk);
	retv = adc::Calibrate();
	if (retv == retvOk)
		UartCmdCli.Printf("ADC calibration success\n\r");
	else
		UartCmdCli.Printf("ADC calibration fail\n\r");
	gpio::SetupPin(PA1, PullAir, Analog); // ADC1 pin
	adc::SelectChannel(1);
	adc::Enable();
	if (retv == retvOk)
		UartCmdCli.Printf("ADC enable success\n\r");
	else
		UartCmdCli.Printf("ADC enable fail\n\r");
//	adc::EnableDmaRequest();

	adc::Start();
	if (retv == retvOk)
		UartCmdCli.Printf("ADC start success\n\r");
	else
		UartCmdCli.Printf("ADC start fail\n\r");

	xTaskCreate(&BlinkBlueTask, "BLKB", 256, NULL,
			BLINK_TASK_PRIORITY, &BlinkBlueTaskHandle);
	xTaskCreate(&CliTask, "CLI", 256, NULL,
			CLI_TASK_PRIORITY, &CliTaskHandle);
//	vTaskStartScheduler();

	while(1);
}

//Main and FreeRTOS hooks
/////////////////////////////////////////////////////////////////////
void vApplicationMallocFailedHook(){
	UartCmdCli.Printf("[ERR] Malloc failed\n\r");
}

void vApplicationTickHook(){
	uptime += 10;
}
