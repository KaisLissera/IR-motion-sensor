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

Timer_t AdcTrigger;
DmaRx_t AdcDmaRx;

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
//		gpio::TogglePin(LED_IR);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

#define POROG_BUF_SIZE 128
uint8_t PorogBuf[POROG_BUF_SIZE]; //Буфер динамического порога
uint32_t PorogBufPtr = 0; //Указатель последнего элемента буфера динамического порога
uint32_t PorogAkkum = 0; //Акуумулятор порога

void CliTask(void *pvParameters){
	AdcTrigger.StartCount();
	while(1){
		uint32_t Threshold = 0;
//		UartCmdCli.Printf("%d\n\r", AdcDmaRx.GetNumberOfBytesInBuffer());
		while(AdcDmaRx.GetNumberOfBytesInBuffer()){
			PorogBuf[PorogBufPtr % POROG_BUF_SIZE] = AdcDmaRx.ReadFromBuffer();
			PorogAkkum += PorogBuf[PorogBufPtr % POROG_BUF_SIZE];

			if (PorogBufPtr > POROG_BUF_SIZE){
				PorogAkkum -= PorogBuf[(PorogBufPtr + 1) % POROG_BUF_SIZE];
				Threshold = PorogAkkum/POROG_BUF_SIZE;
			}
			else
				Threshold = PorogAkkum/PorogBufPtr;

			if (PorogBuf[PorogBufPtr % POROG_BUF_SIZE] > Threshold)
				gpio::ActivatePin(LED_R);
			else
				gpio::DeactivatePin(LED_R);

		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

int main() {
	flash::SetFlashLatency(1);
	rcc::EnableHSE();
	rcc::SetupPLL(pllSrcHsePrediv, 16, 4);
	rcc::EnablePLL();
	rcc::SwitchSysClk(sysClkPll);

	gpio::SetupPin(LED_B, PullAir, GeneralOutput);
	gpio::SetupPin(LED_G, PullAir, GeneralOutput);
	gpio::SetupPin(LED_R, PullAir, GeneralOutput);
	gpio::SetupPin(LED_IR, PullAir, GeneralOutput);

	gpio::ActivatePin(LED_IR);

	UartCmd.Init(UART_PARAMS);
	UartCmd.EnableCharMatch('\r', UART_IRQ_PRIORITY);
	UartCmd.EnableDmaRequest(); // USART1 on DMA channels 2 and 3
	UartCmd.Enable();

	UartCmdDmaTx.Init(UART_DMA_TX, DMA_IRQ_PRIORITY);
	UartCmdDmaRx.Init(UART_DMA_RX);
	UartCmdDmaRx.Start();

	// Timer for ADC
	// 1 MHz timer frequency, 200 kHz trigger frequency
	AdcTrigger.Init(TIM15, 3000000, 14*4, UpCounter);
	AdcTrigger.SetMasterMode(TriggerOnUpdate);

	// ADC setup
	rcc::EnableHSI14();
	adc::Init(hsi14, adc8bit, adcTim15Trgo, adcSample7_5Clk);
	adc::Calibrate();
	gpio::SetupPin(ADC_IR_1, PullAir, Analog); // ADC1 pin
//	gpio::SetupPin(ADC_IR_2, PullAir, Analog); // ADC2 pin - not used
	adc::SelectChannel(1); // Using only IR channel 1
	adc::Enable();
	adc::EnableDmaRequest(); // ADC on DMA channel 1
	adc::Start();

	// ADC DMA setup
	AdcDmaRx.Init(DMA1_Channel1, (uint32_t)&ADC1->DR);
	AdcDmaRx.Start();

	xTaskCreate(&BlinkBlueTask, "BLKB", 256, NULL,
			BLINK_TASK_PRIORITY, &BlinkBlueTaskHandle);
	xTaskCreate(&CliTask, "CLI", 256, NULL,
			CLI_TASK_PRIORITY, &CliTaskHandle);
	vTaskStartScheduler();

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
