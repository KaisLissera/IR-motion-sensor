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
#include <math.h> // abs(x)
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

Timer_t LedTimer;
Timer_t IrLedTimer;

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
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

#define MAXIMUM_LENGTH 2048 // Must be degree of 2
#define DECIMATOR_SIZE 128 // Must be degree of 2

// Peak filter
class IirPeak_t{
public:
	int32_t a0;
	int32_t b1;
	int32_t b2;
	int32_t Output[2]; // Previous outputs
	IirPeak_t(int32_t _a0, int32_t _b1, int32_t _b2){
		a0 = _a0; b1 = _b1; b2 = _b2;
	}

	// Not working
	void CalculateCoefficients(uint32_t f0, uint32_t fd, uint32_t bandWidth){
		// Calculation and normalization
		b1 = ceil(2*exp(-M_PI*bandWidth/fd)*cos(2*M_PI*f0/fd) * 256);
		b2 = ceil(-exp(-2*M_PI*bandWidth/fd) * 256);
		a0 = ceil((1-exp(-2*M_PI*bandWidth/fd)) * 256);
	}

	inline int32_t GetCurrentAbsOutput(uint32_t CurrentInput){
		int32_t temp = Output[1];
		Output[1] = Output[0];
		Output[0] = CurrentInput*a0 + Output[1]*b1 + temp*b2;
		if (Output[0] >= 0)
			Output[0] = Output[0] >> 8;
		else
			Output[0] = -((-Output[0]) >> 8);
		return abs(Output[0]);
	}
};

void CliTask(void *pvParameters){
	IirPeak_t CarrierFilter(211, -240, 15);
//	CarrierFilter.CalculateCoefficients(36000, 200000, 1000);
	UartCmdCli.Printf("[SYS] Calculated filter coefficients a0 = %d, b1 = %d, b2 = %d\n\r",
			CarrierFilter.a0, CarrierFilter.b1, CarrierFilter.b2);

	uint32_t DecimatorAkkum = 0;
	uint32_t DecimatorCounter = 0;

	uint32_t max1 = 0;
	uint32_t max2 = 0;
	uint32_t max1counter = 0;
	uint32_t max2counter = MAXIMUM_LENGTH >> 1;

	uint32_t min1 = 0;
	uint32_t min2 = 0;

	AdcTrigger.StartCount();
	while(1){
//		UartCmdCli.Printf("%d\n\r", AdcDmaRx.GetNumberOfBytesInBuffer()); // For testing
		while(AdcDmaRx.GetNumberOfBytesInBuffer()){
			DecimatorAkkum += CarrierFilter.GetCurrentAbsOutput(AdcDmaRx.ReadFromBuffer());
//			DecimatorAkkum += AdcDmaRx.ReadFromBuffer();
			DecimatorCounter++;
			if (DecimatorCounter == DECIMATOR_SIZE){
				uint32_t temp = DecimatorAkkum >> ReturnDegreeOfTwo(DECIMATOR_SIZE);
				DecimatorAkkum = 0;
				DecimatorCounter = 0;

				if (temp > max1)
					max1 = temp;
				if (temp > max2)
					max2 = temp;
				if (temp < min1)
					min1 = temp;
				if (temp < min2)
					min2 = temp;

				max1counter++;
				max2counter++;
				if (max1counter == MAXIMUM_LENGTH){
					max1 = 0;
					min1 = max2;
					max1counter = 0;
				}
				if (max2counter == MAXIMUM_LENGTH){
					max2 = 0;
					min2 = max1;
					max2counter = 0;
				}

				UartCmdCli.Printf("%d    %d\n\r", temp, max1);
				if ((max1 > 15) or (max2 > 15))
					gpio::ActivatePin(LED_R);
				else
					gpio::DeactivatePin(LED_R);
			}
		}
//		PorogBuf[PorogBufPtr & (POROG_BUF_SIZE - 1)] = AdcDmaRx.ReadFromBuffer();
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

int main() {
	flash::SetFlashLatency(1);
	rcc::EnableHSE();
	rcc::SetupPLL(pllSrcHsePrediv, 16, 4);
	rcc::EnablePLL();
	rcc::SwitchSysClk(sysClkPll);

//	gpio::SetupPin(LED_B, PullAir, GeneralOutput);
	LedTimer.Init(TIM3, 10000, 10000, UpCounter);
	LedTimer.ConfigureChannel(PC8, AF0, 3, PWM1);
	LedTimer.LoadCompareValue(3, 5000);
	LedTimer.SetChannelAbility(3, Enable);
	LedTimer.StopCount(5001);
//	LedTimer.StartCount();

	gpio::SetupPin(LED_G, PullAir, GeneralOutput);
	gpio::SetupPin(LED_R, PullAir, GeneralOutput);

	UartCmd.Init(UART_PARAMS);
	UartCmd.EnableCharMatch('\r', UART_IRQ_PRIORITY);
	UartCmd.EnableDmaRequest(); // USART1 on DMA channels 2 and 3
	UartCmd.Enable();

	UartCmdDmaTx.Init(UART_DMA_TX, DMA_IRQ_PRIORITY);
	UartCmdDmaRx.Init(UART_DMA_RX);
	UartCmdDmaRx.Start();

	// Timer for ADC
	// 1 MHz timer frequency, 200 kHz trigger frequency
	AdcTrigger.Init(TIM15, 3000000, 14, UpCounter);
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

	//IR LED setup
//	gpio::SetupPin(LED_IR, PullAir, GeneralOutput);
//	gpio::ActivatePin(PA8);
	IrLedTimer.Init(TIM1, 360000, 10, UpCounter);
	TIM1->BDTR |= TIM_BDTR_MOE;
	IrLedTimer.ConfigureChannel(LED_IR, AF2, 1, PWM1);
	IrLedTimer.LoadCompareValue(1, 5);
	IrLedTimer.SetChannelAbility(1, Enable);
	IrLedTimer.StartCount();

	xTaskCreate(&BlinkBlueTask, "BLKB", 256, NULL,
			BLINK_TASK_PRIORITY, &BlinkBlueTaskHandle);
	xTaskCreate(&CliTask, "CLI", 256, NULL,
			CLI_TASK_PRIORITY, &CliTaskHandle);

	UartCmdCli.Printf("[SYS] Starting sheduler\n\r");
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
