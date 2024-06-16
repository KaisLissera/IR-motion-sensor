/*
 * main.cpp
 *
 *  Created on: 2024.02.08
 *      Author: Kais Lissera
 */

#include <stm32f0xx.h>
//
#include <stdint.h>
#include <math.h> // abs(x)
//
#include <rcc_F072.h>
#include <gpio_F072.h>
#include <tim_F072.h>
#include <interface_F072.h>
#include <adc_F072.h>

#include <board.h>

#include <FreeRTOS.h>
#include <lib.h>
#include <task.h>

//RTOS tasks declarations and priorities
/////////////////////////////////////////////////////////////////////
#define BLINK_TASK_PRIORITY 1
#define CLI_TASK_PRIORITY 1

#define UART_IRQ_PRIORITY configSTM_MAX_SYSCALL_INTERRUPT_PRIORITY
#define DMA_IRQ_PRIORITY configSTM_MAX_SYSCALL_INTERRUPT_PRIORITY

TaskHandle_t BlinkBlueTaskHandle;
void BlinkBlueTask(void *pvParametrs);

TaskHandle_t CliTaskHandle;
void CliTask(void *pvParametrs);

// Peripheral declaration
/////////////////////////////////////////////////////////////////////

// Debug UART shell
#define UART_BUFFER_SIZE 128
Uart_t UartCmd;
DmaChannel_t DmaChannel2 = {.Channel = DMA1_Channel2, .Number = 2, .Irq = DMA1_Channel2_3_IRQn};
DmaChannel_t DmaChannel3 = {.Channel = DMA1_Channel3, .Number = 2, .Irq = DMA1_Channel2_3_IRQn};
uint8_t CmdTxBuffer[UART_BUFFER_SIZE];
uint8_t CmdRxBuffer[UART_BUFFER_SIZE];
DmaTx_t CmdTxDma(&DmaChannel2, CmdTxBuffer, UART_BUFFER_SIZE);
DmaRx_t CmdRxDma(&DmaChannel3, CmdRxBuffer, UART_BUFFER_SIZE);
Cli_t CmdCli(&CmdTxDma, &CmdRxDma);

// ADC
#define ADC_BUFFER_SIZE 512
Timer_t AdcTrigger;
DmaChannel_t DmaChannel1 = {.Channel = DMA1_Channel1, .Number = 1, .Irq = DMA1_Channel1_IRQn};
uint8_t AdcRxBuffer[ADC_BUFFER_SIZE];
DmaRx_t AdcRxDma(&DmaChannel1, AdcRxBuffer, ADC_BUFFER_SIZE);

// IR transmitter
Timer_t IrCarrierTimer;
Timer_t IrEncoderTimer;

//IRQ Handlers
/////////////////////////////////////////////////////////////////////
extern "C"{
	void USART1_IRQHandler(){
		UartCmd.IrqHandler(); // Interrupt on character match
	}

	void DMA1_Channel2_3_IRQHandler(){
		CmdTxDma.IrqHandler(); // Interrupt on DMA transfer complete
	}

	void HardFault_Handler(){
	}
}

//Main and RTOS tasks
/////////////////////////////////////////////////////////////////////

void BlinkBlueTask(void *pvParameters){
//	uint32_t brigthness = 0;
	while(1){
//		UartCmdCli.Printf("[SYS] Blink blue task\n\r");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

#define DECIMATOR_SIZE 64 // Must be degree of 2
#define MAXIMUM_LENGTH 16*1024 // Approximately 5 seconds

// Peak filter
class IirPeak_t{
public:
	int32_t a0;
	int32_t b1;
	int32_t b2;
	int32_t Output[2] = {0, 0}; // Previous outputs
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
		Output[0] = Output[0] >> 8;
		return abs(Output[0]);
	}
};

void CliTask(void *pvParameters){
	IirPeak_t CarrierFilter(30, 211, -240);
	CmdCli.Printf("[SYS] Filter coefficients a0 = %d, b1 = %d, b2 = %d\n\r",
			CarrierFilter.a0, CarrierFilter.b1, CarrierFilter.b2);

	uint32_t DecimatorAccumulator = 0;
	uint32_t DecimatorCounter = 0;

	uint32_t max1 = 0;
	uint32_t max2 = 0;
	uint32_t max = 0;
	uint32_t max1counter = 0;
	uint32_t max2counter = MAXIMUM_LENGTH >> 1;

	uint32_t temp = 0;
	uint32_t gain = 1;
	AdcTrigger.StartCount();
	while(1){
		while(AdcRxDma.GetNumberOfBytesReady()){
			// Auto gain
			temp = AdcRxDma.ReadChar();
			if (temp > max1)
				max1 = temp;
			if (temp > max2)
				max2 = temp;
			// Iir peak filter for carrier frequency
			DecimatorAccumulator += CarrierFilter.GetCurrentAbsOutput(temp*gain);
			DecimatorCounter++;
			if (DecimatorCounter == DECIMATOR_SIZE){
				temp = DecimatorAccumulator / DECIMATOR_SIZE;
				DecimatorAccumulator = 0;
				DecimatorCounter = 0;
				// Auto gain gain update
				if (max1 > max2)
					max = max1;
				else
					max = max2;

				if (max*gain > 256)
					gain = gain >> 1;
				else if ((max*gain < 128) and (gain < 32))
					gain = gain << 1;

				max1counter++;
				max2counter++;
				if (max1counter == MAXIMUM_LENGTH){
					max1 = 0;
					max1counter = 0;
				}
				if (max2counter == MAXIMUM_LENGTH){
					max2 = 0;
					max2counter = 0;
				}

				CmdCli.Printf("%d  gain=%d\n\r", temp, gain, max);
				if (temp > 32)
					gpio::ActivatePin(LED_R);
				else
					gpio::DeactivatePin(LED_R);
			}
		}
	}
}

int main() {
	rcc::ResetAll();
	rcc::DisableAllClocks();
#ifdef USE_BOOTLOADER
	//Move interrupt vector table to RAM
	__disable_irq();
//	volatile uint32_t *VTable = (volatile uint32_t*)0x20000000;
//	for (uint32_t i = 0; i < 48; i++){
//	VTable[i] = *(volatile uint32_t*)(0x8002000 + i*4);
//	}

	for (uint32_t i = 0; i < 48; i++){
		volatile uint32_t* pRamTable = (uint32_t*)(0x20000000 + i*4);
		*pRamTable = *(volatile uint32_t*)(0x8002000 + i*4);
	}
	rcc::EnableClkSYSCFGCOMP();
	SYSCFG->CFGR1 = 0b11;
	__enable_irq();
#endif

	// Clock setup
	flash::SetFlashLatency(1);
	rcc::EnableHSE();
	rcc::SetupPLL(pllSrcHsePrediv, 16, 4);
	rcc::EnablePLL();
	rcc::SwitchSysClk(sysClkPll);

	uint32_t coreClockHz = rcc::GetCurrentSystemClock();
	uint32_t ahbClockHz = rcc::GetCurrentAHBClock(coreClockHz);
	uint32_t apbClockHz = rcc::GetCurrentAPBClock(ahbClockHz);
	uint32_t timClockHz = rcc::GetCurrentTimersClock(apbClockHz);

	// Enable peripheral clocks
	rcc::EnableClkAHB(RCC_AHBENR_GPIOCEN);
	rcc::EnableClkAHB(RCC_AHBENR_GPIOBEN);
	rcc::EnableClkAHB(RCC_AHBENR_GPIOAEN);
	rcc::EnableClkAHB(RCC_AHBENR_DMA1EN);

	rcc::EnableClkAPB1(RCC_APB1ENR_TIM2EN);
	rcc::EnableClkAPB1(RCC_APB1ENR_TIM14EN);

	rcc::EnableClkAPB2(RCC_APB2ENR_USART1EN);
	rcc::EnableClkAPB2(RCC_APB2ENR_ADCEN);
	rcc::EnableClkAPB2(RCC_APB2ENR_TIM1EN);
	rcc::EnableClkAPB2(RCC_APB2ENR_TIM15EN);

	// RGB LED setup
	gpio::SetupPin(LED_B, PullAir, GeneralOutput);
	gpio::SetupPin(LED_G, PullAir, GeneralOutput);
	gpio::SetupPin(LED_R, PullAir, GeneralOutput);

	UartCmd.Init(USART1, apbClockHz, 115200);
	gpio::SetupPin(PA9, PullAir, AlternateFunction, AF1);
	gpio::SetupPin(PA9, PullAir, AlternateFunction, AF1);
	UartCmd.EnableCharMatch('\r', USART1_IRQn ,UART_IRQ_PRIORITY);
	UartCmd.EnableDmaRequest(USART_CR3_DMAT | USART_CR3_DMAR);
	UartCmd.Enable();

	CmdTxDma.Init((uint32_t)&USART1->TDR);
	CmdRxDma.Init((uint32_t)&USART1->RDR);
	CmdRxDma.Start();

	// Timer for ADC
	// 1 MHz timer frequency, 200 kHz trigger frequency
	AdcTrigger.Init(TIM15, timClockHz, 3000000, 15-1, UpCounter);
	AdcTrigger.SetMasterMode(triggerOnUpdate);

	// ADC setup
	rcc::EnableHSI14();
	adc::Init(adcClkHsi14, adcResolution8bit, adcTriggerTim15Trgo, adcSample7_5Clk);
	adc::Calibrate();
	gpio::SetupPin(ADC_IR_1, PullAir, Analog); // ADC1 pin
//	gpio::SetupPin(ADC_IR_2, PullAir, Analog); // ADC2 pin - not used
	adc::SelectChannel(1); // Using only IR channel 1
	adc::Enable();
	adc::EnableDmaRequest(); // ADC on DMA channel 1
	adc::Start();

	// ADC DMA setup
	AdcRxDma.Init((uint32_t)&ADC1->DR);
	AdcRxDma.Start();

	//IR LED setup
	IrCarrierTimer.Init(TIM1, timClockHz, 360000, 10-1, UpCounter);
	IrCarrierTimer.ConfigureChannel(LED_IR, AF2, 1, outputComparePWM1);
	IrCarrierTimer.LoadCompareValue(1, 5);
	IrCarrierTimer.SetChannelAbility(1, Enable);
	IrCarrierTimer.SetSlaveMode(slaveModeTrigger, Itr1);
	IrCarrierTimer.SetOnePulseMode(32);
	IrCarrierTimer.StartCount();

	IrEncoderTimer.Init(TIM2, timClockHz, 360000, 10*64-1, UpCounter);
	IrEncoderTimer.SetMasterMode(triggerOnUpdate);
	IrEncoderTimer.StartCount();

	// Buzzer
	gpio::SetupPin(PB1, PullAir, GeneralOutput);
	gpio::DeactivatePin(PB1);

	xTaskCreate(&CliTask, "CLI", 256, NULL,
			CLI_TASK_PRIORITY, &CliTaskHandle);

	CmdCli.Printf("[SYS] Starting sheduler\n\r");
	vTaskStartScheduler();

	while(1);
}

//Main and FreeRTOS hooks
/////////////////////////////////////////////////////////////////////
void vApplicationMallocFailedHook(){
	CmdCli.Printf("[ERR] Malloc failed\n\r");
}
