/*
 * adc_F072.h
 *
 *  Created on: Feb 16, 2024
 *      Author: KONSTANTIN
 */

#ifndef ADC_H_
#define ADC_H_

#include <lib_F072.h>
#include <rcc_F072.h>

typedef enum{
	adcClkHsi14 	= 0b00,
	adcClkPclkDiv2 	= 0b01,
	adcClkPclkDiv4 	= 0b10
}AdcClk_t;

typedef enum{
	adcSample1_5Clk 	= 0b000,
	adcSample7_5Clk 	= 0b001,
	adcSample13_5Clk 	= 0b010,
	adcSample28_5Clk 	= 0b011,
	adcSample41_5Clk 	= 0b100,
	adcSample55_5Clk 	= 0b101,
	adcSample71_5Clk 	= 0b110,
	adcSample239_5Clk 	= 0b111,
}AdcSampleTime_t;

typedef enum{
	adcResolution12bit 	= 0b00,
	adcResolution10bit 	= 0b01,
	adcResolution8bit 	= 0b10,
	adcResolution6bit 	= 0b11
}AdcRes_t;

typedef enum{
	adcTriggerTim1Trgo 	= 0b000,
	adcTriggerTim1Cc4 	= 0b001,
	adcTriggerTim2Trgo 	= 0b010,
	adcTriggerTim3Trgo 	= 0b011,
	adcTriggerTim15Trgo = 0b100,
}AdcTrigger_t;

namespace adc{
	void Init(AdcClk_t ClkSource, AdcRes_t AdcResBit ,AdcTrigger_t TriggerSource,
			AdcSampleTime_t AdcSampling = adcSample1_5Clk){
		rcc::EnableClkADC();
		ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE_Msk;
		ADC1->CFGR2 |= ClkSource << ADC_CFGR2_CKMODE_Pos; // Select ADC clock
		ADC1->CFGR1 |= ADC_CFGR1_OVRMOD; // Disable overrun
		ADC1->SMPR = AdcSampling;
		ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // Single conversion mode
//		ADC1->CFGR1 |= ADC_CFGR1_CONT; // Continuous conversion mode
		// ADC resolution setup
		ADC1->CFGR1 &= ~ADC_CFGR1_RES_Msk;
		ADC1->CFGR1 |= AdcResBit << ADC_CFGR1_RES_Pos;
		//External trigger
		ADC1->CFGR1 |= 0b01 << ADC_CFGR1_EXTEN_Pos; // Conversion on rising edge
		ADC1->CFGR1 |= TriggerSource << ADC_CFGR1_EXTSEL_Pos;
	}

	uint8_t EnableDmaRequest(){
		if (ADC1->CR & ADC_CR_ADSTART)
			return retvFail; // ADC is not stopped
		ADC1->CFGR1 |= ADC_CFGR1_DMACFG; // DMA circular mode
		ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
		return retvOk;
	}

	uint8_t SelectChannel(uint8_t ChNumber){
		if (ADC1->CR & ADC_CR_ADSTART)
			return retvFail; // ADC is not stopped
		ADC1->CHSELR |= 0b1UL << ChNumber;
		return retvOk;
	}

	uint8_t DeselectChannel(uint8_t ChNumber){
		if (ADC1->CR & ADC_CR_ADSTART)
			return retvFail; // ADC is not stopped
		ADC1->CHSELR &= ~(0b1UL << ChNumber);
		return retvOk;
	}

	uint8_t Start(){
		if (!(ADC1->CR & ADC_CR_ADEN))
			return retvFail; // ADC is disabled
		if ((ADC1->CR & ADC_CR_ADDIS))
			return retvFail; // Pending ADC disable
		ADC1->CR = ADC_CR_ADSTART;
		return retvOk;
	}

	uint8_t Stop(){
		if (ADC1->CR & ADC_CR_ADSTART){
			ADC1->CR = ADC_CR_ADSTP; // Stop conversion if not stopped
		}
		return retvOk;
	}

	uint8_t Calibrate(uint32_t Timeout = 0xFFF){
		if (ADC1->CR & ADC_CR_ADEN_Msk)
			return retvFail; // ADC is not disabled
		if (ADC1->CFGR1 & ADC_CFGR1_DMAEN_Msk)
			return retvFail; // DMA request generation not disabled
		// Calibration start and wait until ready
		ADC1->CR = ADC_CR_ADCAL;
		while (ADC1->CR & ADC_CR_ADCAL){
			Timeout--;
			if (Timeout == 0)
				return retvTimeout; //Unable to calibrate ADC
		}
		return retvOk;
	}

	uint8_t Enable(uint32_t Timeout = 0xFFF){
		ADC1->ISR = ADC_ISR_ADRDY; // Clear ready flag
		ADC1->CR = ADC_CR_ADEN;
		while (ADC1->ISR & ADC_ISR_ADRDY){
			Timeout--;
			if (Timeout == 0)
				return retvTimeout; //Unable to enable ADC
		}
		return retvOk;
	}

	uint8_t Disable(uint32_t Timeout = 0xFFF){
		if (ADC1->CR & ADC_CR_ADSTART){
			ADC1->CR = ADC_CR_ADSTP; // Stop conversion if not
		}
		ADC1->CR = ADC_CR_ADDIS;
		return retvOk;
	}
}

#endif /* ADC_H_ */
