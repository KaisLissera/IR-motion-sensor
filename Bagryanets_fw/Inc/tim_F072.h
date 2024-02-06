/*
 * ezhtim.h
 *
 *  Created on: Aug 2, 2023
 *      Author: KONSTANTIN
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_

#include <stdint.h>
//
#include <lib_F072.h>
//
#include <gpio_F072.h>
#include <rcc_F072.h>
#include <uart_F072.h>

typedef enum {
	DownCounter 	= 0,
	UpCounter 		= 1
} CounterDirection_t;

typedef enum { // Not full
	Frozen = 0b0000,
	ActiveOnMatch = 0b0001,
	InactiveOnMatch = 0b0010,
	Toggle = 0b0011,
	ForceInactive = 0b0101,
	ForceActive = 0b0110,
	PWM1 = 0b0110,
	PWM2 = 0b0111
} OutputCompare_t;

class Timer_t {
private:
	TIM_TypeDef*  Timer;
public:
	Timer_t (TIM_TypeDef*  _Timer) {
		Timer = _Timer;
	}

	//Minimum Frequency = SYS_CLK/65536, 1.2 kHz if core clock 80 MHz
	void Init(uint32_t Frequency, uint32_t ReloadValue, CounterDirection_t Dir) {
		ezhrcc::EnableClkTIM(Timer);
		//Frequency
		Timer -> PSC = (uint32_t)(SYS_CLK/Frequency) + 1;
		if(Dir)
			Timer -> CR1 |= TIM_CR1_DIR; // 1 - Up counter
		else
			Timer -> CR1 &= ~TIM_CR1_DIR; // 0 - Down counter
		Timer -> ARR = ReloadValue;
	}

	void ConfigureChannel(GPIO_TypeDef* Gpio, uint32_t Pin, AltFunction_t Af, uint8_t ChannelNumber, OutputCompare_t CompareType) {
		//TIM channel pin initialization
		ezhgpio::SetupPin(Gpio, Pin, NoPullUpDown, AlternateFunction, Af); // PA5 AF1
		//Channel Compare Type
		switch (ChannelNumber) {
		case 1:
			Timer -> CCMR1 |= (CompareType << TIM_CCMR1_OC1M_Pos);
			Timer -> CCER |= TIM_CCER_CC1E;
			break;
		case 2:
			Timer -> CCMR1 |= (CompareType << TIM_CCMR1_OC2M_Pos);
			Timer -> CCER |= TIM_CCER_CC2E;
			break;
		case 3:
			Timer -> CCMR2 |= (CompareType << TIM_CCMR2_OC3M_Pos);
			Timer -> CCER |= TIM_CCER_CC3E;
			break;
		case 4:
			Timer -> CCMR2 |= (CompareType << TIM_CCMR2_OC4M_Pos);
			Timer -> CCER |= TIM_CCER_CC4E;
			break;
		default:
			ASSERT_SIMPLE(0); //Bad timer channel number
		}
	}

	void SetChannelAbility(uint8_t ChannelNumber, Ability_t Able) {
		switch (ChannelNumber) {
		case 1:
			Timer -> CCER |= TIM_CCER_CC1E;
			break;
		case 2:
			Timer -> CCER |= TIM_CCER_CC2E;
			break;
		case 3:
			Timer -> CCER |= TIM_CCER_CC3E;
			break;
		case 4:
			Timer -> CCER |= TIM_CCER_CC4E;
			break;
		default:
			ASSERT_SIMPLE(0); //Bad timer channel number
		} // switch end
	}

	void LoadCompareValue(uint8_t ChannelNumber, uint16_t OutputCompareValue) {
		switch (ChannelNumber) {
		case 1:
			Timer -> CCR1 = OutputCompareValue;
			break;
		case 2:
			Timer -> CCR2 = OutputCompareValue;
			break;
		case 3:
			Timer -> CCR3 = OutputCompareValue;
			break;
		case 4:
			Timer -> CCR4 = OutputCompareValue;
			break;
		default:
			ASSERT_SIMPLE(0); //Bad timer channel number
		} //switch end
	}

	void StartCount() {
		Timer -> CR1 |= TIM_CR1_CEN;
	}

	void StopCount(uint16_t LoadCounterValue = 0) {
		Timer -> CR1 &= ~TIM_CR1_CEN;
		Timer -> CNT = LoadCounterValue;
	}
}; // Timer_t end

#endif /* INC_TIM_H_ */
