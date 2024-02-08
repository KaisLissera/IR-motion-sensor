/*
 * gpio.cpp
 *
 *  Created on: Aug 12, 2023
 *      Author: KONSTANTIN
 */

#include <gpio_F072.h>

//ezhgpio
/////////////////////////////////////////////////////////////////////

void gpio::SetupPin(GPIO_TypeDef* Gpio, uint8_t Pin, PinPupd_t Pupd, PinMode_t Mode, AltFunction_t Af) {
	//Enable port clock
	rcc::EnableClkGPIO(Gpio);
	// Setup mode
	Gpio -> MODER &= ~(0b11UL << (Pin*2));
	Gpio -> MODER |= Mode << (Pin*2);
	// Setup pull-up/pull-down
	Gpio -> PUPDR &= ~(0b11UL << (Pin*2));
	Gpio -> PUPDR |= Pupd << (Pin*2);
	// Setup pin alternate function
	if(Pin < 8) {
		Gpio -> AFR[0] &= ~(0b0000UL << (4*Pin));
		Gpio -> AFR[0] |= Af << (4*Pin);
	}
	else{
		Gpio -> AFR[1] &= ~(0b0000UL << (4*Pin));
		Gpio -> AFR[1] |= Af << (4*Pin);
	}
}

//PIN_Input = 00; PIN_GeneralOutput = 01; PIN_AlternateFunction = 10; PIN_Analog = 11
void gpio::SetPinMode(GPIO_TypeDef* Gpio, uint32_t Pin ,uint32_t Mode) {
	Gpio -> MODER &= ~(0b11UL << (Pin*2));
	Gpio -> MODER |= Mode << (Pin*2);
}

//PIN_NoPUPD = 00; PIN_PU = 01; PIN_PD = 10
void gpio::SetPinPupd(GPIO_TypeDef* Gpio, uint32_t Pin ,uint32_t Pupd) {
	Gpio -> PUPDR &= ~(0b11UL << (Pin*2));
	Gpio -> PUPDR |= Pupd << (Pin*2);
}

void gpio::SetPinAltFunction(GPIO_TypeDef* Gpio, uint32_t Pin, uint32_t Af) {
	if(Pin < 8) {
		Gpio -> AFR[0] &= ~(0b0000UL << (4*Pin));
		Gpio -> AFR[0] |= Af << (4*Pin);
	}
	else{
		Gpio -> AFR[1] &= ~(0b0000UL << (4*Pin));
		Gpio -> AFR[1] |= Af << (4*Pin);
	}
} //GPIO_ezh::SetAltFuncPIN

uint8_t gpio::GetPinInput(GPIO_TypeDef* Gpio, uint32_t Pin) {
	if((Gpio->IDR & (0b1UL << Pin)) == 0)
		return 0;
	else
		return 1;
}

//Simple buttons
/////////////////////////////////////////////////////////////////////

ButtonState_t Button_t::CheckState() {
	uint8_t CurrentState = gpio::GetPinInput(Gpio, Pin);
	if(CurrentState == IdleState) {
		if(CurrentState != PreviousState) {
			PreviousState = CurrentState;
			return Released;
		}
		else
			return Idle;
	} else {
		if(CurrentState != PreviousState) {
			PreviousState = CurrentState;
			return Pressed;
		}
		else
			return HoldDown;
	}
}
