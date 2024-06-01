/*
 * gpio.h
 *
 *  Created on: Aug 12, 2023
 *      Author: Kais Lissera
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include <stdint.h>
//
#include <lib.h>
//
#include <rcc_F072.h>

//GPIO setup
/////////////////////////////////////////////////////////////////////

typedef enum {
	Input 				= 0b00UL,
	GeneralOutput 		= 0b01UL,
	AlternateFunction 	= 0b10UL,
	Analog 				= 0b11UL
} PinMode_t;

typedef enum {
	PullAir 	= 0b00UL,
	PullUp 		= 0b01UL,
	PullDown 	= 0b10UL
} PinPupd_t;

typedef enum {
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
} AltFunction_t;

namespace gpio {
	void SetupPin(GPIO_TypeDef* Gpio, uint8_t Pin, PinPupd_t Pupd, PinMode_t Mode, AltFunction_t Af = AF0);
	void SetPinMode(GPIO_TypeDef* _GPIO, uint32_t _PIN ,uint32_t mode);
	void SetPinPupd(GPIO_TypeDef* _GPIO, uint32_t _PIN ,uint32_t pupd);
	void SetPinAltFunction(GPIO_TypeDef* _GPIO, uint32_t _PIN, uint32_t AltFunc);
	//
	inline void ActivatePin(GPIO_TypeDef* _GPIO, uint32_t _PIN) {
		_GPIO->BSRR = 0b1UL << _PIN;
	}
	inline void DeactivatePin(GPIO_TypeDef* _GPIO, uint32_t _PIN) {
		_GPIO->BSRR = 0b1UL << (_PIN + 16);
	}
	inline void TogglePin(GPIO_TypeDef* _GPIO, uint32_t _PIN) {
		_GPIO->ODR ^= 0b1UL << _PIN;
	}
	uint8_t GetPinInput(GPIO_TypeDef* _GPIO, uint32_t _PIN);
}//GPIO end

//Simple buttons
/////////////////////////////////////////////////////////////////////

typedef enum {
	Idle,
	HoldDown,
	Pressed,
	Released
} ButtonState_t;

class Button_t {
private:
	GPIO_TypeDef* Gpio;
	uint8_t Pin;
	uint8_t IdleState;
	uint8_t PreviousState;
public:
	Button_t(GPIO_TypeDef* _Gpio, uint8_t _Pin, PinPupd_t _PupdType) {
		Gpio =_Gpio;
		Pin = _Pin;
		//
		switch(_PupdType) {
		case(PullUp):
			IdleState = 1;
			PreviousState = 1;
			break;
		case(PullDown):
			IdleState = 0;
			PreviousState = 0;
			break;
		default:
			ASSERT_SIMPLE(0);
		}
	}

	void Init(){
		gpio::SetupPin(Gpio,Pin,PullUp,Input);
	}

	ButtonState_t CheckState();
};

#endif /* INC_GPIO_H_ */
