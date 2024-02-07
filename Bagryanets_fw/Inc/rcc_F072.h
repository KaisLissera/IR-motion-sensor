/*
 * rcc_F072.h
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include <stdint.h>
//
#include <lib_F072.h>

//Convert milliseconds to FreeRTOS ticks
#ifdef FREE_RTOS_TICK
constexpr uint32_t MsToFreeRtosTick(uint32_t ms) {
	return ms*FREE_RTOS_TICK/1000;
}
#endif

//Simple delays
/////////////////////////////////////////////////////////////////////

//Attention! SysTick delay conflicts with FreeRTOS
#if (USE_SYSTICK_DELAY == 1)
volatile static uint32_t systickCount;
void DelayMs(uint32_t ms);

extern "C"
void SysTick_Handler(void);
#endif

void BlockingDelay(uint32_t ms);

//Functions to setup system clock and peripheral clock control
/////////////////////////////////////////////////////////////////////

//Frequencies of the internal oscillators in Hz
#define HSI_FREQ_HZ     8000000UL
#define HSI14_FREQ_HZ   14000000UL // Dedicated for ADC
#define HSI48_FREQ_HZ   48000000UL // Primarily for USB
#define LSI_FREQ_HZ     40000UL
#define LSE_FREQ_HZ     32768UL

typedef enum {
	apbDiv1 = 0b000, apbDiv2 = 0b100, apbDiv4 = 0b101, apbDiv8 = 0b110,	apbDiv16 = 0b111
} ApbDiv_t; // PCLK prescaler  - RCC_CFGR_PPRE

typedef enum {
    ahbDiv1		= 0b0000,
    ahbDiv2		= 0b1000,
    ahbDiv4		= 0b1001,
    ahbDiv8		= 0b1010,
    ahbDiv16	= 0b1011,
    ahbDiv64	= 0b1100,
    ahbDiv128	= 0b1101,
    ahbDiv256	= 0b1110,
    ahbDiv512	= 0b1111
} AhbDiv_t; // HCLK prescaler - RCC_CFGR_HPRE

typedef enum {
	sysClkHsi = 0b00, sysClkHse = 0b01, sysClkPll = 0b10, sysClkHsi48 = 0b11
} SysClkSource_t; // System clock switch status

typedef enum {
	pllSrcHsiDiv2 = 0b00, pllSrcHsiPrediv = 0b01, pllSrcHsePrediv = 0b10, pllSrcHsi48Prediv = 0b11
} PllSource_t; // PLL input clock source

namespace rcc {
	uint8_t EnableLSI(uint32_t Timeout = 0xFFF);
	uint8_t EnableHSI(uint32_t Timeout = 0xFFF);
	uint8_t EnableHSE(uint32_t Timeout = 0xFFF);
	uint8_t EnableHSI14(uint32_t Timeout = 0xFFF);
	uint8_t EnableHSI48(uint32_t Timeout = 0xFFF);
	uint8_t EnablePLL(uint32_t Timeout = 0xFFF);
	// Need to add timeout until RDY bit is cleared !!!!
	inline void DisableLSI() {RCC -> CSR &= ~RCC_CSR_LSION;};
	inline void DisableHSI() {RCC -> CR &= ~RCC_CR_HSION;};
	inline void DisableHSI14() {RCC -> CR2 &= ~RCC_CR2_HSI14ON;};
	inline void DisableHSI48() {RCC -> CR2 &= ~RCC_CR2_HSI48ON;};
	inline void DisableHSE() {RCC -> CR &= ~RCC_CR_HSEON;};
	inline void DisablePLL() {RCC -> CR &= ~RCC_CR_PLLON;};
	//
	uint8_t SetSysClk(SysClkSource_t SysClkSource, uint32_t Timeout = 0xFFF);
	void SetBusDividers(AhbDiv_t AhbDiv, ApbDiv_t ApbDiv);

	// 2 <= pllMul <= 16, 1 <= pllPrediv <= 16
	uint8_t SetupPLL(PllSource_t pllSrc, uint32_t pllMul, uint32_t pllPrediv);
	//
	uint32_t GetCurrentSystemClock();
	uint32_t GetCurrentAHBClock();
	uint32_t GetCurrentAPBClock();
	//
//	uint8_t SetSystemClk48MHz();
//	uint8_t SetSystemClk8MHz();
//	uint8_t SetSystemClk4MHz();

	//Functions to enable peripheral clocks
/////////////////////////////////////////////////////////////////////
	//AHB
	inline void EnableClkTSC(void) {RCC->AHBENR |= RCC_AHBENR_TSCEN;}
	inline void EnableClkGPIO(GPIO_TypeDef* Gpio) {
		if(Gpio == GPIOA)
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		else if(Gpio == GPIOB)
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		else if(Gpio == GPIOC)
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
		else if(Gpio == GPIOD)
			RCC->AHBENR |= RCC_AHBENR_GPIODEN;
		else if(Gpio == GPIOE)
			RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
		else if(Gpio == GPIOF)
			RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
		else
			ASSERT_SIMPLE(0); // Bad GPIO name
	}
	inline void EnableClkCRC(void) {RCC->AHBENR |= RCC_AHBENR_CRCEN;}
	inline void EnableClkFLITF(void) {RCC->AHBENR |= RCC_AHBENR_FLITFEN;}
	inline void EnableClkSRAM(void) {RCC->AHBENR |= RCC_AHBENR_SRAMEN;}
	inline void EnableClkDMA(DMA_TypeDef* Dma) {
		if(Dma == DMA1)
			RCC->AHBENR |= RCC_AHBENR_DMA1EN;
#ifdef DMA2
		else if(Dma == DMA2)
			RCC->AHBENR |= RCC_AHBENR_DMA2EN;
#endif
		else
			ASSERT_SIMPLE(0); // Bad DMA name
	}

	//APB
	inline void EnableClkDBGMCU(void) {RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;}
	inline void EnableClkTIM(TIM_TypeDef* Tim) {
		if(Tim == TIM1)
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		else if(Tim == TIM2)
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		else if(Tim == TIM3)
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		else if(Tim == TIM6)
			RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
		else if(Tim == TIM7)
			RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
		else if(Tim == TIM14)
			RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		else if(Tim == TIM15)
			RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
		else if(Tim == TIM16)
			RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
		else if(Tim == TIM17)
			RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
		else
			ASSERT_SIMPLE(0); // Bad timer name
	}
	inline void EnableClkUART(USART_TypeDef* Uart) {
		if(Uart == USART1)
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		else if(Uart == USART2)
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		else if(Uart == USART3)
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		else if(Uart == USART4)
			RCC->APB1ENR |= RCC_APB1ENR_USART4EN;
#ifdef USART6
		else if(Uart == USART5)
			RCC->APB1ENR |= RCC_APB1ENR_USART5EN;
#endif
#ifdef USART6
		else if(Uart == USART6)
			RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
#endif
#ifdef USART7
		else if(Uart == USART7)
			RCC->APB2ENR |= RCC_APB2ENR_USART7EN;
#endif
#ifdef USART8
		else if(Uart == USART8)
			RCC->APB2ENR |= RCC_APB2ENR_USART8EN;
#endif
		else
			ASSERT_SIMPLE(0); // Bad UART name
	}
		inline void EnableClkSPI(SPI_TypeDef* Spi) {
		if(Spi == SPI1)
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		else if(Spi == SPI2)
			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		else
			ASSERT_SIMPLE(0); // Bad SPI name
	}
	inline void EnableClkADC(void) {RCC->APB2ENR |= RCC_APB2ENR_ADCEN;}
	inline void EnableClkSYSCFGCOMP(void) {RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;}
	inline void EnableClkCEC(void) {RCC->APB1ENR |= RCC_APB1ENR_CECEN;}
	inline void EnableClkDAC(void) {RCC->APB1ENR |= RCC_APB1ENR_DACEN;}
	inline void EnableClkPWR(void) {RCC->APB1ENR |= RCC_APB1ENR_PWREN;}
	inline void EnableClkCRS(void) {RCC->APB1ENR |= RCC_APB1ENR_CRSEN;}
	inline void EnableClkCAN(void) {RCC->APB1ENR |= RCC_APB1ENR_CANEN;}
	inline void EnableClkUSB(void) {RCC->APB1ENR |= RCC_APB1ENR_USBEN;}
	inline void EnableClkI2C(I2C_TypeDef* I2c) {
		if(I2c == I2C1)
			RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		else if(I2c == I2C2)
			RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		else
			ASSERT_SIMPLE(0); // Bad I2C name
	}
	inline void EnableClkWWDG(void) {RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;}

	//Functions to disable peripheral clocks
/////////////////////////////////////////////////////////////////////
	//AHB
	inline void DisableClkTSC(void) {RCC->AHBENR &= ~RCC_AHBENR_TSCEN;}
	inline void DisableClkGPIO(GPIO_TypeDef* Gpio) {
		if(Gpio == GPIOA)
			RCC->AHBENR &= ~RCC_AHBENR_GPIOAEN;
		else if(Gpio == GPIOB)
			RCC->AHBENR &= ~RCC_AHBENR_GPIOBEN;
		else if(Gpio == GPIOC)
			RCC->AHBENR &= ~RCC_AHBENR_GPIOCEN;
		else if(Gpio == GPIOD)
			RCC->AHBENR &= ~RCC_AHBENR_GPIODEN;
		else if(Gpio == GPIOE)
			RCC->AHBENR &= ~RCC_AHBENR_GPIOEEN;
		else if(Gpio == GPIOF)
			RCC->AHBENR &= ~RCC_AHBENR_GPIOFEN;
		else
			ASSERT_SIMPLE(0); // Bad GPIO name
	}
	inline void DisableClkCRC(void) {RCC->AHBENR &= ~RCC_AHBENR_CRCEN;}
	inline void DisableClkFLITF(void) {RCC->AHBENR &= ~RCC_AHBENR_FLITFEN;}
	inline void DisableClkSRAM(void) {RCC->AHBENR &= ~RCC_AHBENR_SRAMEN;}
	inline void DisableClkDMA(DMA_TypeDef* Dma) {
		if(Dma == DMA1)
			RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;
#ifdef DMA2
		else if(Dma == DMA2)
			RCC->AHBENR &= ~RCC_AHBENR_DMA2EN;
#endif
		else
			ASSERT_SIMPLE(0); // Bad DMA name
	}

	//APB
	inline void DisableClkDBGMCU(void) {RCC->APB2ENR &= ~RCC_APB2ENR_DBGMCUEN;}
	inline void DisableClkTIM(TIM_TypeDef* Tim) {
		if(Tim == TIM1)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
		else if(Tim == TIM2)
			RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
		else if(Tim == TIM3)
			RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
		else if(Tim == TIM6)
			RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
		else if(Tim == TIM7)
			RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
		else if(Tim == TIM14)
			RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
		else if(Tim == TIM15)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN;
		else if(Tim == TIM16)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN;
		else if(Tim == TIM17)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN;
		else
			ASSERT_SIMPLE(0); // Bad timer name
	}
	inline void DisableClkUART(USART_TypeDef* Uart) {
		if(Uart == USART1)
			RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
		else if(Uart == USART2)
			RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
		else if(Uart == USART3)
			RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
		else if(Uart == USART4)
			RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN;
#ifdef USART6
		else if(Uart == USART5)
			RCC->APB1ENR &= ~RCC_APB1ENR_USART5EN;
#endif
#ifdef USART6
		else if(Uart == USART6)
			RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
#endif
#ifdef USART7
		else if(Uart == USART7)
			RCC->APB2ENR &= ~RCC_APB2ENR_USART7EN;
#endif
#ifdef USART8
		else if(Uart == USART8)
			RCC->APB2ENR &= ~RCC_APB2ENR_USART8EN;
#endif
		else
			ASSERT_SIMPLE(0); // Bad UART name
	}
		inline void DisableClkSPI(SPI_TypeDef* Spi) {
		if(Spi == SPI1)
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
		else if(Spi == SPI2)
			RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
		else
			ASSERT_SIMPLE(0); // Bad SPI name
	}
	inline void DisableClkADC(void) {RCC->APB2ENR &= ~RCC_APB2ENR_ADCEN;}
	inline void DisableClkSYSCFGCOMP(void) {RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGCOMPEN;}
	inline void DisableClkCEC(void) {RCC->APB1ENR &= ~RCC_APB1ENR_CECEN;}
	inline void DisableClkDAC(void) {RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;}
	inline void DisableClkPWR(void) {RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;}
	inline void DisableClkCRS(void) {RCC->APB1ENR &= ~RCC_APB1ENR_CRSEN;}
	inline void DisableClkCAN(void) {RCC->APB1ENR &= ~RCC_APB1ENR_CANEN;}
	inline void DisableClkUSB(void) {RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;}
	inline void DisableClkI2C(I2C_TypeDef* I2c) {
		if(I2c == I2C1)
			RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
		else if(I2c == I2C2)
			RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
		else
			ASSERT_SIMPLE(0); // Bad I2C name
	}
	inline void DisableClkWWDG(void) {RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;}
};//rcc end

//Sleep and low power modes
/////////////////////////////////////////////////////////////////////

namespace power {
	// If wakeup enabled pin forced low, WKUP on rising edge
	// PA0
	inline void EnableWakeup1(void) {PWR->CSR |= PWR_CSR_EWUP1;}
	// PC13
	inline void EnableWakeup2(void) {PWR->CSR |= PWR_CSR_EWUP2;}
	// PE6
	inline void EnableWakeup3(void) {PWR->CSR |= PWR_CSR_EWUP3;}
	// PA2
	inline void EnableWakeup4(void) {PWR->CSR |= PWR_CSR_EWUP4;}
	// PC5
	inline void EnableWakeup5(void) {PWR->CSR |= PWR_CSR_EWUP5;}
	// PB5
	inline void EnableWakeup6(void) {PWR->CSR |= PWR_CSR_EWUP6;}
	// PB15
	inline void EnableWakeup7(void) {PWR->CSR |= PWR_CSR_EWUP7;}

	inline void EnterSleep() { // Core clock disabled, all peripherals running
		PWR->CR = PWR_CR_CWUF | PWR_CR_CSBF; //Clear wakeup flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		__WFI();
	} // Wake-up - interrupt/wake-up event

	inline void EnterStop() { // All clocks are stopped
		PWR->CR = PWR_CR_CWUF | PWR_CR_CSBF; //Clear wakeup flags
		PWR->CR &= ~PWR_CR_PDDS;
		PWR->CR |= PWR_CR_LPDS;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		__WFI();
	} // Wake-up - EXTI line

	inline void EnterStandby() { // VDD domain povwered-off
		PWR->CR = PWR_CR_CWUF | PWR_CR_CSBF; //Clear wakeup flags
		PWR->CR |= PWR_CR_PDDS;
		PWR->CR |= PWR_CR_LPDS;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	    __WFI();
	} // Wake-up - WKUP pin, RTC, NRST, IWDG
};

//FLASH setup
/////////////////////////////////////////////////////////////////////

namespace flash {
	void SetFlashLatency(uint8_t AhbClkMHz);
}

//NVIC setup
/////////////////////////////////////////////////////////////////////

namespace nvic {
	inline void SetupIrq(IRQn_Type IRQ, uint8_t Priority) {
		NVIC_EnableIRQ(IRQ);
		NVIC_SetPriority(IRQ, Priority);
	}
}

#endif /* INC_RCC_H_ */
