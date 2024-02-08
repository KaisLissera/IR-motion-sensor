/*
 * rcc_F072.cpp
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#include <lib_F072.h>
//
#include <rcc_F072.h>

//Simple delays
/////////////////////////////////////////////////////////////////////

#if (USE_SYSTICK_DELAY == 1)
extern "C"
void SysTick_Handler(void) {
	if(systickCount > 0) systickCount--;
}

void DelayMs(uint32_t ms) {
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn, 0);
	systickCount = ms;
	SysTick->VAL = 0x0u;
	// In stm32f0x7x SysTick frequency equals core frequency HCLK divided by 8
	uint32_t Clk = rcc::GetCurrentSystemClock();
	SysTick->LOAD = (uint32_t)(Clk >> 13); // SYS_CLK/8/1000 + 1 - precise value
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; //Clock, interrupt, systick enable
	while(systickCount);
}
#endif

void BlockingDelay(uint32_t ms) {
	uint32_t Clk = rcc::GetCurrentSystemClock();
	uint32_t temp = ms*Clk >> 4; // Check why 20!!!
	for(volatile uint32_t i = 0; i < temp; i++) {};
}

//rcc
/////////////////////////////////////////////////////////////////////

uint8_t rcc::EnableLSI(uint32_t Timeout) {
	RCC->CSR |= RCC_CSR_LSION;
	while(!(RCC->CSR & RCC_CSR_LSIRDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start LSI
	}
	return retvOk;
}

uint8_t rcc::EnableHSI8(uint32_t Timeout) {
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start HSI
	}
	return retvOk;
}

uint8_t rcc::EnableHSI14(uint32_t Timeout) {
	RCC->CR2 |= RCC_CR2_HSI14ON;
	while(!(RCC->CR2 & RCC_CR2_HSI14RDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start HSI14
	}
	return retvOk;
}

uint8_t rcc::EnableHSI48(uint32_t Timeout) {
	RCC->CR2 |= RCC_CR2_HSI48ON;
	while(!(RCC->CR2 & RCC_CR2_HSI48RDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start HSI14
	}
	return retvOk;
}

uint8_t rcc::EnableHSE(uint32_t Timeout) {
	RCC->CR &= ~RCC_CR_HSEBYP; //HSE must not be bypassed
	RCC->CR |= RCC_CR_HSEON; //HSE on
	while(!(RCC->CR & RCC_CR_HSERDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start HSE
	}
	return retvOk;
}

uint8_t rcc::EnablePLL(uint32_t Timeout) {
	RCC->CR |= RCC_CR_PLLON; //Enable PLL, PLL must NOT be used as system clock
	while(!(RCC->CR & RCC_CR_PLLRDY)) { // PLL locked
		Timeout--;
		if (Timeout == 0)
			return retvFail; //PLL not locked
	}
	return retvOk;
}

// sysClkHsi, sysClkHse, sysClkPll, sysClkHsi48
uint8_t rcc::SwitchSysClk(SysClkSource_t SysClkSource, uint32_t Timeout) {
	RCC->CFGR &= ~RCC_CFGR_SW; // Clear
	RCC->CFGR |= (SysClkSource << RCC_CFGR_SW_Pos); // Switch system clock
	// Check system clock switch status
	while((RCC->CFGR & RCC_CFGR_SWS_Msk) != ((uint32_t)SysClkSource << RCC_CFGR_SWS_Pos)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to switch system clock
	}
	return retvOk;
}

// 2 <= pllMul <= 16, 1 <= pllPrediv <= 16
uint8_t rcc::SetupPLL(PllSource_t pllSrc, uint32_t pllMul, uint32_t pllPreDiv) {
	rcc::DisablePLL();
	// Check arguments
	if (pllPreDiv < 1 or pllPreDiv > 16)
		return retvBadValue;
	if (pllMul < 2 and pllMul > 16)
		return retvBadValue;
	// Transform to register values
	pllPreDiv = pllPreDiv - 1; // 1,2,3 => 0b0000, 0b0001
	// 0b1110, 0b1111 - PLLMUL x16
	pllMul = pllMul - 2; // 2,3,4 => 0b0000, 0b0001
    //
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV_Msk;
	RCC->CFGR2 |= pllPreDiv << RCC_CFGR2_PREDIV_Pos;
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= pllMul << RCC_CFGR_PLLMUL_Pos;
	// Select pll source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC;
	RCC->CFGR |= pllSrc << RCC_CFGR_PLLSRC_Pos;

    uint8_t retVal = rcc::EnablePLL();
    return retVal;
}

// AHB, APB
void rcc::SetBusDividers(AhbDiv_t AhbDiv, ApbDiv_t ApbDiv) {
    uint32_t Temp = RCC->CFGR;
    Temp &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE_Msk);  // Clear bits
    Temp |= AhbDiv  << RCC_CFGR_HPRE_Pos;
    Temp |= ApbDiv << RCC_CFGR_PPRE_Pos;
    RCC->CFGR = Temp;
}

uint32_t rcc::GetCurrentSystemClock() {
    uint32_t Temp = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;  // System clock switch status
    switch(Temp) {
        case sysClkHsi48: return HSI48_FREQ_HZ;
        case sysClkHsi: return HSI8_FREQ_HZ;
#ifdef HSE_FREQ_HZ
        case sysClkHse: return HSE_FREQ_HZ;
#endif
        case sysClkPll: {
            uint32_t PllSource = (RCC->CFGR & RCC_CFGR_PLLSRC_Msk) >> RCC_CFGR_PLLSRC_Pos;
            uint32_t pllPreDiv = (RCC->CFGR2 & RCC_CFGR2_PREDIV_Msk) >> RCC_CFGR2_PREDIV_Pos;
            pllPreDiv = pllPreDiv + 1; // Conversion to divider
            uint32_t pllMul = (RCC->CFGR & RCC_CFGR_PLLMUL_Msk) >> RCC_CFGR_PLLMUL_Pos;
            pllMul = pllMul + 2; // Conversion to multiplier
            switch(PllSource) {
            	case pllSrcHsiDiv2:
            		return HSI8_FREQ_HZ*pllMul/(pllPreDiv*2);
                case pllSrcHsiPrediv:
                	return HSI8_FREQ_HZ*pllMul/pllPreDiv;
#ifdef HSE_FREQ_HZ
                case pllSrcHsePrediv:
                	return HSE_FREQ_HZ*pllMul/pllPreDiv;
#endif
                case pllSrcHsi48Prediv:
                	return HSI48_FREQ_HZ*pllMul/pllPreDiv;
            } // Switch on PLL source
        } break; //case sysClkPll:
    } // Switch on system clock status
    return retvFail;
}

uint32_t rcc::GetCurrentAHBClock() {
	uint32_t Temp = ((RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos);
	uint32_t SysClk = GetCurrentSystemClock();
	switch(Temp) {
		case ahbDiv1: return SysClk;
		case ahbDiv2: return SysClk >> 1;
		case ahbDiv4: return SysClk >> 2;
		case ahbDiv8: return SysClk >> 3;
		case ahbDiv16: return SysClk >> 4;
		case ahbDiv64: return SysClk >> 6;
		case ahbDiv128: return SysClk >> 7;
		case ahbDiv256: return SysClk >> 8;
		case ahbDiv512: return SysClk >> 9;
		default:
			return retvFail;
	}
}

uint32_t rcc::GetCurrentAPBClock() {
	uint32_t Temp = ((RCC->CFGR & RCC_CFGR_PPRE_Msk) >> RCC_CFGR_PPRE_Pos);
	uint32_t AhbClk = GetCurrentAHBClock();
	switch(Temp) {
		case apbDiv1: return AhbClk;
		case apbDiv2: return AhbClk >> 1;
		case apbDiv4: return AhbClk >> 2;
		case apbDiv8: return AhbClk >> 3;
		case apbDiv16: return AhbClk >> 4;
		default:
			return retvFail;
	}
}

//flash
/////////////////////////////////////////////////////////////////////

// Setup Flash latency depending on CPU frequency
void flash::SetFlashLatency(uint8_t AhbClkMHz) {
    uint32_t Temp = FLASH->ACR;
    Temp &= ~FLASH_ACR_LATENCY_Msk;
    Temp |= FLASH_ACR_PRFTBE; // Enable prefetch
    if (AhbClkMHz <= 24)
    	Temp |= 0b000 << FLASH_ACR_LATENCY_Pos;
    else
    	Temp |= 0b001 << FLASH_ACR_LATENCY_Pos;

    FLASH->ACR = Temp;
//    while(FLASH->ACR != tmp);
}
