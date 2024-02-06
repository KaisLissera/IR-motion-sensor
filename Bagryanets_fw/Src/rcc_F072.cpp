/*
 * ezhrcc.cpp
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
	SysTick->LOAD = (uint32_t)(SYS_CLK/1000 - 1);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; //Clock, interrupt, systick enable
	while(SystickCount);
}
#endif

void BlockingDelay(uint32_t ms) {
	uint32_t temp = SYS_CLK/20;
	for(volatile uint32_t i = 0; i < temp; i++) {};
}

//ezhrcc
/////////////////////////////////////////////////////////////////////

uint8_t ezhrcc::EnableLSI(uint32_t Timeout) {
	RCC->CSR |= RCC_CSR_LSION;
	while(!(RCC->CSR & RCC_CSR_LSIRDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start LSI
	}
	return retvOk;
}

uint8_t ezhrcc::EnableHSI(uint32_t Timeout) {
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start HSI
	}
	return retvOk;
}

uint8_t ezhrcc::SetFrequencyMSI(MSIFreq_t Frequency) {
	//MSIRANGE must NOT be modified when MSI is ON and NOT ready
	if(((RCC->CR & RCC_CR_MSIRDY) == 0) and ((RCC->CR & RCC_CR_MSION) == 1))
		return retvFail;
	//
	uint32_t Temp = RCC->CR;
	Temp &= ~RCC_CR_MSIRANGE_Msk;
	Temp |= Frequency << RCC_CR_MSIRANGE_Pos;
	Temp |= RCC_CR_MSIRGSEL; // Frequency range for MSI defined in CR MSIRANGE
	RCC->CR = Temp;
	return retvOk;
}

uint8_t ezhrcc::EnableMSI(MSIFreq_t Frequency, uint32_t Timeout) {
	// Setup frequency
	if(SetFrequencyMSI(Frequency) == retvFail)
		return retvFail;
	// Enable
	RCC -> CR |= RCC_CR_MSION;
	while(!(RCC->CR & RCC_CR_MSIRDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start MSI
	}
	return retvOk;
}

uint8_t ezhrcc::EnableHSE(uint32_t Timeout) {
	RCC->CR &= ~RCC_CR_HSEBYP; //HSE must not be bypassed
	RCC->CR |= RCC_CR_HSEON; //HSE on
	while(!(RCC->CR & RCC_CR_HSERDY)) {
		Timeout--;
		if (Timeout == 0)
			return retvFail; //Unable to start HSE
	}
	return retvOk;
}

uint8_t ezhrcc::EnablePLL(uint32_t Timeout) {
	RCC->CR |= RCC_CR_PLLON; //Enable PLL, PLL must NOT be used as system clock
	while(!(RCC->CR & RCC_CR_PLLRDY)) { // PLL locked
		Timeout--;
		if (Timeout == 0)
			return retvFail; //PLL not locked
	}
	return retvOk;
}

uint8_t ezhrcc::BypassHSE(uint32_t timeout) {
	RCC->CR &= ~RCC_CR_HSEON; //Disable HSE before enabling bypass
	RCC->CR &= ~RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON; //HSE enabled
	while(!(RCC->CR & RCC_CR_HSERDY)) {
		timeout--;
		if (timeout == 0)
			return retvFail; //Unable to bypass HSE
	}
	return retvOk;
}

uint8_t ezhrcc::SetSysClk(SysClkSource_t SysClkSource, uint32_t Timeout) {
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

//M - 1..8 - main & audio PLL division factor
//R - 2,4,6,8 - main PLL division factor for system clock
//N - 8..86 - main PLL multiplication factor
uint8_t ezhrcc::SetPLL(PllSource_t pllSrc, uint32_t M, uint32_t R, uint32_t N) {
	// Check arguments
	if(!(M >= 1 or M <= 8))
		return retvBadValue;
	if(!(N >= 8 and N <= 86))
		return retvBadValue;
	if(!(R == 2 or R == 4 or R == 6 or R == 8))
		return retvBadValue;
	// Transform to register values
    R = (R/2) - 1; // 2,4,6,8 => 0b00, 0b01, 0b10, 0b11
    M = (M - 1); // 1,2,3.. => 0b000, 0b001, 0b010..
    //
	uint32_t Temp = RCC->PLLCFGR;
	Temp &= ~(RCC_PLLCFGR_PLLSRC |RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLR |
			RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLREN);
	Temp |= ((uint32_t)pllSrc << RCC_PLLCFGR_PLLSRC_Pos) |
			(M << RCC_PLLCFGR_PLLM_Pos) |
			(N << RCC_PLLCFGR_PLLN_Pos) |
			(R << RCC_PLLCFGR_PLLR_Pos) |
			(1UL << RCC_PLLCFGR_PLLREN_Pos); // PLL system clock output enable
    RCC->PLLCFGR = Temp;
    ezhrcc::EnablePLL();
    return retvOk;
}

// AHB, APB1, APB2
void ezhrcc::SetBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div) {
    uint32_t Temp = RCC->CFGR;
    Temp &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);  // Clear bits
    Temp |= ((uint32_t)AHBDiv)  << RCC_CFGR_HPRE_Pos;
    Temp |= ((uint32_t)APB1Div) << RCC_CFGR_PPRE1_Pos;
    Temp |= ((uint32_t)APB2Div) << RCC_CFGR_PPRE2_Pos;
    RCC->CFGR = Temp;
}

uint32_t ezhrcc::GetCurrentSystemClock() {
    uint32_t Temp, MSIRange;
    // Get MSI Range frequency
    if((RCC->CR & RCC_CR_MSIRGSEL) == 0)
    	Temp = (RCC->CSR & RCC_CSR_MSISRANGE) >> RCC_CSR_MSISRANGE_Pos;  // MSISRANGE from RCC_CSR applies
    else
    	Temp = (RCC->CR & RCC_CR_MSIRANGE) >> RCC_CR_MSIRANGE_Pos; // MSIRANGE from RCC_CR applies
    // MSI frequency in Hz
    switch(Temp) {
    	case (msi100kHz):
			MSIRange = 100000; break;
    	case (msi200kHz):
			MSIRange = 200000; break;
    	case (msi400kHz):
			MSIRange = 400000; break;
    	case (msi800kHz):
			MSIRange = 800000; break;
    	case (msi1MHz):
			MSIRange = 1000000; break;
    	case (msi2MHz):
			MSIRange = 2000000; break;
    	case (msi4MHz):
			MSIRange = 4000000; break;
    	case (msi8MHz):
			MSIRange = 8000000; break;
    	case (msi16MHz):
			MSIRange = 1600000; break;
    	case (msi24MHz):
			MSIRange = 2400000; break;
    	case (msi32MHz):
			MSIRange = 3200000; break;
    	case (msi48MHz):
			MSIRange = 4800000; break;
    	default:
    		MSIRange = 4000000;
    } // switch(Temp)
    Temp = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;  // System clock switch status
    switch(Temp) {
        case sysClkMsi: return MSIRange;
        case sysClkHsi16: return HSI_FREQ_HZ;
#ifdef HSE_FREQ_HZ
        case sysClkHse: return HSE_FREQ_HZ;
#endif
        case sysClkPll: {
            uint32_t PllSource, M, R, N;
            PllSource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
            // M - main & audio PLL division factor
            M = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1;
            // R - main PLL division factor for system clock
            R = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1)*2;
            // N - main PLL multiplication factor
            N = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
            switch(PllSource) {
            	case pllSrcMsi:
            		return MSIRange * N / (R * M);
                case pllSrcHsi16:
                	return HSI_FREQ_HZ * N / (R * M);
#ifdef HSE_FREQ_HZ
                case pllSrcHse:
                	return HSE_FREQ_HZ * N / (R * M);
#endif
            } // Switch on PLL source
        } break; //case sysClkPll:
    } // Switch on system clock status
    return retvFail;
}

uint32_t ezhrcc::GetCurrentAHBClock() {
	uint32_t Temp = ((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos);
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

uint32_t ezhrcc::GetCurrentAPB1Clock() {
	uint32_t Temp = ((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos);
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

uint32_t ezhrcc::GetCurrentAPB2Clock() {
	uint32_t Temp = ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos);
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

uint8_t ezhrcc::SetSystemClk80MHz() {
	uint8_t retv;
	retv = ezhrcc::EnableHSI();
	if(retv != retvOk)
		return retv;
	flash::SetFlashLatency(80,highVoltageRange);
	retv = power::SetVoltageRange(highVoltageRange);
	if(retv != retvOk)
		return retv;
	retv = ezhrcc::SetPLL(pllSrcHsi16, 1, 2, 10); // 80 MHz
	if(retv != retvOk)
		return retv;
	retv = ezhrcc::SetSysClk(sysClkPll);
	if(retv != retvOk)
		return retv;
	return retvOk;
}

uint8_t ezhrcc::SetSystemClk2MHz() {
	uint8_t retv;
	flash::SetFlashLatency(2,lowVoltageRange);
	retv = ezhrcc::SetFrequencyMSI(msi2MHz);
	if(retv != retvOk)
		return retv;
	retv = ezhrcc::SetSysClk(sysClkMsi);
	if(retv != retvOk)
		return retv;
	retv = power::SetVoltageRange(lowVoltageRange);
	if(retv != retvOk)
		return retv;
	retv = power::EnterLowPowerRunMode();
	if(retv != retvOk)
		return retv;
	return retvOk;
}

uint8_t ezhrcc::SetSystemClk200kHz() {
	uint8_t retv;
	flash::SetFlashLatency(0,lowVoltageRange);
	retv = ezhrcc::SetFrequencyMSI(msi200kHz);
	if(retv != retvOk)
		return retv;
	retv = ezhrcc::SetSysClk(sysClkMsi);
	if(retv != retvOk)
		return retv;
	retv = power::SetVoltageRange(lowVoltageRange);
	if(retv != retvOk)
		return retv;
	retv = power::EnterLowPowerRunMode();
	if(retv != retvOk)
		return retv;
	return retvOk;
}

//power
/////////////////////////////////////////////////////////////////////

// High voltage range - 80 MHz max system clock
// Low voltage range - 26 MHz max system clock
retv_t power::SetVoltageRange(VoltRange_t VoltRange) {
	// Check clock frequency
	if((ezhrcc::GetCurrentAHBClock() > 26000000) and (VoltRange == lowVoltageRange))
		return retvTooHighSystemClock;
	//
	ezhrcc::EnableClkPWR();
    uint32_t Temp = PWR->CR1;
    Temp &= ~PWR_CR1_VOS;
    Temp |= VoltRange << PWR_CR1_VOS_Pos;
    PWR->CR1 = Temp;
    return retvOk;
}

retv_t power::EnterLowPowerRunMode() {
	// Check clock frequency
	if((ezhrcc::GetCurrentAHBClock() > 2000000))
		return retvTooHighSystemClock;
	//
	ezhrcc::EnableClkPWR();
	PWR->CR1 |= PWR_CR1_LPR;
	while((PWR->SR2 & PWR_SR2_REGLPF) != 0); // Check if regulator in low power run mode
	return retvOk;
}

void power::ExitLowPowerRunMode() {
	PWR->CR1 &= ~PWR_CR1_LPR;
	while((PWR->SR2 & PWR_SR2_REGLPF) == 0); // Wait regulator to enter main mode
}

//flash
/////////////////////////////////////////////////////////////////////

// Setup Flash latency depending on CPU frequency and voltage
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
