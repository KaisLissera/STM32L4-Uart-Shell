/*
 * rcc.h
 *
 *  Created on: 2023.07.31
 *      Author: Kais Lissera
 */

#ifndef INC_EZHRCC_H_
#define INC_EZHRCC_H_

#include <stm32l4xx.h>
//
#include <stdint.h>
//
#include <board.h>
#include <ezhlib.h>
#include <ezhuart.h>

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
#define HSI_FREQ_HZ     16000000UL
#define LSI_FREQ_HZ     32000UL
#define LSE_FREQ_HZ     32768UL

typedef enum {
	msi100kHz 	= 0b0000,
	msi200kHz 	= 0b0001,
	msi400kHz 	= 0b0010,
	msi800kHz 	= 0b0011,
	msi1MHz 	= 0b0100,
	msi2MHz 	= 0b0101,
	msi4MHz 	= 0b0110,
	msi8MHz 	= 0b0111,
	msi16MHz 	= 0b1000,
	msi24MHz 	= 0b1001,
	msi32MHz 	= 0b1010,
	msi48MHz 	= 0b1011
} MSIFreq_t;

typedef enum {
	apbDiv1 = 0b000, apbDiv2 = 0b100, apbDiv4 = 0b101, apbDiv8 = 0b110,	apbDiv16 = 0b111
} APBDiv_t;

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
} AHBDiv_t;

typedef enum {
	sysClkMsi = 0b00, sysClkHsi16 = 0b01, sysClkHse = 0b10, sysClkPll = 0b11
} SysClkSource_t;

typedef enum {
	pllSrcNone = 0b00, pllSrcMsi = 0b01, pllSrcHsi16 = 0b10, pllSrcHse = 0b11
} PllSource_t;

namespace ezhrcc {
	uint8_t EnableLSI(uint32_t Timeout = 0xFFF);
	uint8_t EnableHSI(uint32_t Timeout = 0xFFF);
	uint8_t EnableHSE(uint32_t Timeout = 0xFFF); // Not tested
	uint8_t EnableMSI(MSIFreq_t Frequency, uint32_t Timeout = 0xFFF); // Not implemented
	uint8_t SetFrequencyMSI(MSIFreq_t Frequency);
	uint8_t EnablePLL(uint32_t Timeout = 0xFFF);
	uint8_t BypassHSE(uint32_t Timeout = 0xFFF); // Not tested
	//
	inline void DisableLSI() {RCC -> CSR &= ~RCC_CSR_LSION;};
	inline void DisableHSI() {RCC -> CR &= ~RCC_CR_HSION;};
	inline void DisableMSI() {RCC -> CR &= ~RCC_CR_MSION;};
	inline void DisableHSE() {RCC -> CR &= ~RCC_CR_HSEON;};
	inline void DisablePLL() {RCC -> CR &= ~RCC_CR_PLLON;};
	//
	uint8_t SetSysClk(SysClkSource_t SysClkSource, uint32_t Timeout = 0xFFF);
	void SetBusDividers(AHBDiv_t AHBDiv, APBDiv_t APB1Div, APBDiv_t APB2Div);
	//
	uint8_t SetPLL(PllSource_t pllSrc, uint32_t M, uint32_t R, uint32_t N);
	//
	uint32_t GetCurrentSystemClock();
	uint32_t GetCurrentAHBClock();
	uint32_t GetCurrentAPB1Clock();
	uint32_t GetCurrentAPB2Clock();
	//
	uint8_t SetSystemClk80MHz();
	uint8_t SetSystemClk2MHz();
	uint8_t SetSystemClk200kHz();

	//Functions to enable peripheral clocks
/////////////////////////////////////////////////////////////////////
	//AHB1
#ifdef DMA2D
	inline void EnableClkDMA2D(void) {RCC->AHB1ENR |= RCC_AHB1ENR_DMA2DEN;}
#endif
	inline void EnableClkTSC(void) {RCC->AHB1ENR |= RCC_AHB1ENR_TSCEN;}
	inline void EnableClkCRC(void) {RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;}
	inline void EnableClkFLASH(void) {RCC->AHB1ENR |= RCC_AHB1ENR_FLASHEN;}
	inline void EnableClkDMA(DMA_TypeDef* Dma) {
		if(Dma == DMA1)
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		else if(Dma == DMA2)
			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		else
			ezhAssert(0); // Bad DMA name
	}

	//AHB2
	inline void EnableClkRNG(void) {RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;}
#ifdef HASH
	inline void EnableClkHASH(void) {RCC->AHB2ENR |= RCC_AHB2ENR_HASHEN;}
#endif
#ifdef AES
	inline void EnableClkAES(void) {RCC->AHB2ENR |= RCC_AHB2ENR_AESEN;}
#endif
#ifdef DCMI
	inline void EnableClkDCMI(void) {RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;}
#endif
	inline void EnableClkADC(void) {RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;}
	inline void EnableClkOTGFS(void) {RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;}
	inline void EnableClkGPIO(GPIO_TypeDef* Gpio) {
		if(Gpio == GPIOA)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		else if(Gpio == GPIOB)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		else if(Gpio == GPIOC)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		else if(Gpio == GPIOD)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
		else if(Gpio == GPIOE)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
		else if(Gpio == GPIOF)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
		else if(Gpio == GPIOG)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
		else if(Gpio == GPIOH)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
#ifdef GPIOI
		else if(Gpio == GPIOI)
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOIEN;
#endif
		else
			ezhAssert(0); // Bad GPIO name
	}

	//AHB3
	inline void EnableClkQSPI(void) {RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;}
	inline void EnableClkFMC(void) {RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;}

	//APB1
	inline void EnableClkLPTIM(LPTIM_TypeDef* Lptim) {
		if(Lptim == LPTIM1)
			RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
		else if(Lptim == LPTIM2)
			RCC->APB1ENR2 |= RCC_APB1ENR2_LPTIM2EN;
		else
			ezhAssert(0); // Bad LPTIM name
	}
	inline void EnableClkOPAMP(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_OPAMPEN;}
	inline void EnableClkDAC(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;}
	inline void EnableClkPWR(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;}
	inline void EnableClkCAN(CAN_TypeDef* Can) {
		if(Can == CAN1)
			RCC->APB1ENR1 |= RCC_APB1ENR1_CAN1EN;
#ifdef CAN2
		else if(Can == CAN2)
			RCC->APB1ENR2 |= RCC_APB1ENR2_CAN2EN;
#endif
		else
			ezhAssert(0); // Bad CAN name
	}
#ifdef CRS
	inline void EnableClkCRS(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_CRSEN;}
#endif
	inline void EnableClkI2C(I2C_TypeDef* I2c) {
		if(I2c == I2C1)
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
		else if(I2c == I2C2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;
		else if(I2c == I2C3)
			RCC->APB1ENR1 |= RCC_APB1ENR1_I2C3EN;
#ifdef I2C4
		else if(I2c == I2C4)
			RCC->APB1ENR1 |= RCC_APB1ENR2_I2C4EN;
#endif
		else
			ezhAssert(0); // Bad I2C name
	}
	inline void EnableClkUART(USART_TypeDef* Uart) {
		if(Uart == LPUART1)
			RCC->APB1ENR1 |= RCC_APB1ENR2_LPUART1EN;
		else if(Uart == USART2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
		else if(Uart == USART3)
			RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
		else if(Uart == UART4)
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
		else if(Uart == UART5)
			RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
		else
			ezhAssert(0); // Bad UART name
	}
	inline void EnableClkSPI(SPI_TypeDef* Spi) {
		if(Spi == SPI1)
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // APB2!
		else if(Spi == SPI2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
		else if(Spi == SPI3)
			RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
		else
			ezhAssert(0); // Bad SPI name
	}
	inline void EnableClkWWDG(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_WWDGEN;}
#ifdef RTCAPB
	inline void EnableClkRTCAPB(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;}
#endif
	inline void EnableClkLCD(void) {RCC->APB1ENR1 |= RCC_APB1ENR1_LCDEN;}
	inline void EnableClkTIM(TIM_TypeDef* Tim) {
		if(Tim == TIM1)
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // APB2
		else if(Tim == TIM2)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
		else if(Tim == TIM3)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
		else if(Tim == TIM4)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
		else if(Tim == TIM5)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
		else if(Tim == TIM6)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
		else if(Tim == TIM7)
			RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
		else if(Tim == TIM8)
			RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // APB2
		else if(Tim == TIM15)
			RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // APB2
		else if(Tim == TIM16)
			RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; // APB2
		else if(Tim == TIM17)
			RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; // APB2
		else
			ezhAssert(0); // Bad timer name
	}
	inline void EnableClkSWP(void) {RCC->APB1ENR2 |= RCC_APB1ENR2_SWPMI1EN;}

	//APB2
	inline void EnableClkDFSDM(void) {RCC->APB2ENR |= RCC_APB2ENR_DFSDM1EN;}
	inline void EnableClkSAI(SAI_TypeDef* Sai) {
		if(Sai == SAI1)
			RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
		else if(Sai == SAI2)
			RCC->APB2ENR |= RCC_APB2ENR_SAI2EN;
		else
			ezhAssert(0); // Bad SAI name
	}
	inline void EnableClkSDMMC(void) {RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;}
	inline void EnableClkFW(void) {RCC->APB2ENR |= RCC_APB2ENR_FWEN;}
	inline void EnableClkSYSCFG(void) {RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;}

	//Functions to disable peripheral clocks
/////////////////////////////////////////////////////////////////////
	//AHB1
#ifdef DMA2D
	inline void DisableClkDMA2D(void) {RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2DEN;}
#endif
	inline void DisableClkTSC(void) {RCC->AHB1ENR &= ~RCC_AHB1ENR_TSCEN;}
	inline void DisableClkCRC(void) {RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;}
	inline void DisableClkFLASH(void) {RCC->AHB1ENR &= ~RCC_AHB1ENR_FLASHEN;}
	inline void DisableClkDMA(DMA_TypeDef* Dma) {
		if(Dma == DMA1)
			RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
		else if(Dma == DMA2)
			RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
		else
			ezhAssert(0); // Bad DMA name
	}

	//AHB2
	inline void DisableClkRNG(void) {RCC->AHB2ENR &= ~RCC_AHB2ENR_RNGEN;}
#ifdef HASH
	inline void DisableClkHASH(void) {RCC->AHB2ENR &= ~RCC_AHB2ENR_HASHEN;}
#endif
#ifdef AES
	inline void DisableClkAES(void) {RCC->AHB2ENR &= ~RCC_AHB2ENR_AESEN;}
#endif
#ifdef DCMI
	inline void DisableClkDCMI(void) {RCC->AHB2ENR &= ~RCC_AHB2ENR_DCMIEN;}
#endif
	inline void DisableClkADC(void) {RCC->AHB2ENR &= ~RCC_AHB2ENR_ADCEN;}
	inline void DisableClkOTGFS(void) {RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;}
	inline void DisableClkGPIO(GPIO_TypeDef* Gpio) {
		if(Gpio == GPIOA)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
		else if(Gpio == GPIOB)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
		else if(Gpio == GPIOC)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOCEN;
		else if(Gpio == GPIOD)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIODEN;
		else if(Gpio == GPIOE)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOEEN;
		else if(Gpio == GPIOF)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOFEN;
		else if(Gpio == GPIOG)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOGEN;
		else if(Gpio == GPIOH)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOHEN;
#ifdef GPIOI
		else if(Gpio == GPIOI)
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOIEN;
#endif
		else
			ezhAssert(0); // Bad GPIO name
	}

	//AHB3
	inline void DisableClkQSPI(void) {RCC->AHB3ENR &= ~RCC_AHB3ENR_QSPIEN;}
	inline void DisableClkFMC(void) {RCC->AHB3ENR &= ~RCC_AHB3ENR_FMCEN;}

	//APB1
	inline void DisableClkLPTIM(LPTIM_TypeDef* Lptim) {
		if(Lptim == LPTIM1)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_LPTIM1EN;
		else if(Lptim == LPTIM2)
			RCC->APB1ENR2 &= ~RCC_APB1ENR2_LPTIM2EN;
		else
			ezhAssert(0); // Bad LPTIM name
	}
	inline void DisableClkOPAMP(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_OPAMPEN;}
	inline void DisableClkDAC(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_DAC1EN;}
	inline void DisableClkPWR(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;}
	inline void DisableClkCAN(CAN_TypeDef* Can) {
		if(Can == CAN1)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_CAN1EN;
#ifdef CAN2
		else if(Can == CAN2)
			RCC->APB1ENR2 &= ~RCC_APB1ENR2_CAN2EN;
#endif
		else
			ezhAssert(0); // Bad CAN name
	}
#ifdef CRS
	inline void DisableClkCRS(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_CRSEN;}
#endif
	inline void DisableClkI2C(I2C_TypeDef* I2c) {
		if(I2c == I2C1)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C1EN;
		else if(I2c == I2C2)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C2EN;
		else if(I2c == I2C3)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C3EN;
#ifdef I2C4
		else if(I2c == I2C4)
			RCC->APB1ENR1 &= ~RCC_APB1ENR2_I2C4EN;
#endif
		else
			ezhAssert(0); // Bad I2C name
	}
	inline void DisableClkUART(USART_TypeDef* Uart) {
		if(Uart == LPUART1)
			RCC->APB1ENR1 &= ~RCC_APB1ENR2_LPUART1EN;
		else if(Uart == USART2)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
		else if(Uart == USART3)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART3EN;
		else if(Uart == UART4)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART4EN;
		else if(Uart == UART5)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_UART5EN;
		else
			ezhAssert(0); // Bad UART name
	}
	inline void DisableClkSPI(SPI_TypeDef* Spi) {
		if(Spi == SPI1)
			RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN; // APB2!
		else if(Spi == SPI2)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI2EN;
		else if(Spi == SPI3)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN;
		else
			ezhAssert(0); // Bad SPI name
	}
	inline void DisableClkWWDG(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_WWDGEN;}
#ifdef RTCAPB
	inline void DisableClkRTCAPB(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_RTCAPBEN;}
#endif
	inline void DisableClkLCD(void) {RCC->APB1ENR1 &= ~RCC_APB1ENR1_LCDEN;}
	inline void DisableClkTIM(TIM_TypeDef* Tim) {
		if(Tim == TIM1)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN; // APB2
		else if(Tim == TIM2)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM2EN;
		else if(Tim == TIM3)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN;
		else if(Tim == TIM4)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM4EN;
		else if(Tim == TIM5)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM5EN;
		else if(Tim == TIM6)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM6EN;
		else if(Tim == TIM7)
			RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM7EN;
		else if(Tim == TIM8)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM8EN; // APB2
		else if(Tim == TIM15)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN; // APB2
		else if(Tim == TIM16)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN; // APB2
		else if(Tim == TIM17)
			RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN; // APB2
		else
			ezhAssert(0); // Bad timer name
	}
	inline void DisableClkSWP(void) {RCC->APB1ENR2 &= ~RCC_APB1ENR2_SWPMI1EN;}

	//APB2
	inline void DisableClkDFSDM(void) {RCC->APB2ENR &= ~RCC_APB2ENR_DFSDM1EN;}
	inline void DisableClkSAI(SAI_TypeDef* Sai) {
		if(Sai == SAI1)
			RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN;
		else if(Sai == SAI2)
			RCC->APB2ENR &= ~RCC_APB2ENR_SAI2EN;
		else
			ezhAssert(0); // Bad SAI name
	}
	inline void DisableClkSDMMC(void) {RCC->APB2ENR &= ~RCC_APB2ENR_SDMMC1EN;}
	inline void DisableClkFW(void) {RCC->APB2ENR &= ~RCC_APB2ENR_FWEN;}
	inline void DisableClkSYSCFG(void) {RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;}
};//ezhrcc end

constexpr uint32_t ReturnDivToPLLR(uint32_t div){
	if(div == 2)
		return 0b00UL;
	if(div == 4)
		return 0b01UL;
	if(div == 6)
		return 0b10UL;
	if(div == 8)
		return 0b11UL;
	return 0;
}

//Sleep and low power modes
/////////////////////////////////////////////////////////////////////

typedef enum {
	DetectionOnHigh = 0,
	DetectionOnLow	= 1,
} WakeupEdge_t;

typedef enum {
	highVoltageRange = 0b01, lowVoltageRange = 0b10
} VoltRange_t;

namespace power {
	retv_t SetVoltageRange(VoltRange_t VoltRange);
	retv_t EnterLowPowerRunMode();
	void ExitLowPowerRunMode();

	inline void EnableWakeup1(WakeupEdge_t Edge) {
		if(Edge == DetectionOnHigh)
			PWR->CR4 &= ~PWR_CR4_WP1;
		else
			PWR->CR4 |= PWR_CR4_WP1;
		PWR->CR3 |= PWR_CR3_EWUP1;
	}

	inline void EnableWakeup2(WakeupEdge_t Edge) {
		if(Edge == DetectionOnHigh)
			PWR->CR4 &= ~PWR_CR4_WP2;
		else
			PWR->CR4 |= PWR_CR4_WP2;
		PWR->CR3 |= PWR_CR3_EWUP2;
	}

	inline void EnableWakeup3(WakeupEdge_t Edge) {
		if(Edge == DetectionOnHigh)
			PWR->CR4 &= ~PWR_CR4_WP3;
		else
			PWR->CR4 |= PWR_CR4_WP3;
		PWR->CR3 |= PWR_CR3_EWUP3;
	}

	inline void EnableWakeup4(WakeupEdge_t Edge) {
		if(Edge == DetectionOnHigh)
			PWR->CR4 &= ~PWR_CR4_WP4;
		else
			PWR->CR4 |= PWR_CR4_WP4;
		PWR->CR3 |= PWR_CR3_EWUP4;
	}

	inline void EnableWakeup5(WakeupEdge_t Edge) {
		if(Edge == DetectionOnHigh)
			PWR->CR4 &= ~PWR_CR4_WP5;
		else
			PWR->CR4 |= PWR_CR4_WP5;
		PWR->CR3 |= PWR_CR3_EWUP5;
	}

	inline void EnableExtiWakeUpEXAMPLE() {
		ezhrcc::EnableClkSYSCFG();
		EXTI->PR1 = EXTI_PR1_PIF13_Msk; //Clear EXTI interrupt
		SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13_Msk;
		SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // EXTI 13 line connected to PC13
		EXTI->FTSR1 |= EXTI_FTSR1_FT13; // EXTI 13 line trigger on falling edge
		EXTI->RTSR1 |= EXTI_RTSR1_RT13;
		EXTI->IMR1 &= ~EXTI_IMR1_IM13; // No interrupt mask
		EXTI->EMR1 |= EXTI_EMR1_EM13; // Event mask
	}

	inline void EnterShutdown() {
		PWR->SCR = 0x1F; //Clear wake up flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 &= ~PWR_CR1_LPMS;
		PWR->CR1 |= PWR_CR1_LPMS_SHUTDOWN;
	    __WFI();
	}

	inline void EnterStandby() {
		PWR->SCR = 0x1F; //Clear wakeup flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 &= ~PWR_CR1_LPMS;
		PWR->CR1 |= PWR_CR1_LPMS_STANDBY;
	    __WFI();
	}
	// Does not work, can't restore clock
	inline void EnterStop0() {
		PWR->SCR = 0x1F; //Clear wakeup flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 &= ~PWR_CR1_LPMS;
		PWR->CR1 |= PWR_CR1_LPMS_STOP0;
		__SEV();
		__WFE();
		__WFE();
	}
	// Does not work, can't restore clock
	inline void EnterStop1() {
		PWR->SCR = 0x1F; //Clear wakeup flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 &= ~PWR_CR1_LPMS;
		PWR->CR1 |= PWR_CR1_LPMS_STOP1;
		__SEV();
		__WFE();
		__WFE();
	}
	// Does not work, can't restore clock
	inline void EnterStop2() {
		PWR->SCR = 0x1F; //Clear wakeup flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		PWR->CR1 &= ~PWR_CR1_LPMS;
		PWR->CR1 |= PWR_CR1_LPMS_STOP2;
		__SEV();
		__WFE();
		__WFE();
	}

	// Does not work with FreeRTOS
	inline void EnterSleep() {
		PWR->SCR = 0x1F; //Clear wakeup flags
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		__SEV();
		__WFE();
		__WFE();
	}
};

//Simple functions to measure execution time
/////////////////////////////////////////////////////////////////////

namespace tick_counter {
	inline void Init(void) {
		CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT -> CYCCNT = 0;
	}
	inline void Start(void) {DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;}
	inline void Pause(void) {DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;}
	inline uint32_t GetValue(void) {return DWT->CYCCNT;}
	inline void Clear(void) {
		DWT -> CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
		DWT -> CYCCNT = 0;
	}
}; //tick_counter end

//FLASH setup
/////////////////////////////////////////////////////////////////////


namespace flash {
	void SetFlashLatency(uint8_t AhbClkMHz, VoltRange_t VoltRange);
}

//NVIC setup
/////////////////////////////////////////////////////////////////////

namespace ezhnvic {
	inline void SetupIrq(IRQn_Type IRQ, uint8_t Priority) {
		NVIC_EnableIRQ(IRQ);
		NVIC_SetPriority(IRQ, Priority);
	}
}

#endif /* INC_EZHRCC_H_ */
