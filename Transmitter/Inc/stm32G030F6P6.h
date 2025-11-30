/*
 * stm32G030F6P6.h
 *
 *  Created on: Aug 16, 2025
 *      Author: kiscs
 */

#ifndef STM32G030F6P6_H_
#define STM32G030F6P6_H_


#include <setup.h>
#include <stdint.h>



/*BUS addresses */

#define	APB1_BUS_ADDRESS			0x40000000
#define APB2_BUS_ADDRESS			0x40010000
#define	AHB_BUS_ADDRESS				0x40020000
#define GPIOA_ADDRESS				0x50000000


/*Base address*/

#define GPIOA_BASE_ADDRESS			(GPIOA_ADDRESS + 0x0000)
#define USART2_BASE_ADDRESS			(APB1_BUS_ADDRESS + 0x4400)
#define ADC_BASE_ADDRESS			(APB2_BUS_ADDRESS + 0x2400)
#define RCC_BASE_ADDRESS 			(AHB_BUS_ADDRESS + 0x1000)

/* Peripheral registers mapping*/

#define GPIOA						((GPIO_register_map*) GPIOA_BASE_ADDRESS)
#define USART2						((USART_register_map*) USART2_BASE_ADDRESS)
#define ADC							((ADC_register_map*) ADC_BASE_ADDRESS)
#define RCC							((RCC_register_map*) RCC_BASE_ADDRESS)


	/*****GPIO REGISTER MAP TYPEDEF STRUCT *****/

typedef struct{

	volatile uint32_t MODER;		// offset 0x00 (Mode register)
	volatile uint32_t OTYPER;		// offset 0x04 (Output type register)
	volatile uint32_t OSPEEDR;		// offset 0x08 (Output speed register)
	volatile uint32_t PUPDR;		// offset 0x0C (Pull-up/Pull-down register)
	volatile uint32_t IDR;			// offset 0x10 (Input data register)
	volatile uint32_t ODR;			// offset 0x14 (Output data register)
	volatile uint32_t BSRR;			// offset 0x18 (Bit set/reset register)
	volatile uint32_t LCKR;			// offset 0x1C (Configuration lock register)
	volatile uint32_t AFR[2];		// offset 0x20 & 0x24 (Alternate function low/high register)
	volatile uint32_t BRR;			// offset 0x28 (Bit reset register)

} GPIO_register_map;



	/*****RCC REGISTER MAP TYPEDEF STRUCT *****/

typedef struct {
    volatile uint32_t CR;        // 0x00 (Clock control register)
    volatile uint32_t ICSCR;     // 0x04 (Internal clock sources calibration register)
    volatile uint32_t CFGR;      // 0x08 (Clock configuration register)
    volatile uint32_t PLLCFGR;   // 0x0C (PLL configuration register)
    volatile uint32_t RESERVED0; // 0x10
    volatile uint32_t CRRCR;     // 0x14 (Clock recovery RC register)
    volatile uint32_t CIER;      // 0x18 (Clock interrupt enable register)
    volatile uint32_t CIFR;      // 0x1C (Clock interrupt flag register)
    volatile uint32_t CICR;      // 0x20 (Clock interrupt clear register)
    volatile uint32_t IOPRSTR;   // 0x24 (IO port reset register)
    volatile uint32_t AHBRSTR;   // 0x28 (AHB peripheral reset register)
    volatile uint32_t APBRSTR1;  // 0x2C (APB peripheral reset register 1)
    volatile uint32_t APBRSTR2;  // 0x30 (APB peripheral reset register 2)
    volatile uint32_t IOPENR;    // 0x34 (IO port clock enable register)
    volatile uint32_t AHBENR;    // 0x38 (AHB peripheral clock enable register)
    volatile uint32_t APBENR1;   // 0x3C (APB peripheral clock enable register 1)
    volatile uint32_t APBENR2;   // 0x40 (APB peripheral clock enable register 2)
    volatile uint32_t IOPSMENR;  // 0x44 (IO port clock enable in sleep mode register)
    volatile uint32_t AHBSMENR;  // 0x48 (AHB peripheral clock enable in sleep mode register)
    volatile uint32_t APBSMENR1; // 0x4C (APB peripheral clock enable in sleep mode register 1)
    volatile uint32_t APBSMENR2; // 0x50 (APB peripheral clock enable in sleep mode register 2)
    volatile uint32_t CCIPR;     // 0x54 (Peripherals independent clock configuration register 1)
    volatile uint32_t CCIPR2;    // 0x58 (Peripherals independent clock configuration register 2)
    volatile uint32_t BDCR;      // 0x5C (Backup domain control register)
    volatile uint32_t CSR;       // 0x60 (Control/status register)
} RCC_register_map;


	/*****USART REGISTER MAP TYPEDEF STRUCT *****/

typedef struct {
    volatile uint32_t CR1;      // 0x00 (Control register 1)
    volatile uint32_t CR2;      // 0x04 (Control register 2)
    volatile uint32_t CR3;      // 0x08 (Control register 3)
    volatile uint32_t BRR;      // 0x0C (Baud rate register)
    volatile uint32_t GTPR;     // 0x10 (Guard time and prescaler register)
    volatile uint32_t RTOR;     // 0x14 (Receiver timeout register)
    volatile uint32_t RQR;      // 0x18 (Request register)
    volatile uint32_t ISR;      // 0x1C (Interrupt & status register)
    volatile uint32_t ICR;      // 0x20 (Interrupt flag clear register)
    volatile uint32_t RDR;      // 0x24 (Receive data register)
    volatile uint32_t TDR;      // 0x28 (Transmit data register)
    volatile uint32_t PRESC;    // 0x2C (Prescaler register)
} USART_register_map;


	/*****ADC REGISTER MAP TYPEDEF STRUCT *****/

typedef struct {
    volatile uint32_t ISR;      // 0x00 (Interrupt and status register)
    volatile uint32_t IER;      // 0x04	(Interrupt enable register)
    volatile uint32_t CR;       // 0x08 (Control register)
    volatile uint32_t CFGR1;    // 0x0C (Configuration register 1)
    volatile uint32_t CFGR2;    // 0x10 (Configuration register 2)
    volatile uint32_t SMPR;     // 0x14 (Sampling time register)
    volatile uint32_t RESERVED0;// 0x18
    volatile uint32_t RESERVED1;// 0x1C
    volatile uint32_t AWD1TR;   // 0x20 (Watchdog threshold register 1)
    volatile uint32_t AWD2TR;   // 0x24 (Watchdog threshold register 2)
    volatile uint32_t CHSELR;   // 0x28 (Channel selection register)
    volatile uint32_t AWD3TR;   // 0x2C (Watchdog threshold register 3)
    volatile uint32_t RESERVED2;// 0x30
    volatile uint32_t RESERVED3;// 0x34
    volatile uint32_t RESERVED4;// 0x38
    volatile uint32_t RESERVED5;// 0x3C
    volatile uint32_t DR;       // 0x40 (data register)
} ADC_register_map;



#endif /* STM32G030F6P6_H_ */
