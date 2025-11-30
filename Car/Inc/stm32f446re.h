
#ifndef STM32F446RE_H_
#define STM32F446RE_H_

#include <stdint.h>


/*BUS addresses*/
#define APB2								0x40010000u											// ADC1, TIM1
#define AHB1								0x40020000u											// RCC, GPIOA

/*Offsets*/
#define RCC_OFFSET							((0x40023800u) - (AHB1))
#define GPIOA_OFFSET				 		((0x40020000u) - (AHB1))
#define GPIOB_OFFSET                        ((0x40020400u) - (AHB1))
#define GPIOC_OFFSET          			    ((0x40020800u) - (AHB1))
#define ADC1_OFFSET							((0x40012000u) - (APB2))
#define TIM1_OFFSET							((0x40010000u) - (APB2))
#define TIM8_OFFSET							((0x40010400) - (APB2))
#define USART1_OFFSET						((0x40011000u) - (APB2))

/*Base addresses*/
#define RCC_BASE_ADDRESS					((AHB1) + (RCC_OFFSET))
#define GPIOA_BASE_ADDRESS					((AHB1) + (GPIOA_OFFSET))
#define GPIOB_BASE_ADDRESS                  ((AHB1) + (GPIOB_OFFSET))
#define GPIOC_BASE_ADDRESS      			((AHB1) + (GPIOC_OFFSET))
#define ADC1_BASE_ADDRESS					((APB2) + (ADC1_OFFSET))
#define TIM1_BASE_ADDRESS					((APB2) + (TIM1_OFFSET))
#define TIM8_BASE_ADDRESS					((APB2) + (TIM8_OFFSET))
#define USART1_BASE_ADDRESS					((APB2) + (USART1_OFFSET))
#define NVIC_BASE_ADDRESS  				     0xE000E100u


/* Peripheral registers mapping*/
#define RCC									((RCC_register_map*) RCC_BASE_ADDRESS)
#define GPIOA								((GPIO_register_map*) GPIOA_BASE_ADDRESS)
#define GPIOB                               ((GPIO_register_map*) GPIOB_BASE_ADDRESS)
#define GPIOC               			    ((GPIO_register_map*) GPIOC_BASE_ADDRESS)
#define ADC1								((ADC_register_map*) ADC1_BASE_ADDRESS)
#define TIM1								((TIM_register_map*) TIM1_BASE_ADDRESS)
#define TIM8								((TIM_register_map*) TIM8_BASE_ADDRESS)
#define USART1								((USART_register_map*) USART1_BASE_ADDRESS)
#define NVIC               					((NVIC_register_map*) NVIC_BASE_ADDRESS)

/***** RCC REGISTER MAP TYPEDEF STRUCT *****/
typedef struct
{
    volatile uint32_t CR;           // offset 0x00 (Clock control register)
    volatile uint32_t PLLCFGR;      // offset 0x04 (PLL configuration register)
    volatile uint32_t CFGR;         // offset 0x08 (Clock configuration register)
    volatile uint32_t CIR;          // offset 0x0C (Clock interrupt register)
    volatile uint32_t AHB1RSTR;     // offset 0x10 (AHB1 peripheral reset register)
    volatile uint32_t AHB2RSTR;     // offset 0x14 (AHB2 peripheral reset register)
    volatile uint32_t AHB3RSTR;     // offset 0x18 (AHB3 peripheral reset register)
    volatile uint32_t RESERVED0;    // offset 0x1C (Reserved)
    volatile uint32_t APB1RSTR;     // offset 0x20 (APB1 peripheral reset register)
    volatile uint32_t APB2RSTR;     // offset 0x24 (APB2 peripheral reset register)
    volatile uint32_t RESERVED1[2]; // offset 0x28–0x2C (Reserved)
    volatile uint32_t AHB1ENR;      // offset 0x30 (AHB1 peripheral clock enable register)
    volatile uint32_t AHB2ENR;      // offset 0x34 (AHB2 peripheral clock enable register)
    volatile uint32_t AHB3ENR;      // offset 0x38 (AHB3 peripheral clock enable register)
    volatile uint32_t RESERVED2;    // offset 0x3C (Reserved)
    volatile uint32_t APB1ENR;      // offset 0x40 (APB1 peripheral clock enable register)
    volatile uint32_t APB2ENR;      // offset 0x44 (APB2 peripheral clock enable register)
    volatile uint32_t RESERVED3[2]; // offset 0x48–0x4C (Reserved)
    volatile uint32_t AHB1LPENR;    // offset 0x50 (AHB1 clock enable in low power mode)
    volatile uint32_t AHB2LPENR;    // offset 0x54 (AHB2 clock enable in low power mode)
    volatile uint32_t AHB3LPENR;    // offset 0x58 (AHB3 clock enable in low power mode)
    volatile uint32_t RESERVED4;    // offset 0x5C (Reserved)
    volatile uint32_t APB1LPENR;    // offset 0x60 (APB1 clock enable in low power mode)
    volatile uint32_t APB2LPENR;    // offset 0x64 (APB2 clock enable in low power mode)
    volatile uint32_t RESERVED5[2]; // offset 0x68–0x6C (Reserved)
    volatile uint32_t BDCR;         // offset 0x70 (Backup domain control register)
    volatile uint32_t CSR;          // offset 0x74 (Clock control & status register)
    volatile uint32_t RESERVED6[2]; // offset 0x78–0x7C (Reserved)
    volatile uint32_t SSCGR;        // offset 0x80 (Spread spectrum clock generation register)
    volatile uint32_t PLLI2SCFGR;   // offset 0x84 (PLLI2S configuration register)
    volatile uint32_t PLLSAICFGR;   // offset 0x88 (PLLSAI configuration register)
    volatile uint32_t DCKCFGR;      // offset 0x8C (Dedicated clocks configuration register)
    volatile uint32_t CKGATENR;     // offset 0x90 (Clocks gated enable register)
    volatile uint32_t DCKCFGR2;     // offset 0x94 (Dedicated clocks configuration register 2)
} RCC_register_map;

/***** GPIO REGISTER MAP TYPEDEF STRUCT *****/
typedef struct
{
    volatile uint32_t MODER;   // offset 0x00 (Mode register)
    volatile uint32_t OTYPER;  // offset 0x04 (Output type register)
    volatile uint32_t OSPEEDR; // offset 0x08 (Output speed register)
    volatile uint32_t PUPDR;   // offset 0x0C (Pull-up/Pull-down register)
    volatile uint32_t IDR;     // offset 0x10 (Input data register)
    volatile uint32_t ODR;     // offset 0x14 (Output data register)
    volatile uint32_t BSRR;    // offset 0x18 (Bit set/reset register)
    volatile uint32_t LCKR;    // offset 0x1C (Configuration lock register)
    volatile uint32_t AFR[2];  // offset 0x20 (AFRL: alternate function low), 0x24 (AFRH: alternate function high)
} GPIO_register_map;

/***** ADC REGISTER MAP TYPEDEF STRUCT *****/
typedef struct
{
    volatile uint32_t SR;       // 0x00: status register
    volatile uint32_t CR1;      // 0x04: control register 1
    volatile uint32_t CR2;      // 0x08: control register 2
    volatile uint32_t SMPR1;    // 0x0C: sample time register 1
    volatile uint32_t SMPR2;    // 0x10: sample time register 2
    volatile uint32_t JOFR1;    // 0x14: injected channel data offset register 1
    volatile uint32_t JOFR2;    // 0x18: injected channel data offset register 2
    volatile uint32_t JOFR3;    // 0x1C: injected channel data offset register 3
    volatile uint32_t JOFR4;    // 0x20: injected channel data offset register 4
    volatile uint32_t HTR;      // 0x24: watchdog higher threshold register
    volatile uint32_t LTR;      // 0x28: watchdog lower threshold register
    volatile uint32_t SQR1;     // 0x2C: regular sequence register 1
    volatile uint32_t SQR2;     // 0x30: regular sequence register 2
    volatile uint32_t SQR3;     // 0x34: regular sequence register 3
    volatile uint32_t JSQR;     // 0x38: injected sequence register
    volatile uint32_t JDR1;     // 0x3C: injected data register 1
    volatile uint32_t JDR2;     // 0x40: injected data register 2
    volatile uint32_t JDR3;     // 0x44: injected data register 3
    volatile uint32_t JDR4;     // 0x48: injected data register 4
    volatile uint32_t DR;       // 0x4C: regular data register
} ADC_register_map;

/***** TIM1 & TIM8 REGISTER MAP TYPEDEF STRUCT *****/
typedef struct
{
    volatile uint32_t CR1;     // offset 0x00 (Control register 1)
    volatile uint32_t CR2;     // offset 0x04 (Control register 2)
    volatile uint32_t SMCR;    // offset 0x08 (Slave mode control register)
    volatile uint32_t DIER;    // offset 0x0C (DMA/Interrupt enable register)
    volatile uint32_t SR;      // offset 0x10 (Status register)
    volatile uint32_t EGR;     // offset 0x14 (Event generation register)
    volatile uint32_t CCMR1;   // offset 0x18 (Capture/compare mode register 1)
    volatile uint32_t CCMR2;   // offset 0x1C (Capture/compare mode register 2)
    volatile uint32_t CCER;    // offset 0x20 (Capture/compare enable register)
    volatile uint32_t CNT;     // offset 0x24 (Counter)
    volatile uint32_t PSC;     // offset 0x28 (Prescaler)
    volatile uint32_t ARR;     // offset 0x2C (Auto-reload register)
    volatile uint32_t RCR;     // offset 0x30 (Repetition counter register)
    volatile uint32_t CCR[4];  // offset 0x34–0x40 (Capture/compare registers 1–4)
    volatile uint32_t BDTR;    // offset 0x44 (Break and dead-time register)
    volatile uint32_t DCR;     // offset 0x48 (DMA control register)
    volatile uint32_t DMAR;    // offset 0x4C (DMA address for full transfer)
} TIM_register_map;

/***** USART REGISTER MAP TYPEDEF STRUCT *****/
typedef struct
{
    volatile uint32_t SR;   // offset 0x00 (Status register)
    volatile uint32_t DR;   // offset 0x04 (Data register)
    volatile uint32_t BRR;  // offset 0x08 (Baud rate register)
    volatile uint32_t CR1;  // offset 0x0C (Control register 1)
    volatile uint32_t CR2;  // offset 0x10 (Control register 2)
    volatile uint32_t CR3;  // offset 0x14 (Control register 3)
    volatile uint32_t GTPR; // offset 0x18 (Guard time & prescaler register)
} USART_register_map;

/***** EXTI REGISTER MAP TYPEDEF STRUCT *****/
typedef struct
{
    volatile uint32_t IMR;   // offset 0x00 (Interrupt mask register)
    volatile uint32_t EMR;   // offset 0x04 (Event mask register)
    volatile uint32_t RTSR;  // offset 0x08 (Rising trigger selection register)
    volatile uint32_t FTSR;  // offset 0x0C (Falling trigger selection register)
    volatile uint32_t SWIER; // offset 0x10 (Software interrupt event register)
    volatile uint32_t PR;    // offset 0x14 (Pending register)
} EXTI_register_map;

/*****NVIC TYPEDEF STRUCT*****/

typedef struct {
    volatile uint32_t ISER[8];   			     // 0x000 - Interrupt Set Enable
    volatile uint32_t RESERVED0[24];
    volatile uint32_t ICER[8];    				 // 0x080 - Interrupt Clear Enable
    volatile uint32_t RESERVED1[24];
    volatile uint32_t ISPR[8];     				 // 0x100 - Interrupt Set Pending
    volatile uint32_t RESERVED2[24];
    volatile uint32_t ICPR[8];    				 // 0x180 - Interrupt Clear Pending
    volatile uint32_t RESERVED3[24];
    volatile uint32_t IABR[8];    				 // 0x200 - Active Bit Register
    volatile uint32_t RESERVED4[56];
    volatile uint8_t  IPR[240];   				 // 0x300 - Interrupt Priority
} NVIC_register_map;

/*****USART SELECTION STRUCT*****/
typedef struct
{
	volatile uint16_t forward;
	volatile uint16_t backward;
	volatile uint16_t setpoint;
	volatile uint16_t deadzone_vrx;
	volatile uint16_t deadzone_vry;
	volatile int16_t previous_setp_calc;

}USART_values;

#endif /* STM32F446RE_H_ */
