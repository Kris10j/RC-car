
#ifndef SETUP_H_
#define SETUP_H_



#include <stdint.h>
#include "stm32f446re.h"
#include "interrupt.h"



void GPIO_initialize (void);
void ADC1_initialize (void);
void TIM1_initialize (void);
void TIM8_initialize (void);
uint32_t TIM1_formulas(void);
uint16_t TIM1_value_scale(uint16_t adc_value);
uint16_t ADC1_servo (void);
void ADC1_start_conversion (void);
void USART1_RX_initialize (void);
void USART1_RX_interrupt (void);
void USART1_baud_setting_9600 (void);
void USART1_selection (USART_values *sum_values);
void turn_selection(float u);
void car_start_position(USART_values* start);
void USART1_reset (void);
void simple_delay(void);
void enable_fpu(void);

/****************RCC MACROS ****************/

/*Peripheral clock enable*/
#define RCC_AHB1ENR_GPIOA 						(1u << 0)								// GPIO clock
#define RCC_AHB1ENR_GPIOB  					    (1u << 1)								// GPIOB clock
#define RCC_AHB1ENR_GPIOC     				    (1u << 2)								// GPIOC clock
#define RCC_APB2ENR_USART1						(1u << 4)								// USART1 clock
#define RCC_APB2ENR_TIM1						(1u << 0)								// TIM1 clock
#define RCC_APB2ENR_TIM8						(1u << 1)								// TIM8 clock
#define RCC_APB2ENR_ADC1						(1u << 8)								// ADC1 clock

/****************GPIO MACROS ****************/

/*MODER - Port mode register*/
#define AF										2u										// 0b10 (Alternate function mode)
#define ANALOG									3u										// 0b11 (Analog mode)

/*MODER - set and clear PA0, PA8, PA9, PA10, PA11, PB7, */
#define CLEAR_MODER_PA0							~(3u << (0 * 2))
#define CLEAR_MODER_PA1							~(3u << (1 * 2))
#define CLEAR_MODER_PA4							~(3u << (4 * 2))
#define CLEAR_MODER_PA8							~(3u << (8 * 2))
#define CLEAR_MODER_PA9							~(3u << (9 * 2))
#define CLEAR_MODER_PA10						~(3u << (10 * 2))
#define CLEAR_MODER_PA11						~(3u << (11 * 2))
#define CLEAR_MODER_PB7							~(3u << (7 * 2))
#define CLEAR_MODER_PC6							~(3u << (6*2))
#define SET_PA0_ANALOG							(ANALOG << (0 * 2))
#define SET_PA1_OUTPUT							(1 << (1 * 2))
#define SET_PA4_OUTPUT							(1 << (4 * 2))
#define SET_PA8_AF								(AF << (8 * 2))
#define SET_PA9_AF								(AF << (9 * 2))
#define SET_PA10_AF								(AF << (10 * 2))
#define SET_PA11_AF								(AF << (11 * 2))
#define SET_PB7_AF								(AF << (7 * 2))
#define SET_PC6_AF								(AF << (6*2))

/*PUPDR - No pull-up, pull-down configuration*/
#define CLEAR_PA0_NO_PULL						~(3u << (0 * 2))
#define CLEAR_PA1_NO_PULL						~(3u << (1 * 2))
#define CLEAR_PA4_NO_PULL						~(3u << (4 * 2))
#define CLEAR_PA7_NO_PULL						~(3u << (7 * 2))
#define CLEAR_PA8_NO_PULL						~(3u << (8 * 2))
#define CLEAR_PA9_NO_PULL						~(3u << (9 * 2))
#define CLEAR_PA10_NO_PULL						~(3u << (10 * 2))
#define CLEAR_PA11_NO_PULL						~(3u << (11 * 2))
#define CLEAR_PB7_NO_PULL						~(3u << (7 * 2))
#define CLEAR_PC6_NO_PULL						~(3u << (6*2))
#define SET_PA1_PULL_DOWN						(1u << (1 * 2))
#define SET_PA4_PULL_DOWN						(1u << (4 * 2))
#define SET_PB7_PULL							(1u << (7 * 2))

/*OTYPER - clear PA8, PA9, PA10, PA11 [00 - to configure the output type of the I/O port]*/
#define CLEAR_PA1_OUT_PUSH_PULL					~(1u << 1)
#define CLEAR_PA4_OUT_PUSH_PULL					~(1u << 4)
#define CLEAR_PA8_OUT_PUSH_PULL					~(1u << 8)
#define CLEAR_PA9_OUT_PUSH_PULL					~(1u << 9)
#define CLEAR_PA10_OUT_PUSH_PULL				~(1u << 10)
#define CLEAR_PA11_OUT_PUSH_PULL				~(1u << 11)
#define CLEAR_PC6_OUT_PUSH_PULL					~(1u << 6)

/*OSPEEDR - output speed register*/
#define HIGH_SPEED								3u

/*OSPEEDR - set and clear PA8, PA9, PA10, PA11  [11: High speed]*/
#define CLEAR_OSPEEDR_PA1						~(3u << (1 * 2))
#define CLEAR_OSPEEDR_PA4						~(3u << (4 * 2))
#define CLEAR_OSPEEDR_PA8						~(3u << (8 * 2))
#define CLEAR_OSPEEDR_PA9						~(3u << (9 * 2))
#define CLEAR_OSPEEDR_PA10						~(3u << (10 * 2))
#define CLEAR_OSPEEDR_PA11						~(3u << (11 * 2))
#define CLEAR_OSPEEDR_PC6						~(3u << (6*2))
#define SET_PA1_HIGH_SPEED						(HIGH_SPEED << (1 * 2))
#define SET_PA4_HIGH_SPEED						(HIGH_SPEED << (4 * 2))
#define SET_PA8_HIGH_SPEED						(HIGH_SPEED << (8 * 2))
#define SET_PA9_HIGH_SPEED						(HIGH_SPEED << (9 * 2))
#define SET_PA10_HIGH_SPEED						(HIGH_SPEED << (10 * 2))
#define SET_PA11_HIGH_SPEED						(HIGH_SPEED << (11 * 2))
#define SET_PC6_HIGH_SPEED						(HIGH_SPEED << (6*2))

/*AFR - alternate function, low registers*/
#define AFR1									1u								// 0001: AF1
#define AFR7									7u								// 0111: AF7

/*AFR set and clear PA8, PA9, PA10, PA11, PB7*/
#define CLEAR_AFR1_PA8							~(15u << (0 * 4))				//AFR high register
#define CLEAR_AFR1_PA9							~(15u << (1 * 4))				//AFR high register
#define CLEAR_AFR1_PA10							~(15u << (2 * 4))				//AFR high register
#define CLEAR_AFR1_PA11							~(15u << (3 * 4))				//AFR high register
#define CLEAR_AFR7_PB7							~(15u << (7 * 4))				//AFR low register
#define CLEAR_AFR3_PC6							~(15u << (6 * 4))				//AFR high register
#define SET_AFR1_PA8							(AFR1 << (0 * 4))				//AFR high register
#define SET_AFR1_PA9							(AFR1 << (1 * 4))				//AFR high register
#define SET_AFR1_PA10							(AFR1 << (2 * 4))				//AFR high register
#define SET_AFR1_PA11							(AFR1 << (3 * 4))				//AFR high register
#define SET_AFR7_PB7							(AFR7 << (7 * 4))				//AFR low register
#define SET_AFR3_PC6							(3u  << (6*4))

/*BSRR - Bit set/reset register - AIN 1, AIN2 enable/disable*/
//PA1 (AIN1)
#define BSRR_AIN1_PA1_SET						 (1 << 1)						//PA1 HIGH
#define BSRR_AIN1_PA1_RESET						 (1 << 17)						//PA1 LOW

//PA4 (AIN2)
#define BSRR_AIN2_PA4_SET						 (1 << 4)						//PA4 HIGH
#define BSRR_AIN2_PA4_RESET 					 (1 << 20)						//PA4 LOW

/****************ADC MACROS ****************/

/*SMPR2 - Sampling time register*/
#define SMP0									3u								//SMP0

/*SMPR2 - Sampling time register set 56 cycles, [2:0]*/
#define CLEAR_SMPR_PA0							~(7u << (3 * 0))
#define SET_SMPR_PA0							(SMP0 << (3 * 0))

/*CR1 - Control register 1*/
#define RES_10_BIT								1u								// resolution 10 bit [01]

/*CR1 - Resolution register [1:0]*/
#define CLEAR_RES								~(3u << 24)
#define SET_RES_10_BIT							(RES_10_BIT << 24)

/*CR2 - Control register 2*/
#define ALIGN									0u								//Right alignment = 0
#define CONT_SINGLE								0u								//Single conversion = 0
#define ADON_ENABLE								1u								//Enable ADC = 1

/*CR2 -  Set and clear right data alignment (ALIGN), bit 11*/
#define CLEAR_RIGHT_ALIGN						~(1 << 11)
#define SET_RIGHT_ALIGN							(ALIGN << 11)

/*CR2 - Set and clear Single conversion mode, bit 1*/
#define	CLEAR_CONT_SINGLE						~(1u << 1)
#define SET_CONT_SINGLE							(CONT_SINGLE << 1)

/*CR2 - A/D converter ON/OFF (ADON), bit 0*/
#define CLEAR_ADON								~(1 << 0)
#define SET_ADON_ENABLE							(ADON_ENABLE << 0)

/*CR2 - Start conversion of regular channels (SWSTART), bit 30*/
#define SWSTART_ENABLE							(1u << 30)						//Starts conversion of regular channels = 1

/*SR - Status register*/
#define EOC_CONV_NOT_COMP					   	~(1 << 1)						//Conversion not complete = 0
#define EOC_CONV_COMP							(1 << 1)						//Conversion complete = 1

/*SQR1 - Regular sequence register 1*/
#define SQR1_CONV								~(0xFu << 20)					//Regular channel sequence length -> 1 conversion [0000]
/*SQR3 - Regular sequence register 3*/
#define SQR1_CH_0								~(0x1Fu << 0)					//1st conversion, select 0 channel
/*Deadzone*/
#define DEADZONE_MIN							442u
#define DEADZONE_MAX							582u
/*ADC max*/
#define ADC_MAX_1023							1023u
#define ADC_MIN_0								0u

/****************USART MACROS ****************/

/*DR -> Receiver enable*/
#define CLEAR_RECEIVER							~(1u << 2)						//RE = 0
#define RECEIVER_ENABLE							(1u << 2)						//RE = 1

/*DR -> World length settings*/
#define CLEAR_WORLD_LENGTH						~(3u << 12)
#define SET_WORLD_LENGTH_8_BIT					(0u << 12)						//M = 0 (1 Start bit, 8 Data bits, n Stop bit)

/*DR -> Oversampling mode (OVER8)*/
#define CLEAR_OVERSAMPLING						~(1u << 15)
#define SET_OVERSAMPLING_16						(0u << 15)						//Oversampling by 16

/*CR2 -> STOP bits [STOP]*/
#define CLEAR_STOP_BITS							~(3u << 12)
#define SET_STOP_BIT_1							(0u << 12)						//1 stop bit

/*USARTDIV standart generation*/
#define F_CLOCK									16000000u						//Basic clock = 16 MHz
#define OVER8									0u								//Oversampling 16 = 0
#define RX_BAUD_9600							9600u							//Baud rate = 9600

/*CR1: USART1 enable [UE]*/
#define USART1_DISABLE							USART1 -> CR1 &= ~(1u << 13)	//UE = 0
#define USART1_ENABLE							USART1 -> CR1 |= (1u << 13)		//UE = 1

/****************TIM1 (PWM) MACROS ****************/

/*Constants macros*/
#define ARR_799									799u
#define ARR_19999								19999u
#define F_CLK									16000000u						//Hz
#define PSC_0									0u
#define PSC_15									15u

/*CR1: Counter enable*/
#define COUNTER_ENABLE							(1 << 0)						//CEN = 1

/*Output compare register*/
#define OC1PE_ENABLE							(1u << 3)						//CCMR1, Output Compare 1 preload enable, bit 3
#define OC2PE_ENABLE							(1u << 11)						//CCMR1, Output Compare 2 preload enable, bit 11
#define OC3PE_ENABLE            			    (1u << 3)        			    //CCMR2, Output Compare 3 preload enable, bit 3
#define OC1M_PWM_MODE_1							(6u << 4)						//CCMR1, Output Compare 1 mode: PWM mode 1 (0b110), bit-> 6:4
#define OC2M_PWM_MODE_1							(6u << 12)						//CCMR1, Output Compare 2 mode: PWM mode 1 (0b110), bit-> 14:12
#define OC3M_PWM_MODE_1           			    (6u << 4)						//CCMR2, Output Compare 3 mode: PWM mode 1 (0b110), bit-> 6:4
#define CC1S_CLEAR								~(3u << 0)						//CCMR1, CC1 channel is configured as output (0b00), bit-> 1:0
#define CC2S_CLEAR								~(3u << 8)						//CCMR1, CC2 channel is configured as output (0b00), bit-> 9:8
#define CC3S_CLEAR               			    ~(3u << 0)         			    //CCMR2, CC3 channel is configured as output (0b00), bit-> 1:0
/*CCER - Capture/compare enable register*/
#define CC1E_OUTPUT_ENABLE					   	(1 << 0)						//channel 1 signal is output, bit 0
#define CC2E_OUTPUT_ENABLE						(1 << 4)						//channel 2 signal is output, bit 4
#define CC3E_OUTPUT_ENABLE						(1 << 8)						//channel 3 signal is output, bit 8

#define CC1P_ACTIVE_HIGH						~(1 << 1)						//capture/compare 1 output polarity active high, bit 1
#define CC2P_ACTIVE_HIGH						~(1 << 5)						//capture/compare 2 output polarity active high, bit 5
#define CC3P_ACTIVE_HIGH						~(1 << 9)						//capture/compare 3 output polarity active high, bit 9

/*BDTR*/
#define MAIN_OUTPUT_ENABLE						(1u << 15)						//MOE = 1 (Main Output Enable)
/*EGR*/
#define EGR_UPDATE_GANERATION_ENABLE			(1u << 0)						//Re-initialize the counter and generates an update of the registers.

/****************Interrupt MACROS ****************/

/*Usart1 interrupt register flags*/
#define FRAMING_ERROR							(1u << 1)						//check FE 1
#define NOISE_DETECTED							(1u << 2)						//check NF 1
#define OVERRUN_ERROR							(1u << 3)						//check ORE 1
#define RXNE_READY								(1u << 5)						//RXNE: Read data register not empty, 1: Received data is ready to be read.
#define RXNE_INTERRUPT_ENABLE					(1u << 5)						//usart interrupt enable

/****************OTHER MACROS ****************/
#define FEEDBACK_MIN 480
#define FEEDBACK_MAX 991
#define	FEEDBACK_MID ((FEEDBACK_MAX + FEEDBACK_MIN)/2)
#define TRANSMITTER_MID 512
#define FEEDBACK_OFFSET 5

#endif /* SETUP_H_ */
