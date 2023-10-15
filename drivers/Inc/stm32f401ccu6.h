/*
 * stm32f401ccu6.h
 *
 *  Created on: Aug 11, 2023
 *      Author: elamrani
 */

#ifndef DRIVERS_INC_STM32F401CCU6_H_
#define DRIVERS_INC_STM32F401CCU6_H_

#include <stdint.h>

/****************** Macros for base addresses of interior MCU peripherals ***************************/
// Bus base adresses
#define AHB1_BA				0x50000000UL
#define AHB2_BA				0x40020000UL
#define APB1_BA				0x40007400UL
#define APB2_BA				0x40000000UL

// base adresses
#define EXTI_BA 			((EXTI_RegType *)0x40013C00UL)
#define SYSCFG_BA			((SYSCFG_RegType *)0x40013800UL)
#define SRAM1_BA			0x20000000UL
#define FLASH_BA			0x08000000UL

// GPIOs bus adresses
#define GPIOA_BA			((GPIO_RegType*)0x40020000UL)
#define GPIOB_BA			((GPIO_RegType *)0x40020400UL)
#define GPIOC_BA			((GPIO_RegType*)0x40020800UL)
#define GPIOD_BA			((GPIO_RegType*)0x40020C00UL)
#define GPIOE_BA			((GPIO_RegType*)0x40021000UL)
#define GPIOH_BA			((GPIO_RegType*)0x40021C00UL)

//define SPI base adresses
#define SPI1				((SPI_RegType*)0x40013000) //on APB2
#define SPI4				((SPI_RegType*)0x40013400) //on APB2
#define SPI3_I2S3			((SPI_RegType*)0x40003C00) //on APB1
#define SPI2_I2S2			((SPI_RegType*)0x40003800) //on APB1

//define I2C base addresses
#define I2C1				((I2C_RegType*)0x40005400) 	// on APB1
#define I2C2           		((I2C_RegType*)0x40005800)	// on APB1
#define I2C3 				((I2C_RegType*)0x40005C00)	// on APB1

//define the USART base addresses
#define USART1				((USART_RegType*)0x40011000) //on APB2
#define USART2				((USART_RegType*)0x40004400) //on APB1
#define USART6				((USART_RegType*)0x40011400) //on APB2

#define RCC 				((RCC_RegType *)0x40023800)
/****************************************************************************************************/





/***************** Macros for base addresses of interior ARM processor peripherals *******************/


//define arm processor NVIC peripheral base adresses
#define NVIC_ISER_BA 			((uint32_t*)0xE000E100UL)
#define NVIC_ICER_BA 			((uint32_t*)0XE000E180UL)
#define NVIC_IPR_BA				((uint32_t*)0xE000E400UL)
#define NVIC_ICPR_BA			((uint32_t*)0xE000E280UL)

/******************************************************************************************************/





/*********************macros for clock enable functions*************************************/
#define ENABLE_SYSCFG_CLK() 		(RCC->APB2ENR |= (1 << 14))

#define SPI_CLK_ENABLE(SPI_NUMBER)    do{\
											if(SPI_NUMBER==2)	RCC->APB1ENR |= (1<<14);\
											else if(SPI_NUMBER==3)	RCC->APB1ENR |= (1<<15);\
											else if(SPI_NUMBER==1)	RCC->APB2ENR |= (1<<12);\
											else if(SPI_NUMBER==4)	RCC->APB2ENR |= (1<<13);\
										}while(0)


#define SPI_CLK_DISABLE(SPI_NUMBER)    do{\
											if(SPI_NUMBER==2)		RCC->APB1ENR &= ~(1<<14);\
											else if(SPI_NUMBER==3)	RCC->APB1ENR &= ~(1<<15);\
											else if(SPI_NUMBER==1)	RCC->APB2ENR &= ~(1<<12);\
											else if(SPI_NUMBER==4)	RCC->APB2ENR &= ~(1<<13);\
										}while(0)


#define I2C_CLK_ENABLE(I2C_BaseAddr)	do{\
											if(I2C_BaseAddr==I2C1) 		RCC->APB1ENR |= (1<<21);\
											else if(I2C_BaseAddr==I2C2) RCC->APB1ENR |= (1<<22);\
											else if(I2C_BaseAddr==I2C3)	RCC->APB2ENR |= (1<<23);\
										}while(0)


#define I2C_CLK_DISABLE(I2C_BaseAddr)	do{\
											if(I2C_BaseAddr==I2C1) 		RCC->APB1ENR &= ~(1<<21);\
											else if(I2C_BaseAddr==I2C2) RCC->APB1ENR &= ~(1<<22);\
											else if(I2C_BaseAddr==I2C3)	RCC->APB2ENR &= ~(1<<23);\
										}while(0)

#define USART_CLK_ENABLE(USART_BaseAddr)	do{\
												if(USART_BaseAddr==USART1) 			RCC->APB2ENR |= (1<<4);\
												else if(USART_BaseAddr==USART2) 	RCC->APB1ENR |= (1<<17);\
												else if(USART_BaseAddr==USART6)		RCC->APB2ENR |= (1<<5);\
											}while(0)

#define USART_CLK_DISABLE(USART_BaseAddr)	do{\
												if(USART_BaseAddr==USART1) 			RCC->APB2ENR &= ~(1<<4);\
												else if(USART_BaseAddr==USART2) 	RCC->APB1ENR &= ~(1<<17);\
												else if(USART_BaseAddr==USART6)		RCC->APB2ENR &= ~(1<<5);\
											}while(0)





/*********************************************************************************************/




/***************** macros for constatnts in order to give more readibility to the code****************/
//define states
#define HIGH 				((uint8_t)1)
#define LOW  				((uint8_t)0)
#define SET  				HIGH
#define RESET 				LOW
#define ENABLE  			HIGH
#define DISABLE 			LOW


//IRQ numbers for EXTI
#define IRQ_N_EXTI0			6
#define IRQ_N_EXTI1			7
#define IRQ_N_EXTI2			8
#define IRQ_N_EXTI3			9
#define IRQ_N_EXTI4			10
#define IRQ_N_EXTI9_5		23
#define IRQ_N_EXTI15_10		40
//IRQ numbers for SPI
#define IRQ_N_SPI1			35
#define IRQ_N_SPI2			36
#define IRQ_N_SPI3			51
#define IRQ_N_SPI4			84

//Positions of configuration  bits for register CR1 for SPI
#define SPI_CR1_BIDIMODE 	15
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_CRCEN		13
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_DFF			11
#define SPI_CR1_RXONLY		10
#define SPI_CR1_SSM			9
#define SPI_CR1_SSI			8
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SPE			6
#define SPI_CR1_BR			3
#define SPI_CR1_MSTR		2
#define SPI_CR1_CPOL		1
#define SPI_CR1_CPHA		0

//Positions of configuration  bits for register CR2 for SPI
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_RFR				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7



//Positions of configuration  bits for register SR for SPI
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

//Positions of configuration  bits for register CR1 for I2C
#define I2C_CR1_PE 				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15


//Positions of configuration  bits for register CR2 for I2C
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12


//Positions of configuration  bits for register OAR1 for I2C
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD_1_7		1
#define I2C_OAR1_ADD_8_9		8
#define I2C_OAR1_ADDMODE		15



//Positions of configuration  bits for register OAR2 for I2C
#define I2C_OAR2_ENDUAL			0
#define I2C_OAR2_ADD2			1


//Positions of configuration  bits for register SR1 for I2C
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

//Positions of configuration  bits for register SR2 for I2C
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

//Positions of configuration  bits for register CCR for I2C
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_F_S				15


//Positions of configuration  bits for register SR for USART
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

//Positions of configuration  bits for register CR1 for USART
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15



//Positions of configuration  bits for register CR2 for USART
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14



//Positions of configuration  bits for register CR3 for USART
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

/*****************************************************************************************************/



/********************************* enums for new data types *******************************/
typedef enum {
	GPIO_PIN_0 = 0,
	GPIO_PIN_1,
	GPIO_PIN_2 ,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15
} GPIO_PIN;
/******************************************************************************************/





/***************************** Structures for Registers of the MCU ******************************/

// Strcuture for RCC registers
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	uint32_t Reserved1[2];
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t Reserved2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	uint32_t Reserved3[2];
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t Reserved4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	uint32_t Reserved5[2];
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t Reserved6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t Reserved7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	uint32_t Reserved8;
	volatile uint32_t DCKCFGR;
} RCC_RegType;


//Structure for GPIOx registers
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
} GPIO_RegType;


//Structure for EXTI registers
typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RegType;


//structure for SYSCFG registers
typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;
} SYSCFG_RegType;

//Structure for SPIx registers
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SR;
} SPI_RegType;


// This structure defines the registers of an I2C interface
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
} I2C_RegType;

// This structure defines the registers of an USART interface
typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
} USART_RegType;



/***************************************************************************************************/




#endif /* DRIVERS_INC_STM32F401CCU6_H_ */
