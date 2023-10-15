/*
 * GPIO.h
 *
 *  Created on: Aug 12, 2023
 *      Author: elamrani
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f401ccu6.h"

/********************************	MACROS	****************************************************/
//Macro for enabling or disabling the clock for a GPIO port
#define GPIO_ChangeClk(port, enable) \
	do {\
		if(enable){\
			RCC->AHB1ENR |= (1 << port);\
		}else{\
			RCC->AHB1ENR &= ~(1 << port);\
		}\
	}while(0)
/******************************************************************************************/


/************************************ Enums for various pin configurations ****************************/
typedef enum{
	INPUT=0,
	OUTPUT,
	AF,
	ANALOG,
	IRQ_FE,
	IRQ_RE,
	IRQ_FE_AND_RE
} PIN_MODE;

typedef enum{
	PushPull=0,
	OpenDrain
} OTYPE;

typedef enum{
	LowSpeed=0,
	MediumSpeed,
	HighSpeed,
	VHighSpeed
} OSPEED;

typedef enum{
	NO=0,
	PU,
	PD
} PUPD;

typedef enum {
	AF0 = 0,
	AF1,
	AF2 ,
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
} AF_MODE;

typedef enum{
	GPIOA=0,
	GPIOB,
	GPIOC,
	GPIOD,
	GPIOE,
	GPIOH=7
} GPIOx;
/***************************************************************************************************/




/******************* structures that handles the configuration of registers for gpio********************/
// Configuration structure for a GPIO pin
typedef struct{
	PIN_MODE pin_mode;
	OTYPE otype;
	OSPEED ospeed;
	PUPD pupd;
	AF_MODE AF;
} GPIO_PINConfig;

// Handle structure for a GPIO pin
typedef struct {
	GPIOx port;
	GPIO_PIN PIN;
	GPIO_PINConfig Config;
} GPIO_PINHandle;
/******************************************************************************************************/



/******************************* Function prototypes***************************************/
// Function prototypes for GPIO operations
void GPIO_Init(GPIO_PINHandle *pinConfig);
void GPIO_WritePin(GPIO_RegType* port, GPIO_PIN pin, uint8_t value);
uint8_t GPIO_ReadPin(GPIO_RegType* port, GPIO_PIN pin);
void GPIO_TogglePin(GPIO_RegType* port, GPIO_PIN pin);
void GPIO_SetPin(GPIO_RegType *port,GPIO_PIN pin );
void GPIO_ResetPin(GPIO_RegType *port,GPIO_PIN pin );


// Functions for handling GPIO interrupts
void GPIO_IRQEnable(uint8_t IRQ_Number, uint8_t Enable);
void GPIO_IRQPriority(uint8_t IRQ_Number, uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t pin_number);
/******************************************************************************************/


#endif /* GPIO_H_ */
