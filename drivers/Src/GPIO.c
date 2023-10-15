/*
 * GPIO.c
 *
 *  Created on: Aug 12, 2023
 *      Author: elamrani
 *      This file have the implementation of the functions
 *      of which the prototypes are defined in GPIO.h
 */
#include "GPIO.h"

// Function to initialize a GPIO pin
void GPIO_Init(GPIO_PINHandle *pinConfig){

	GPIO_RegType *port;

	// set port
	if(pinConfig->port == GPIOA)
		port = GPIOA_BA;
	else if(pinConfig->port == GPIOB)
		port = GPIOB_BA;
	else if(pinConfig->port == GPIOC)
		port = GPIOC_BA;
	else if(pinConfig->port == GPIOD)
		port = GPIOD_BA;
	else if(pinConfig->port == GPIOE)
		port = GPIOE_BA;
	else if(pinConfig->port == GPIOH)
		port = GPIOH_BA;


	PIN_MODE pin_mode = pinConfig->Config.pin_mode;

	if( pin_mode == INPUT ){
		port->MODER |= (((uint32_t)pin_mode) << (2 * pinConfig->PIN));
		port->PUPDR |=(((uint32_t)pinConfig->Config.pupd) << (2 * pinConfig->PIN));
	}else if(pin_mode == OUTPUT){
		port->MODER |= (((uint32_t)pin_mode) << (2 * pinConfig->PIN));
		port->OTYPER |= (((uint32_t)pinConfig->Config.otype) << pinConfig->PIN);
		port->PUPDR |= (((uint32_t)pinConfig->Config.pupd) << (2 * pinConfig->PIN));
		port->OSPEEDR |= (((uint32_t)pinConfig->Config.ospeed) << (2 * pinConfig->PIN));
	}else if( pin_mode == AF){
		port->MODER |= (((uint32_t)pin_mode) << (2 * pinConfig->PIN));
		port->OTYPER |= (((uint32_t)pinConfig->Config.otype) << pinConfig->PIN);
		port->PUPDR |= (((uint32_t)pinConfig->Config.pupd) << (2 * pinConfig->PIN));
		if(  pinConfig->PIN > 7){
			port->AFRH |= (((uint32_t)pinConfig->Config.AF) << (4* (pinConfig->PIN - 8)));
		}else {
			port->AFRL |= (((uint32_t)pinConfig->Config.AF) << (4* pinConfig->PIN ));
		}

	}else if(pin_mode ==ANALOG){
		port->MODER |= (((uint32_t)pin_mode) << (2 * pinConfig->PIN));

	}else{
		port->MODER |= (((uint32_t)pin_mode) << (2 * pinConfig->PIN));
		port->PUPDR |=(((uint32_t)pinConfig->Config.pupd) << (2 * pinConfig->PIN));

		// set the interrupt trigger
		if(pin_mode == IRQ_FE){
			EXTI_BA->FTSR |= (1 << pinConfig->PIN);
			EXTI_BA->RTSR &= ~(1 << pinConfig->PIN);
		}else if(pin_mode == IRQ_RE){
			EXTI_BA->FTSR &= ~(1 << pinConfig->PIN);
			EXTI_BA->RTSR |= (1 << pinConfig->PIN);
		}else if(pin_mode == IRQ_FE_AND_RE){
			EXTI_BA->FTSR |= (1 << pinConfig->PIN);
			EXTI_BA->RTSR |= (1 << pinConfig->PIN);
		}

		//set the port on the syscfg registers
		uint8_t temp_1 = pinConfig->PIN / 4;
		uint8_t temp_2 = pinConfig->PIN % 4;
		ENABLE_SYSCFG_CLK();
		uint32_t temp_3=(((uint32_t)pinConfig->port) << (4 * temp_2));
		SYSCFG_BA->EXTICR[temp_1] |= temp_3;

		//set the mask
		EXTI_BA->IMR |= (1 << pinConfig->PIN);

	}
}


// Function to write a value to a GPIO pin
void GPIO_WritePin(GPIO_RegType *port, GPIO_PIN pin, uint8_t value){
	port->ODR |= ((uint32_t)value) << pin;
}


// Function to read the value of a GPIO pin
uint8_t GPIO_ReadPin(GPIO_RegType *port, GPIO_PIN pin){
	return (uint8_t)((port->IDR >> pin)&(0x1));
}


// Function to toggle the value of a GPIO pin
void GPIO_TogglePin(GPIO_RegType *port, GPIO_PIN pin){

	if((port->ODR >> pin)&(0x00000001)){
		port->ODR &= ~((uint32_t)1) << pin;
	}else{
		port->ODR |= ((uint32_t)1) << pin;
	}
}


// Function to set a GPIO pin high
void GPIO_SetPin(GPIO_RegType *port,GPIO_PIN pin ){
	port->BSRR = (((uint32_t)1) << pin);
}


// Function to set a GPIO pin low
void GPIO_ResetPin(GPIO_RegType* port,GPIO_PIN pin ){
	//port->BSRR = (((uint32_t)0) << pin);
	port->BSRR = (((uint32_t)1) << (pin+15));
}


// Function to enable or disable a GPIO interrupt
void GPIO_IRQEnable(uint8_t IRQ_Number, uint8_t Enable){
	uint8_t temp_1= IRQ_Number / 32;
	uint8_t temp_2= IRQ_Number % 32;
	if(Enable){
		*(NVIC_ISER_BA + temp_1) |= (1 << temp_2);
	}else{
		*(NVIC_ICER_BA + temp_1) |= (1 << temp_2);
	}
}

// Function to set the priority of a GPIO interrupt
void GPIO_IRQPriority(uint8_t IRQ_Number, uint8_t IRQ_Priority){
	uint8_t temp_1= IRQ_Number / 4;
	uint8_t temp_2= IRQ_Number % 4;
	*(NVIC_IPR_BA + temp_1) |= ((IRQ_Priority << 4) << (8*temp_2));
}


// Function to handle a GPIO interrupt
void GPIO_IRQHandling(uint8_t pin_number){
	if(EXTI_BA->PR & (1<<pin_number))
		EXTI_BA->PR |= (1 << pin_number);
}


