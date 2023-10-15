/*
 * SPI.h
 *
 *  Created on: Sep 6, 2023
 *      Author: elamrani
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f401ccu6.h"

/*
 * Macros for Cummunication Modes
 */
#define SPI_FULL_DUPLEX 			0
#define SPI_HALF_DUPLEX 			1
#define SPI_SIMPLEX_RECEIVE_ONLY 	2
/*
 * Macros for choice of slave or master
 */
#define SPI_MASTER 	1
#define SPI_SLAVE	0
/*
 * Macros for SPI SSM : software slave select management
 */
#define SPI_SSM_ENA		1
#define SPI_SSM_DIS		0

/*
 * global variables
 */
extern volatile uint16_t DATA_RECEIVED_WITH_IT;
extern volatile uint16_t DATA_TO_TRANSMIT_IT;
extern volatile uint8_t DATA_RECEIVEED_FLAG_IT;



/*
 * Type for Data Frame Format
 */
typedef enum {
	_8_bits=0,
	_16_bits
} SPI_DFF_t;

/*
 * Type For baud rate prescaler
 */
typedef enum {
	DIVISION_2=0,
	DIVISION_4,
	DIVISION_8,
	DIVISION_16,
	DIVISION_32,
	DIVISION_64,
	DIVISION_128,
	DIVISION_256
} SPI_Prescaler_BD_t;




/*
 * SPI struct
 */
typedef struct {
	uint8_t MODE;
	SPI_Prescaler_BD_t BaudRate;
	SPI_DFF_t DFF; 					//data frame format
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t LSBFIRST;
	uint8_t Master;
	uint8_t SSM; 					//software slave select management
} SPI_Param_t;

/*
 * Struct for interrupts to activate
 */

typedef struct{//0 : disbaled and 1 : enabled --> these bits are in SPI_CR2
	uint8_t SPI_TXEIE; // Tx is empty
	uint8_t SPI_RXNEIE; // Rx is empty
	uint8_t SPI_ERRIE; // Errors : Overrun , Mode Fault or CRC Error
} SPI_Interrupts_t;


//functions

// to transform the SPI pointer to its number {1,2,3,4}
uint8_t SPI_Pointer_to_number(SPI_RegType * SPI);

// for full-duplex communication using SPI
void SPI_INIT(SPI_Param_t * Config, SPI_RegType * SPI);

// to enable or disable SPI
void SPI_ENABLE(SPI_RegType * SPI, uint8_t enable);

//send data
void SPI_SendDATA(SPI_RegType * SPI, SPI_Param_t * Config , uint8_t * data, uint32_t len);
//receive data
uint8_t * SPI_ReceiveDATA(SPI_RegType * SPI, SPI_Param_t * Config , uint32_t len);


//helper functions to transmit and receive data
uint8_t SPI_Tx_IS_BUSY(SPI_RegType * SPI);
uint8_t SPI_Rx_IS_READY(SPI_RegType * SPI);

//function for send and receive data using interrupt
void SPI_ReceiveDATAWithInterrupt(SPI_RegType * SPI);

void SPI_SendDATAWithInterrupt(SPI_RegType * SPI, SPI_Param_t * Config );

// Function to enable interrupts in MCU
void SPI_Enable_Interrupts (SPI_RegType *SPI,SPI_Interrupts_t *Config_IT);



// Functions for handling GPIO interrupts
void SPI_IRQEnable(uint8_t IRQ_Number, uint8_t Enable);
void SPI_IRQPriority(uint8_t IRQ_Number, uint8_t IRQ_Priority);
void SPI_IRQHandling(SPI_RegType * SPI, SPI_Param_t * Config);





#endif /* SPI_H_ */
