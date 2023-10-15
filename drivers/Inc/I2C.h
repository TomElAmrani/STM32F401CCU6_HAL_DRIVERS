/*
 * I2C.h
 *
 *  Created on: Oct 13, 2023
 *      Author: elamrani
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f401ccu6.h"

//enums to define types of configuration of the I2C
typedef enum __attribute__((__packed__)) {
	ADDR_7_BITS,
	ADDR_10_BITS
} Addr_Mode;

typedef enum __attribute__((__packed__)) {
	STANDARD, 	// 100Kb
	FAST		// 400Kb
} Speed_Mode;

typedef enum __attribute__((__packed__)) {
	ENABLED_ACK,
	DISABLED_ACK
} Byte_Ack;

typedef enum __attribute__((__packed__)) {
	ENABLED_STR,
	DISABLED_STR
} SCL_Stretching;





// This structure defines the different parameters that configures I2C interface
typedef struct{
	Addr_Mode 		ADDR_MODE;
	Speed_Mode 		SPEED_MODE;
	Byte_Ack 		BYTE_ACK;
	SCL_Stretching  SCL_STR;
	uint16_t address;
} I2C_Param_t;


//initiate the I2C interface communication
void I2C_init(I2C_RegType * I2C, I2C_Param_t * Config);


// Send data in master mode
void I2C_SendDATA_MasterMode(I2C_RegType * I2C, Addr_Mode ADDR_MODE , uint16_t address,uint8_t * data, uint32_t max_len);

// Receive Data in master mode
void I2C_ReceiveDATA_MasterMode(I2C_RegType * I2C, Addr_Mode ADDR_MODE , uint16_t address,uint8_t * data, uint32_t max_len);

// Send data in slave mode
void I2C_SendDATA_SlaveMode(I2C_RegType * I2C, uint8_t * data, uint32_t len);

// Receive data in slave mode
void I2C_ReceiveDATA_SlaveMode(I2C_RegType * I2C, uint8_t * data, uint32_t max_len);



#endif /* I2C_H_ */
