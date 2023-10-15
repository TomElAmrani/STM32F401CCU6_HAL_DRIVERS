/*
 * I2C.c
 *
 *  Created on: Oct 13, 2023
 *      Author: elamrani
 */
#include "I2C.h"

//initiate the I2C interface communication
void I2C_init(I2C_RegType * I2C, I2C_Param_t * Config){
	//enable clock for the interface
	I2C_CLK_ENABLE(I2C);

	//disable the peripheral
	I2C->CR1 |= (1<<I2C_CR1_PE);

	//set the address mode
	I2C->OAR1 = 0; // clear the register
	if (Config->ADDR_MODE == ADDR_10_BITS) I2C->OAR1 |= 1<<I2C_OAR1_ADDMODE;

	//set speed mode
	I2C->CR2 &=~(0b111111);
	if (Config->SPEED_MODE == STANDARD ) I2C->CR2 |= (0b10);  	//2MH
	else if(Config->SPEED_MODE == STANDARD)	I2C->CR2 |= (0b100);//4MH

	//reset the whole register
	I2C->CR1 = 0;
	// clock stretching enabled
	if(Config->SCL_STR == DISABLED_STR) I2C->CR1 |= (1<< I2C_CR1_NOSTRETCH);
	// set Ack bit
	if(Config->BYTE_ACK == ENABLED_ACK) I2C->CR1 |= (1<< I2C_CR1_ACK);

	//reset the address
	I2C->OAR1 &= ~(0x3F);

	//set the address
	if(Config->ADDR_MODE == ADDR_7_BITS){
		I2C->OAR1 |= ((uint8_t)Config->address<<1);
	}else if(Config->ADDR_MODE == ADDR_10_BITS){
		I2C->OAR1 |= Config->address;
	}

	//set the RCC value to the minimum 0x04
	I2C->CCR =0x0;
	I2C->CCR |= 0x04;
	if(Config->SPEED_MODE==FAST)	I2C->CCR |= (1<<I2C_CCR_F_S);

	//set the rise time
	//in fast mode if pclk=8MHZ and because the max rise time is 300ns , the value to set is 3
	//while in standard mode , the max rise time is 1000ns , then the value to set is 9
	I2C->TRISE = 0x0;
	if(Config->SPEED_MODE == FAST){
		I2C->TRISE |= 0x03;
	}else{
		I2C->TRISE |= 0x09;
	}


	//Enable the peripheral
	I2C->CR1 |= (1<<I2C_CR1_PE);

}


// Send data in master mode
void I2C_SendDATA_MasterMode(I2C_RegType * I2C, Addr_Mode  ADDR_MODE , uint16_t address,uint8_t * data, uint32_t len){

	address= address & (0x3F);

	while(I2C->SR2 & I2C_SR2_BUSY);
	//start the communication
	I2C->CR1 |= 1<<I2C_CR1_START;
	//wait until the generation of the start bit
	while(!(I2C->SR1 & I2C_SR1_SB));
	//send the address and bit(R/W)=0 to read data
	if(ADDR_MODE == ADDR_7_BITS){
		I2C->DR = address<<1;
	}else if(ADDR_MODE == ADDR_10_BITS){
		I2C->DR= (0xF0)|((address>>8)<<1);
		//check the ack
		if(I2C->SR1 & I2C_SR1_AF) return;
		//wiat until the DR  is empty
		while(!(I2C->SR1 & I2C_SR1_TxE));
		//write the emaining of the address
		I2C->DR = (0xF) & address;
		//check the ack
		if(I2C->SR1 & I2C_SR1_AF) return;
	}


	//start sending data
	if(I2C->SR1 & 1<<I2C_SR2_TRA){
		//start sending data
		while(len){
			while(!(I2C->SR1 & 1<<I2C_SR1_TxE));
			//send data
			I2C->DR = *data;
			//check the ack
			if(I2C->SR1 & (1<<I2C_SR1_AF)) return;
			len--;
			data++;
		}
	}

}

// Receive Data in master mode
void I2C_ReceiveDATA_MasterMode(I2C_RegType * I2C, Addr_Mode ADDR_MODE , uint16_t address,uint8_t * data, uint32_t max_len){
	address= address & (0x3F);

	while(I2C->SR2 & I2C_SR2_BUSY);
	//start the communication
	I2C->CR1 |= 1<<I2C_CR1_START;
	//wait until the generation of the start bit
	while(!(I2C->SR1 & I2C_SR1_SB));
	//send the address and bit(R/W)=0 to read data
	if(ADDR_MODE == ADDR_7_BITS){
		I2C->DR = address<<1;
	}else if(ADDR_MODE == ADDR_10_BITS){
		I2C->DR= (0xF0)|((address>>8)<<1);
		//check the ack
		if(I2C->SR1 & I2C_SR1_AF) return;
		//wiat until the DR  is empty
		while(!(I2C->SR1 & I2C_SR1_TxE));
		//write the emaining of the address
		I2C->DR = (0xF) & address;
		//check the ack
		if(I2C->SR1 & I2C_SR1_AF) return;
	}

	//start receiving data
	if(!(I2C->SR1 & I2C_SR2_TRA)){
	while(max_len && !(I2C->SR1 & 1<<I2C_SR1_STOPF)){
		while(!(I2C->SR1 & I2C_SR1_RxNE));
		*data=I2C->DR;
		//set the ACK when receiving data
		I2C->CR1 |= (1<<I2C_CR1_ACK);
		data++;
		max_len--;
	}
	}


}


// Send data in slave mode
void I2C_SendDATA_SlaveMode(I2C_RegType * I2C, uint8_t * data, uint32_t len){
	//enable the peripheral
	I2C->CR1 |= 1<<I2C_CR1_PE;

	//Wait to receive the first bit
	while(!(I2C->SR1 & 1<<I2C_SR1_RxNE));
	I2C->DR;
	//set the ACK when receiving data
	I2C->CR1 |= (1<<I2C_CR1_ACK);
	/*
	* Here we have two possibilities:
	* we re gonna receive the match for address if it is of 7 bits
	* or we need to receive the next bit
	*/
	if(!(I2C->SR1 & 1<<I2C_SR1_ADDR)){
		while(!(I2C->SR1 & 1<<I2C_SR1_RxNE));
		I2C->DR;
		//set the ACK when receiving data
		I2C->CR1 |= (1<<I2C_CR1_ACK);

		if(!(I2C->SR1 & I2C_SR1_ADDR)) return;
	}

	if(I2C->SR1 & 1<<I2C_SR2_TRA){
		//start sending data
		while(len){
			while(!(I2C->SR1 & 1<<I2C_SR1_TxE));
			//send data
			I2C->DR = *data;
			//check the ack
			if(I2C->SR1 & (1<<I2C_SR1_AF)) return;
			len--;
			data++;
		}
	}

}

// Receive data in slave mode
void I2C_ReceiveDATA_SlaveMode(I2C_RegType * I2C, uint8_t * data, uint32_t max_len){
	//enable the peripheral
	I2C->CR1 |= 1<<I2C_CR1_PE;

	//Wait to receive the first bit
	while(!(I2C->SR1 & I2C_SR1_RxNE));
	I2C->DR;
	//set the ACK when receiving data
	I2C->CR1 |= (1<<I2C_CR1_ACK);
	/*
	* Here we have two possibilities:
	* we re gonna receive the match for address if it is of 7 bits
	* or we need to receive the next bit
	*/
	if(!(I2C->SR1 & I2C_SR1_ADDR)){
		while(!(I2C->SR1 & I2C_SR1_RxNE));
		I2C->DR;
		//set the ACK when receiving data
		I2C->CR1 |= (1<<I2C_CR1_ACK);

		if(!(I2C->SR1 & I2C_SR1_ADDR)) return;
	}

	if(!(I2C->SR1 & I2C_SR2_TRA)){
	while(max_len && !(I2C->SR1 & I2C_SR1_STOPF)){
		while(!(I2C->SR1 & 1<<I2C_SR1_RxNE));
		*data=I2C->DR;
		//set the ACK when receiving data
		I2C->CR1 |= (1<<I2C_CR1_ACK);
		data++;
		max_len--;
	}
	}

}
