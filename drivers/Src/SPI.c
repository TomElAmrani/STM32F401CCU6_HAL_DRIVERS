/*
 * SPI.c
 *
 *  Created on: Sep 6, 2023
 *      Author: elamrani
 */
#include "SPI.h"
#include <stdlib.h>

//extern variables
volatile uint16_t DATA_RECEIVED_WITH_IT;
volatile uint16_t DATA_TO_TRANSMIT_IT = 0x0;
volatile uint8_t DATA_RECEIVEED_FLAG_IT = 0;


void SPI_INIT(SPI_Param_t * Config, SPI_RegType * SPI){
	uint8_t SPI_N=SPI_Pointer_to_number(SPI);
	SPI_CLK_ENABLE(SPI_N);

	//reset the register
	SPI->CR1 &= 0;


	if (Config->MODE == SPI_FULL_DUPLEX){
		//BIDI MODE is cleared
	}else if(Config->MODE == SPI_HALF_DUPLEX){
		//BIDI MODE IS SET
		SPI->CR1 |= 1<<SPI_CR1_BIDIMODE;
		//BIDIOE should be set to transmit data
	}else if(Config->MODE == SPI_SIMPLEX_RECEIVE_ONLY ){
		//BIDI MODE is cleared
		//RXONLY should be set
		SPI->CR1 |= 1<<SPI_CR1_RXONLY;
	}


	//Software Slave select management
	SPI->CR1 |= Config->SSM << SPI_CR1_SSM ;

	//for master configuration : Select the BR[2:0] bits to define the serial clock baud rate
	//APB2 -> PCLK_max=84MH
	//APB1 -> PCLK_max=42MH
	//BR=0 -> PCLK/2
	//BR=1 -> PCLK/4
	//BR=2 -> PCLK/8 	...
	//BR=7 -> PCLK/256
	if(Config->Master){
		//baud rate prescaler
		SPI->CR1 |= (((uint32_t)Config->BaudRate)&(0b111))<<SPI_CR1_BR;

		if(Config->SSM) SPI->CR1 |= 1 << SPI_CR1_SSI;
	}

	//DFF: Data Frame Format (8 bits or 16 bits)
	SPI->CR1 |= (Config->DFF & 1)<<SPI_CR1_DFF;

	//set the mode : CPOL and CPHA
	SPI->CR1 |= (Config->CPOL & 1)<<SPI_CR1_CPOL;
	SPI->CR1 |= (Config->CPHA & 1)<<SPI_CR1_CPHA;

	//set or reset LSBFIRST
	SPI->CR1 |= (Config->LSBFIRST & 1)<<SPI_CR1_LSBFIRST;

	//for master : the SSOE should be set to make the NSS pin in output mode (one master)
	if(Config->Master){
		SPI->CR2 |= 1<<SPI_CR2_SSOE;
	}

	//MSTR:0 -> Slave
	//MSTR:1 -> Master
	SPI->CR1 |= (Config->Master << SPI_CR1_MSTR);


}

void SPI_ENABLE(SPI_RegType * SPI, uint8_t enable){
	if(enable)
		SPI->CR1 |= 1 << SPI_CR1_SPE;
	else
		SPI->CR1 &= ~(1 << SPI_CR1_SPE);
}

uint8_t SPI_Pointer_to_number(SPI_RegType * SPI){
	uint8_t num;
	if(SPI == SPI1)
		num = 1;
	else if(SPI== SPI2_I2S2)
		num=2;
	else if(SPI== SPI3_I2S3)
		num=3;
	else if(SPI==SPI4)
		num=4;

	return num;
}


void SPI_SendDATA(SPI_RegType * SPI, SPI_Param_t * Config , uint8_t * data, uint32_t len){
	while(len>0){
		while( !(SPI->SR & 1<<SPI_SR_TXE) );

		if( !(Config->DFF) ){
			SPI->DR = *((uint8_t*)data);
			data++;
			len--;
		}else{
			SPI->DR = *((uint16_t*)data);
			(uint16_t*)data++;
			len-=2;
		}

	}
}

uint8_t * SPI_ReceiveDATA(SPI_RegType * SPI, SPI_Param_t * Config , uint32_t len){
	uint8_t * DataReceived = malloc(len);

	if( !(Config->DFF) ){
		for(uint8_t i=0;i<len;i++){
			while(!SPI_Rx_IS_READY(SPI1));
			DataReceived[i]=SPI->DR;
		}
	}else{
		for(uint8_t i=0;i<(len/2+len%2) ;i++){
			while(!SPI_Rx_IS_READY(SPI1));
			uint16_t temp= SPI->DR;
			DataReceived[i*2]= (uint8_t)(temp & (0xFF));
			DataReceived[i*2+1]=  (uint8_t)(temp>>8);
		}
	}
	return DataReceived;
}

uint8_t SPI_Tx_IS_BUSY(SPI_RegType * SPI){
	return (uint8_t)(SPI->SR & 1<<SPI_SR_BSY) ;
}

uint8_t SPI_Rx_IS_READY(SPI_RegType * SPI){
	return  (uint8_t)(SPI->SR & 1<<SPI_SR_RXNE) ;
}

//function for send and receive data using interrupt
void SPI_ReceiveDATAWithInterrupt(SPI_RegType * SPI){
	DATA_RECEIVED_WITH_IT = SPI->DR;
	DATA_RECEIVEED_FLAG_IT=1;
}

void SPI_SendDATAWithInterrupt(SPI_RegType * SPI, SPI_Param_t * Config ){
	if( !(Config->DFF) ){
		SPI->DR = (uint8_t)DATA_TO_TRANSMIT_IT;
	}else{
		SPI->DR = (uint16_t)DATA_TO_TRANSMIT_IT;
	}
}


// Function to enable or disable a SPI interrupt
void SPI_IRQEnable(uint8_t IRQ_Number, uint8_t Enable){
	uint8_t temp_1= IRQ_Number / 32;
	uint8_t temp_2= IRQ_Number % 32;
	if(Enable){
		*(NVIC_ISER_BA + temp_1) |= (1 << temp_2);
	}else{
		*(NVIC_ICER_BA + temp_1) |= (1 << temp_2);
	}
}

// Function to set the priority of a SPI interrupt
void SPI_IRQPriority(uint8_t IRQ_Number, uint8_t IRQ_Priority){
	uint8_t temp_1= IRQ_Number / 4;
	uint8_t temp_2= IRQ_Number % 4;
	*(NVIC_IPR_BA + temp_1) |= ((IRQ_Priority << 4) << (8*temp_2));
}


// Function to handle a SPI interrupt
void SPI_IRQHandling(SPI_RegType * SPI, SPI_Param_t * Config){
	if(SPI->CR2 & (1<<SPI_CR2_TXEIE)){//TX empty has lanched the interrupt
		SPI_ReceiveDATAWithInterrupt(SPI);
	}else if(SPI->CR2 & (1<<SPI_CR2_RXNEIE)){//RX not empty lanched the interrupt
		SPI_SendDATAWithInterrupt(SPI,Config);
	}

}
//enable interrupts function

void SPI_Enable_Interrupts (SPI_RegType *SPI ,SPI_Interrupts_t *Config_IT){
	//reset interrupt bits
	SPI->CR2 &= ~((1<< SPI_CR2_TXEIE)|
					(1<<SPI_CR2_RXNEIE)|
					(1<<SPI_CR2_ERRIE));

	//set interrupt bits
	if(Config_IT->SPI_TXEIE){
		SPI->CR2 |= 1<< SPI_CR2_TXEIE;
	}else if(Config_IT->SPI_RXNEIE){
		SPI->CR2 |= 1 << SPI_CR2_RXNEIE;
	}else if(Config_IT->SPI_ERRIE){
		SPI->CR2 |= SPI_CR2_ERRIE;
	}
}

