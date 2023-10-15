/*
 * UART.c
 *
 *  Created on: Oct 13, 2023
 *      Author: elamrani
 */
#include "UART.h"

//init the UART
void init(USART_Handle_t *uart_handle){
	//enable the clock
	USART_CLK_ENABLE(uart_handle->USART);
	//reset the registers
	uart_handle->USART->CR1 = 0;
	uart_handle->USART->CR2 = 0;
	uart_handle->USART->CR3 = 0;
	//Enable the USART by writing the UE bit in USART_CR1 register to 1.
	uart_handle->USART->CR1 |= ((uart_handle->Config.word_length)<<USART_CR1_UE);
	//Program the M bit in USART_CR1 to define the word length.
	uart_handle->USART->CR1 |= ((uart_handle->Config.word_length)<<USART_CR1_M);
	//Program the number of stop bits in USART_CR2.
	uart_handle->USART->CR2 |= ((uart_handle->Config.stop_bits)<<USART_CR2_STOP);
	//Select the desired baud rate using the baud rate register USART_BRR
	uart_handle->USART->BRR = decimalToHex16Bit(uart_handle->Config.baudrate);
	// Oversampling mode by 8 or 16
	uart_handle->USART->CR1 |= ((uart_handle->Config.oversampling_mode)<<USART_CR1_OVER8);
	//set the parity
	uart_handle->USART->CR1 |= ((uart_handle->Config.parity)<<USART_CR1_PCE);
}


uint32_t USART_ReceiveData(USART_Handle_t *uart_handle, uint8_t *data, uint32_t max_len){
	uint32_t len=0;

	//Set the RE bit USART_CR1. This enables the receiver which begins searching for a start bit
	uart_handle->USART->CR1 |= (1<<USART_CR1_RE);

	while(max_len){
		//wait until the data is received
		while(!(uart_handle->USART->SR & 1<<USART_SR_RXNE));

		//check if there are  error flags
		//ORE: Overrun error
		//NF: Noise detected flag
		//FE: Framing error
		//PE: Parity error
		//IDLE: IDLE line detected
		if( uart_handle->USART->SR & (1<<USART_SR_ORE | 1<<USART_SR_NF | 1<<USART_SR_FE | 1<<USART_SR_PE | 1<<USART_SR_IDLE))
			return len;


		if(uart_handle->Config.parity == ENABLE_PARITY){ *data=(uart_handle->USART->DR)>>1;}
		else{ *data=uart_handle->USART->DR;}
		data++;
		max_len--;
		len++;
	}
	return len;
}

void USART_SendData(USART_Handle_t *uart_handle,uint8_t *data, uint32_t len){
	//Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	uart_handle->USART->CR1 |= (1<<USART_CR1_TE);
	//wait the buffer to be empty
	while(uart_handle->USART->SR & (1<<USART_SR_TXE));
	//send the first byte
	uart_handle->USART->DR=*data;
	len--;

	while(len){
		//wait until the data is transmitted
		while(!(uart_handle->USART->SR & 1<<USART_SR_TC));
		//send the data
		uart_handle->USART->DR=*data;

		data++;
		len--;
	}

}


// Function to convert decimal to 16-bit hexadecimal
uint16_t decimalToHex16Bit(float decimalValue) {
    // Extracting the integer part (mantissa) and fractional part
	uint32_t mantissa = (uint32_t)decimalValue;
    float fraction = decimalValue - mantissa;

    // Converting the 12-bit mantissa to hexadecimal
    mantissa &= 0xFFF; // To ensure only the lower 12 bits are considered

    // Converting the fractional part to a 4-bit hexadecimal representation
    uint32_t fractionHex = (uint32_t)(fraction * 16);

    // Combining the mantissa and fractional part
    uint16_t result = (uint16_t)((mantissa << 4) | fractionHex);

    return result;
}
