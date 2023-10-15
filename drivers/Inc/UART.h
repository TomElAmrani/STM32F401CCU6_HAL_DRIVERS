/*
 * UART.h
 *
 *  Created on: Oct 13, 2023
 *      Author: elamrani
 */
#include "stm32f401ccu6.h"

#ifndef UART_H_
#define UART_H_

// we will focus only on the UART , the asynchronous communication

/*
 * if Oversampling by 8 (OVER8=1) and fPCLK = 8 MHz ,
 * the value in the BRR that corresponds to the desired baudrate :
 */
#define BR_CLK8_2_point_4_KBps_SAMP8		416.625
#define BR_CLK8_9_point_6_KBps_SAMP8		104.125
#define BR_CLK8_19_point_2_KBps_SAMP8		52.125
#define BR_CLK8_57_point_6_KBps_SAMP8		17.375
#define BR_CLK8_115_point_2_KBps_SAMP8		8.625
#define BR_CLK8_230_point_4_KBps_SAMP8		4.375
#define BR_CLK8_460_point_8_KBps_SAMP8		2.125
#define BR_CLK8_896_KBps_SAMP8				1.125
/*
 * if Oversampling by 16 (OVER8=0) and fPCLK = 8 MHz ,
 * the value in the BRR that corresponds to the desired baudrate :
 */
#define BR_CLK8_2_point_4_KBps_SAMP16		208.3125
#define BR_CLK8_9_point_6_KBps_SAMP16		52.0625
#define BR_CLK8_19_point_2_KBps_SAMP16		26.0625
#define BR_CLK8_57_point_6_KBps_SAMP16		8.6875
#define BR_CLK8_115_point_2_KBps_SAMP16		4.3125
#define BR_CLK8_230_point_4_KBps_SAMP16		2.1875
#define BR_CLK8_460_point_8_KBps_SAMP16		1.0625




typedef enum{
	DISABLE_PARITY,
	ENABLE_PARITY
} PARITY;

typedef enum{
	W_8_bits,
	W_9_bits
} Word_Length;

typedef enum{
	SAMPLE_8,
	SAMPLE_16
} Oversampling_Mode;

typedef enum{
	STOP_0_5_bits,
	STOP_1_bits,
	STOP_1_5_bits,
	STOP_2_bits
} Stop_Bits;

typedef struct{
	Stop_Bits stop_bits;
	Oversampling_Mode oversampling_mode;
	Word_Length word_length;
	PARITY parity;
	uint16_t baudrate;
} USART_Param_t;

typedef struct {
	USART_RegType *USART;
	USART_Param_t Config;
} USART_Handle_t;

//init the UART
void init(USART_Handle_t *uart_handle);

//this function returns the length of the data that has been received
uint32_t USART_ReceiveData(USART_Handle_t *uart_handle, uint8_t *data, uint32_t max_len);
//send data that is in the "data" paramter of length "len"
void USART_SendData(USART_Handle_t *uart_handle,uint8_t *data, uint32_t len);

// Function to convert decimal to 16-bit hexadecimal
uint16_t decimalToHex16Bit(float decimalValue);


#endif /* UART_H_ */
