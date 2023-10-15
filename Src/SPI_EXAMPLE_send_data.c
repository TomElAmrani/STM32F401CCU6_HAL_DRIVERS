/*
 * SPI_EXAMPLE_send_data.c
 *
 *  Created on: Sep 7, 2023
 *      Author: elamrani
 */
#include <string.h>
#include "GPIO.h"
#include "SPI.h"
#include "I2C.h"
#include "UART.h"
#include <stdlib.h>

//Global variables
uint8_t * DataReceived;
char Msg[]= "hello from el amrani";

//to use in interrupt service routine
SPI_Param_t SPI1_CONFIG;


//prototypes
void SPI1_GPIO_INIT();
void SPI_PROGRAM();
void delay(){
	for(int i=0;i<500000;i++);
}


//Without using interrupt
/*
int main(){
	SPI1_GPIO_INIT();

	while(1){
		delay();
		SPI_PROGRAM();
		delay();delay();delay();delay();delay();
	}

	return 0;
}
*/

//With using interrupt

int main(){
	SPI1_GPIO_INIT();
	SPI_INIT( &SPI1_CONFIG , SPI1);

	SPI_Interrupts_t *Config_IT=malloc(sizeof(SPI_Interrupts_t));
	Config_IT->SPI_RXNEIE = 1;
	SPI_Enable_Interrupts ( SPI1 , Config_IT);
	free(Config_IT);

	SPI_ENABLE(SPI1, ENABLE);




	while(1){
		if(DATA_RECEIVEED_FLAG_IT){
			//code to use the data received;
			DATA_RECEIVED_WITH_IT;
		}
		DATA_RECEIVEED_FLAG_IT=0;

	}

	return 0;
}



void SPI1_GPIO_INIT(){
	/*
	 * PA4 : SPI1_NSS
	 * PA5 : SPI1_SCLK
	 * PA6 : SPI1_MISO
	 * PA7 : SPI1_MOSI
	 */
	GPIO_ChangeClk(GPIOA, ENABLE);
	GPIO_PINHandle *gpio_handle=malloc(sizeof(GPIO_PINHandle));
	gpio_handle->port = GPIOA;
	gpio_handle->Config.AF = AF5;
	gpio_handle->Config.ospeed= VHighSpeed;
	gpio_handle->Config.pin_mode = AF;
	gpio_handle->Config.pupd = NO;


	//SPI1_NSS
	gpio_handle->PIN = GPIO_PIN_4;
	GPIO_Init(gpio_handle);
	//SPI1_SCLK
	gpio_handle->PIN = GPIO_PIN_5;
	GPIO_Init(gpio_handle);
	//SPI1_MISO
	gpio_handle->PIN = GPIO_PIN_6;
	GPIO_Init(gpio_handle);
	//SPI1_MOSI
	gpio_handle->PIN = GPIO_PIN_7;
	GPIO_Init(gpio_handle);

}

void SPI_PROGRAM(){
	SPI_Param_t * Config= malloc(sizeof(SPI_Param_t ));
	Config->BaudRate = DIVISION_256;
	Config->CPHA=0;
	Config->CPOL=0;
	Config->DFF=0;
	Config->LSBFIRST=0;
	Config->MODE=SPI_FULL_DUPLEX;
	Config->Master=SPI_MASTER;
	Config->SSM=0;

	SPI_INIT(Config, SPI1);

	SPI_ENABLE(SPI1, ENABLE);

	SPI_SendDATA(SPI1, Config, (uint8_t *)Msg, (uint32_t)strlen(Msg));

	DataReceived=SPI_ReceiveDATA(SPI1,Config,15);

	while( SPI_Tx_IS_BUSY(SPI1) );

	SPI_ENABLE(SPI1, DISABLE);

	free(Config);
}


void SPI1_IRQHandler(void){
	SPI_IRQHandling(SPI1, &SPI1_CONFIG );
}

