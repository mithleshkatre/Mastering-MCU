/*
 * 006Spi_tx_testing.c
 *
 *  Created on: 01-Apr-2021
 *      Author: katre
 */

#include "stm32f401xx.h"

#include <string.h>

//COMMOND CODE
#define COMMAND_LED_CTRL     	 0x50;
#define COMMAND_SENSOR_READ      0x51;
#define COMMAND_LED_READ         0x52;
#define COMMAND_PRINT            0x53;
#define COMMAND_ID_READ          0x54;


#define LED_ON			1
#define LED_OFF         0

//ARDUINO ANALOG PIN

#define ANALOG_PIN0     0
#define ANALOG_PIN1     1
#define ANALOG_PIN2     2
#define ANALOG_PIN3     3
#define ANALOG_PIN4     4


// AURDUINO LED
#define LED_PIN    9

void delay(void){

	for(uint32_t i=0;i<500000;i++);
}

/*
 * SPI2
 * PB14 - SPI2_MISO
 * PB15  - SPI2_MOSI
 * PB13  - SPI2_SCLK
 * PB12  - SPI2_NSS
 * ALT function mode: 5
 * port B
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	memset(&SPI2Handle,0,sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl( GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte ==0xf5)
	{
		return 1;//ack
	}
	else
	{
		return 0;
	}
}

int main(){

	uint8_t dummy_write= 0xff;
	uint8_t dummy_read;

	// button function
	GPIO_ButtonInit();

	// this function is used to initialize the GPIO Pins to behave as SPI2 pins
	SPI2_GPIOInits();

    // this function is used to initialize the SPI2 peripheral parameter
	SPI2_Inits();

	// this make NSS signal internally high and avoid MODF error
	//SPI_SSIConfig(SPI2,ENABLE);

	SPI_SSOEConfig(SPI2,ENABLE);
	while(1){
		while(!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13)));

		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);


		//1. cmd_led_ctrl   <pin no(1)>     <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		// send cmd
		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

	    // send some dummy bits(1 byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//verify the response
		if(SPI_VerifyResponse(ackbyte))
		{

			args[0]=LED_PIN;
			args[1]=LED_ON;
			//send argument
			SPI_SendData(SPI2, args, 2);
		}

		// lets conform SPI is not Busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));


		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

	}
	return 0;
}
