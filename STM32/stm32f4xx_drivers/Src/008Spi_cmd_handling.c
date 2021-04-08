/*
 * 006Spi_tx_testing.c
 *
 *  Created on: 01-Apr-2021
 *      Author: katre
 */

#include "stm32f401xx.h"

#include <string.h>



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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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

int main(){

	char user_data[] = "Hello word";

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

		// first send the length information

		uint8_t dataLen =strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// to send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// lets conform SPI is not Busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

	}
	return 0;
}
