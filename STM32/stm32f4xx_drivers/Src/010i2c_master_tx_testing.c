/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: 06-Apr-2021
 *      Author: katre
 */


#include "stm32f401xx.h"

#include <string.h>

void delay(void){

	for(uint32_t i=0;i<500000;i++);
}

#define MY_ADDER    0x61;
#define SLAVE_ADDER   0x68

I2C_Handle_t I2C1Handle;

// some date

uint8_t some_data[] = "We are testing I2C master Tx\n";

/*
 PB6 -> I2C1_SCL
 PB7 -> I2C1_SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// sclk
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&I2CPins);

	// sclk
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDER;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_PeriClockControl(I2C1, ENABLE);
	I2C_Init(&I2C1Handle);
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

	// i2c pin inits
	I2C1_GPIOInits();

	// i2c peripheral configuration
	I2C1_Inits();

	// gpio button for i2c1
	GPIO_ButtonInit();

	// enable the I2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	while(1)
	{
		// wait for button is pressed
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();


	// send data to slave

	I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDER);

	}
	while(1);
	return 0;
}

