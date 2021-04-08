/*
 * 012i2c_master_rx_testingIT.c
 *
 *  Created on: 07-Apr-2021
 *      Author: katre
 */


#include "stm32f401xx.h"

#include <string.h>

//extern void initialise_monitor_handles();

void delay(void){

	for(uint32_t i=0;i<500000;i++);
}

#define MY_ADDER    0x61;
#define SLAVE_ADDER   0x68

I2C_Handle_t I2C1Handle;

// receive data

uint8_t rcv_data[32] ;

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

	uint8_t commondcode;
	uint8_t len;


	//initialise_monitor_handles();

	//printf("Application is running\n");


	// i2c pin inits
	I2C1_GPIOInits();

	// i2c peripheral configuration
	I2C1_Inits();

	// gpio button for i2c1
	GPIO_ButtonInit();

	I2C_IRQInterrupConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterrupConfig(IRQ_NO_I2C1_ER, ENABLE);

	// enable the I2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		// wait for button is pressed
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		delay();

		commondcode = 0x51;

		while(I2C_MasterSendDataIT(&I2C1Handle, &commondcode, 1, SLAVE_ADDER, I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDER, I2C_ENABLE_SR) != I2C_READY);


		commondcode = 0x52;

		while(I2C_MasterSendDataIT(&I2C1Handle, &commondcode, 1, SLAVE_ADDER, I2C_ENABLE_SR) != I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_data, 1, SLAVE_ADDER, I2C_DISABLE_SR) != I2C_READY);

	}
	while(1);
	return 0;
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}
	else if(AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		printf("Error : Ack Failure");

		I2C_CloseSendData(&I2C1Handle);

		// generate stop condition
		I2C_GenerateStopCondition(I2C1);

		while(1);
	}
}

