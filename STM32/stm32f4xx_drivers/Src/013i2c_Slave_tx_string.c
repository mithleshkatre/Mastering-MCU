/*
 * 013i2c_Slave_tx_string.c
 *
 *  Created on: 09-Apr-2021
 *      Author: katre
 */


#include "stm32f401xx.h"

#include <string.h>

//extern void initialise_monitor_handles();

void delay(void){

	for(uint32_t i=0;i<500000;i++);
}


#define SLAVE_ADDER   0x68
#define MY_ADDER	SLAVE_ADDER
I2C_Handle_t I2C1Handle;

// receive data

uint8_t tx_buf[32] = "STM32 Slave mode received\n" ;

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

	I2C_IRQInterrupConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterrupConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	// enable the I2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

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

	static uint8_t commandCode = 0;
	static uint8_t cnt = 0;
	if(AppEv == I2C_EV_DATA_REQ)
		{
			//master wants some data. slave has to send it
			if(commandCode == 0x51)
			{
				//send the length information to master
				I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buf));
			}

			else if(AppEv ==  0x51)
			{
			//send contents of tx_buf
				I2C_SlaveSendData(pI2CHandle->pI2Cx, tx_buf[cnt++]);
			}

		else if(AppEv == I2C_EV_DATA_RCV)
		{
			//Data is Waiting for slave to read
			commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

		}

		else if(AppEv == I2C_ERROR_AF)
		{
			//this happen only during slave txing
			commandCode = 0xff;
		}

		else if(AppEv == I2C_EV_STOP)
		{


		}
}

}

















