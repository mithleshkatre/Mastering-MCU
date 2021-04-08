/*
 * 004_button_interrupt.c
 *
 *  Created on: 28-Mar-2021
 *      Author: katre
 */

#include <string.h>
#include "stm32f401xx.h"

#define LOW              0
#define BTN_PRESSED      LOW

void delay(void){

	for(uint32_t i=0;i<500000/2;i++);
}

int main(){

	GPIO_Handle_t GpioLed, GPIOBtn;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));
	// led
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	// button
		GPIOBtn.pGPIOx = GPIOC;
		GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
		GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

		GPIO_PeriClockControl( GPIOC, ENABLE);
		GPIO_Init(&GPIOBtn);

	// IRQ configure
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
		GPIO_IRQInterrupConfig(IRQ_NO_EXTI9_5, ENABLE);



		while(1);



	return 0;
}


 void EXTI9_5_IRQHandler(void)
 {
	 GPIO_IRQHandling(GPIO_PIN_NO_13);
	 GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5 );
 }

