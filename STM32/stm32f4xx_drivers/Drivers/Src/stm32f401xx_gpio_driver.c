/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Mar 13, 2021
 *      Author: katre
 */
//******************************************************************************


#include "stm32f401xx_gpio_driver.h"


/************************************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- this function enable or disable peripheral clock for GPIO port
 *
 * @param[in]       - base address of GPIO peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
***************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();

		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}
	}

	else{
		//disable
	}

}
	/************************************************************************************
	 * @fn				- GPIO_Init
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp =0;
	// 1. configure mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// non interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}


	else{
		// for interrupt
	}

	temp=0;
	 //2. configure the speed

		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;  // setting

	temp=0;
	// 3. configure the pull up and pull down

		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~ (0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;
	// 4. configure the out type

		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~ ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER |= temp;


	// 5. configure the output type
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~ (0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}

}

/************************************************************************************
	 * @fn				- GPIO_DeInit
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

		if(pGPIOx==GPIOA){
			GPIOA_REG_RESET();

		}else if(pGPIOx==GPIOB){
			GPIOB_REG_RESET();
		}
		else if(pGPIOx==GPIOC){
			GPIOC_REG_RESET();
		}
		else if(pGPIOx==GPIOD){
			GPIOD_REG_RESET();
		}
		else if(pGPIOx==GPIOE){
			GPIOH_REG_RESET();
		}
		else if(pGPIOx==GPIOH){
			GPIOH_REG_RESET();
		}

}

/************************************************************************************
	 * @fn				- GPIO_ReadFromInputPin
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;

}

/************************************************************************************
	 * @fn				- GPIO_ReadFromInputPin
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint32_t value;
		value = (uint32_t)(pGPIOx->IDR);
		return value;

}


/************************************************************************************
	 * @fn				- GPIO_WriteToOutputPin
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET ){
		// for write 1
		pGPIOx->ODR |=(1 << PinNumber);

	}else{
		// for write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}



/************************************************************************************
	 * @fn				- GPIO_WriteToOutputPort
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;


}



/************************************************************************************
	 * @fn				- GPIO_ToggleOutputPin
	 *
	 * @brief			- GPIO register configuration
	 *
	 * @param[in]       - base address of GPIO peripheral
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber);

}



//IRQ configuration and ISR

void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi );

void GPIO_IRQHandling(uint8_t PinNumber);



