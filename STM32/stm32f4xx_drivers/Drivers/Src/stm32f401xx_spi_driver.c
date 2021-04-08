/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: 31-Mar-2021
 *      Author: katre
 */



#include "stm32f401xx_spi_driver.h"


static void spi_txe_intrrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_intrrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_intrrupt_handle(SPI_Handle_t *pSPIHandle);


/************************************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			- this function enable or disable peripheral clock for SPI port
 *
 * @param[in]       - base address of SPIx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
***************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pSPIx==SPI1){
			SPI1_PCLK_EN();

		}else if(pSPIx==SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4){
			SPI3_PCLK_EN();
		}
	}

	else{
		//disable
	}

}

/************************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			-
 *
 * @param[in]       - base address of SPIx peripheral
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
***************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
   // first lets configure the SPI_CR1 Reg.
	uint32_t temp =0;
	//configure the device mode
	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << 2;

	//2. configure the bus config

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// bidi mode should be clear
		temp &= ~(1<<15);

	}

	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi mode should be set
		temp |= (1<<15);

	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
			// bidi mode should be set
		temp &= ~(1<<15);
		temp |=   (1<<10);

		}

	// configure the spi serial clock speed (baud rate)
	temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << 3;

	// configure the DFF

	temp |= pSPIHandle->SPI_Config.SPI_DFF << 11;

	// configure the CPHA
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << 0;

	// configure the CPOL
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << 1;

	pSPIHandle->pSPIx->CR1 = temp;
}


/************************************************************************************
 * @fn				- SPI_DeInit
 *
 * @brief			- this function enable or disable peripheral clock for SPI port
 *
 * @param[in]       - base address of SPIx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
***************************************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{

	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

}



/************************************************************************************
 * @fn				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]       -
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- this is blocking call
 *
 *
***************************************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. wait until TXE is Set

		//while((pSPIx->SR & (1<<1)));
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)== FLAG_RESET);

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// 1. load the data in the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			pSPIx->DR = *(uint8_t*)pTxBuffer;
			Len--;
			(uint8_t*)pTxBuffer++;
		}
	}
}




/************************************************************************************
 * @fn				- SPI_ReceiveData
 *
 * @brief			-
 *
 * @param[in]       -
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- this is blocking call
 *
 *
***************************************************************************************/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. wait until RXNE is Set

		//while((pSPIx->SR & (1<<1)));
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)== FLAG_RESET);

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// 1. load the data in the DR
		  *((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			 *(uint8_t*)pRxBuffer = pSPIx->DR ;
			Len--;
			(uint8_t*)pRxBuffer++;
		}
	}
}



/************************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 * @brief			- this function enable or disable peripheral clock for SPI port
 *
 * @param[in]       - base address of SPIx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- this is blocking call
 *
 *
***************************************************************************************/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}


}
/************************************************************************************
 * @fn				- SPI_SSIConfig
 *
 * @brief			- this function enable or disable peripheral clock for SPI port
 *
 * @param[in]       - base address of SPIx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- this is blocking call
 *
 *
***************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}


}

/************************************************************************************
 * @fn				- SPI_SSOEConfig
 *
 * @brief			-
 *
 * @param[in]       - base address of SPIx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- this is blocking call
 *
 *
***************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}


}


/*
 * IRQ configuration and ISR Handling
 */


/************************************************************************************
	 * @fn				- SPI_IRQInterrupConfig
	 *
	 * @brief			-
	 *
	 * @param[in]       -
	 * @param[in]		- Enable or DISABLE
	 * @param[in]		-
	 *
	 * @return			- none
	 *
	 * @Note			- none
	 *
	 *
	***************************************************************************************/

void SPI_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(IRQNumber <=31){
			//progress ISER0 register
			*NVIC_ISER0 |=(1 << IRQNumber);

			}
		else if (IRQNumber > 31  || IRQNumber <64){
			//progress ISER1 register
			*NVIC_ISER1 |=(1 << (IRQNumber % 32));

			}


		else if (IRQNumber > 64  || IRQNumber < 96){
			//progress ISER2 register
			*NVIC_ISER2 |=(1 << (IRQNumber % 64));
			}
	}
	else
	{
		if(IRQNumber <=31){
			//progress ICER0 register
			*NVIC_ICER0 |=(1 << IRQNumber);

			}
		else if (IRQNumber > 31  || IRQNumber <64){
			//progress ICER1 register
			*NVIC_ISER1 |=(1 << (IRQNumber % 32));

			}

		else if (IRQNumber > 64  || IRQNumber < 96){
			//progress ICER2 register
			*NVIC_ISER2 |=(1 << (IRQNumber % 64));

			}
	}
}
/************************************************************************************
		 * @fn				- SPI_IRQPriorityConfig
		 *
		 * @brief			-
		 *
		 * @param[in]       -
		 * @param[in]		-
		 * @param[in]		-
		 *
		 * @return			- none
		 *
		 * @Note			- none
		 *
		 *
***************************************************************************************/

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx =IRQNumber /4;
	uint8_t iprx_section =IRQNumber %4;

	uint8_t shift_amount =(8 * iprx_section)+(8 - NO_PR_BITS_IMPLEMENTED);

   *(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}

/************************************************************************************
		 * @fn				-SPI_SendDataIT
		 *
		 * @brief			-
		 *
		 * @param[in]       -
		 * @param[in]		-
		 * @param[in]		-
		 *
		 * @return			- none
		 *
		 * @Note			- none
		 *
		 *
***************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//1. save the Tx buffer address and Len information in some global variable
	   pSPIHandle->pTxBuffer = pTxBuffer;
	   pSPIHandle->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that
	// no other code can take over same SPI peripheral untill transmission is over
	   pSPIHandle->TxState = SPI_BUSY_IN_TX;

	// 3. Enable the TXEIE control bit to get Interrupt whenever TXE flag is set in SR
	   pSPIHandle->pSPIx->CR2 |= (1<< SPI_CR2_TXEIE);
	//4. Data Transmission will be handled by the ISR code
	}

	return state;
}

/************************************************************************************
		 * @fn				-SPI_ReceiveDataIT
		 *
		 * @brief			-
		 *
		 * @param[in]       -
		 * @param[in]		-
		 * @param[in]		-
		 *
		 * @return			- none
		 *
		 * @Note			- none
		 *
		 *
***************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
		//1. save the Tx buffer address and Len information in some global variable
		   pSPIHandle->pRxBuffer = pRxBuffer;
		   pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral untill transmission is over
		   pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the TXEIE control bit to get Interrupt whenever TXE flag is set in SR
		   pSPIHandle->pSPIx->CR2 |= (1<< SPI_CR2_RXNEIE);
		//4. Data Transmission will be handled by the ISR code
		}

		return state;
}


/************************************************************************************
		 * @fn				-SPI_IRQHandling
		 *
		 * @brief			-
		 *
		 * @param[in]       -
		 * @param[in]		-
		 * @param[in]		-
		 *
		 * @return			- none
		 *
		 * @Note			- none
		 *
		 *
***************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle )
{
	uint8_t temp1, temp2;
	// first lets check the TXE
	temp1= pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2= pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_txe_intrrupt_handle(pSPIHandle);
	}

	// check for RXNE
	temp1= pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2= pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
		{
			// handle RXNE
			spi_rxne_intrrupt_handle(pSPIHandle);
		}

	// check for over flag
		temp1= pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
		temp2= pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

		if(temp1 && temp2)
			{
				// handle RXNE
				spi_ovr_err_intrrupt_handle(pSPIHandle);
			}
}

/*
 * some helper function implementation
 */

static void spi_txe_intrrupt_handle(SPI_Handle_t *pSPIHandle)
{
	  if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
			// 16 bit DFF
		   // 1. load the data in the DR
			*((uint16_t*)pSPIHandle->pTxBuffer) = pSPIHandle->pSPIx->DR;
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
			}
			else
			{
			*(uint8_t*)pSPIHandle->pTxBuffer = pSPIHandle->pSPIx->DR ;
			pSPIHandle->TxLen--;
			(uint8_t*)pSPIHandle->pTxBuffer++;
			}

	  if(! pSPIHandle->TxLen)
	  {
		  // Tx Len is Zero , so close the spi transmission and inform the application that
		  //Tx is over

		  //this prevents interrupt from setting up of TXE flag
		  pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_TXEIE);
		  pSPIHandle->pTxBuffer = NULL;
		  pSPIHandle->TxLen =0;
		  pSPIHandle->TxLen = SPI_READY;
		  SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	  }

}

static void spi_rxne_intrrupt_handle(SPI_Handle_t *pSPIHandle)
{
	  if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
			// 16 bit DFF
		   // 1. load the data in the DR
			*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			(uint16_t*)pSPIHandle->pRxBuffer--;
			(uint16_t*)pSPIHandle->pRxBuffer--;

			}
			else
			{
				//8 bit
			*(uint8_t*)pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR ;
			pSPIHandle->RxLen--;
			(uint8_t*)pSPIHandle->pRxBuffer--;
			}

	  if(! pSPIHandle->RxLen)
	  {
		  // Reception is completed
		  //lets turn off the rxne interrupt

		  pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_RXNEIE);
		  pSPIHandle->pTxBuffer = NULL;
		  pSPIHandle->TxLen =0;
		  pSPIHandle->TxLen = SPI_READY;
		  SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	  }

}
static void spi_ovr_err_intrrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState |= SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	  SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	  pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_TXEIE);
	  pSPIHandle->pTxBuffer = NULL;
	  pSPIHandle->TxLen =0;
	  pSPIHandle->TxLen = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{

	  pSPIHandle->pSPIx->CR2 &= ~(1<< SPI_CR2_RXNEIE);
	  pSPIHandle->pTxBuffer = NULL;
	  pSPIHandle->TxLen =0;
	  pSPIHandle->TxLen = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
//this is weak implementation, the application may override this function
}





