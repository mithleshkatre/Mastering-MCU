/*
 * stm32f401xx_usart_driver.c
 *
 *  Created on: 08-Apr-2021
 *      Author: katre
 */

#include "stm32f401xx_usart_driver.h"

uint16_t AHB_PreScaler1[8]= {2,4,8, 16, 64, 128,256,512};
uint16_t APB_PreScaler1[4]= {2,4,8, 16};

/************************************************************************************
 * @fn				- USART_PeriClockControl
 *
 * @brief			- this function enable or disable peripheral clock for USART port
 *
 * @param[in]       - base address of USARTx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
***************************************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}

}

/************************************************************************************
 * @fn				- USART_Init
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
void USART_Init(USART_Handle_t *pUSARTHandle)
{


	//***********************************CR1*******************************************
	//USART_Mode
	uint32_t temp = 0;

	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		temp |= (1 << USART_CR1_TE);
	}

	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX )
		{
		temp |= (1 << USART_CR1_RE);
		}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		temp |= (1 << USART_CR1_TE) | (1 << USART_CR1_RE);
	}

	//USART_WordLength

	 temp |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	 //USART_ParityControl

	 if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	 {
		 temp |= (1 << USART_CR1_PCE );
	 }
	 else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	 {
		 temp |= (1 << USART_CR1_PCE );
		 temp |= (1 << USART_CR1_PS);
	 }

	 // store temp value in CR1
	 pUSARTHandle->pUSARTx->CR1 = temp;

	 //*********************************CR2******************************************

	 temp=0;

     temp |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

     pUSARTHandle->pUSARTx->CR2 = temp;

	 //*********************************CR3******************************************

     temp = 0;

     if(pUSARTHandle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_CTS)
     {
    	 temp |= (1 << USART_CR3_CTSE);
     }

     else if(pUSARTHandle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_RTS)
     {
    	 temp |= (1 << USART_CR3_RTSE);
     }

     else if(pUSARTHandle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_CTS_RTS)
     {
    	 temp |= (1 << USART_CR3_RTSE);
    	 temp |= (1 << USART_CR3_CTSE);
     }

     pUSARTHandle->pUSARTx->CR3 = temp;

     //********************************* baud rate***************************************

     USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return SET;
	}
	return RESET;
}


void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 & (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/************************************************************************************
 * @fn				- USART_SendData
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

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	for(uint32_t i = 0; i < Len ;i++)
	{

		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)));

		// for 9 bit
		if(pUSARTHandle->USART_Config.USART_WordLength  == USART_WORDLEN_9BITS)
		{

			pdata = ((uint16_t*)pTxBuffer);
			pUSARTHandle->pUSARTx->DR = (*pdata & ((uint16_t)0x1FF));


			// parity check (none)
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// parity set
				pTxBuffer++;
			}
		}
		else
		{
			// 8 bit transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0x0FF);
		}

	}

	while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)));
}

/************************************************************************************
 * @fn				- USART_ReceiveData
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
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{


	for(uint32_t i = 0; i < Len ;i++)
	{

		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE)));

		// for 9 bit
		if(pUSARTHandle->USART_Config.USART_WordLength  == USART_WORDLEN_9BITS)
		{

			// parity check (none)
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
			{
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & ((uint16_t)0x1FF));

				pRxBuffer++;
				pRxBuffer++;

			}
			else
			{
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & ((uint8_t)0xFF));

				pRxBuffer++;
			}
		}
		else
		{
			// 8 bit transfer
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE )
			{

				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x0FF);

			}
			else
			{
				// parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x07F);
			}

			pRxBuffer++;
		}

	}
}


/************************************************************************************
 * @fn				- RCC_GetPCLK1Value
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
uint32_t RCC_GetPCLK1Value1(void)
{
	uint32_t pclk1, systemCLK, temp, ahb1p ,apb1p;
	uint32_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0)
	{
		systemCLK = 16000000;
	}
	else if(clksrc == 1)
	{
		systemCLK = 8000000;
	}
	else if(clksrc == 2)
	{
		//systemCLK = RCC_GetPLLOutputCLK();
	}

	//ahb1p
	temp = ((RCC->CFGR >> 4) & 0xf);

	if(temp < 8)
	{
		ahb1p =1;
	}
	else
	{
		ahb1p = AHB_PreScaler1[temp -8];
	}

	//apb1p
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p =1;
	}
	else
	{
		apb1p = APB_PreScaler1[temp -4];
	}

	pclk1 = (systemCLK/ahb1p)/apb1p;
	return pclk1;

}


/************************************************************************************
 * @fn				- RCC_GetPCLK2Value
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
uint32_t RCC_GetPCLK2Value2(void)
{
	uint32_t pclk2, systemCLK, temp, ahb1p, apb2p;
	uint32_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0)
	{
		systemCLK = 16000000;
	}
	else if(clksrc == 1)
	{
		systemCLK = 8000000;
	}
	else if(clksrc == 2)
	{
		//systemCLK = RCC_GetPLLOutputCLK();
	}

	//ahb1p
	temp = ((RCC->CFGR >> 4) & 0xf);

	if(temp < 8)
	{
		ahb1p =1;
	}
	else
	{
		ahb1p = AHB_PreScaler1[temp -8];
	}

	//apb2p
	temp = ((RCC->CFGR >> 13) & 0x7);

	if(temp < 4)
	{
		apb2p =1;
	}
	else
	{
		apb2p = APB_PreScaler1[temp -4];
	}

	pclk2 = (systemCLK/ahb1p)/apb2p;
	return pclk2;

}

/************************************************************************************
 * @fn				- RCC_GetPCLK2Value
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
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t pCLKx, usartdiv;

	uint32_t M_part, F_part;

	uint32_t tempreg =0;

	if(pUSARTx == USART1 || pUSARTx == USART1  )
	{
		RCC_GetPCLK2Value2();
	}
	else
	{
		RCC_GetPCLK1Value1();
	}

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{

		// over8 = 1  ////  over sampling by 8
		usartdiv = ((25 * pCLKx)/(2 * BaudRate));
	}
	else
	{
		// over8 = 0    and    over sampling by 16
		usartdiv = ((25 * pCLKx)/(4 * BaudRate));
	}

	//calculating the mentissa part
	M_part = usartdiv /100;

	tempreg |= M_part << 4;

	//calculating the fraction part

	F_part = usartdiv -(M_part * 100);

	// calculating the final fraction

	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// 8 bit
		F_part =(((F_part * 8)+ 50) / 100) & ((uint8_t)0x07);


	}
	else
	{
		F_part =((((F_part * 16)+50)/100) & (uint16_t)0x0F);
	}

	tempreg |= F_part;

	pUSARTx->BRR = tempreg;
}


/*********************************************************************
* @fn - USART_SendDataWithIT
*
* @brief -
*
* @param[in] -
* @param[in] -
* @param[in] -
*
* @return -
*
* @Note -
**************************************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
 uint8_t txstate = pUSARTHandle->TxBusyState;

 	 if(txstate != USART_BUSY_IN_TX)
 	 {
 		 pUSARTHandle->TxLen = Len;

 		 pUSARTHandle->pTxBuffer = pTxBuffer;

 		 pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

 		 //Implement the code to enable interrupt for TXE
 		 pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);

 		 //Implement the code to enable interrupt for TC
 		 pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);

 	 }
 return txstate;

}

/*********************************************************************
* @fn 			 - USART_ReceiveDataWithIT*
*
* @brief 		 -
*
* @param[in]	 -
* @param[in] 	 -
* @param[in] 	 -
*
* @return        -
*
* @Note			 -
*
*****************************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{

 uint8_t rxstate = pUSARTHandle->RxBusyState;

 	 if(rxstate != USART_BUSY_IN_RX)
 	 {
 		 pUSARTHandle->RxLen = Len;

 		 pUSARTHandle->pRxBuffer = pRxBuffer;

 		 pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

 		 (void)pUSARTHandle->pUSARTx->DR;

 		 //Implement the code to enable interrupt for RXNE
 		 pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);

 	 }

 return rxstate;
}



/*
 * IRQ configuration and ISR Handling
 */


/************************************************************************************
	 * @fn				- USART_IRQInterrupConfig
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

void USART_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
		 * @fn				- USART_IRQPriorityConfig
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

void USART_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx =IRQNumber /4;
	uint8_t iprx_section =IRQNumber %4;

	uint8_t shift_amount =(8 * iprx_section)+(8 - NO_PR_BITS_IMPLEMENTED);

   *(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}

/************************************************************************************
		 * @fn				- USART_IRQPriorityConfig
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

void USART_IRQHandling(USART_Handle_t *pUSARTHandle )
{
	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;


/*************************Check for TC flag **********************************
**********/
	//Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	//Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC
		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{

			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}

	}


/*************************Check for TXE flag *********************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
				pUSARTHandle->TxBusyState = USART_READY;

				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}


/*************************Check for RXNE flag ********************************
************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);
	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data
						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data
						//read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
					else
					{
						//Parity is used, so , 7 bits will be of user data and1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
					}
					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=1;
				}
			}

			//if of >0
			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;

				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag *********************************
***********/

	//Note : CTS feature is not applicable for UART4 and UART5
	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);

	if(temp1 && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}


	/*************************Check for IDLE detection flag **********************
**********************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}



/*************************Check for Overrun detection flag *******************
*************************/

	//Implement the code to check the status of ORE flag in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if(temp1 && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag *******************************
*************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used. temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;
	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;

		if(temp1 & ( 1 << USART_SR_FE))
		{
	/*
 	 This bit is set by hardware when a desynchronization, excessive noise or a break character
 	 is detected. It is cleared by a software sequence (an read to the USART_SR register
 	 followed by a read to the USART_DR register).
	*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
		/*
 	 	 This bit is set by hardware when noise is detected on a received frame. It is cleared by a
 	 	 software sequence (an read to the USART_SR register followed by a read to the USART_DR register).
		 */

			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}


/*********************************************************************
* @fn - USART_ApplicationEventCallback
*
* @brief -
*
* @param[in] -
* @param[in] -
* @param[in] -
** @return -
*
* @Note -
************************************************************************/
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}































