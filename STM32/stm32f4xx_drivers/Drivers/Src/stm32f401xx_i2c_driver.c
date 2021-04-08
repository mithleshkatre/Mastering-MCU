/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 02-Apr-2021
 *      Author: katre
 */

#include "stm32f401xx_i2c_driver.h"


uint16_t AHB_PreScaler[8]= {2,4,8, 16, 64, 128,256,512};
uint16_t APB_PreScaler[4]= {2,4,8, 16};

void RCC_GetPLLOutputCLK(void);
uint32_t RCC_GetPCLK1Value(void);

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);


static void I2C_MasterHandleRXNEInterrupt( I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt( I2C_Handle_t *pI2CHandle);


void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);  // SlaveAddr is Slave address + r/ rw bit =0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1);  // SlaveAddr is Slave address + r/ rw bit =1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;

	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// device in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// first disable the ACK

				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				//clear the ADDER flag(sr1,sr2)
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;

				(void)dummyRead;

			}
		}
		else
		{
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;

			(void)dummyRead;
		}


	}
	else
	{
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;

		(void)dummyRead;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

	}
	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	}
}


/************************************************************************************
 * @fn				- I2C_PeripheralControl
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

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}


}

/************************************************************************************
 * @fn				- I2C_PeriClockControl
 *
 * @brief			- this function enable or disable peripheral clock for I2C port
 *
 * @param[in]       - base address of I2Cx peripheral
 * @param[in]		- Enable or DISABLE
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 *
***************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pI2Cx==I2C1){
			I2C1_PCLK_EN();

		}else if(pI2Cx==I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx==I2C3){
			I2C3_PCLK_EN();
		}
	}

	else{
		//disable
	}

}

//void RCC_GetPLLOutputCLK(void)
//{

//}


uint32_t RCC_GetPCLK1Value(void)
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
		ahb1p = AHB_PreScaler[temp -8];
	}

	//apb1p
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p =1;
	}
	else
	{
		apb1p = APB_PreScaler[temp -4];
	}

	pclk1 = (systemCLK/ahb1p)/apb1p;
	return pclk1;

}


/************************************************************************************
 * @fn				- I2C_Init
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

void I2C_Init(I2C_Handle_t *pI2Handle)
{
	uint8_t tempreg = 0;

	//ack control bit
	tempreg |= pI2Handle->I2C_Config.I2C_ACKControl << 10;
	pI2Handle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2

	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2Handle->pI2Cx->CR2 = (tempreg & 0x3f);

	//program the device own address

	tempreg = 0;
	tempreg |= pI2Handle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1<< 14);
	pI2Handle->pI2Cx->OAR1 = tempreg;

	//CCR calculation

	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is in standard mode
		ccr_value = (RCC_GetPCLK1Value())/(2*pI2Handle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xfff);
	}
	else
	{
		//mode is fast mode;
		tempreg |= (1 <<15);
		tempreg |= (pI2Handle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value |= (RCC_GetPCLK1Value())/(3*pI2Handle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			ccr_value |= (RCC_GetPCLK1Value())/(25*pI2Handle->I2C_Config.I2C_SCLSpeed);
		}

		tempreg |= (ccr_value & 0xfff);
	}

	pI2Handle->pI2Cx->CCR |= tempreg;

	// TRISE Configuration

	if(pI2Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			// mode is in standard mode

			tempreg = (RCC_GetPCLK1Value()/1000000U)+1;
		}
		else
		{
			//mode is fast mode;
		tempreg = ((RCC_GetPCLK1Value()*300)/1000000000U)+1;

		}
	pI2Handle->pI2Cx->TRISE = (tempreg & 0x3f);


}
/************************************************************************************
 * @fn				- I2C_MasterSendData
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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Conform that start generation is completed by checking the SB flag in the SR1
	// NOTE: until SB is cleared SCL will be stretched (pulled to Low)

	 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	 //3. send the address of the slave with e/rw bit set to w(0) (total 8 bit)

	 I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	 //4. Confirm that address phase is completed by checking the ADDR flag in the SR1

	 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR) );

	 //5. Clear the ADDR flag according to its software sequence
	 // note: until ADDR is cleared SCL will be stretched (pulled to Low)

	 I2C_ClearADDRFlag(pI2CHandle);

	 //6. send data until Len Become 0

	 while(Len > 0)
	 {
		 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		 pI2CHandle->pI2Cx->DR = *pTxbuffer;
		 pTxbuffer++;
		 Len--;
	 }

	 //7.when len becomes zero wait for TXE=1 and BTf=1 before generate the STOP condition
	 // Note: TXE =1, BTF =1, mean that both SR and DR are empty and next transmission should begin
	 // When BTF =1 SCL will be Stretched (pulled to low)

	 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	 //8. Generating the stop condition and master need not to wait for the completion of stop condition
	 // note: generating stop , automatically clear the BTF
	 if(Sr == I2C_DISABLE_SR){
	 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	 }


}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{

	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;

}
/************************************************************************************
 * @fn				- I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Conform that start generation is completed by checking the SB flag in the SR1
	// NOTE: until SB is cleared SCL will be stretched (pulled to Low)

	 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB) );

	//3. send the address of the slave with e/rw bit set to R(1) (total 8 bit)

	 I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4.wait and  Confirm that address phase is completed by checking the ADDR flag in the SR1

	 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR) );

	// procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// disable acking

		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag

		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE become 1
		 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate stop condition
		 if(Sr == I2C_DISABLE_SR){
			 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		 }

		//Read data in buffer

		 *pRxbuffer = pI2CHandle->pI2Cx->DR;

	}

	//procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len become zero
		for(uint32_t i = Len; i > 0; i-- )
		{
			// wait until RxNE become 1
			 while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


			if(i == 2) //if last two byte are remaining
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				// generate stop condition
				 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read the data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxbuffer++;
		}
	}
	//re- enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		// enable acking
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		// disable
		pI2Cx->CR1 &= ~ (1 << I2C_CR1_ACK);
	}
}

/************************************************************************************
 * @fn				- I2C_MasterSendDataIT
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

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
			if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))

				{

				pI2CHandle->pTxBuffer = pTxbuffer;
				pI2CHandle->TxLen = Len;
				pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
				pI2CHandle->DevAddr = SlaveAddr;
				pI2CHandle->Sr = Sr;

				// Implement code to generate START condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


				// Implement code to enable ITBUFEN control bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

				// Implement code to enable ITEVFEN control bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

				// Implement code to enable ITERREN control bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
				}

			return busystate;
}

/************************************************************************************
 * @fn				- I2C_MasterReceiveDataIT
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

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

			if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
				{

				pI2CHandle->pTxBuffer = pRxbuffer;
				pI2CHandle->TxLen = Len;
				pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
				pI2CHandle->DevAddr = SlaveAddr;
				pI2CHandle->Sr = Sr;

				// Implement code to generate START condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


				// Implement code to enable ITBUFEN control bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

				// Implement code to enable ITEVFEN control bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

				// Implement code to enable ITERREN control bit
				pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

				}
			return busystate;
}


/************************************************************************************
		 * @fn				- I2C_IRQInterrupConfig
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
void I2C_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
		 * @fn				- I2C_IRQPriorityConfig
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx =IRQNumber /4;
	uint8_t iprx_section =IRQNumber %4;

	uint8_t shift_amount =(8 * iprx_section)+(8 - NO_PR_BITS_IMPLEMENTED);

   *(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}

 static void I2C_MasterHandleTXEInterrupt( I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen >0)
	{
		//1. load the data in DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}

}

 static void I2C_MasterHandleRXNEInterrupt( I2C_Handle_t *pI2CHandle)
{
	// we have to do data reception
	if(pI2CHandle->RxSize == 1)
	{
	*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
	pI2CHandle->RxLen--;
	}

		if(pI2CHandle->RxSize > 1)
		{
			if(pI2CHandle->RxSize == 2)
			{
				// clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
			}

			// read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
		}

		if(pI2CHandle->RxLen == 0)
		{
			//close the I2C data reception and notify the application

			//1. generate the stop condition
			if(pI2CHandle->Sr == I2C_DISABLE_SR)
			{
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//2. close the i2c rx
			I2C_CloseReceiveData(pI2CHandle);

			//3. notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
		}
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C , uint8_t data)
{
	pI2C->DR = data;

}



uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	return (uint8_t) pI2C->DR;

}

/************************************************************************************
		 * @fn				- I2C_IRQPriorityConfig
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

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	// Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle for interrupt generated by SB event
	// note: SB flag is only in master mode
	if(temp1 && temp3)
	{
		// the interrupt is generated because SB even
		// this block will not  be executed in slave mode because for slave SB is always zero
		//In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle for interrupt generated by ADDR event
	// Note: When master mode: Address is send
	//       When Slave mode : address matched with own addrss
	if(temp1 && temp3)
	{
		// the interrupt is generated by ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. handle for interrupt generated by BTF (byte transfer finished) event
	if(temp1 && temp3)
	{
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// make sure tht TXE is also SET
			if(pI2CHandle->pI2Cx->SR1 == I2C_SR1_TxE)
			{
			// BTF, TXE =1
			if(pI2CHandle->TxLen == 0)
			{
				//1. generate the stopcondition
				if(pI2CHandle->Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
				//2.reset all the member element of the handle structure
				I2C_CloseSendData(pI2CHandle);

				//3. notify the application about transmission complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
		   }

	   }
	else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
	  {
		;
	  }
	}



	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4.  Handle for interrupt generated by STOP event
	// Note: stop detection flag is applicable only slave mode, for master this flag with
	if(temp1 && temp3)
	{
		// STOP flag is set
		// clear the STOPF (i.e 1) read SR1 2) write to SR1)

		pI2CHandle->pI2Cx->SR1 |= 0x0000;

		// notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE);
	//5. Handle for interrupt generated by TXE event

	if(temp1 && temp2 & temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
		// TXE flag is set
		// We have to do the data transmission

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);

			}
			else
			{    //slave
				//make sure that slave is in transmision mode
				if(pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_TRA))
				{
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
				}
			}
		}
	}



	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	//6.  Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
		// RXNE flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{

			I2C_MasterHandleRXNEInterrupt(pI2CHandle);

		}
		else
		{
			  //slave
			//make sure that slave is in received mode
			if(! (pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}

		}
	}
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<< I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<< I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<< I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1<< I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
 uint32_t temp1,temp2;

 //Know the status of ITERREN control bit in the CR2
 temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);
/***********************Check for Bus error***********************************
*/
 	 temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
 	 if(temp1 && temp2 )
 	 	 {
 		 //This is Bus error
 		 //Implement the code to clear the buss error flag
 		 pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

 		 //Implement the code to notify the application about the error
 		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
 	 	 }

/***********************Check for arbitration lost error**********************
**************/
 	 temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
 	 if(temp1 && temp2)
 	 {
 		 //This is arbitration lost error
 		 //Implement the code to clear the arbitration lost error flag
 		 pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
 		 //Implement the code to notify the application about the error
 		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
 	 }


/***********************Check for ACK failure error**************************
**********/
 	 temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
 	 if(temp1 && temp2)
 	 {
 		 //This is ACK failure error
 		 //Implement the code to clear the ACK failure error flag pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
 		 //Implement the code to notify the application about the error
 		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
 	 }
/***********************Check for Overrun/underrun error**********************
**************/
 	 temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
 	 if(temp1 && temp2)
 	 {
 		 //This is Overrun/underrun
 		 //Implement the code to clear the Overrun/underrun error flag
 		 pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
 		 //Implement the code to notify the application about the error
 		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
 	 }
/***********************Check for Time out error******************************
******/
 	 temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
 	 if(temp1 && temp2)
 	 {
 		 //This is Time out error
 		 //Implement the code to clear the Time out error flag
 		 pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
 		 //Implement the code to notify the application about the error
 		 I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
 	 }
}





