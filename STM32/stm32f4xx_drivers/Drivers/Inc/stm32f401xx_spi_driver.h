/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: 31-Mar-2021
 *      Author: katre
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

/*
 * configure structure for SPIx peripheral
 */

typedef struct{

	uint8_t SPI_DeviceMode;  // @SPI_DeviceMode
	uint8_t SPI_BusConfig;   //  @SPI_BusConfig
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;


//this is the handle structure for SPIx

typedef struct{

	SPI_RegDef_t *pSPIx;  //this holds the base address of spix port
	SPI_Config_t SPI_Config; //this holds spix pin configuration setting
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint8_t  TxLen;
	uint8_t  RxLen;
	uint8_t  TxState;
	uint8_t  RxState;

}SPI_Handle_t;

/*
 * SPI application state
 */
#define SPI_READY    		 0
#define SPI_BUSY_IN_RX       1
#define SPI_BUSY_IN_TX       2

/*
 * @SPI_DeviceMode
 */

#define  SPI_DEVICE_MODE_MASTER		1
#define  SPI_DEVICE_MODE_SLAVE		0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD           			1
#define SPI_BUS_CONFIG_HD          				1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY           1

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2           	 0
#define SPI_SCLK_SPEED_DIV4           	 1
#define SPI_SCLK_SPEED_DIV8           	 2
#define SPI_SCLK_SPEED_DIV16          	 3
#define SPI_SCLK_SPEED_DIV32             4
#define SPI_SCLK_SPEED_DIV64             5
#define SPI_SCLK_SPEED_DIV128            6
#define SPI_SCLK_SPEED_DIV256            7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS       0
#define SPI_DFF_16BITS      1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH       1
#define SPI_CPOL_LOW        0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH       1
#define SPI_CPHA_LOW        0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN          1
#define SPI_SSM_DI          0


/*
 * SPI_ related status flags defination
 */
#define   SPI_TXE_FLAG      (1<< SPI_SR_TXE)
#define   SPI_RXNE_FLAG      (1<< SPI_SR_RXNE)
#define   SPI_BUSY_FLAG      (1<< SPI_SR_BSY)

/*
 * possible SPI Application event
 */

#define SPI_EVENT_TX_CMPLT     1
#define SPI_EVENT_RX_CMPLT     2
#define SPI_EVENT_OVR_ERR      3
#define SPI_EVENT_CRC_ERR      4

//**************************************************************************************
// 								API supported by driver
//****************************************************************************************

//peripheral clock setup

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//init and de-init

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*
 * other peripheral
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/*
 * IRQ configuration and ISR Handling
 */

void SPI_IRQInterrupConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle );


/*
 * application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);





#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
