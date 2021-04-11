/*
 * stm32f401xx.h
 *
 *  Created on: Mar 13, 2021
 *      Author:Mithlesh katre
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include<stdint.h>
#include <stddef.h>

#define __vo      volatile
#define __weak   __attribute__((weak))

//***********************START : Processor specific details*******************************
/*
 * ARM cortex  mx processor NVIC ISERx register address
 */

#define NVIC_ISER0         ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1         ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2         ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3         ((__vo uint32_t*)0xE000E10c)


/*
 * ARM cortex  mx processor NVIC IcERx register address
 */

#define NVIC_ICER0         ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1         ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2         ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3         ((__vo uint32_t*)0xE000E18c)

/*
 * ARM cortex  mx processor NVIC Priority register address
 */
#define NVIC_PR_BASE_ADDR    ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED     4

// base addresses of flash,ram,rom

#define FLASH_BASEADDR         0x08000000UL  //512KB
#define SRAM_BASEADDR          0x20000000UL  //96KB
#define ROM                    0x1FFF0000UL  //30KB

//AHBx APBx Bus peripheral base addresses

#define PERIPH_BASE            0x40000000U
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE 	   0x40010000U
#define AHB1PERIPH_BASE        0x40020000U
#define AHB2PERIPH_BASE        0x50000000U


//AHB1 BUS HANGING PERIPHERALS

#define GPIOA_BASEADDR         (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR         (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR         (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR         (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR         (AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR         (AHB1PERIPH_BASE + 0x1C00)

#define RCC_BASEADDR           (AHB1PERIPH_BASE + 0x3800)


//APB1 BUS HANGING PERIPHERALS

#define I2C1_BASEADDR               (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR               (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR             (APB1PERIPH_BASE + 0x4400)


//APB2 BUS HANGING PERIPHERALS

#define SPI1_BASEADDR               (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR               (APB2PERIPH_BASE + 0x3400)

#define USART1_BASEADDR             (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR             (APB2PERIPH_BASE + 0x1400)

#define SYSCFG_BASEADDR             (APB2PERIPH_BASE + 0x3800)

#define EXTI_BASEADDR               (APB2PERIPH_BASE + 0x3C00)


//*****************************PERIPHERAL REGISTER DEFINATION STRUCTURE************************************

//GPIOx REGISTOR

typedef struct{

	__vo uint32_t MODER;    //1. GPIO port mode register                              offset  0x00
	__vo uint32_t OTYPER;   //2. GPIO port output type register
	__vo uint32_t OSPEEDR;	//3. GPIO port output speed register
	__vo uint32_t PUPDR;	//4. GPIO port pull-up/pull-down register
	__vo uint32_t IDR;		//5. GPIO port input data register
	__vo uint32_t ODR;		//6. GPIO port output data register
	__vo uint32_t BSRR;		//7. GPIO port bit set/reset register
	__vo uint32_t LCKR;		//8. GPIO port configuration lock register
	__vo uint32_t AFR[2];	//9.10. GPIO alternate function high AND low register     offset  0x24

} GPIO_RegDef_t;


// PERIPHERAL BASE ADDRESSES TYPECASTED TO RCC_RegDef_t

typedef struct{

	__vo uint32_t CR;    			//1.RCC clock control register
	__vo uint32_t PLLCFGR;  		//2. RCC PLL configuration register
	__vo uint32_t CFGR;				//3. RCC clock configuration register
	__vo uint32_t CIR;				//4. RCC clock interrupt register
	__vo uint32_t AHB1RSTR;			//5. RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;			//6.CC AHB2 peripheral reset register
	 uint32_t  RESERVED0[2];		//7.8.
	__vo uint32_t APB1RSTR;			//9.RCC APB1 peripheral reset register for (RCC_APB1RSTR)
	__vo uint32_t APB2RSTR;         //10. RCC APB2 peripheral reset register (RCC_APB2RSTR)
	uint32_t  RESERVED1[2];			//11.12
	__vo uint32_t AHB1ENR;   		//13. RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
	__vo uint32_t AHB2ENR;			//14. RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
	uint32_t  RESERVED2[2];			//15.16.
	__vo uint32_t APB1ENR;			//17. RCC APB1 peripheral clock enable register (RCC_APB1ENR)
	__vo uint32_t APB2ENR;			//18. RCC APB2 peripheral clock enable register
	uint32_t  RESERVED3[2];			//19.20.
	__vo uint32_t AHB1LPENR;		//21. RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;		//22. RCC AHB2 peripheral clock enable in low power mode register
	uint32_t  RESERVED4[2];			//23.24.
	__vo uint32_t APB1LPENR;		//25.6.3.15 RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;		//26. RCC APB2 peripheral clock enabled in low power mode register
	uint32_t  RESERVED5[2];			//27.28
	__vo uint32_t BDCR;				//296.3.17 RCC Backup domain control register (RCC_BDCR)
	__vo uint32_t CSR;				//30. RCC clock control & status register (RCC_CSR)
	uint32_t  RESERVED6[2];			//31.32
	__vo uint32_t  SSCGR;			//33. RCC spread spectrum clock generation register (RCC_SSCGR)
	__vo uint32_t PLLI2SCFGR;			//34. RCC PLLI2S configuration register (RCC_PLLI2SCFGR)
	__vo uint32_t Reserved;		//35.36
	__vo uint32_t DCKCFGR;			//37  RCC Dedicated Clocks Configuration Register (RCC_DCKCFGR)

} RCC_RegDef_t;


// DEFINEING THE STRUCTURE DEFINATION OF EXTI registers

typedef struct{

	__vo uint32_t IMR;		//nterrupt mask register 0x00
	__vo uint32_t EMR; 	 //10.3.2 Event mask register (EXTI_EMR)
	__vo uint32_t RTSR; 	 //10.3.3 Rising trigger selection register (EXTI_RTSR)
	__vo uint32_t FTSR; 	//10.3.4 Falling trigger selection register (EXTI_FTSR)
	__vo uint32_t SWIER;	//10.3.5 Software interrupt event register (EXTI_SWIER)
	__vo uint32_t PR;		///10.3.6 Pending register (EXTI_PR) 0x14

}EXTI_RegDef_t;


// DEFINEING THE STRUCTURE DEFINATION OF EXTI registers

typedef struct{

	__vo uint32_t MEMRMP;    //7.2.1 SYSCFG memory remap register (SYSCFG_MEMRMP)
	__vo uint32_t PMC;       //7.2.2 SYSCFG peripheral mode configuration register (SYSCFG_PMC)
	__vo uint32_t EXTICR[4];   //7.2.3 SYSCFG external interrupt configuration register 1,2,3,4
	__vo uint32_t CMPCR;     //7.2.7 Compensation cell control register (SYSCFG_CMPCR)

}SYSCFG_RegDef_t;


typedef struct{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;



typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;


}I2C_RegDef_t;



typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USART_RegDef_t;

//PERIPHERAL BASE ADDRESSES TYPECASTED TO GPIO_RegDef_t

#define GPIOA          ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB          ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC          ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD          ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE          ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH          ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC            ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI           ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG         ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1            ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI2            ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3            ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1			 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			 ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)


//********************************ENABLE CLOCK*************************************************

//CLOCK ENABLE MACRO FOR GPIOx PERIPHERAL

#define GPIOA_PCLK_EN()     ((RCC->AHB1ENR) |=(1<<0))
#define GPIOB_PCLK_EN()     ((RCC->AHB1ENR) |=(1<<1))
#define GPIOC_PCLK_EN()     ((RCC->AHB1ENR) |=(1<<2))
#define GPIOD_PCLK_EN()     ((RCC->AHB1ENR) |=(1<<3))
#define GPIOE_PCLK_EN()     ((RCC->AHB1ENR) |=(1<<4))
#define GPIOH_PCLK_EN()     ((RCC->AHB1ENR) |=(1<<7))


//CLOCK ENABLE MACRO FOR I2Cx PERIPHERAL

#define I2C1_PCLK_EN()       ((RCC->APB1ENR) |=(1<<21))
#define I2C2_PCLK_EN()       ((RCC->APB1ENR) |=(1<<22))
#define I2C3_PCLK_EN()       ((RCC->APB1ENR) |=(1<<23))


//CLOCK ENABLE MACRO FOR SPIx PERIPHERAL

#define SPI2_PCLK_EN()       ((RCC->APB1ENR) |=(1<<14))
#define SPI3_PCLK_EN()       ((RCC->APB1ENR) |=(1<<14))

#define SPI1_PCLK_EN()       ((RCC->APB2ENR) |=(1<<12))
#define SPI4_PCLK_EN()       ((RCC->APB2ENR) |=(1<<13))



//CLOCK ENABLE MACRO FOR USARTx PERIPHERAL

#define USART2_PCLK_EN()     ((RCC->APB1ENR) |=(1<<17))

#define USART1_PCLK_EN()     ((RCC->APB2ENR) |=(1<<4))
#define USART6_PCLK_EN()     ((RCC->APB2ENR) |=(1<<5))

//CLOCK ENABLE MACRO FOR USARTx PERIPHERAL

#define SYSCFG_PCLK_EN()     ((RCC->APB2ENR) |=(1<<14))


//CLOCK ENABLE MACRO FOR EXTIx PERIPHERAL

#define EXTI_PCLK_EN()        ((RCC->APB2ENR) |= )

//******************************DISABLE CLOCK****************************************************


//CLOCK DISABLE MACRO FOR GPIOx PERIPHERAL

#define GPIOA_PCLK_DI()     ((RCC->AHB1ENR) &= ~(1<<0))
#define GPIOB_PCLK_DI()     ((RCC->AHB1ENR) &= ~(1<<1))
#define GPIOC_PCLK_DI()     ((RCC->AHB1ENR) &= ~(1<<2))
#define GPIOD_PCLK_DI()     ((RCC->AHB1ENR) &= ~(1<<3))
#define GPIOE_PCLK_DI()     ((RCC->AHB1ENR) &= ~(1<<4))
#define GPIOH_PCLK_DI()     ((RCC->AHB1ENR) &= ~(1<<7))


//CLOCK DISABLE MACRO FOR I2Cx PERIPHERAL

#define I2C1_PCLK_DI()       ((RCC->APB1ENR) &= ~(1<<21))
#define I2C2_PCLK_DI()       ((RCC->APB1ENR) &= ~(1<<22))
#define I2C3_PCLK_DI()       ((RCC->APB1ENR) &= ~(1<<23))


//CLOCK DISABLE MACRO FOR SPIx PERIPHERAL

#define SPI2_PCLK_DI()       ((RCC->APB1ENR) &= ~(1<<14))
#define SPI3_PCLK_DI()       ((RCC->APB1ENR) &= ~(1<<14))

#define SPI1_PCLK_DI()       ((RCC->APB2ENR) &= ~(1<<12))
#define SPI4_PCLK_DI()       ((RCC->APB2ENR) &= ~(1<<13))



//CLOCK DISABLE MACRO FOR USARTx PERIPHERAL

#define USART2_PCLK_DI()     ((RCC->APB1ENR) &= ~(1<<17))

#define USART1_PCLK_DI()     ((RCC->APB2ENR) &= ~(1<<4))
#define USART6_PCLK_DI()     ((RCC->APB2ENR) &= ~(1<<5))

//CLOCK DISABLE MACRO FOR USARTx PERIPHERAL

#define SYSCFG_PCLK_DI()     ((RCC->APB2ENR) &= ~(1<<14))


//CLOCK DISABLE MACRO FOR EXTIx PERIPHERAL

#define EXTI_PCLK_DI()



//****************macros for reset GPIOx peripheral*************************

#define GPIOA_REG_RESET()         do{(RCC->AHB1ENR |= (1<<0)); (RCC->AHB1ENR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()         do{(RCC->AHB1ENR |= (1<<1)); (RCC->AHB1ENR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()         do{(RCC->AHB1ENR |= (1<<2)); (RCC->AHB1ENR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()         do{(RCC->AHB1ENR |= (1<<3)); (RCC->AHB1ENR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()         do{(RCC->AHB1ENR |= (1<<4)); (RCC->AHB1ENR &= ~(1<<4)); }while(0)
#define GPIOH_REG_RESET()         do{(RCC->AHB1ENR |= (1<<7)); (RCC->AHB1ENR &= ~(1<<7)); }while(0)


// @ SYSconfig EXTIRCR macro
#define GPIO_BASEADDR_TO_CODE(x)   ((x==GPIOA)? 0: \
									(x==GPIOB)? 1: \
									(x==GPIOC)? 2: \
									(x==GPIOD)? 3: \
									(x==GPIOE)? 4: \
									(x==GPIOH)? 5:0)


/*
 * interrupt request number of stm32f4XXX
 */
#define IRQ_NO_EXTI0  		  6
#define IRQ_NO_EXTI1   		  7
#define IRQ_NO_EXTI2 		  8
#define IRQ_NO_EXTI3  		  9
#define IRQ_NO_EXTI4  		  10
#define IRQ_NO_EXTI9_5  	  23
#define IRQ_NO_EXTI15_10	  40

/*
 * spi vector table intrupt
 */

#define IRQ_NO_SPI1       35
#define IRQ_NO_SPI2       36
#define IRQ_NO_SPI3       51
#define IRQ_NO_SPI4       84

/*
 * I2C vectror table interrupt
 */
#define IRQ_NO_I2C1_EV       31
#define IRQ_NO_I2C1_ER       32
#define IRQ_NO_I2C2_EV       33
#define IRQ_NO_I2C2_ER       34
#define IRQ_NO_I2C3_EV       79
#define IRQ_NO_I2C4_ER       80


// macros for all the possible priority level
#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI15      15



/*
 * bit position defination SPI_CR1
 */
#define SPI_CR1_CPHA           0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI  			8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY  	   10
#define SPI_CR1_DFF			   11
#define SPI_CR1_CRCNEXT        12
#define SPI_CR1_CRCEN		   13
#define SPI_CR1_BIDIOE         14
#define SPI_CR1_BIDIMODE       15

/*
 * bit position defination SPI_CR2
 */
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_ FRF			4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE  		6
#define SPI_CR2_TXEIE			7


/*
 * bit position defination SPI_SR
 */
#define SPI_SR_RXNE           0
#define SPI_SR_TXE            1
#define SPI_SR_CHSIDE         2
#define SPI_SR_UDR            3
#define SPI_SR_CRCIERR        4
#define SPI_SR_MODF           5
#define SPI_SR_OVR            6
#define SPI_SR_BSY            7
#define SPI_SR_IFRE           8

/*
 * ********************************I2C register bit position******************************************************
 */
//bit position for CR1

#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15


// BIT POSITION FOR CR2
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

// BIT POSITION FOR SR1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15


// BIT POSITION FOR SR2
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8


// BIT POSITION FOR CCR
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_F_S				15



/*****************************************************************************
*************
*Bit position definitions of USART peripheral
*****************************************************************************
*************/
/*
* Bit position definitions USART_CR1
*/
#define USART_CR1_SBK 			0
#define USART_CR1_RWU 			1
#define USART_CR1_RE 			2
#define USART_CR1_TE 			3
#define USART_CR1_IDLEIE	 	4
#define USART_CR1_RXNEIE 		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXEIE 		7
#define USART_CR1_PEIE 			8
#define USART_CR1_PS 			9
#define USART_CR1_PCE 			10
#define USART_CR1_WAKE 			11
#define USART_CR1_M 			12
#define USART_CR1_UE 			13
#define USART_CR1_OVER8 		15
/*
* Bit position definitions USART_CR2
*/
#define USART_CR2_ADD 			0
#define USART_CR2_LBDL 			5
#define USART_CR2_LBDIE			 6
#define USART_CR2_LBCL 			8
#define USART_CR2_CPHA 			9
#define USART_CR2_CPOL 			10
#define USART_CR2_STOP 			12
#define USART_CR2_LINEN 		14
/*
* Bit position definitions USART_CR3
*/#define USART_CR3_EIE 		0
#define USART_CR3_IREN 			1
#define USART_CR3_IRLP 			2
#define USART_CR3_HDSEL 		3
#define USART_CR3_NACK 			4
#define USART_CR3_SCEN 			5
#define USART_CR3_DMAR 			6
#define USART_CR3_DMAT 			7
#define USART_CR3_RTSE 			8
#define USART_CR3_CTSE 			9
#define USART_CR3_CTSIE 		10
#define USART_CR3_ONEBIT 		11
/*
* Bit position definitions USART_SR
*/
#define USART_SR_PE 			0
#define USART_SR_FE 			1
#define USART_SR_NE 			2
#define USART_SR_ORE 			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE 			5
#define USART_SR_TC 			6
#define USART_SR_TXE 			7
#define USART_SR_LBD 			8
#define USART_SR_CTS 			9

//*************************some generic micros********************************************

#define  ENABLE 			1
#define  DISABLE 		    0
#define  SET				ENABLE
#define  RESET 			    DISABLE

#define  GPIO_PIN_SET       SET
#define  GPIO_PIN_RESET     RESET
#define FLAG_RESET          RESET
#define FLAG_SET            SET




#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_usart_driver.h"
#endif /* INC_STM32F401XX_H_ */



