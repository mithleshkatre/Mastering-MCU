/*
 * stm32f401xx.h
 *
 *  Created on: Mar 13, 2021
 *      Author:Mithlesh katre
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include<stdint.h>

#define __vo    volatile
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


//PERIPHERAL BASE ADDRESSES TYPECASTED TO GPIO_RegDef_t

#define GPIOA          ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB          ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC          ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD          ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE          ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH          ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC            ((RCC_RegDef_t*)RCC_BASEADDR  )

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

#define EXTI_PCLK_EN()

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

//*************************some generic micros********************************************

#define  ENABLE 			1
#define  DISABLE 		    0
#define  SET				ENABLE
#define  RESET 			    DISABLE

#define  GPIO_PIN_SET       SET
#define  GPIO_PIN_RESET     RESET



#include "stm32f401xx_gpio_driver.h"

#endif /* INC_STM32F401XX_H_ */
