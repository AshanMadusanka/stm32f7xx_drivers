//
// Created by Ashan on 19/05/2025.
//

#ifndef STM32F746XX_H
#define STM32F746XX_H


#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/***************************** Processor Specific Details ***********************/

#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100U) /*!< Interrupt Set Enable Register 0 */
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104U) /*!< Interrupt Set Enable Register 0 */
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108U) /*!< Interrupt Set Enable Register 0 */
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10CU) /*!< Interrupt Set Enable Register 0 */

#define NVIC_ICER0          ((__vo uint32_t*)0XE000E180U) /*!< Interrupt Clear Enable Register 0 */
#define NVIC_ICER1          ((__vo uint32_t*)0XE000E184U) /*!< Interrupt Clear Enable Register 0 */
#define NVIC_ICER2          ((__vo uint32_t*)0XE000E188U) /*!< Interrupt Clear Enable Register 0 */
#define NVIC_ICER3          ((__vo uint32_t*)0XE000E18CU) /*!< Interrupt Clear Enable Register 0 */

#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400U) /*!< Interrupt Priority Register Base Address */


#define NO_PRIORITY_BITS_IMPLEMENTED 4 /*!< Number of priority bits implemented in the NVIC */

/*  Base addresses of FLASH and SRAM */

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20010000U
#define SRAM2_BASEADDR 0x2004C000U
#define ROM_BASEADDR   0x1FF00000U

/* Base addresses of AHBx and APBx bus peripherals */

#define PERIPH_BASEADDR 0x40000000U
#define APB1_BASEADDR  (PERIPH_BASEADDR)
#define APB2_BASEADDR  0x40010000U
#define AHB1_BASEADDR  0x40020000U
#define AHB2_BASEADDR  0x50000000U
#define AHB3_BASEADDR  0xA0000000U

/* Base addresses of peripherals which are hanging on AHB1 Bus */

#define GPIOA_BASEADDR (AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR (AHB1_BASEADDR + 0x2400)
#define GPIOK_BASEADDR (AHB1_BASEADDR + 0x2800)
#define RCC_BASEADDR   (AHB1_BASEADDR + 0x3800)


/* Base addresses of peripherals which are hanging on APB1 Bus */

#define I2C1_BASEADDR (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1_BASEADDR + 0x5C00)
#define I2C4_BASEADDR (APB1_BASEADDR + 0x6000)

#define SPI2_BASEADDR (APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1_BASEADDR + 0x5000)

/* Base addresses of peripherals which are hanging on APB2 Bus */

#define SPI1_BASEADDR (APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR (APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR (APB2_BASEADDR + 0x5000)
#define SPI6_BASEADDR (APB2_BASEADDR + 0x5400)

#define USART1_BASEADDR (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2_BASEADDR + 0x1400)
#define EXTI_BASEADDR  (APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR (APB2_BASEADDR + 0x3800)


/*Return port code for given GPIO address*/

#define GPIO_BASEADDR_TO_CODE(x) \
        ((x == GPIOA) ? 0 : \
         (x == GPIOB) ? 1 : \
         (x == GPIOC) ? 2 : \
         (x == GPIOD) ? 3 : \
         (x == GPIOE) ? 4 : \
         (x == GPIOF) ? 5 : \
         (x == GPIOG) ? 6 : \
         (x == GPIOH) ? 7 : \
         (x == GPIOI) ? 8 : \
         (x == GPIOJ) ? 9 : \
         (x == GPIOK) ? 10 : 0)

/*********************Peripheral register definition structures**************/

typedef struct {

   __vo uint32_t MODER;    /*!< GPIO port mode register, Address offset: 0x00 */
   __vo uint32_t OTYPER;   /*!< GPIO port output type register, Address offset: 0x04 */
   __vo uint32_t OSPEEDR; /*!< GPIO port output speed register, Address offset: 0x08 */
   __vo uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register, Address offset: 0x0C */
   __vo uint32_t IDR;     /*!< GPIO port input data register, Address offset: 0x10 */
   __vo uint32_t ODR;     /*!< GPIO port output data register, Address offset: 0x14 */
   __vo uint32_t BSRR;    /*!< GPIO port bit set/reset register, Address offset: 0x18 */
   __vo uint32_t LCKR;    /*!< GPIO port configuration lock register, Address offset: 0x1C */
   __vo uint32_t AFR[2];  /*!< GPIO alternate function low/high registers, Address offset: 0x20-0x24 */

}GPIO_RegDef_t;

typedef struct {

   __vo uint32_t CR1;       /*!< I2C Control register 1, Address offset: 0x00 */
   __vo uint32_t CR2;       /*!< I2C Control register 2, Address offset: 0x04 */
   __vo uint32_t OAR1;      /*!< I2C Own address register 1, Address offset: 0x08 */
   __vo uint32_t OAR2;      /*!< I2C Own address register 2, Address offset: 0x0C */
   __vo uint32_t TIMINGR;    /*!< I2C Timing register, Address offset: 0x10 */
   __vo uint32_t TIMEOUTR;   /*!< I2C Timeout register, Address offset: 0x14 */
   __vo uint32_t ISR;       /*!< I2C Interrupt and status register, Address offset: 0x18 */
   __vo uint32_t ICR;       /*!< I2C Interrupt clear register, Address offset: 0x1C */
   __vo uint32_t PECR;      /*!< I2C PEC register, Address offset: 0x20 */
   __vo uint32_t RXDR;      /*!< I2C Receive data register, Address offset: 0x24 */
   __vo uint32_t TXDR;      /*!< I2C Transmit data register, Address offset: 0x28 */

}I2C_RegDef_t;

typedef struct {
    __vo uint32_t CR1;        /*!< SPI Control register 1, Address offset: 0x00 */
    __vo uint32_t CR2;       /*!< SPI Control register 2, Address offset: 0x04 */
    __vo uint32_t SR;        /*!< SPI Status register, Address offset: 0x08 */
    __vo uint32_t DR;        /*!< SPI Data register, Address offset: 0x0C */
    __vo uint32_t CRCPR;     /*!< SPI CRC polynomial register, Address offset: 0x10 */
    __vo uint32_t RXCRCR;    /*!< SPI RX CRC register, Address offset: 0x14 */
    __vo uint32_t TXCRCR;    /*!< SPI TX CRC register, Address offset: 0x18 */
    __vo uint32_t I2SCFGR;   /*!< SPI I2S configuration register, Address offset: 0x1C */
    __vo uint32_t I2SPR;     /*!< SPI I2S prescaler register, Address offset: 0x20 */

}SPI_RegDef_t;

typedef struct {

   __vo uint32_t CR1;       /*!< USART Control register 1, Address offset: 0x00 */
   __vo uint32_t CR2;       /*!< USART Control register 2, Address offset: 0x04 */
   __vo uint32_t CR3;       /*!< USART Control register 3, Address offset: 0x08 */
   __vo uint32_t BRR;       /*!< USART Baud rate register, Address offset: 0x0C */
   __vo uint32_t GTPR;      /*!< USART Guard time and prescaler register, Address offset: 0x10 */
   __vo uint32_t RTOR;      /*!< USART Receiver timeout register, Address offset: 0x14 */
   __vo uint32_t RQR;       /*!< USART Request register, Address offset: 0x18 */
   __vo uint32_t ISR;       /*!< USART Interrupt and status register, Address offset: 0x1C */
   __vo uint32_t ICR;       /*!< USART Interrupt flag clear register, Address offset: 0x20 */
   __vo uint32_t RDR;       /*!< USART Receive data register, Address offset: 0x24 */
   __vo uint32_t TDR;       /*!< USART Transmit data register, Address offset: 0x28 */

}USART_RegDef_t;

typedef struct {
  __vo uint32_t CR;        /*!< RCC clock control register, Address offset: 0x00 */
  __vo uint32_t PLLCFGR;   /*!< RCC PLL configuration register, Address offset: 0x04 */
  __vo uint32_t CFGR;      /*!< RCC clock configuration register, Address offset: 0x08 */
  __vo uint32_t CIR;       /*!< RCC clock interrupt register, Address offset: 0x0C */
  __vo uint32_t AHB1RSTR; /*!< RCC AHB1 peripheral reset register, Address offset: 0x10 */
  __vo uint32_t AHB2RSTR; /*!< RCC AHB2 peripheral reset register, Address offset: 0x14 */
  __vo uint32_t AHB3RSTR; /*!< RCC AHB3 peripheral reset register, Address offset: 0x18 */
 uint32_t RESERVED0; /*!< Reserved, Address offset: 0x1C */
  __vo uint32_t APB1RSTR; /*!< RCC APB1 peripheral reset register, Address offset: 0x20 */
  __vo uint32_t APB2RSTR; /*!< RCC APB2 peripheral reset register, Address offset: 0x24 */
   uint32_t RESERVED1[2]; /*!< Reserved, Address offset: 0x28-0x2C */
  __vo uint32_t AHB1ENR ;  /*!< RCC AHB1 peripheral clock enable register, Address offset: 0x30 */
  __vo uint32_t AHB2ENR;  /*!< RCC AHB2 peripheral clock enable register, Address offset: 0x34 */
  __vo uint32_t AHB3ENR;  /*!< RCC AHB3 peripheral clock enable register, Address offset: 0x38 */
   uint32_t RESERVED2; /*!< Reserved, Address offset: 0x3C */
  __vo uint32_t APB1ENR;  /*!< RCC APB1 peripheral clock enable register, Address offset: 0x40 */
  __vo uint32_t APB2ENR;  /*!< RCC APB2 peripheral clock enable register, Address offset: 0x44 */
   uint32_t RESERVED3[2]; /*!< Reserved, Address offset: 0x48-0x4C */
  __vo uint32_t AHB1LPENR; /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __vo uint32_t AHB2LPENR; /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __vo uint32_t AHB3LPENR; /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
   uint32_t RESERVED4; /*!< Reserved, Address offset: 0x5C */
  __vo uint32_t APB1LPENR; /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __vo uint32_t APB2LPENR; /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
   uint32_t RESERVED5[2]; /*!< Reserved, Address offset: 0x68-0x6C */
  __vo uint32_t BDCR;     /*!< RCC Backup domain control register, Address offset: 0x70 */
  __vo uint32_t CSR;      /*!< RCC clock control & status register, Address offset: 0x74 */
   uint32_t RESERVED6[2]; /*!< Reserved, Address offset: 0x78-0x7C */
  __vo uint32_t SSCGR;    /*!< RCC spread spectrum clock generation register, Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR; /*!< RCC PLLI2S configuration register, Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR; /*!< RCC PLLSAI configuration register, Address offset: 0x88 */
  __vo uint32_t DCKCFGR;  /*!< RCC Dedicated Clocks Configuration Register, Address offset: 0x8C */
  __vo uint32_t DCKCFGR2; /*!< RCC Dedicated Clocks Configuration Register 2, Address offset: 0x90 */

}RCC_RegDef_t;

typedef struct {

 uint32_t IMR; /*!< EXTI Interrupt mask register, Address offset: 0x00 */
 uint32_t EMR; /*!< EXTI Event mask register, Address offset: 0x04 */
 uint32_t RTSR; /*!< EXTI Rising trigger selection register, Address offset: 0x08 */
 uint32_t FTSR; /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
 uint32_t SWIER; /*!< EXTI Software interrupt event register, Address offset: 0x10 */
 uint32_t PR; /*!< EXTI Pending register, Address offset: 0x14 */

}EXTI_RegDef_t;

typedef struct {

 uint32_t MEMRMP;      /*!< SYSCFG memory remap register, Address offset: 0x00 */
 uint32_t PMC;         /*!< SYSCFG peripheral mode configuration register, Address offset: 0x04 */
 uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
 uint32_t RESVED[2]; /*!< Reserved, Address offset: 0x18-0x1C */
 uint32_t CMPCR;       /*!< SYSCFG compensation cell control register, Address offset: 0x20 */

}SYSCFG_RegDef_t;

/*Peripheral definitions (Peripheral base addresses type cast to xxx Regdef_t) */

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ ((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK ((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5 ((SPI_RegDef_t *)SPI5_BASEADDR)
#define SPI6 ((SPI_RegDef_t *)SPI6_BASEADDR)

#define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)
#define I2C4 ((I2C_RegDef_t *)I2C4_BASEADDR)

/*Clock Enable Macros for GPIOx Peripherals*/

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*Clock Enable Macros for GPIOx Peripherals*/

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |= (1 << 10))

/*Clock Enable Macros for I2Cx Peripherals*/

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))
#define I2C4_PCLK_EN() (RCC->APB1ENR |= (1 << 24))

/*Clock Enable Macros for SPIx Peripherals*/

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN() (RCC->APB2ENR |= (1 << 21))

/*Clock Enable Macros for USARTx Peripherals*/

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/*Clock Disable Macros for GPIOx Peripherals*/

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 10))

/*Clock Disable Macros for I2Cx Peripherals*/

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))
#define I2C4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 24))

/*Clock Disable Macros for SPIx Peripherals*/

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI5_PCLK_DI() (RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 21))

/*Clock Disable Macros for USARTx Peripherals*/

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

/*Macros to reset GPIOx Peripherals*/

#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &=~ (1 << 0));} while (0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &=~ (1 << 1));} while (0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &=~ (1 << 2));} while (0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &=~ (1 << 3));} while (0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &=~ (1 << 4));} while (0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &=~ (1 << 5));} while (0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &=~ (1 << 6));} while (0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &=~ (1 << 7));} while (0)
#define GPIOI_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &=~ (1 << 8));} while (0)
#define GPIOJ_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &=~ (1 << 9));} while (0)
#define GPIOK_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &=~ (1 << 10));} while (0)


/*IRQ(Interupt Request) Numbers of STM32F746xx MCU*/
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5   23
#define IRQ_NO_EXTI15_10 40
#define IRQ_NO_SPI1      35
#define IRQ_NO_SPI2      36
#define IRQ_NO_SPI3      51
#define IRQ_NO_SPI4      84
#define IRQ_NO_SPI5      85
#define IRQ_NO_SPI6      86

/*Generic Macros*/

#define ENABLE        1
#define DISABLE       0
#define SET           ENABLE
#define RESET         DISABLE
#define GPIO_PIN_SET  SET
#define GPIO_PIN_RESET RESET
#define FLAGSET        SET
#define FLAGRESET      RESET


/** A bit of position definition of SPI_CR1 Register **/
#define SPI_CR1_CPHA        0U
#define SPI_CR1_CPOL        1U
#define SPI_CR1_MSTR        2U
#define SPI_CR1_BR          3U
#define SPI_CR1_SPE         6U
#define SPI_CR1_LSBFIRST    7U
#define SPI_CR1_SSI         8U
#define SPI_CR1_SSM         9U
#define SPI_CR1_RXONLY      10U
#define SPI_CR1_CRCL        11U
#define SPI_CR1_CRCNEXT     12U
#define SPI_CR1_CRCEN       13U
#define SPI_CR1_BIDIOE      14U
#define SPI_CR1_BIDIMODE    15U



/** A bit of position definition of SPI_CR2 Register **/

#define SPI_CR2_RXDMAEN     0U
#define SPI_CR2_TXDMAEN     1U
#define SPI_CR2_SSOE        2U
#define SPI_CR2_NSSP        3U
#define SPI_CR2_FRF         4U
#define SPI_CR2_ERRIE       5U
#define SPI_CR2_RXNEIE      6U
#define SPI_CR2_TXEIE       7U
#define SPI_CR2_DS          8U
#define SPI_CR2_FRXTH       12U
#define SPI_CR2_LDMARX      13U
#define SPI_CR2_LDMATX      14U

/** A bit of position definition of SPI_SR Register **/

#define SPI_SR_RXNE         0U
#define SPI_SR_TXE          1U
#define SPI_SR_CHSIDE       2U
#define SPI_SR_UDR          3U
#define SPI_SR_CRCERR       4U
#define SPI_SR_MODF         5U
#define SPI_SR_OVR          6U
#define SPI_SR_BSY          7U
#define SPI_SR_FRE          8U
#define SPI_SR_FRLVL        9U
#define SPI_SR_FTLVL        11U

/** A bit of position definition of I2C CR1 Register **/

#define I2C_CR1_PE          0U
#define I2C_CR1_TXIE        1U
#define I2C_CR1_RXIE        2U
#define I2C_CR1_ADDRIE      3U
#define I2C_CR1_NACKIE      4U
#define I2C_CR1_STOPIE      5U
#define I2C_CR1_TCIE        6U
#define I2C_CR1_ERRIE       7U
#define I2C_CR1_DNF         8U
#define I2C_CR1_ANFOFF      12U
#define I2C_CR1_TXDMAEN     14U
#define I2C_CR1_RXDMAEN     15U
#define I2C_CR1_SBC         16U
#define I2C_CR1_NOSTRETCH   17U
#define I2C_CR1_GCEN        19U
#define I2C_CR1_SMBHEN     20U
#define I2C_CR1_SMBDEN     21U
#define I2C_CR1_ALERT      22U
#define I2C_CR1_PECEN      23U

/** A bit of position definition of I2C CR2 Register **/

#define I2C_CR2_SADD        0U
#define I2C_CR2_RD_WRN      10U
#define I2C_CR2_ADD10       11U
#define I2C_CR2_HEAD10R     12U
#define I2C_CR2_START       13U
#define I2C_CR2_STOP        14U
#define I2C_CR2_NACK        15U
#define I2C_CR2_NBYTES      16U
#define I2C_CR2_RELOAD      24U
#define I2C_CR2_AUTOEND     25U
#define I2C_CR2_PECBYTE     26U

/** A bit of position definition of I2C ISR Register **/

#define I2C_ISR_TXE         0U
#define I2C_ISR_TXIS        1U
#define I2C_ISR_RXNE        2U
#define I2C_ISR_ADDR        3U
#define I2C_ISR_NACKF       4U
#define I2C_ISR_STOPF       5U
#define I2C_ISR_TC          6U
#define I2C_ISR_TCR         7U
#define I2C_ISR_BERR        8U
#define I2C_ISR_ARLO        9U
#define I2C_ISR_OVR         10U
#define I2C_ISR_PECERR      11U
#define I2C_ISR_TIMEOUT     12U
#define I2C_ISR_ALERT       13U
#define I2C_ISR_BUSY        15U
#define I2C_ISR_DIR         16U
#define I2C_ISR_ADDCODE     17U

/** A bit of position definition of I2C TIMINGR Register **/

#define I2C_TIMINGR_SCLL    0U
#define I2C_TIMINGR_SCLH    8U
#define I2C_TIMINGR_SDADEL  16U
#define I2C_TIMINGR_SCLDEL  20U
#define I2C_TIMINGR_PRESC   28U

#include "stm32f746xx_gpio_driver.h"
#include "stm32f746xx_i2c_driver.h"
#include "stm32f746xx_spi_driver.h"
#endif //STM32F746XX_H
