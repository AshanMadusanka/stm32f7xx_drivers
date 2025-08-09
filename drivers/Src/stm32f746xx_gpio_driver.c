/*
 * stm32f746xx_gpio_driver.c
 *
 *  Created on: May 24, 2025,
 *      Author: Ashan
 *
 *  This file contains the implementation of GPIO driver functions for the STM32F746xx microcontroller.
 */

#include "stm32f746xx_gpio_driver.h"
#include "stm32f746xx.h"

/**
 * @brief  Enables or disables the peripheral clock for the given GPIO port.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @param  EnorDi: ENABLE to enable the clock, DISABLE to disable the clock.
 * @retval None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {

    if(EnorDi == ENABLE){

      if (pGPIOx == GPIOA) {

          GPIOA_PCLK_EN();
      }
      else if(pGPIOx == GPIOB) {

          GPIOB_PCLK_EN();

      }
      else if(pGPIOx == GPIOC) {

          GPIOC_PCLK_EN();
      }
      else if(pGPIOx == GPIOD) {

          GPIOD_PCLK_EN();
      }
      else if(pGPIOx == GPIOE) {

          GPIOE_PCLK_EN();
      }
      else if(pGPIOx == GPIOF) {

          GPIOF_PCLK_EN();
      }

      else if(pGPIOx == GPIOG) {

          GPIOG_PCLK_EN();
      }
      else if(pGPIOx == GPIOH) {

          GPIOH_PCLK_EN();
      }
      else if(pGPIOx == GPIOI) {

          GPIOI_PCLK_EN();
      }

      else if(pGPIOx == GPIOJ) {

          GPIOJ_PCLK_EN();
      }
      else if(pGPIOx == GPIOK) {

          GPIOK_PCLK_EN();
      }
  }
  else {

      if (pGPIOx == GPIOA) {

          GPIOA_PCLK_DI();
      }
      else if(pGPIOx == GPIOB) {

          GPIOB_PCLK_DI();
      }
      else if(pGPIOx == GPIOC) {

          GPIOC_PCLK_DI();
      }
      else if(pGPIOx == GPIOD) {

          GPIOD_PCLK_DI();
      }
      else if(pGPIOx == GPIOE) {

          GPIOE_PCLK_DI();
      }
      else if(pGPIOx == GPIOF) {

          GPIOF_PCLK_DI();
      }

      else if(pGPIOx == GPIOG) {

          GPIOG_PCLK_DI();
      }
      else if(pGPIOx == GPIOH) {

          GPIOH_PCLK_DI();
      }
      else if(pGPIOx == GPIOI) {

          GPIOI_PCLK_DI();
      }

      else if(pGPIOx == GPIOJ) {

          GPIOJ_PCLK_DI();
      }
      else if(pGPIOx == GPIOK) {

          GPIOK_PCLK_DI();
      }
  }
}

/**
 * @brief  Initializes the GPIO pin with the specified configuration.
 * @param  pGPIOHandle: Pointer to the GPIO handle structure containing pin configuration.
 * @retval None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

    GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);
    uint32_t temp = 0;

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG) {

        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the bits

        pGPIOHandle->pGPIOx->MODER |= temp;
    }
    else {

        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

            // Configure FTSR
            EXTI->RTSR &=~ (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {

            // Configure RTSR
            EXTI->FTSR &=~ (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else {

            // Configure Both  FTSR, RTSR
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Unmask the interrupt for the pin

        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; // Determine the EXTI line register (0-3)
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; // Determine the bit position within the register
        SYSCFG_PCLK_EN();
        uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG->EXTICR[temp1] &= ~(0xF << (4 * temp2)); // Clear bits
        SYSCFG->EXTICR[temp1] |= port_code << (4 *temp2);

    }

    temp = 0;

    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the bit
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    temp =pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the bits
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the bits
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT) {
        if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) < 8) {
            temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear the bits
            pGPIOHandle->pGPIOx->AFR[0] |= temp;
            temp = 0;
        }


        else {
            temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8));
            pGPIOHandle->pGPIOx->AFR[1] &= ~(0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8))); // Clear the bits
            pGPIOHandle->pGPIOx->AFR[1] |= temp;
        }
    }
}

/**
 * @brief  Resets the GPIO port to its default state.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @retval None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

    if (pGPIOx == GPIOA) {

        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB) {

        GPIOB_REG_RESET();
    }
    else if(pGPIOx == GPIOC) {

        GPIOC_REG_RESET();
    }
    else if(pGPIOx == GPIOD) {

        GPIOD_REG_RESET();
    }
    else if(pGPIOx == GPIOE) {

        GPIOE_REG_RESET();
    }
    else if(pGPIOx == GPIOF) {

        GPIOF_REG_RESET();
    }
    else if(pGPIOx == GPIOG) {

        GPIOG_REG_RESET();
    }
    else if(pGPIOx == GPIOH) {

        GPIOH_REG_RESET();
    }
    else if(pGPIOx == GPIOI) {

        GPIOI_REG_RESET();
    }
    else if(pGPIOx == GPIOJ) {

        GPIOJ_REG_RESET();
    }
    else if(pGPIOx == GPIOK) {

        GPIOK_REG_RESET();
    }
}

/**
 * @brief  Reads the value of a specific GPIO input pin.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @param  PinNumber: Pin number to read.
 * @retval uint8_t: Value of the pin (0 or 1).
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    uint8_t value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x1);
    return value;
}

/**
 * @brief  Reads the value of the entire GPIO input port.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @retval uint16_t: Value of the input port.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    uint16_t value = pGPIOx->IDR;
    return value;
}

/**
 * @brief  Writes a value to a specific GPIO output pin.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @param  PinNumber: Pin number to write to.
 * @param  Value: Value to write (GPIO_PIN_SET or GPIO_PIN_RESET).
 * @retval None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {

    if(Value == GPIO_PIN_SET) {
        pGPIOx->ODR |= 1 << PinNumber;
    }
    else {

        pGPIOx->ODR &=~(1 << PinNumber);
    }
}

/**
 * @brief  Writes a value to the entire GPIO output port.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @param  Value: Value to write to the port.
 * @retval None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {

    pGPIOx->ODR = Value;

}

/**
 * @brief  Toggles the state of a specific GPIO output pin.
 * @param  pGPIOx: Pointer to the GPIO port base address.
 * @param  PinNumber: Pin number to toggle.
 * @retval None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

    pGPIOx->ODR ^= (1 << PinNumber);

}

/**
 * @brief  Configures the interrupt for a specific GPIO pin.
 * @param  IRQNumber: IRQ number of the GPIO pin.
 * @param  IRQPriority: Priority of the interrupt.
 * @param  EnorDi: ENABLE to enable the interrupt, DISABLE to disable it.
 * @retval None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi) {

    if(EnorDi == ENABLE) {

        if(IRQNumber <=31) {

            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber >= 32 && IRQNumber <64) {

            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber <96) {

            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
    }
    else {

        if(IRQNumber <=31) {

            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if(IRQNumber >= 32 && IRQNumber <64) {

            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if(IRQNumber >= 64 && IRQNumber <96) {

            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }

    

}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {

    uint8_t iprx = IRQNumber / 4 ; // Determine the interrupt priority register (IPRx Register)
    uint8_t iprx_section = IRQNumber % 4 ; // Determine the section within the register
    uint8_t shift_amount = (8 *iprx_section) + (8-NO_PRIORITY_BITS_IMPLEMENTED); // Calculate the shift amount for the priority bits;

    *(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shift_amount; // Set the priority for the interrupt


}

/**
 * @brief  Handles the interrupt for a specific GPIO pin.
 * @param  PinNumber: Pin number that triggered the interrupt.
 * @retval None
 */
void GPIO_IRQHandler(uint8_t PinNumber) {

    if(EXTI->PR & (1 << PinNumber)) { // Check if the interrupt is pending for the pin
        EXTI->PR |= (1 << PinNumber); // Clear the pending interrupt flag
        // Call the user-defined ISR function for the pin
        // This function should be defined by the user to handle the interrupt
        // Example: User_GPIO_IRQHandling(PinNumber);
    }
}

