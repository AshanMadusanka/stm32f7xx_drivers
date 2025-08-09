/*
 * stm32f746xx_gpio_driver.h
 *
 *  Created on: May 24, 2025,
 *      Author: Ashan
 */

#ifndef INC_STM32F746XX_GPIO_DRIVER_H_
#define INC_STM32F746XX_GPIO_DRIVER_H_

#include "stm32f746xx.h"





/* GPIO Pin Configuration structure */

typedef struct {

    uint8_t GPIO_PinNumber;        /*!< Specifies the GPIO pin to be configured. This parameter can be a value of @ref GPIO_PIN_NUMBERS */
    uint8_t GPIO_PinMode;          /*!< Specifies the operating mode for the selected pin. This parameter can be a value of @ref GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;         /*!< Specifies the speed for the selected pin. This parameter can be a value of @ref GPIO_PIN_SPEEDS */
    uint8_t GPIO_PinPuPdControl;   /*!< Specifies the Pull-up or Pull-down activation for the selected pin. This parameter can be a value of @ref GPIO_PIN_PUPD */
    uint8_t GPIO_PinOPType;        /*!< Specifies the operating type for the selected pin. This parameter can be a value of @ref GPIO_PIN_OP_TYPES */
    uint8_t GPIO_PinAltFunMode;    /*!< Specifies the alternate function mode for the selected pin. This parameter can be a value of @ref GPIO_PIN_ALTFUN_MODE */

}GPIO_PinConfig_t;

/*This is a Handle structure for GPIO Pin*/

typedef struct {

    GPIO_RegDef_t *pGPIOx;      /*!< This holds the base address of the GPIO port to which the pin belongs */
    GPIO_PinConfig_t GPIO_PinConfig; /*!< This holds GPIO pin configuration settings */

}GPIO_Handle_t;

#define GPIO_MODE_IN      0x00U /*!< Input mode */
#define GPIO_MODE_OUT     0x01U /*!< Output mode */
#define GPIO_MODE_ALT     0x02U /*!< Alternate function mode */
#define GPIO_MODE_ANALOG  0x03U /*!< Analog mode */
#define GPIO_MODE_IT_FT   0x04U /*!< Interrupt mode falling edge */
#define GPIO_MODE_IT_RT   0x05U /*!< Interrupt mode rising edge */
#define GPIO_MODE_IT_RFT  0x06U /*!< Interrupt mode rising and falling edge */

/*GPIO pin possible output types*/

#define GPIO_OP_TYPE_PP   0x00U /*!< Push-pull output type */
#define GPIO_OP_TYPE_OD   0x01U /*!< Open-drain output type */

/*GPIO pin possible speed types*/

#define GPIO_SPEED_LOW    0x00U /*!< Low speed */
#define GPIO_SPEED_MEDIUM 0x01U /*!< Medium speed */
#define GPIO_SPEED_FAST   0x02U /*!< Fast speed */
#define GPIO_SPEED_HIGH   0x03U /*!< High speed */

/*GPIO pin possible pull-up/pull-down configurations*/

#define GPIO_NO_PUPD      0x00U /*!< No pull-up/pull-down */
#define GPIO_PIN_PU       0x01U /*!< Pull-up */
#define GPIO_PIN_PD       0x02U /*!< Pull-down */

/*GPIO pins*/

#define GPIO_PIN_0        0x00U /*!< Pin 0 */
#define GPIO_PIN_1        0x01U /*!< Pin 1 */
#define GPIO_PIN_2        0x02U /*!< Pin 2 */
#define GPIO_PIN_3        0x03U /*!< Pin 3 */
#define GPIO_PIN_4        0x04U /*!< Pin 4 */
#define GPIO_PIN_5        0x05U /*!< Pin 5 */
#define GPIO_PIN_6        0x06U /*!< Pin 6 */
#define GPIO_PIN_7        0x07U /*!< Pin 7 */
#define GPIO_PIN_8        0x08U /*!< Pin 8 */
#define GPIO_PIN_9        0x09U /*!< Pin 9 */
#define GPIO_PIN_10       0x0AU /*!< Pin 10 */
#define GPIO_PIN_11       0x0BU /*!< Pin 11 */
#define GPIO_PIN_12       0x0CU /*!< Pin 12 */
#define GPIO_PIN_13       0x0DU /*!< Pin 13 */
#define GPIO_PIN_14       0x0EU /*!< Pin 14 */
#define GPIO_PIN_15       0x0FU /*!< Pin 15 */



/*Peripheral Clock setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*Init and De Init*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data Read and Write*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ Configuration and ISR Handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);

#endif /* INC_STM32F746XX_GPIO_DRIVER_H_ */
