//
// Created by ashan on 27/06/2025.
//

#ifndef STM32F746XX_I2C_DRIVER_H
#define STM32F746XX_I2C_DRIVER_H

#include "stm32f746xx.h"
#include <stdint.h>

/**
 * @brief I2C Device Mode
 *
 * @note These macros define the possible device modes for the I2C peripheral.
 */
typedef struct {

    uint32_t I2C_SCLSpeed;        /*!< Specifies the I2C SCL speed. Possible values from @ref I2C_SCL_Speed */
    uint8_t I2C_DeviceAddress;    /*!< Specifies the I2C device address */
    uint8_t I2C_ACKControl;       /*!< Specifies whether to enable or disable ACK. Possible values from @ref I2C_ACK_Control */
    uint16_t I2C_FMDutyCycle;    /*!< Specifies the I2C Fast Mode Duty Cycle. Possible values from @ref I2C_FM_Duty_Cycle */

}I2C_Config_t;

typedef struct {

    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxRxState; /*!< Possible values from @ref I2C_State */
    uint8_t DevAddr; /*!< Device address */
    uint32_t RxSize; /*!< Size of the data to be received */
    uint8_t Sr; /*!< Repeated Start condition */

}I2C_Handle_t;


#define I2C_READY       0
#define I2C_BUSY_IN_RX  1
#define I2C_BUSY_IN_TX  2

/**
 * @brief I2C SCL Speed
 *
 * @note These macros define the possible SCL speeds for the I2C peripheral.
 */
#define I2C_SCL_SPEED_SM    100000U /*!< Standard Mode speed */
#define I2C_SCL_SPEED_FM4K  400000U /*!< Fast Mode speed */
#define I2C_SCL_SPEED_FM2K  200000U /*!< Fast Mode speed */

/**
 * @brief I2C ACK Control
 *
 * @note These macros define the possible ACK control configurations for the I2C peripheral.
 */

#define I2C_ACK_ENABLE 1 /*!< Enable ACK */
#define I2C_ACK_DISABLE 0 /*!< Disable ACK */

/**
 * @brief I2C Fast Mode Duty Cycle
 *
 * @note These macros define the possible Fast Mode Duty Cycle configurations for the I2C peripheral.
 */
#define I2C_FM_DUTY_2       0 /*!< Fast Mode Duty Cycle 2 */
#define I2C_FM_DUTY_16_9    1 /*!< Fast Mode Duty Cycle 16/9 */

/** I2C Flag Names */
#define I2C_FLAG_TXE    (1 << I2C_ISR_TXE) /*!< Transmit Data Register Empty */
#define I2C_FLAG_TXIS   (1 << I2C_ISR_TXIS) /*!< Transmit Interrupt Status */
#define I2C_FLAG_RXNE   (1 << I2C_ISR_RXNE) /*!< Receive Data Register Not Empty */
#define I2C_FLAG_TC     (1 << I2C_ISR_TC)   /*!< Transfer Complete Flag */
#define I2C_FLAG_OVR    (1 << I2C_ISR_OVR)  /*!< Overrun/Underrun Flag */
#define I2C_FLAG_NACKF  (1 << I2C_ISR_NACKF) /*!< NACK Received Flag */
#define I2C_FLAG_ARLO   (1 << I2C_ISR_ARLO)  /*!< Address Matched Flag */
#define I2C_FLAG_BERR   (1 << I2C_ISR_BERR)  /*!< Bus Error Flag */
#define I2C_FLAG_STOPF  (1 << I2C_ISR_STOPF) /*!< Stop Detection Flag */
#define I2C_FLAG_TIMEOUT (1 << I2C_ISR_TIMEOUT) /*!< Timeout Flag */

/** I2C Status Register Flags */
#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); 
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop);
void I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop);
void I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif //STM32F746XX_I2C_DRIVER_H
