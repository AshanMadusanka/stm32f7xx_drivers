/*
 * stm32f746xx_usart.h
 *
 *  Created on: Aug 11, 2025
 *      Author: Ashan
 */

 #ifndef STM32F746XX_USART_H
 #define STM32F746XX_USART_H

 #include "stm32f746xx.h"
 #include <stdint.h>



 /* ============================= Driver types ============================= */
 typedef struct {
	 uint32_t USART_Baud;          /* Desired baud rate, e.g., 115200 */
	 uint8_t  USART_Mode;          /* @USART_Mode */
	 uint8_t  USART_WordLength;    /* @USART_WordLen */
	 uint8_t  USART_Parity;        /* @USART_Parity */
	 uint8_t  USART_StopBits;      /* @USART_StopBits */
	 uint8_t  USART_HwFlowCtl;     /* @USART_HWFlow */
	 uint8_t  USART_OverSampling;  /* @USART_OverSampling */

 } USART_Config_t;

 typedef struct {
	 USART_RegDef_t *pUSARTx;
	 USART_Config_t  USART_Config;
 } USART_Handle_t;

 /* ============================= Driver macros ============================= */
 /* States */
 #define USART_READY        0U
 #define USART_BUSY_IN_RX   1U
 #define USART_BUSY_IN_TX   2U

 /* Modes */
 #define USART_MODE_TX      0x00U
 #define USART_MODE_RX      0x01U
 #define USART_MODE_TXRX    0x02U

 /* Word length */
 #define USART_WORDLEN_8BITS 0x00U
 #define USART_WORDLEN_9BITS 0x01U

 /* Parity control */
 #define USART_PARITY_NONE   0x00U
 #define USART_PARITY_EVEN   0x01U
 #define USART_PARITY_ODD    0x02U

 /* Stop bits (encoded as values to be placed in CR2 STOP[13:12]) */
 #define USART_STOPBITS_1    0x00U
 #define USART_STOPBITS_0_5  0x01U
 #define USART_STOPBITS_2    0x02U
 #define USART_STOPBITS_1_5  0x03U

 /* Hardware flow control */
 #define USART_HW_FLOW_NONE  0x00U
 #define USART_HW_FLOW_RTS   0x01U
 #define USART_HW_FLOW_CTS   0x02U
 #define USART_HW_FLOW_RTS_CTS 0x03U

 /* Oversampling */
 #define USART_OVERSAMPLING_16 0x00U
 #define USART_OVERSAMPLING_8  0x01U

 /*USART BAUD RATE*/

 #define USART_BAUDRATE_2400   2400
 #define USART_BAUDRATE_9600   9600
 #define USART_BAUDRATE_115200 115200

 /* Flags */
 #define USART_FLAG_TXE     (1U << USART_ISR_TXE)
 #define USART_FLAG_RXNE    (1U << USART_ISR_RXNE)
 #define USART_FLAG_TC      (1U << USART_ISR_TC)
 #define USART_FLAG_IDLE    (1U << USART_ISR_IDLE)
 #define USART_FLAG_ORE     (1U << USART_ISR_ORE)
 #define USART_FLAG_FE      (1U << USART_ISR_FE)
 #define USART_FLAG_PE      (1U << USART_ISR_PE)

 /* ============================= API prototypes ============================= */
 void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
 void USART_Init(USART_Handle_t *pUSARTHandle);
 void USART_DeInit(USART_RegDef_t *pUSARTx);

 void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
 void USART_SetBaudRate(USART_Handle_t *pUSARTHandle, uint32_t PeriphClkHz, uint32_t BaudRate);

 void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
 void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

 uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
 uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

 uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
 void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);

 void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
 void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
 void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

 __weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t app_event);

 /* Application event codes */
 #define USART_EVENT_TX_CMPLT   0x01U
 #define USART_EVENT_RX_CMPLT   0x02U
 #define USART_EVENT_IDLE       0x03U
 #define USART_EVENT_ERROR      0x04U


 #endif // STM32F746XX_USART_H



#endif /* INC_STM32F746XX_USART_H_ */
