/*
 * stm32f746xx_usart.c
 *
 *  Created on: Aug 11, 2025
 *      Author: Ashan
 */

#include "stm32f746xx_usart.h"

/* ============================= Static helpers ============================= */
static inline void usart_write_brr(USART_RegDef_t *pUSARTx, uint32_t periphClkHz, uint32_t baud, uint8_t overs8)
{
	if (overs8 == USART_OVERSAMPLING_8) {
		// BRR = (2 * PCLK) / baud when OVER8=1 (integer)
		uint32_t div = (periphClkHz + (baud / 2U)) / baud; // rounded
		pUSARTx->BRR = div;
	} else {
		// OVER8=0: BRR = PCLK / baud (integer)
		uint32_t div = (periphClkHz + (baud / 2U)) / baud; // rounded
		pUSARTx->BRR = div;
	}
}

/* ============================= Clock control ============================= */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5) {
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}

/* ============================= Init/Deinit ============================= */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Enable peripheral clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Disable USART before configuration
	pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_UE);

	// Word length
	if (pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS) {
		pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_M);
	} else {
		pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_M);
	}

	// Parity
	if (pUSARTHandle->USART_Config.Parity == USART_PARITY_NONE) {
		pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_PCE);
	} else {
		pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_PCE);
		if (pUSARTHandle->USART_Config.Parity == USART_PARITY_ODD) {
			pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_PS);
		} else {
			pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_PS);
		}
	}

	// Stop bits
	pUSARTHandle->pUSARTx->CR2 &= ~(0x3U << USART_CR2_STOP);
	pUSARTHandle->pUSARTx->CR2 |= ((pUSARTHandle->USART_Config.StopBits & 0x3U) << USART_CR2_STOP);

	// Hardware flow control
	pUSARTHandle->pUSARTx->CR3 &= ~((1U << USART_CR3_RTSE) | (1U << USART_CR3_CTSE));
	if (pUSARTHandle->USART_Config.HwFlowCtl == USART_HW_FLOW_RTS) {
		pUSARTHandle->pUSARTx->CR3 |= (1U << USART_CR3_RTSE);
	} else if (pUSARTHandle->USART_Config.HwFlowCtl == USART_HW_FLOW_CTS) {
		pUSARTHandle->pUSARTx->CR3 |= (1U << USART_CR3_CTSE);
	} else if (pUSARTHandle->USART_Config.HwFlowCtl == USART_HW_FLOW_RTS_CTS) {
		pUSARTHandle->pUSARTx->CR3 |= (1U << USART_CR3_RTSE) | (1U << USART_CR3_CTSE);
	}

	// Mode
	pUSARTHandle->pUSARTx->CR1 &= ~((1U << USART_CR1_TE) | (1U << USART_CR1_RE));
	if (pUSARTHandle->USART_Config.Mode == USART_MODE_TX) {
		pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_TE);
	}
	else if (pUSARTHandle->USART_Config.Mode == USART_MODE_RX)
    {
        pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_RE);
    }
    else if (pUSARTHandle->USART_Config.Mode == USART_MODE_TXRX) {
        pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_RE) | (1U << USART_CR1_TE);
	}

	// Baud rate
	usart_write_brr(pUSARTHandle->pUSARTx,
					pUSARTHandle->USART_Config.PeriphClk,
					pUSARTHandle->USART_Config.Baud,
					pUSARTHandle->USART_Config.OverSampling);

	// Enable USART
	pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_UE);

	// Reset state
	pUSARTHandle->TxRxState = USART_READY;
}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	// Disable peripheral and clear key registers (simple deinit)
	pUSARTx->CR1 = 0x0U;
	pUSARTx->CR2 = 0x0U;
	pUSARTx->CR3 = 0x0U;
	pUSARTx->BRR = 0x0U;
	// Leave clocks intact; user may disable via PeriClockControl
}

/* ============================= Peripheral control ============================= */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		pUSARTx->CR1 |= (1U << USART_CR1_UE);
	} else {
		pUSARTx->CR1 &= ~(1U << USART_CR1_UE);
	}
}

void USART_SetBaudRate(USART_Handle_t *pUSARTHandle, uint32_t PeriphClkHz, uint32_t BaudRate)
{
	usart_write_brr(pUSARTHandle->pUSARTx, PeriphClkHz, BaudRate, pUSARTHandle->USART_Config.OverSampling);
}

/* ============================= Blocking APIs ============================= */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	for (uint32_t i = 0; i < Len; i++) {
		while (!(pUSARTHandle->pUSARTx->ISR & (1U << USART_ISR_TXE))) {
			// wait for TXE
		}
		pUSARTHandle->pUSARTx->TDR = pTxBuffer[i];
	}
	// Wait for TC
	while (!(pUSARTHandle->pUSARTx->ISR & (1U << USART_ISR_TC))) {
	}
	// Clear TC
	pUSARTHandle->pUSARTx->ICR |= (1U << USART_ICR_TCCF);
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for (uint32_t i = 0; i < Len; i++) {
		while (!(pUSARTHandle->pUSARTx->ISR & (1U << USART_ISR_RXNE))) {
			// wait for RXNE
		}
		pRxBuffer[i] = (uint8_t)pUSARTHandle->pUSARTx->RDR;
	}
}

/* ============================= Interrupt APIs ============================= */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	if (pUSARTHandle->TxRxState != USART_READY) return pUSARTHandle->TxRxState;

	pUSARTHandle->pTxBuffer = pTxBuffer;
	pUSARTHandle->TxLen = Len;
	pUSARTHandle->TxRxState = USART_BUSY_IN_TX;

	// Enable TXEIE and TCIE
	pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_TXEIE) | (1U << USART_CR1_TCIE);
	return USART_READY;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	if (pUSARTHandle->TxRxState != USART_READY) return pUSARTHandle->TxRxState;

	pUSARTHandle->pRxBuffer = pRxBuffer;
	pUSARTHandle->RxLen = Len;
	pUSARTHandle->TxRxState = USART_BUSY_IN_RX;

	// Enable RXNEIE and IDLEIE (optional)
	pUSARTHandle->pUSARTx->CR1 |= (1U << USART_CR1_RXNEIE) | (1U << USART_CR1_IDLEIE);
	return USART_READY;
}

/* ============================= Flag APIs ============================= */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	return (pUSARTx->ISR & FlagName) ? SET : RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	// Map known ISR flags to ICR clears where applicable
	if (FlagName & (1U << USART_ISR_TC)) pUSARTx->ICR |= (1U << USART_ICR_TCCF);
	if (FlagName & (1U << USART_ISR_IDLE)) pUSARTx->ICR |= (1U << USART_ICR_IDLECF);
	if (FlagName & (1U << USART_ISR_ORE)) pUSARTx->ICR |= (1U << USART_ICR_ORECF);
	if (FlagName & (1U << USART_ISR_FE)) pUSARTx->ICR |= (1U << USART_ICR_FECF);
	if (FlagName & (1U << USART_ISR_PE)) pUSARTx->ICR |= (1U << USART_ICR_PECF);
}

/* ============================= NVIC helpers ============================= */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1U << IRQNumber);
		} else if (IRQNumber <= 63) {
			*NVIC_ISER1 |= (1U << (IRQNumber % 32));
		} else if (IRQNumber <= 95) {
			*NVIC_ISER2 |= (1U << (IRQNumber % 64));
		} else {
			*NVIC_ISER3 |= (1U << (IRQNumber % 96));
		}
	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1U << IRQNumber);
		} else if (IRQNumber <= 63) {
			*NVIC_ICER1 |= (1U << (IRQNumber % 32));
		} else if (IRQNumber <= 95) {
			*NVIC_ICER2 |= (1U << (IRQNumber % 64));
		} else {
			*NVIC_ICER3 |= (1U << (IRQNumber % 96));
		}
	}
}

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4U;
	uint8_t iprx_section = IRQNumber % 4U;
	uint8_t shift_amount = (8U * iprx_section) + (8U - NO_PRIORITY_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/* ============================= IRQ handler ============================= */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t isr = pUSARTHandle->pUSARTx->ISR;
	uint32_t cr1 = pUSARTHandle->pUSARTx->CR1;

	// RXNE
	if ((isr & (1U << USART_ISR_RXNE)) && (cr1 & (1U << USART_CR1_RXNEIE))) {
		if (pUSARTHandle->TxRxState == USART_BUSY_IN_RX && pUSARTHandle->RxLen > 0) {
			*(pUSARTHandle->pRxBuffer++) = (uint8_t)pUSARTHandle->pUSARTx->RDR;
			pUSARTHandle->RxLen--;
			if (pUSARTHandle->RxLen == 0) {
				// Disable RX interrupt and notify
				pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_RXNEIE);
				pUSARTHandle->TxRxState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		} else {
			(void)pUSARTHandle->pUSARTx->RDR; // Drain
		}
	}

	// TXE
	if ((isr & (1U << USART_ISR_TXE)) && (cr1 & (1U << USART_CR1_TXEIE))) {
		if (pUSARTHandle->TxRxState == USART_BUSY_IN_TX) {
			if (pUSARTHandle->TxLen > 0) {
				pUSARTHandle->pUSARTx->TDR = *(pUSARTHandle->pTxBuffer++);
				pUSARTHandle->TxLen--;
			}
			if (pUSARTHandle->TxLen == 0) {
				// Disable TXEIE, wait for TC
				pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_TXEIE);
			}
		}
	}

	// TC
	if ((isr & (1U << USART_ISR_TC)) && (cr1 & (1U << USART_CR1_TCIE))) {
		// Clear TC
		pUSARTHandle->pUSARTx->ICR |= (1U << USART_ICR_TCCF);
		if (pUSARTHandle->TxRxState == USART_BUSY_IN_TX && pUSARTHandle->TxLen == 0) {
			// Disable TCIE and notify
			pUSARTHandle->pUSARTx->CR1 &= ~(1U << USART_CR1_TCIE);
			pUSARTHandle->TxRxState = USART_READY;
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
		}
	}

	// IDLE detection
	if ((isr & (1U << USART_ISR_IDLE)) && (cr1 & (1U << USART_CR1_IDLEIE))) {
		pUSARTHandle->pUSARTx->ICR |= (1U << USART_ICR_IDLECF);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	// Error handling: ORE, FE, PE
	if ((isr & (1U << USART_ISR_ORE)) && (pUSARTHandle->pUSARTx->CR3 & (1U << USART_CR3_EIE))) {
		pUSARTHandle->pUSARTx->ICR |= (1U << USART_ICR_ORECF);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ERROR);
	}
	if ((isr & (1U << USART_ISR_FE)) && (pUSARTHandle->pUSARTx->CR3 & (1U << USART_CR3_EIE))) {
		pUSARTHandle->pUSARTx->ICR |= (1U << USART_ICR_FECF);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ERROR);
	}
	if ((isr & (1U << USART_ISR_PE)) && (cr1 & (1U << USART_CR1_PEIE))) {
		pUSARTHandle->pUSARTx->ICR |= (1U << USART_ICR_PECF);
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_ERROR);
	}
}

/* ============================= Weak callback ============================= */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t app_event)
{
	(void)pUSARTHandle;
	(void)app_event;
}


