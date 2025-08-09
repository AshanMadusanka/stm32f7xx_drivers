//
// Created by ashan on 07/06/2025.
//
#include "stm32f746xx_spi_driver.h"

#include <stdlib.h>

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if(EnorDi == ENABLE) {
        if(pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if(pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if(pSPIx == SPI3) {
            SPI3_PCLK_EN();
        } else if(pSPIx == SPI4) {
            SPI4_PCLK_EN();
        } else if(pSPIx == SPI5) {
            SPI5_PCLK_EN();
        } else if(pSPIx == SPI6) {
            SPI6_PCLK_EN();
        }
    } else {
        if(pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if(pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if(pSPIx == SPI3) {
            SPI3_PCLK_DI();
        } else if(pSPIx == SPI4) {
            SPI4_PCLK_DI();
        } else if(pSPIx == SPI5) {
            SPI5_PCLK_DI();
        } else if(pSPIx == SPI6) {
            SPI6_PCLK_DI();
        }
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {

    uint32_t tempreg = 0;
    uint32_t tempreg2 = 0;
    // Enable the peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // Configure the SPI device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
    // Configure the SPI bus configuration
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {

        tempreg &= ~ (pSPIHandle->SPIConfig.SPI_BusConfig << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {

        tempreg |= (1 << SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {

        tempreg &= ~(1 << SPI_CR1_BIDIOE);
        tempreg |= (1 << SPI_CR1_RXONLY);
    }
    // Configure the SPI clock speed
    tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // Configure the SPI data frame format
    tempreg2 |=(pSPIHandle->SPIConfig.SPI_DS << SPI_CR2_DS);

    // Configure the SPI software slave management
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // Configure the SPI clock polarity
    tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    // Set FRXTH if using 8-bit data size
    if (pSPIHandle->SPIConfig.SPI_DS == SPI_DS_8BITS) {
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_FRXTH);
    } else {
        pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_FRXTH);
    }

    pSPIHandle->pSPIx->CR1 = tempreg;
    pSPIHandle->pSPIx->CR2 |= tempreg2;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    pSPIx->CR1 = 0; // Reset the control register
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName) {

    if (pSPIx->SR & FlagName) {
        return FLAGSET;
    }
    return FLAGRESET ;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
                // Loop until all data is sent
                while (Len > 0) {

                    // Wait until TXE (Transmit buffer empty) flag is set
                    while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAGRESET);

                    // Check if DS (Data Frame Format) is 16-bit
                    if(((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16BITS) {

                        // Load 16 bits of data into the data register
                        pSPIx->DR = *(uint16_t*)pTxBuffer;
                        // Decrement length by 2 bytes
                        Len--;
                        Len--;
                        // Increment buffer pointer by 2 bytes
                        (uint16_t*)pTxBuffer++;
                    }
                    else {
                        // Load 8 bits of data into the data register
                        // Note: The data register is 16 bits wide, so we can write 8 bits directly
                      *((__vo uint8_t*)&pSPIx->DR) = *pTxBuffer;

                        // Decrement length by 1 byte
                        Len--;
                        // Increment buffer pointer by 1 byte
                        pTxBuffer++;
                    }
                }

            }

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
    // Implementation for receiving data through SPI
    // Loop until all data is sent
    while (Len > 0) {

        // Wait until RXNE (Transmit buffer empty) flag is set
        while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAGRESET);

        // Check if DS is 16-bit
        if(((pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16BITS) {

            // Load pTxBuffer with 16 bits of data from the data register
            *((uint16_t*)pRxBuffer )= pSPIx->DR ;
            // Decrement length by 2 bytes
            Len--;
            Len--;
            // Increment buffer pointer by 2 bytes
            (uint16_t*)pRxBuffer++;
        }
        else {
            // Load pTxBuffer with 8 bits of data from the data register
            *pRxBuffer = *((__vo uint8_t*)&pSPIx->DR);
            // Decrement length by 1 byte
            Len--;
            // Increment buffer pointer by 1 byte
            pRxBuffer++;
        }
    }
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    // Implementation for configuring SPI interrupt

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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    // Implementation for configuring SPI interrupt priority
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle) {
    // Implementation for handling SPI interrupts
    // This function should check the status of the SPI peripheral and handle the interrupt accordingly
    uint32_t temp1, temp2;
    // Check if the TXE interrupt is enabled
    temp1 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    temp2 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    if (temp1 && temp2) {
        // Call the TXE interrupt handler
        spi_txe_interrupt_handle(pSPIHandle);
    }

    // Check if the RXNE interrupt is enabled
    temp1 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
    temp2 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    if (temp1 && temp2) {
        // Call the RXNE interrupt handler
        spi_rxne_interrupt_handle(pSPIHandle);
    }
    // Check if the OVR interrupt is enabled
    temp1 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
    temp2 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    if(temp1 && temp2) {
        // Call the OVR interrupt handler
        spi_ovr_interrupt_handle(pSPIHandle);
    }
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    // Check if the TxLen is greater than 0
    if (pSPIHandle->TxLen > 0) {
        // Check if the data frame size is 16 bits
        if(((pSPIHandle->pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16BITS) {
            // Load 16 bits of data into the data register
            pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
            // Decrement the length by 2 bytes
            pSPIHandle->TxLen -= 2;
            // Increment the buffer pointer by 2 bytes
            pSPIHandle->pTxBuffer += 2;
        } else {
            // Load 8 bits of data into the data register
            *((__vo uint8_t*)&pSPIHandle->pSPIx->DR) = *pSPIHandle->pTxBuffer;
            // Decrement the length by 1 byte
            pSPIHandle->TxLen--;
            // Increment the buffer pointer by 1 byte
            pSPIHandle->pTxBuffer++;
        }
    }

    // If TxLen is zero, disable the TXEIE interrupt and set TxState to ready
    if (pSPIHandle->TxLen == 0) {
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT); // Notify the application that transmission is complete
    }

}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    // Check if DS is 16-bit
    if(((pSPIHandle->pSPIx->CR2 >> SPI_CR2_DS) & 0xF) == SPI_DS_16BITS) {

        // Load pTxBuffer with 16 bits of data from the data register
        *((uint16_t*)pSPIHandle->pRxBuffer )= pSPIHandle->pSPIx->DR ;
        // Decrement length by 2 bytes
        pSPIHandle->RxLen--;
        pSPIHandle->RxLen--;
        // Increment buffer pointer by 2 bytes
        (uint16_t*)pSPIHandle->pRxBuffer++;
    }
    else {
        // Load pTxBuffer with 8 bits of data from the data register
        *pSPIHandle->pRxBuffer = *((__vo uint8_t*)&pSPIHandle->pSPIx->DR);
        // Decrement length by 1 byte
        pSPIHandle->RxLen--;
        // Increment buffer pointer by 1 byte
        pSPIHandle->pRxBuffer++;
    }
    // If RxLen is zero, disable the RXNEIE interrupt and set RxState to ready
    if(pSPIHandle->RxLen == 0) {

        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT); // Notify the application that reception is complete
    }

}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    uint8_t temp;
    // Clear the OVR flag by reading the data register
    if(pSPIHandle->TxState != SPI_BUSY_IN_TX) {
        // If not busy in transmission, read the data register to clear the OVR flag
        temp = pSPIHandle->pSPIx->DR; // Read the data register
        temp = pSPIHandle->pSPIx->SR; // Read the status register to clear the OVR flag
        (void)temp; // Prevent unused variable warning
    }
    // Reset the RxState to ready
    pSPIHandle->RxState = SPI_READY;
    // Notify the application about the OVR error
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {

    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL; // Clear the TxBuffer pointer
    pSPIHandle->TxLen = 0; // Reset the TxLen
    pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {

    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL; // Clear the RxBuffer pointer
    pSPIHandle->RxLen = 0; // Reset the RxLen
    pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {

    uint8_t temp;
    temp = pSPIx->DR; // Read the data register to clear the OVR flag
    temp = pSPIx->SR; // Read the status register to clear the OVR flag

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE); // Set the SPE bit to enable the SPI peripheral
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE); // Clear the SPE bit to disable the SPI peripheral
    }
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI); // Set the SSI bit to enable internal slave select
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI); // Clear the SSI bit to disable internal slave select
    }
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE); // Set the SSOE bit to enable software slave management
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE); // Clear the SSOE bit to disable software slave management
    }
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {

    uint8_t state = pSPIHandle->TxState;
    if (state != SPI_BUSY_IN_TX) {
        // Set the TxState to busy
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // Store the TxBuffer and Len in the handle
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        // Enable the TXEIE interrupt
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }
    return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {

    uint8_t state = pSPIHandle->RxState;
    if (state != SPI_BUSY_IN_RX) {
        // Set the RxState to busy
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // Store the RxBuffer and Len in the handle
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        // Enable the RXNEIE interrupt
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }
    return state;

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {

    // This is a weak implementation of the application event callback function.

}