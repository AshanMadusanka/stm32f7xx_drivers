
//
// Created by ashan on 27/06/2025.
//

#include "stm32f746xx_i2c_driver.h"
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static  void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
void I2C_Init(I2C_Handle_t *pI2CHandle) {

    uint32_t tempreg = 0;

    // Enable the I2C1 peripheral clock
    I2C_PeriClockControl(pI2CHandle->pI2Cx , ENABLE);

    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1; // Shift left to set the address in the correct position
    tempreg |= (1 << 15); // Set the ADDR bit in the OAR1 register
    pI2CHandle->pI2Cx->OAR1 |= tempreg; // Set the device address in the OAR1 register

    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM) {
        tempreg |= (0 << I2C_TIMINGR_PRESC); // Set the SCL speed to Standard Mode
        tempreg |= (0x04 << I2C_TIMINGR_SCLDEL); // Set the SCL low period
        tempreg |= (0x02 << I2C_TIMINGR_SDADEL); // Set the SCL high period
        tempreg |= (0xF << I2C_TIMINGR_SCLH); // Set the SCL high time
        tempreg |= (0x13 << I2C_TIMINGR_SCLL); // Set the SCL low time

        pI2CHandle->pI2Cx->TIMINGR |= tempreg; // Write the timing configuration to the TIMINGR register
    } else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM2K) {
        tempreg |= (0 << I2C_TIMINGR_PRESC); // Set the SCL speed to Standard Mode
        tempreg |= (0x03 << I2C_TIMINGR_SCLDEL); // Set the SCL low period
        tempreg |= (0x01 << I2C_TIMINGR_SDADEL); // Set the SCL high period
        tempreg |= (0x13 << I2C_TIMINGR_SCLH); // Set the SCL high time
        tempreg |= (0x2F << I2C_TIMINGR_SCLL); // Set the SCL low time

        pI2CHandle->pI2Cx->TIMINGR |= tempreg; // Write the timing configuration to the TIMINGR register

    } else if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_FM4K) {
        tempreg |= (0 << I2C_TIMINGR_PRESC); // Set the SCL speed to Standard Mode
        tempreg |= (0x03 << I2C_TIMINGR_SCLDEL); // Set the SCL low period
        tempreg |= (0x01 << I2C_TIMINGR_SDADEL); // Set the SCL high period
        tempreg |= (0x09 << I2C_TIMINGR_SCLH); // Set the SCL high time
        tempreg |= (0x13 << I2C_TIMINGR_SCLL); // Set the SCL low time

        pI2CHandle->pI2Cx->TIMINGR |= tempreg; // Write the timing configuration to the TIMINGR register
    }
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr , uint8_t SrOrStop) {


 // 1. Clear and configure CR2 with address, byte count, and direction
    uint32_t temp = pI2CHandle->pI2Cx->CR2;

    /**
     * Clear the SADD (slave address), NBYTES (number of bytes), and RD_WRN (read/write) fields in the I2C_CR2 register.
     * This prepares the register for setting new values for these fields before initiating an I2C transfer.
     * - SADD: 7 bits for slave address
     * - NBYTES: 8 bits for number of bytes to transfer
     * - RD_WRN: 1 (set) for read
     * - RD_WRN: 0 (clear) for write
     */
    temp &= ~((0x7F << I2C_CR2_SADD) | (0xFF << I2C_CR2_NBYTES) | (1 << I2C_CR2_RD_WRN));
    temp |= ((SlaveAddr << 1) << I2C_CR2_SADD); // 7-bit address shifted
    temp |= (Len << I2C_CR2_NBYTES);            // Number of bytes
    temp |= (1 << I2C_CR2_AUTOEND);            // Auto-end mode
    pI2CHandle->pI2Cx->CR2 = temp;

    // 2. Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 3. Send data bytes
    for(uint32_t i = 0; i < Len; i++) {
        // Wait until TXE flag is set
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) {
            // Consider adding a timeout here
        }
        // Send data byte
        pI2CHandle->pI2Cx->TXDR = pTxBuffer[i];
    }

    // 4. In auto-end mode, STOP is generated automatically after the last byte
    // Wait for STOPF flag
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF));

    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF));

    if (SrOrStop == I2C_DISABLE_SR) {
       I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }





}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop)
{
    uint32_t temp = pI2CHandle->pI2Cx->CR2;

    // Clear SADD, NBYTES, RD_WRN fields
    temp &= ~((0x7F << I2C_CR2_SADD) | (0xFF << I2C_CR2_NBYTES) | (1 << I2C_CR2_RD_WRN));
    temp |= ((SlaveAddr << 1) << I2C_CR2_SADD); // 7-bit address
    temp |= (Len << I2C_CR2_NBYTES);            // Number of bytes
    temp |= (1 << I2C_CR2_RD_WRN);              // Set for read
    temp |= (1 << I2C_CR2_AUTOEND);             // Auto-end mode
    pI2CHandle->pI2Cx->CR2 = temp;

    // Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // Receive data
    for(uint32_t i = 0; i < Len; i++) {
        // Wait until RXNE flag is set
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)) {
            // Optionally add timeout
        }
        // Read data
        pRxBuffer[i] = (uint8_t)pI2CHandle->pI2Cx->RXDR;
    }

    // Wait for STOPF flag (stop condition detected)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF));

    // Clear STOPF flag by writing to ICR
    pI2CHandle->pI2Cx->ICR |= (1 << I2C_ISR_STOPF);

    // If SrOrStop is I2C_ENABLE_SR, keep the bus active for repeated start
    if (SrOrStop == I2C_DISABLE_SR) {
        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
    }

}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        // Enable the I2C peripheral
        pI2Cx->CR1 |= (1 << 0); // Set the PE bit in CR1 register
    } else {
        // Disable the I2C peripheral
        pI2Cx->CR1 &= ~(1 << 0); // Clear the PE bit in CR1 register
    }
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        // Enable the clock for the I2C peripheral
        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
        else if (pI2Cx == I2C4) {
            I2C4_PCLK_EN();
        }
    }else {
        // Disable the clock for the I2C peripheral
        if (pI2Cx == I2C1) {
            I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
        else if (pI2Cx == I2C4) {
            I2C4_PCLK_DI();
        }
    }
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
    // Generate a START condition
    pI2Cx->CR2 |= (1 << I2C_CR2_START);
 
}

static  void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr) {

    SlaveAddr = SlaveAddr << 1; // Shift left to set the address in the correct position
    SlaveAddr &= ~(1 << 0); // Clear the LSB to indicate a write operation
    pI2Cx->CR2 |= SlaveAddr; // Set the address in the CR2 register


}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
    // Generate a STOP condition
    pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName) {
    if (pI2Cx->ISR & FlagName) {
        return 1; // Flag is set
    } else {
        return 0; // Flag is not set
    }
}

void I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop){
    // 1. Save the Tx buffer and length to the handle for use in ISR
    pI2CHandle->pTxBuffer = pTxBuffer;
    pI2CHandle->TxLen = Len;
    pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
    pI2CHandle->DevAddr = SlaveAddr;
    pI2CHandle->Sr = SrOrStop;

    // 2. Configure CR2 for address, nbytes, direction (write)
    uint32_t temp = pI2CHandle->pI2Cx->CR2;
    temp &= ~((0x7F << I2C_CR2_SADD) | (0xFF << I2C_CR2_NBYTES) | (1U << I2C_CR2_RD_WRN));
    temp |= ((SlaveAddr << 1) << I2C_CR2_SADD);
    temp |= (Len << I2C_CR2_NBYTES);
    temp &= ~(1U << I2C_CR2_RD_WRN); // Write
    temp |= (1U << I2C_CR2_AUTOEND); // Auto-end
    pI2CHandle->pI2Cx->CR2 = temp;

    // 3. Enable I2C interrupts: TXIE, STOPIE, NACKIE, ERRIE
    pI2CHandle->pI2Cx->CR1 |= (1U << I2C_CR1_TXIE) | (1U << I2C_CR1_STOPIE) | (1U << I2C_CR1_NACKIE) | (1U << I2C_CR1_ERRIE);

    // 4. Generate START
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
}
void I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t SrOrStop){
    // 1. Save the Rx buffer and length to the handle for use in ISR
    pI2CHandle->pRxBuffer = pRxBuffer;
    pI2CHandle->RxLen = Len;
    pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
    pI2CHandle->DevAddr = SlaveAddr;
    pI2CHandle->Sr = SrOrStop;

    // 2. Configure CR2 for address, nbytes, direction (read)
    uint32_t temp = pI2CHandle->pI2Cx->CR2;
    temp &= ~((0x7F << I2C_CR2_SADD) | (0xFF << I2C_CR2_NBYTES) | (1U << I2C_CR2_RD_WRN));
    temp |= ((SlaveAddr << 1) << I2C_CR2_SADD);
    temp |= (Len << I2C_CR2_NBYTES);
    temp |= (1U << I2C_CR2_RD_WRN); // Read
    temp |= (1U << I2C_CR2_AUTOEND); // Auto-end
    pI2CHandle->pI2Cx->CR2 = temp;

    // 3. Enable I2C interrupts: RXIE, STOPIE, NACKIE, ERRIE
    pI2CHandle->pI2Cx->CR1 |= (1U << I2C_CR1_RXIE) | (1U << I2C_CR1_STOPIE) | (1U << I2C_CR1_NACKIE) | (1U << I2C_CR1_ERRIE);

    // 4. Generate START
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
}

void I2C_IRQInterruptConfig(I2C_Handle_t *pI2CHandle, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        // Enable the I2C interrupt
        NVIC_EnableIRQ(pI2CHandle->IRQn);
    } else {
        // Disable the I2C interrupt
        NVIC_DisableIRQ(pI2CHandle->IRQn);
    }
}

void I2C_IRQPriorityConfig(I2C_Handle_t *pI2CHandle, uint32_t IRQPriority) {
    // Set the IRQ priority for the I2C peripheral
    NVIC_SetPriority(pI2CHandle->IRQn, IRQPriority);
}

void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle) {
    uint32_t isr = pI2CHandle->pI2Cx->ISR;

    // 1. Handle TXIS (Transmit interrupt status, ready to send next byte)
    if (isr & (1U << I2C_ISR_TXIS)) {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX && pI2CHandle->TxLen > 0) {
            pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer++);
            pI2CHandle->TxLen--;
        }
    }

    // 2. Handle RXNE (Receive buffer not empty)
    if (isr & (1U << I2C_ISR_RXNE)) {
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxLen > 0) {
            *(pI2CHandle->pRxBuffer++) = (uint8_t)pI2CHandle->pI2Cx->RXDR;
            pI2CHandle->RxLen--;
        }
    }

    // 3. Handle STOPF (Stop detection flag)
    if (isr & (1U << I2C_ISR_STOPF)) {
        // Clear STOPF by writing 1 to it in ICR
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_STOPF);
        // Reset state
        pI2CHandle->TxRxState = I2C_READY;
        // Optionally, call application callback here
    }

    // 4. Handle NACKF (NACK received)
    if (isr & (1U << I2C_ISR_NACKF)) {
        // Clear NACKF by writing 1 to it in ICR
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_NACKF);
        // Optionally, handle NACK error (reset state, callback, etc.)
        pI2CHandle->TxRxState = I2C_READY;
    }

    // 5. Handle TXE (Transmit buffer empty, legacy)
    if (isr & (1U << I2C_ISR_TXE)) {
        // Do nothing, handled by TXIS
    }

    // 6. Handle TC (Transfer Complete)
    if (isr & (1U << I2C_ISR_TC)) {
        // Optionally, handle transfer complete (for RELOAD mode)
    }

    // 7. Handle errors (BERR, ARLO, OVR, etc.)
    if (isr & (1U << I2C_ISR_BERR)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_BERR);
        pI2CHandle->TxRxState = I2C_READY;
    }
    if (isr & (1U << I2C_ISR_ARLO)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_ARLO);
        pI2CHandle->TxRxState = I2C_READY;
    }
    if (isr & (1U << I2C_ISR_OVR)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_OVR);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // Add more error handling as needed
}

void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle) {
    uint32_t isr = pI2CHandle->pI2Cx->ISR;

    // Bus error
    if (isr & (1U << I2C_ISR_BERR)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_BERR);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // Arbitration lost
    if (isr & (1U << I2C_ISR_ARLO)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_ARLO);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // Overrun/Underrun
    if (isr & (1U << I2C_ISR_OVR)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_OVR);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // PEC error
    if (isr & (1U << I2C_ISR_PECERR)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_PECERR);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // Timeout
    if (isr & (1U << I2C_ISR_TIMEOUT)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_TIMEOUT);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // SMBus alert
    if (isr & (1U << I2C_ISR_ALERT)) {
        pI2CHandle->pI2Cx->ICR |= (1U << I2C_ISR_ALERT);
        pI2CHandle->TxRxState = I2C_READY;
    }
    // Optionally, call application error callback here
}