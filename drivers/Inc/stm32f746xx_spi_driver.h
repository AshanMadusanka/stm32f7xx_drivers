//
// Created by ashan on 07/06/2025.
//

#ifndef STM32F746XX_SPI_DRIVER_H
#define STM32F746XX_SPI_DRIVER_H

#include "stm32f746xx.h"

/**
 * @brief Configuration structure for SPIx peripheral
 */
typedef struct {

    uint8_t SPI_DeviceMode;        /*!< Specifies the SPI device mode. Possible values from @ref SPI_Device_Mode */
    uint8_t SPI_BusConfig;         /*!< Specifies the SPI bus configuration. Possible values from @ref SPI_Bus_Config */
    uint8_t SPI_SclkSpeed;         /*!< Specifies the SPI clock speed. Possible values from @ref SPI_Sclk_Speed */
    uint8_t SPI_DS;               /*!< Specifies the SPI data frame format. Possible values from @ref SPI_Data_Frame_Format */
    uint8_t SPI_CPOL;              /*!< Specifies the SPI clock polarity. Possible values from @ref SPI_Clock_Polarity */
    uint8_t SPI_CPHA;              /*!< Specifies the SPI clock phase. Possible values from @ref SPI_Clock_Phase */
    uint8_t SPI_SSM;               /*!< Specifies the SPI software slave management. Possible values from @ref SPI_Software_Slave_Management */

} SPI_Config_t;

/**
 * @brief Handle structure for SPIx peripheral
 */
typedef struct {

    SPI_RegDef_t *pSPIx;           /*!< Pointer to the SPIx peripheral base address */
    SPI_Config_t SPIConfig;        /*!< Configuration settings for the SPI peripheral */
    uint8_t *pTxBuffer;           /*!< Pointer to the transmit buffer */
    uint8_t *pRxBuffer;           /*!< Pointer to the receive buffer */
    uint32_t TxLen;               /*!< Length of data to be transmitted */
    uint32_t RxLen;               /*!< Length of data to be received */
    uint8_t TxState;             /*!< State of the transmit operation */
    uint8_t RxState;             /*!< State of the receive operation */

} SPI_Handle_t;

/**
 * @brief SPI Device Mode
 *
 * @note These macros define the possible device modes for the SPI peripheral.
 */

#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0


/**
 * @brief SPI Bus Configuration
 *
 * @note These macros define the possible bus configurations for the SPI peripheral.
 */
#define SPI_BUS_CONFIG_FD       1 /*!< Full Duplex */
#define SPI_BUS_CONFIG_HD       2 /*!< Half Duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3 /*!< Simplex Receive Only */

/**
 * @brief SPI Clock Speed
 *
 * @note These macros define the possible clock speeds for the SPI peripheral.
 */

#define SPI_SCLK_SPEED_DIV2     0 /*!< Clock speed divided by 2 */
#define SPI_SCLK_SPEED_DIV4     1 /*!< Clock speed divided by 4 */
#define SPI_SCLK_SPEED_DIV8     2 /*!< Clock speed divided by 8 */
#define SPI_SCLK_SPEED_DIV16    3 /*!< Clock speed divided by 16 */
#define SPI_SCLK_SPEED_DIV32    4 /*!< Clock speed divided by 32 */
#define SPI_SCLK_SPEED_DIV64    5 /*!< Clock speed divided by 64 */
#define SPI_SCLK_SPEED_DIV128   6 /*!< Clock speed divided by 128 */
#define SPI_SCLK_SPEED_DIV256   7 /*!< Clock speed divided by 256 */


/**
 * @brief SPI Data Frame Format
 *
 * @note These macros define the possible data frame formats for the SPI peripheral.
 */

#define SPI_DS_8BITS           7 /*!< 8-bit data frame format */
#define SPI_DS_16BITS          15 /*!< 16-bit data frame format */

/**
 * @brief SPI Clock Polarity
 *
 * @note These macros define the possible clock polarities for the SPI peripheral.
 */

#define SPI_CPOL_LOW            0 /*!< Clock polarity low */
#define SPI_CPOL_HIGH           1 /*!< Clock polarity high */

/**
 * @brief SPI Clock Phase
 *
 * @note These macros define the possible clock phases for the SPI peripheral
 *
 */

#define SPI_CPHA_LOW            0 /*!< Clock phase low */
#define SPI_CPHA_HIGH           1 /*!< Clock phase high */

/**
 * @brief SPI Software Slave Management
 *
 * @note These macros define the possible software slave management configurations for the SPI peripheral.
 */

#define SPI_SSM_EN              1 /*!< Software slave management enabled */
#define SPI_SSM_DI              0 /*!< Software slave management disabled */


#define SPI_TXE_FLAG (1<< SPI_SR_TXE)
#define SPI_RXNE_FLAG (1<< SPI_SR_RXNE)
#define SPI_BUSY_FLAG (1<< SPI_SR_BSY)
#define SPI_OVR_FLAG (1<< SPI_SR_OVR)
#define SPI_MODF_FLAG (1<< SPI_SR_MODF)
#define SPI_CRCERR_FLAG (1<< SPI_SR_CRCERR)
#define SPI_FRE_FLAG (1<< SPI_SR_FRE)
#define SPI_UDR_FLAG (1<< SPI_SR_UDR)
#define SPI_CHSIDE_FLAG (1<< SPI_SR_CHSIDE)

#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2


#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_ERR  3
#define SPI_EVENT_CRC_ERR 4

/**
 * @brief Enables or disables the peripheral clock for the given SPI peripheral
 *
 * @param pSPIx Pointer to the SPI peripheral base address
 * @param EnorDi Enable or disable macro (1 to enable, 0 to disable)
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief Initializes the given SPI peripheral with the specified configuration
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);

/**
 * @brief Deinitializes the given SPI peripheral
 *
 * @param pSPIx Pointer to the SPI peripheral base address
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/**
 * @brief Gets the flag status of the specified flag in the SPI peripheral
 *
 * @param pSPIx Pointer to the SPI peripheral base address
 * @param FlagName Name of the flag to check (e.g., SPI_SR_RXNE, SPI_SR_TXE)
 * @return Flag status (1 if set, 0 if reset)
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
/**
 * @brief Sends data through the SPI peripheral
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 * @param pTxBuffer Pointer to the transmit buffer
 * @param Len Length of the data to be sent
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

/**
 * @brief Receives data through the SPI peripheral
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 * @param pRxBuffer Pointer to the receive buffer
 * @param Len Length of the data to be received
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief Configures the interrupt for the given IRQ number
 *
 * @param IRQNumber IRQ number to configure
 * @param EnorDi Enable or disable macro (1 to enable, 0 to disable)
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Configures the priority of the given IRQ number
 *
 * @param IRQNumber IRQ number to configure
 * @param IRQPriority Priority value to set
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Handles the interrupt for the SPI peripheral
 *
 * @param pSPIHandle Pointer to the SPI handle structure
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);
#endif //STM32F746XX_SPI_DRIVER_H
