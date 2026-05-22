#include "w25q128.h"

static inline void W25Q128_CS_Low(W25Q128_Device *self) {
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_RESET);
}

static inline void W25Q128_CS_High(W25Q128_Device *self) {
    HAL_GPIO_WritePin(self->cs_port, self->cs_pin, GPIO_PIN_SET);
}

static void W25Q128_WriteEnable(W25Q128_Device *self) {
    uint8_t cmd = W25Q128_CMD_WRITE_ENABLE;
    W25Q128_CS_Low(self);
    HAL_SPI_Transmit(self->hspi, &cmd, 1, 100);
    W25Q128_CS_High(self);
}

static uint8_t W25Q128_ReadStatusReg1(W25Q128_Device *self) {
    uint8_t cmd = W25Q128_CMD_READ_STATUS_REG1;
    uint8_t status = 0;
    W25Q128_CS_Low(self);
    HAL_SPI_Transmit(self->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(self->hspi, &status, 1, 100);
    W25Q128_CS_High(self);
    return status;
}

static void W25Q128_WaitBusy(W25Q128_Device *self) {
    while ((W25Q128_ReadStatusReg1(self) & 0x01) == 0x01) {
        HAL_Delay(1);
    }
}

static HAL_StatusTypeDef W25Q128_Init_Impl(W25Q128_Device *self) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if (self->cs_port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (self->cs_port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (self->cs_port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    
    GPIO_InitStruct.Pin = self->cs_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(self->cs_port, &GPIO_InitStruct);
    
    W25Q128_CS_High(self);
    
    uint8_t cmd = W25Q128_CMD_RELEASE_POWER;
    W25Q128_CS_Low(self);
    HAL_SPI_Transmit(self->hspi, &cmd, 1, 100);
    W25Q128_CS_High(self);
    
    HAL_Delay(5);
    
    uint32_t id = self->ReadID(self);
    if ((id & 0xFFFFFF) == 0xEF4018 || (id & 0xFFFFFF) == 0xEF4017 || id != 0) {
        return HAL_OK;
    }
    return HAL_ERROR;
}

static uint32_t W25Q128_ReadID_Impl(W25Q128_Device *self) {
    uint8_t cmd = W25Q128_CMD_JEDEC_ID;
    uint8_t buffer[3] = {0};
    W25Q128_CS_Low(self);
    HAL_SPI_Transmit(self->hspi, &cmd, 1, 100);
    HAL_SPI_Receive(self->hspi, buffer, 3, 100);
    W25Q128_CS_High(self);
    
    return (uint32_t)((buffer[0] << 16) | (buffer[1] << 8) | buffer[2]);
}

static HAL_StatusTypeDef W25Q128_Read_Impl(W25Q128_Device *self, uint32_t addr, uint8_t *pBuffer, uint32_t size) {
    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_READ_DATA;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((addr >> 8) & 0xFF);
    cmd[3] = (uint8_t)(addr & 0xFF);
    
    W25Q128_CS_Low(self);
    HAL_SPI_Transmit(self->hspi, cmd, 4, 1000);
    HAL_StatusTypeDef status = HAL_SPI_Receive(self->hspi, pBuffer, size, 5000);
    W25Q128_CS_High(self);
    return status;
}

static HAL_StatusTypeDef W25Q128_WritePage_Impl(W25Q128_Device *self, uint32_t addr, const uint8_t *pBuffer, uint16_t size) {
    if (size > W25Q128_PAGE_SIZE) {
        size = W25Q128_PAGE_SIZE;
    }
    
    W25Q128_WriteEnable(self);
    
    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((addr >> 8) & 0xFF);
    cmd[3] = (uint8_t)(addr & 0xFF);
    
    W25Q128_CS_Low(self);
    HAL_SPI_Transmit(self->hspi, cmd, 4, 100);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(self->hspi, (uint8_t*)pBuffer, size, 1000);
    W25Q128_CS_High(self);
    
    W25Q128_WaitBusy(self);
    return status;
}

static HAL_StatusTypeDef W25Q128_Write_Impl(W25Q128_Device *self, uint32_t addr, const uint8_t *pBuffer, uint32_t size) {
    uint32_t page_offset = addr % W25Q128_PAGE_SIZE;
    uint32_t page_remain = W25Q128_PAGE_SIZE - page_offset;
    
    if (size <= page_remain) {
        return self->WritePage(self, addr, pBuffer, size);
    }
    
    HAL_StatusTypeDef status = self->WritePage(self, addr, pBuffer, page_remain);
    if (status != HAL_OK) return status;
    
    addr += page_remain;
    pBuffer += page_remain;
    size -= page_remain;
    
    while (size >= W25Q128_PAGE_SIZE) {
        status = self->WritePage(self, addr, pBuffer, W25Q128_PAGE_SIZE);
        if (status != HAL_OK) return status;
        addr += W25Q128_PAGE_SIZE;
        pBuffer += W25Q128_PAGE_SIZE;
        size -= W25Q128_PAGE_SIZE;
    }
    
    if (size > 0) {
        status = self->WritePage(self, addr, pBuffer, size);
    }
    return status;
}

static HAL_StatusTypeDef W25Q128_EraseSector_Impl(W25Q128_Device *self, uint32_t sectorAddr) {
    W25Q128_WriteEnable(self);
    W25Q128_WaitBusy(self);
    
    uint8_t cmd[4];
    cmd[0] = W25Q128_CMD_SECTOR_ERASE_4KB;
    cmd[1] = (uint8_t)((sectorAddr >> 16) & 0xFF);
    cmd[2] = (uint8_t)((sectorAddr >> 8) & 0xFF);
    cmd[3] = (uint8_t)(sectorAddr & 0xFF);
    
    W25Q128_CS_Low(self);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(self->hspi, cmd, 4, 100);
    W25Q128_CS_High(self);
    
    W25Q128_WaitBusy(self);
    return status;
}

static HAL_StatusTypeDef W25Q128_EraseChip_Impl(W25Q128_Device *self) {
    W25Q128_WriteEnable(self);
    
    uint8_t cmd = W25Q128_CMD_CHIP_ERASE;
    W25Q128_CS_Low(self);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(self->hspi, &cmd, 1, 100);
    W25Q128_CS_High(self);
    
    W25Q128_WaitBusy(self);
    return status;
}

void W25Q128_Create(W25Q128_Device *self, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    self->hspi = hspi;
    self->cs_port = cs_port;
    self->cs_pin = cs_pin;
    
    self->Init = W25Q128_Init_Impl;
    self->ReadID = W25Q128_ReadID_Impl;
    self->Read = W25Q128_Read_Impl;
    self->WritePage = W25Q128_WritePage_Impl;
    self->Write = W25Q128_Write_Impl;
    self->EraseSector = W25Q128_EraseSector_Impl;
    self->EraseChip = W25Q128_EraseChip_Impl;
}
