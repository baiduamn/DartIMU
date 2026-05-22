#ifndef __W25Q128_H
#define __W25Q128_H

#include "main.h"

#define W25Q128_PAGE_SIZE       256
#define W25Q128_SECTOR_SIZE     4096
#define W25Q128_BLOCK_SIZE      65536
#define W25Q128_TOTAL_SIZE      16777216

#define W25Q128_CMD_WRITE_ENABLE      0x06
#define W25Q128_CMD_WRITE_DISABLE     0x04
#define W25Q128_CMD_READ_STATUS_REG1  0x05
#define W25Q128_CMD_WRITE_STATUS_REG1 0x01
#define W25Q128_CMD_READ_DATA         0x03
#define W25Q128_CMD_PAGE_PROGRAM      0x02
#define W25Q128_CMD_SECTOR_ERASE_4KB  0x20
#define W25Q128_CMD_BLOCK_ERASE_32KB  0x52
#define W25Q128_CMD_BLOCK_ERASE_64KB  0xD8
#define W25Q128_CMD_CHIP_ERASE        0xC7
#define W25Q128_CMD_POWER_DOWN        0xB9
#define W25Q128_CMD_RELEASE_POWER     0xAB
#define W25Q128_CMD_JEDEC_ID          0x9F

struct W25Q128_Device;

typedef struct W25Q128_Device {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    
    HAL_StatusTypeDef (*Init)(struct W25Q128_Device *self);
    uint32_t (*ReadID)(struct W25Q128_Device *self);
    HAL_StatusTypeDef (*Read)(struct W25Q128_Device *self, uint32_t addr, uint8_t *pBuffer, uint32_t size);
    HAL_StatusTypeDef (*WritePage)(struct W25Q128_Device *self, uint32_t addr, const uint8_t *pBuffer, uint16_t size);
    HAL_StatusTypeDef (*Write)(struct W25Q128_Device *self, uint32_t addr, const uint8_t *pBuffer, uint32_t size);
    HAL_StatusTypeDef (*EraseSector)(struct W25Q128_Device *self, uint32_t sectorAddr);
    HAL_StatusTypeDef (*EraseChip)(struct W25Q128_Device *self);
} W25Q128_Device;

void W25Q128_Create(W25Q128_Device *self, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif /* __W25Q128_H */
