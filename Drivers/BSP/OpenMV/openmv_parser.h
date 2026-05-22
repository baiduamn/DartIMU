#ifndef __OPENMV_PARSER_H
#define __OPENMV_PARSER_H

#include "main.h"

typedef struct {
    uint8_t detected;
    int16_t x;
    int16_t y;
    uint16_t width;
    uint16_t height;
} OpenMV_Target_t;

typedef enum {
    STATE_WAIT_HEADER1 = 0,
    STATE_WAIT_HEADER2,
    STATE_WAIT_CMD,
    STATE_WAIT_LEN,
    STATE_RECEIVE_DATA,
    STATE_WAIT_CHECKSUM
} ParserState_t;

struct OpenMV_Parser;

typedef struct OpenMV_Parser {
    UART_HandleTypeDef *huart;
    ParserState_t state;
    uint8_t cmd;
    uint8_t data_len;
    uint8_t data_cnt;
    uint8_t rx_buffer[32];
    uint8_t checksum;
    uint32_t last_recv_time;
    
    OpenMV_Target_t target;
    
    void (*Init)(struct OpenMV_Parser *self, UART_HandleTypeDef *huart);
    uint8_t (*ParseByte)(struct OpenMV_Parser *self, uint8_t byte);
    void (*GetTarget)(struct OpenMV_Parser *self, OpenMV_Target_t *out_target);
} OpenMV_Parser;

void OpenMV_Parser_Create(OpenMV_Parser *self, UART_HandleTypeDef *huart);

#endif /* __OPENMV_PARSER_H */
