#include "openmv_parser.h"
#include <string.h>

#define FRAME_HEADER1 0x55
#define FRAME_HEADER2 0xAA

static void OpenMV_Parser_Init_Impl(OpenMV_Parser *self, UART_HandleTypeDef *huart) {
    self->huart = huart;
    self->state = STATE_WAIT_HEADER1;
    self->cmd = 0;
    self->data_len = 0;
    self->data_cnt = 0;
    self->checksum = 0;
    self->last_recv_time = 0;
    memset(&self->target, 0, sizeof(OpenMV_Target_t));
}

static uint8_t OpenMV_Parser_ParseByte_Impl(OpenMV_Parser *self, uint8_t byte) {
    uint32_t now = HAL_GetTick();
    
    if (self->state != STATE_WAIT_HEADER1 && (now - self->last_recv_time > 10)) {
        self->state = STATE_WAIT_HEADER1;
    }
    self->last_recv_time = now;

    switch (self->state) {
        case STATE_WAIT_HEADER1:
            if (byte == FRAME_HEADER1) {
                self->state = STATE_WAIT_HEADER2;
                self->checksum = byte;
            }
            break;
            
        case STATE_WAIT_HEADER2:
            if (byte == FRAME_HEADER2) {
                self->state = STATE_WAIT_CMD;
                self->checksum += byte;
            } else {
                self->state = STATE_WAIT_HEADER1;
            }
            break;
            
        case STATE_WAIT_CMD:
            self->cmd = byte;
            self->checksum += byte;
            self->state = STATE_WAIT_LEN;
            break;
            
        case STATE_WAIT_LEN:
            self->data_len = byte;
            if (self->data_len > sizeof(self->rx_buffer)) {
                self->state = STATE_WAIT_HEADER1;
            } else {
                self->checksum += byte;
                self->data_cnt = 0;
                if (self->data_len == 0) {
                    self->state = STATE_WAIT_CHECKSUM;
                } else {
                    self->state = STATE_RECEIVE_DATA;
                }
            }
            break;
            
        case STATE_RECEIVE_DATA:
            self->rx_buffer[self->data_cnt++] = byte;
            self->checksum += byte;
            if (self->data_cnt >= self->data_len) {
                self->state = STATE_WAIT_CHECKSUM;
            }
            break;
            
        case STATE_WAIT_CHECKSUM:
            if (self->checksum == byte) {
                if (self->cmd == 0x01 && self->data_len >= 9) {
                    self->target.detected = self->rx_buffer[0];
                    self->target.x = (int16_t)((self->rx_buffer[1] << 8) | self->rx_buffer[2]);
                    self->target.y = (int16_t)((self->rx_buffer[3] << 8) | self->rx_buffer[4]);
                    self->target.width = (uint16_t)((self->rx_buffer[5] << 8) | self->rx_buffer[6]);
                    self->target.height = (uint16_t)((self->rx_buffer[7] << 8) | self->rx_buffer[8]);
                }
                self->state = STATE_WAIT_HEADER1;
                return 1;
            } else {
                self->state = STATE_WAIT_HEADER1;
            }
            break;
            
        default:
            self->state = STATE_WAIT_HEADER1;
            break;
    }
    return 0;
}

static void OpenMV_Parser_GetTarget_Impl(OpenMV_Parser *self, OpenMV_Target_t *out_target) {
    if (out_target) {
        *out_target = self->target;
    }
}

void OpenMV_Parser_Create(OpenMV_Parser *self, UART_HandleTypeDef *huart) {
    self->huart = huart;
    self->Init = OpenMV_Parser_Init_Impl;
    self->ParseByte = OpenMV_Parser_ParseByte_Impl;
    self->GetTarget = OpenMV_Parser_GetTarget_Impl;
    self->Init(self, huart);
}
