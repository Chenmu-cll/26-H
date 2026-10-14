#ifndef __USART_RECEIVE_H
#define __USART_RECEIVE_H

#include "bsp.h"

int  pi_parse_data(char *buffer);         // 解析一行摄像头数据，提取坐标
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart); // DMA半传输回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);     // DMA全传输回调

extern volatile int32_t g_latest_error_x;
extern volatile int32_t g_ball_velocity;  /* 球的速度 */

#endif
