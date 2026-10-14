#ifndef __BSP_H
#define __BSP_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include "usart_receive.h"
#include "pid.h"
#include "step_motor_pid.h"  /* 保留旧PID以备后用 */
#include "step_motor.h"
#include "stdio.h"
#include <math.h>
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint8_t u8;
#define DMA_BUF_SIZE 128

extern uint8_t pi_rx_buf[128];      // 摄像头(USART3) DMA缓存
extern u16 rx_index ;
extern u32 uart_rx_tick ;
extern u8 uart_rx_buffer[128];
extern u8 uart_rx_data;
extern u8 rx_buf[8];
extern int32_t posx;                   /* 电机实时位置（脉冲数） */

/* 外部变量声明 */
extern uint8_t pi_rx_buf[];           // DMA 缓冲区
extern UART_HandleTypeDef huart3;     // USART3 句柄

/* DEBUG 变量 —— 在 Keil Watch 窗口中添加这些变量名即可 */
extern volatile uint32_t g_dma_half_cnt;
extern volatile uint32_t g_dma_full_cnt;
extern volatile uint32_t g_rx_frame_cnt;
extern volatile int32_t  g_last_rx_x;
extern volatile int32_t  g_latest_error_x;  // ★ main.c 直接读这个
extern volatile int32_t  g_ball_velocity;   // ★ 球的速度

/* PID 调试变量 —— Keil Watch 加入这些观察 PID 状态 */
extern volatile float  g_pid_raw_out;       // PID 原始输出（限幅前）
extern volatile float  g_pulse_out;       /* PID输出脉冲数（位置模式，非RPM） */

/* 新 PID（移植自视觉跟踪） */
extern PID ball_pid;          /* 全局PID实例，Keil Watch观察 ball_pid.out / ball_pid.Error0 / ball_pid.err_int */
extern uint8_t ball_direction;/* 电机方向：0=CW, 1=CCW */
#endif
