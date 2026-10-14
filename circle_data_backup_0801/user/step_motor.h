#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include "bsp.h"

void step_motor_poll(float speed_rpm_x);   /* 轮询状态机：统一调度速度/读位置（速度模式，保留向后兼容） */
void Motor_Pos_Control_X(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk);  /* 位置模式 */

#endif
