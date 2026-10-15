#ifndef __JY_TURN_H
#define __JY_TURN_H

#include "main.h"

/* 转圈完成阈值（度）：累计航向变化超过此值判定一圈 */
#define CIRCLE_THRESHOLD 330.0f

/* 外部可读状态 */
extern uint8_t circle_complete;  /* 1=已转完一圈 */
extern float   yaw_accum;        /* 累计角度变化（带符号，度） */

void jy_turn_init(void);
void jy_turn_update(void);

#endif
