/**
 * @file    jy_turn.c
 * @brief   JY901S 航向角度监测 —— 纯监测，不动电机
 *
 * 逻辑：
 *   1. 上电时记录 JY901S 当前 Yaw 作为初始航向
 *   2. 每个周期计算 delta，处理 ±180°环绕后累加到 yaw_accum
 *   3. 当 |yaw_accum| >= CIRCLE_THRESHOLD(330°) 时，circle_complete 置 1
 *   4. 不控制电机，只提供 circle_complete 标志供外部停车判断
 */

#include "jy_turn.h"
#include "jy_read.h"
#include <math.h>

static float yaw_start;   /* 起始航向角          */
static float yaw_prev;    /* 上一次航向角         */
static uint8_t started;   /* 1=已开始监测         */

uint8_t circle_complete = 0;
float   yaw_accum       = 0.0f;

void jy_turn_init(void)
{
    yaw_start       = fAngle[2];   /* 记录上电时的初始航向 */
    yaw_prev        = yaw_start;
    yaw_accum       = 0.0f;
    started         = 1;           /* 上电即开始监测        */
    circle_complete = 0;
}

void jy_turn_update(void)
{
    if (!started || circle_complete) return;

    float yaw_now = fAngle[2];
    float delta = yaw_now - yaw_prev;

    /* ±180°环绕处理 */
    if (delta > 180.0f)       delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;

    yaw_accum += delta;
    yaw_prev   = yaw_now;

    /* 累计角度超过阈值 → 一圈完成 */
    if (fabsf(yaw_accum) >= CIRCLE_THRESHOLD) {
        circle_complete = 1;
    }
}
