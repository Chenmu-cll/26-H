#ifndef __TB6612_H__
#define __TB6612_H__

#include "bsp.h"

extern uint8_t motor_enabled;       /* 电机使能标志位：0=停止，1=启动 */
/* 比赛计时 */
extern uint32_t race_start_tick;    /* 计时开始时刻(ms) */
extern uint32_t race_elapsed_ms;    /* 最终用时(ms) */
extern uint8_t  race_timing;        /* 计时状态：1=计时中，0=停止/未开始 */

/* ===== 匀加速匀减速模式 ===== */
extern uint8_t accel_mode;          /* 0=普通模式, 1=匀加速匀减速模式 */
extern uint8_t accel_state;         /* 匀加速流程状态机 */

/* 匀加速匀减速可调参数宏 */
#define TARGET_PWM0       230       /* 普通模式目标基础速度 */
#define ACCEL_TARGET_PWM  190       /* 匀加速模式目标最高速度（低于普通模式）*/

/* 分段加速：4段式，避免低速编码器死区 */
#define ACCEL_TOTAL_MS    3000      /* 加速总时间 (ms) */
#define ACCEL_PWM1        60        /* 第1段终点PWM (死区出口) */
#define ACCEL_T1_MS       600       /* 第1段时间 (ms) */
#define ACCEL_PWM1_5      95        /* 第2段终点PWM (中间值) */
#define ACCEL_T2_MS       600       /* 第2段时间 (ms) */
#define ACCEL_PWM2        130       /* 第3段终点PWM */
#define ACCEL_T3_MS       600       /* 第3段时间 (ms) */
/* 第4段: ACCEL_PWM2 → ACCEL_TARGET_PWM, 时间 = ACCEL_TOTAL_MS - T1 - T2 - T3 */
#define DECEL_TOTAL_MS    4000      /* 减速前2段时间 (ms) */
#define DECEL_T3_MS       1000      /* 减速第3段：末端缓停 (ms) */

#define DECEL_MIN_PWM     65        /* 减速终点PWM (不降到0，编码器保持反馈) */
#define CRUISE_START_MS_DEFAULT  24000  /* 默认：从按下PG6到开始减速的时间 (ms) */
#define CRUISE_START_MS_MIN       5000  /* 最短开始减速时间 5s */
#define CRUISE_START_MS_MAX      60000  /* 最长开始减速时间 60s */
#define CRUISE_START_MS_STEP      1000  /* 每次按键调整步长 1s */

extern uint32_t cruise_start_ms;     /* 当前开始减速时间(可调) */

/* 匀加速匀减速状态机枚举 */
#define ACCEL_IDLE          0       /* 空闲，等待 PG6 触发 */
#define ACCEL_ACCELERATING  1       /* 匀加速中 */
#define ACCEL_CRUISING      2       /* 匀速巡航中 */
#define ACCEL_DECELERATING  3       /* 匀减速中 */
#define ACCEL_HOLD          4       /* 减速完成，保持最低速，等待循迹/角度停车 */

/* 十字路口起跑/终点检测 */

void MOTOR_Init(void);
void MOTORA_SetSpeed(int32_t Speed);
void MOTORB_SetSpeed(int32_t Speed);
void Control(void);
void delay_control(void);
void Motor_key_scan(void);          /* 按键扫描：按下翻转标志位 */
void PG7_key_scan(void);            /* PG7 按键扫描：匀加速模式切换 */
void PG8_key_scan(void);            /* PG8 按键扫描：开始减速时间+1s */
void PG11_key_scan(void);           /* PG11 按键扫描：开始减速时间-1s */
#endif

