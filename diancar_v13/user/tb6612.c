#include "tb6612.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>   /* abs() */
#include "PID.h"
#include "jy_turn.h"

/* ========== 融合权重：SPEED_W + TRACK_W = 1.0 ========== */
#define SPEED_W        0.45f  // 普通模式：速度环权重（降低让循迹更灵敏）
#define TRACK_W        0.55f  // 普通模式：循迹环权重

/* 匀加速匀减速模式：速度环主导，闭环跟踪斜坡目标 */
#define ACCEL_SPEED_W  0.65f  // 加速/减速时速度环权重
#define ACCEL_TRACK_W  0.35f  // 加速/减速时循迹环权重

int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}

float Velocity_Left,Velocity_Right;

// TB6612和PWM初始化
void MOTOR_Init()
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

}

// A电机控制
void MOTORA_SetSpeed(int32_t Speed)
{
     //右电机: pa
    if (Speed < 0) {            // 反转
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {                    // 正转 / 停止
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abs(Speed));

}

// B电机控制
void MOTORB_SetSpeed(int32_t Speed)
{
     /* 左电机: pb */
    if (Speed > 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, abs(Speed));
}
int PWM_yaw; 				   //差速值
int PWM0, PWML, PWMR;		   //PWM值，左轮值，右轮值

uint8_t motor_enabled = 0;          /* 电机使能标志位，按键翻转 */
static uint32_t key_last_tick = 0;  /* 按键消抖计时 */
static uint8_t  key_last_state = 1; /* 上次按键状态：1=松开 */

/* 比赛计时变量 */
uint32_t race_start_tick = 0;       /* 计时开始时刻(ms) */
uint32_t race_elapsed_ms = 0;       /* 最终用时(ms) */
uint8_t  race_timing = 0;           /* 计时状态：1=计时中，0=停止 */

/* ===== 匀加速匀减速模式变量 ===== */
uint8_t accel_mode = 0;              /* 0=普通模式, 1=匀加速匀减速模式 */
uint8_t accel_state = ACCEL_IDLE;    /* 当前流程状态 */
uint32_t cruise_start_ms = CRUISE_START_MS_DEFAULT; /* 开始减速时间(可调) */
static uint32_t accel_start_tick = 0; /* 流程启动时刻 (uwTick) */
static float accel_start_yaw = 0;    /* PG6按下时的航向角 */

/* 停车检测：PID.c track_line() case 0xE1 直接停车 */

void Control(void)
{
    ///* JY901S 检测到转完一圈 → 停车 */
    //if (circle_complete) {
    //    motor_enabled = 0;
    //    race_timing = 0;
    //    race_elapsed_ms = uwTick - race_start_tick;
    //}

    if (!motor_enabled) {
        PWM0 = 0;
        PWML = 0;
        PWMR = 0;
        /* 清空PID状态，防止下次启动时积分残留导致飞车 */
        PID_left.err_sum = 0;
        PID_left.output = 0;
        PID_right.err_sum = 0;
        PID_right.output = 0;
        MOTORA_SetSpeed(0);
        MOTORB_SetSpeed(0);
        return;
    }

    /* ===== 1. 编码器已由主循环读取到 encoder_left/right ===== */

    /* ===== 2. 匀加速匀减速：根据流程阶段计算 PWM0 ===== */
    if (accel_mode) {
        uint32_t elapsed = uwTick - accel_start_tick;

        if (accel_state == ACCEL_ACCELERATING) {
            /* 分段加速：4段不同斜率，避免低速编码器死区 */
            uint32_t t4_ms = ACCEL_TOTAL_MS - ACCEL_T1_MS - ACCEL_T2_MS - ACCEL_T3_MS;
            if (elapsed >= ACCEL_TOTAL_MS) {
                accel_state = ACCEL_CRUISING;
                PWM0 = ACCEL_TARGET_PWM;
            } else if (elapsed < ACCEL_T1_MS) {
                /* 第1段：0 → ACCEL_PWM1 */
                PWM0 = (uint32_t)ACCEL_PWM1 * elapsed / ACCEL_T1_MS;
            } else if (elapsed < ACCEL_T1_MS + ACCEL_T2_MS) {
                /* 第2段：ACCEL_PWM1 → ACCEL_PWM1_5 */
                uint32_t t = elapsed - ACCEL_T1_MS;
                PWM0 = ACCEL_PWM1 + (uint32_t)(ACCEL_PWM1_5 - ACCEL_PWM1) * t / ACCEL_T2_MS;
            } else if (elapsed < ACCEL_T1_MS + ACCEL_T2_MS + ACCEL_T3_MS) {
                /* 第3段：ACCEL_PWM1_5 → ACCEL_PWM2 */
                uint32_t t = elapsed - ACCEL_T1_MS - ACCEL_T2_MS;
                PWM0 = ACCEL_PWM1_5 + (uint32_t)(ACCEL_PWM2 - ACCEL_PWM1_5) * t / ACCEL_T3_MS;
            } else {
                /* 第4段：ACCEL_PWM2 → ACCEL_TARGET_PWM */
                uint32_t t = elapsed - ACCEL_T1_MS - ACCEL_T2_MS - ACCEL_T3_MS;
                PWM0 = ACCEL_PWM2 + (uint32_t)(ACCEL_TARGET_PWM - ACCEL_PWM2) * t / t4_ms;
            }
        }

        if (accel_state == ACCEL_CRUISING) {
            if (elapsed >= cruise_start_ms) {
                accel_state = ACCEL_DECELERATING; /* 进入减速 */
            }
            PWM0 = ACCEL_TARGET_PWM;  /* 匀速保持 */
        }

        if (accel_state == ACCEL_DECELERATING) {
            /* 分段减速：190→130→50→50，3段，末端缓停1秒 */
            uint32_t decel_elapsed = elapsed - cruise_start_ms;
            uint32_t t1_ms = DECEL_TOTAL_MS * (ACCEL_TARGET_PWM - ACCEL_PWM2) / (ACCEL_TARGET_PWM - DECEL_MIN_PWM);
            uint32_t t2_ms = DECEL_TOTAL_MS - t1_ms;

            if (decel_elapsed >= DECEL_TOTAL_MS + DECEL_T3_MS) {
                accel_state = ACCEL_HOLD;
                PWM0 = DECEL_MIN_PWM;
            } else if (decel_elapsed < t1_ms) {
                /* 第1段：ACCEL_TARGET_PWM → ACCEL_PWM2 */
                PWM0 = ACCEL_TARGET_PWM - (uint32_t)(ACCEL_TARGET_PWM - ACCEL_PWM2) * decel_elapsed / t1_ms;
            } else if (decel_elapsed < t1_ms + t2_ms) {
                /* 第2段：ACCEL_PWM2 → DECEL_MIN_PWM */
                uint32_t t = decel_elapsed - t1_ms;
                PWM0 = ACCEL_PWM2 - (uint32_t)(ACCEL_PWM2 - DECEL_MIN_PWM) * t / t2_ms;
            } else {
                /* 第3段：DECEL_MIN_PWM 保持1秒，末端缓停 */
                PWM0 = DECEL_MIN_PWM;
            }
        }

        if (accel_state == ACCEL_HOLD) {
            PWM0 = DECEL_MIN_PWM;  /* 保持最低速，由循迹/角度逻辑停车 */
        }
    } else {
        PWM0 = TARGET_PWM0;  /* 普通模式：固定基础速度 */
    }

    /* ===== 3. 航向角回归检测 → 停车（普通±30°, 匀加速±15°）===== */
    if (motor_enabled && (uwTick - race_start_tick) >= 15000) {
        float yaw_diff = fAngle[2] - accel_start_yaw;
        float yaw_threshold = accel_mode ? 15.0f : 23.0f;
        /* 角度环绕处理 */
        if (yaw_diff >  180.0f) yaw_diff -= 360.0f;
        if (yaw_diff < -180.0f) yaw_diff += 360.0f;
        if (yaw_diff >= -yaw_threshold && yaw_diff <= yaw_threshold) {
            motor_enabled = 0;
            accel_state = ACCEL_IDLE;
            race_timing = 0;
            race_elapsed_ms = uwTick - race_start_tick;
        }
    }

    /* ===== 4. 循迹环：计算差速 (PWM0 ± 差速) ===== */
    PWM_yaw = yaw_PD_value(Yaw_measure, 0);

    /* 匀加速模式：差速等比缩放，加速时两轮同步攀升 */
    if (accel_mode && PWM0 > 0 && ACCEL_TARGET_PWM > 0) {
        PWM_yaw = PWM_yaw * PWM0 / ACCEL_TARGET_PWM;
    }

    /* 限幅：差速不能超过基础速度，保证起步时两轮始终同向 */
    if (PWM_yaw >  PWM0) PWM_yaw =  PWM0;
    if (PWM_yaw < -PWM0) PWM_yaw = -PWM0;

    int32_t track_left  = PWM0 + PWM_yaw;   // 循迹环左轮目标
    int32_t track_right = PWM0 - PWM_yaw;   // 循迹环右轮目标

    /* ===== 5. 速度环：以循迹环目标作为输入，编码器反馈 ===== */
    PID_left.target_speed  = track_left;
    PID_right.target_speed = track_right;
    SpeedPID_Calculate(&PID_left,  encoder_left);
    SpeedPID_Calculate(&PID_right, encoder_right);

    /* ===== 6. 融合输出 ===== */
    int32_t pwm_left  = (int32_t)(SPEED_W * PID_left.output  + TRACK_W * track_left);
    int32_t pwm_right = (int32_t)(SPEED_W * PID_right.output + TRACK_W * track_right);

    /* ===== 7. 驱动电机 ===== */
    MOTORA_SetSpeed(-pwm_right);
    MOTORB_SetSpeed(-pwm_left);
    PWML = pwm_left;
    PWMR = pwm_right;
}
uint32_t control_tick = 0;
void delay_control()
{
    if(uwTick - control_tick < 20)  //20ms控制周期
    {
        return;
    }
    control_tick = uwTick;
    Control();
}

/* PG6按键扫描：下降沿触发，消抖后翻转motor_enabled */
/* 普通模式：启停切换；匀加速模式：触发匀加速流程 */
#define KEY_PORT    GPIOG
#define KEY_PIN     GPIO_PIN_6

void Motor_key_scan(void)
{
    uint8_t key_now = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN);

    /* 按键未变化，重置计时 */
    if (key_now == key_last_state) {
        key_last_tick = uwTick;
        return;
    }

    /* 消抖时间 30ms */
    if (uwTick - key_last_tick < 30)
        return;

    key_last_state = key_now;

    /* 下降沿（松开→按下） */
    if (key_now == GPIO_PIN_RESET) {

        if (accel_mode) {
            /* ===== 匀加速匀减速模式 ===== */
            if (accel_state == ACCEL_IDLE) {
                /* 触发匀加速流程 */
                motor_enabled = 1;
                accel_start_tick = uwTick;
                accel_start_yaw = fAngle[2];  /* 记录起始航向角 */
                accel_state = ACCEL_ACCELERATING;
                race_start_tick = uwTick;
                race_timing = 1;
                race_elapsed_ms = 0;
            } else {
                /* 流程进行中按PG6 → 紧急停车 */
                motor_enabled = 0;
                accel_state = ACCEL_IDLE;
                race_timing = 0;
                race_elapsed_ms = uwTick - race_start_tick;
            }
        } else {
            /* ===== 普通模式（原逻辑）===== */
            motor_enabled = !motor_enabled;

            if (motor_enabled) {
                /* 电机启动 → 开始计时 */
                race_start_tick = uwTick;
                race_timing = 1;
                race_elapsed_ms = 0;
                accel_start_yaw = fAngle[2];  /* 记录起始航向角 */
            } else {
                /* 电机停止 → 结束计时 */
                race_timing = 0;
                race_elapsed_ms = uwTick - race_start_tick;
            }
        }
    }
}

/* PG7按键扫描：切换匀加速匀减速模式 */
#define PG7_PORT    GPIOG
#define PG7_PIN     GPIO_PIN_7

static uint32_t pg7_last_tick = 0;
static uint8_t  pg7_last_state = 1;

void PG7_key_scan(void)
{
    uint8_t key_now = HAL_GPIO_ReadPin(PG7_PORT, PG7_PIN);

    /* 按键未变化，重置计时 */
    if (key_now == pg7_last_state) {
        pg7_last_tick = uwTick;
        return;
    }

    /* 消抖时间 30ms */
    if (uwTick - pg7_last_tick < 30)
        return;

    pg7_last_state = key_now;

    /* 下降沿（松开→按下）：切换模式 */
    if (key_now == GPIO_PIN_RESET) {
        accel_mode = !accel_mode;

        /* 切换模式时强制停车，防止状态混乱 */
        motor_enabled = 0;
        accel_state = ACCEL_IDLE;
        race_timing = 0;
        PWM0 = 0;
    }
}

/* PG8按键扫描：开始减速时间 +1s */
#define PG8_PORT    GPIOG
#define PG8_PIN     GPIO_PIN_8

static uint32_t pg8_last_tick = 0;
static uint8_t  pg8_last_state = 1;

void PG8_key_scan(void)
{
    uint8_t key_now = HAL_GPIO_ReadPin(PG8_PORT, PG8_PIN);

    if (key_now == pg8_last_state) {
        pg8_last_tick = uwTick;
        return;
    }

    if (uwTick - pg8_last_tick < 30)
        return;

    pg8_last_state = key_now;

    if (key_now == GPIO_PIN_RESET) {
        if (cruise_start_ms + CRUISE_START_MS_STEP <= CRUISE_START_MS_MAX) {
            cruise_start_ms += CRUISE_START_MS_STEP;
        }
    }
}

/* PG11按键扫描：开始减速时间 -1s */
#define PG11_PORT   GPIOG
#define PG11_PIN    GPIO_PIN_11

static uint32_t pg11_last_tick = 0;
static uint8_t  pg11_last_state = 1;

void PG11_key_scan(void)
{
    uint8_t key_now = HAL_GPIO_ReadPin(PG11_PORT, PG11_PIN);

    if (key_now == pg11_last_state) {
        pg11_last_tick = uwTick;
        return;
    }

    if (uwTick - pg11_last_tick < 30)
        return;

    pg11_last_state = key_now;

    if (key_now == GPIO_PIN_RESET) {
        if (cruise_start_ms >= CRUISE_START_MS_STEP + CRUISE_START_MS_MIN) {
            cruise_start_ms -= CRUISE_START_MS_STEP;
        }
    }
}
