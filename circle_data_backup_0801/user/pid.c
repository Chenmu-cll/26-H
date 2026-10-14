#include "pid.h"
#include "step_motor.h"

/* ========== 居中跟踪 PID 参数（从头调参，纯P起步）========== */

/* ---- 调试参数（非static，main.c可以引用）---- */
const float  TUNE_KP    =  0.7f;    /* P增益 */
const float  TUNE_KI    =  0.054f;    /* 积分 */
const float  TUNE_KD    =  0.5f;   /* 微分 */
const uint16_t TUNE_SPEED = 100;    /* RPM */
const uint8_t  TUNE_ACC   = 300;    /* 加速度 */

/* 全局 PID 实例，main.c 中使用 */
PID ball_pid;
uint8_t ball_direction;  /* 0=CW, 1=CCW */

/**
 *@brief  PID初始化（位置式，带deltaT）
 *@param  pid    PID结构体
 *@param  Kp     比例增益
 *@param  Ki     积分增益
 *@param  Kd     微分增益
 *@param  target 目标值
 *@param  max    输出上限
 *@param  min    输出下限
 */
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float target, float max, float min)
{
	pid->Kp   = Kp;
	pid->Ki   = Ki;
	pid->Kd   = Kd;
	pid->Actual   = 0;
	pid->Error0   = 0;
	pid->err_int  = 0;
	pid->err_prev = 0;
	pid->Target   = target;
	pid->out      = 0;
	pid->max      = max;
	pid->min      = min;
	pid->first_run = 1;   // 首次运行位置位，首次调用时种子历史
}

/**
 *@brief  位置式PID计算
 *@param  test  PID结构体
 *@retval  None（结果写入 test->out）
 *
 *@note   deltaT = 0.04s (40ms)，由主循环时序保证。
 *        首次调用时种子 err_prev = err，使微分为0，避免启动尖峰。
 *        积分用梯形法则，输出和积分均限幅到 [min, max]。
 */
void PID_Pro(PID *test)
{
	float err = test->Target - test->Actual;
	float deltaT = 0.04f;  // 40ms 控制周期

	/* 首次运行：种子历史，避免微分尖峰 */
	if (test->first_run) {
		test->err_prev  = err;      // 种子 → 本次微分为 0
		test->err_int   = 0.0f;     // 积分从零开始
		test->first_run = 0;
	}

	/* 微分项：d(err)/dt */
	float err_dev = (err - test->err_prev) / deltaT;

	/* 积分项：梯形法则累加；Ki=0时清零，防止无效累积分饱和 */
	if (test->Ki == 0.0f) {
		test->err_int = 0.0f;
	} else {
		test->err_int = test->err_int + (err + test->err_prev) * deltaT * 0.5f;
	}

	/* PID 输出 */
	float CO = test->Kp * err + test->Ki * test->err_int + test->Kd * err_dev;

	/* 输出限幅 */
	if (CO > test->max) CO = test->max;
	if (CO < test->min) CO = test->min;

	/* 积分限幅（抗积分饱和），与输出限幅同范围 */
	if (test->err_int > test->max) test->err_int = test->max;
	if (test->err_int < test->min) test->err_int = test->min;

	/* 更新状态 */
	test->Error0   = err;
	test->err_prev = err;
	test->out      = CO;
}

/**
 *@brief  应用当前增益和限幅（暂不分远近区，纯P调试）
 */
void Ball_ApplyGains(PID *pid, float err_abs)
{
	(void)err_abs;  /* 暂不使用，后续可在此分远近区 */
	pid->Kp = TUNE_KP;
	pid->Ki = TUNE_KI;
	pid->Kd = TUNE_KD;
	pid->max = CO_MAX;
	pid->min = CO_MIN;
}

/**
 *@brief  居中跟踪控制单步执行
 *@param  offset  摄像头返回的居中偏移量（有符号：正=球偏右，负=球偏左）
 *
 *  内置 20ms 节流：避免 1ms 循环狂发指令导致电机堵转
 */
void Ball_Control_Once(int32_t offset)
{
	/* 节流：每 20ms 只发一次电机指令 */
	{
		static uint32_t last_motor_tick = 0;
		extern volatile uint32_t uwTick;
		if (uwTick - last_motor_tick < 20) return;
		last_motor_tick = uwTick;
	}

	Ball_ApplyGains(&ball_pid, fabsf((float)offset));

	ball_pid.Actual = (float)offset;
	PID_Pro(&ball_pid);                           /* CO = Kp*(-offset) + ... */

	ball_direction = (ball_pid.out > 0.0f) ? 1 : 0;
	Motor_Pos_Control_X(ball_direction, TUNE_SPEED, TUNE_ACC,
	                    (uint32_t)(fabsf(ball_pid.out)));
}
