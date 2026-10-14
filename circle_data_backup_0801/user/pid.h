#ifndef __PID_H
#define __PID_H
#include <math.h>
#include <stdint.h>

/* 输出限幅（脉冲数）—— 调试期保守值，后续可按需放大 */
#define CO_MAX    2000.0f
#define CO_MIN   -2000.0f

/* 调参变量（定义在 pid.c，main.c 可引用） */
extern const float  TUNE_KP;
extern const float  TUNE_KI;
extern const float  TUNE_KD;
extern const uint16_t TUNE_SPEED;
extern const uint8_t  TUNE_ACC;

typedef struct
{
	float Target;       // 设定值 SP（摄像头中心对应的位置）
	float Actual;       // 反馈值 FB（摄像头返回的绝对位置）
	float Kp;           // 比例系数
	float Ki;           // 积分系数
	float Kd;           // 微分系数
	float Error0;       // 当前误差 = Target - Actual
	float err_int;      // 积分累加器
	float err_prev;     // 上次的误差 err[k-1]
	float out;          // PID 输出 CO
	float max;          // 输出上限
	float min;          // 输出下限
	uint8_t first_run;  // 首次运行标志
}PID;

/**
 *@brief  PID初始化（位置式，带deltaT）
 *@param pid    PID结构体
 *@param Kp     比例增益
 *@param Ki     积分增益
 *@param Kd     微分增益
 *@param target 目标值
 @param max    输出上限
 @param min    输出下限
*/
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float target, float max, float min);

/**
 *@brief 位置式PID计算
 *@param test  PID结构体
 *@retval  None（结果写入 test->out）
*/
void PID_Pro(PID *test);

/**
 *@brief  根据误差大小切换增益 + 重写限幅
 *@param  pid      PID结构体
 *@param  err_abs  误差绝对值 |Target - Actual|
*/
void Ball_ApplyGains(PID *pid, float err_abs);

/**
 *@brief  居中跟踪控制单步执行（位置模式：PID输出→脉冲数）
 *@param  offset  摄像头返回的居中偏移量（有符号：正=球偏右，负=球偏左，目标=0）
*/
void Ball_Control_Once(int32_t offset);

#endif
