//
// Created by qwe15 on 2025/9/17.
//

#ifndef _PID_H
#define _PID_H
#include <stdint.h>
//循迹环
extern int PWM0, PWML, PWMR;
typedef struct
{
    float TKp,TKi,TKd;		//PID系数
    float err;				//误差（测量值减去目标值）P的倍数【比例】
    float err_sum;			//误差的累加（I的倍数）
    float err_difference;	//误差的差值（D的倍数）
    float last_err;			//上一次误差
}PID;

void track_line(void);
extern float Yaw_measure;             //转向环测量值
int yaw_PD_value(float yaw_measure,float yaw_calcu); //测量的，目标的

//速度环
typedef struct
{
    float Kp,Ki,Kd;		    //PID系数
    int16_t target_speed;	//目标速度 (编码器脉冲数/周期)
    int16_t current_speed;	//当前速度
    int16_t err;			//误差（测量值减去目标值）P的倍数【比例】
    int16_t err_last;		//上一次误差
    int16_t err_sum;		//误差的累加（I的倍数）
    int16_t output;			//PID输出值pwm
    int16_t output_max;		//输出限幅
} SpeedPID_t;

extern SpeedPID_t PID_left;   //左轮速度环
extern SpeedPID_t PID_right;  //右轮速度环

/* TODO: 编码器速度变量 —— 接入编码器后由中断/定时器更新 */
extern int16_t encoder_left;   //左轮实测速度（脉冲/周期）
extern int16_t encoder_right;  //右轮实测速度（脉冲/周期）

void speed_pid_init(SpeedPID_t *pid, float Kp, float Ki, float Kd, int16_t output_max);
void SpeedPID_Calculate(SpeedPID_t *pid, int16_t real_speed);
void SpeedPID_5ms(void);

#endif
