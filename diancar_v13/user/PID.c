//
// Created by qwe15 on 2025/9/17.
//
#include "stm32f4xx_hal.h"
#include "PID.h"
#include "gwIIC.h"
#include "tim.h"
#include "tb6612.h"


float Yaw_measure;
PID Tpid={28, 0, 0, 0, 0, 0, 0};	//转向环: Kp=28

void track_line(void)
{
    uint8_t d = digital_data;

    switch (d) {
        //===== 线在正中 =====
        case 0xE7: // 11100111 直线（s4s5黑）
            Yaw_measure = 0;
            break;
        //===== 极小幅度修正 =====
        case 0xEF: // 11101111
            Yaw_measure = +0.4f;
            break;
        case 0xF7: // 11110111
            Yaw_measure = -0.4f;
            break;
        //===== 小幅度修正 ±1.2 =====
        case 0xC7: // 11000111 s3-s5
        case 0xCF: // 11001111
            Yaw_measure = +1.2f;
            break;
        case 0xF3: // 11110011
        case 0xE3: // 11100011 s4-s6
            Yaw_measure = -1.2f;
            break;
        //===== 小-中修正 ±1.9 =====
        case 0x87: // 10000111 s2-s5
        case 0xDF: // 11011111
            Yaw_measure = +1.9f;
            break;
        case 0xBF: // 10111111 仅s3
            Yaw_measure = +1.9f;
            break;
        case 0xE1: // 11100001 s4-s7
            Yaw_measure = -1.9f;
            break;
        case 0xFB: // 11111011
        case 0xFD: // 11111101 仅s7
            Yaw_measure = -1.9f;
            break;
        //===== 中幅度修正 ±2.2 =====
        case 0x9F: // 10011111 s3-s5
        case 0x1F: // 00011111
        case 0x83: // 10000011 s2-s6
            Yaw_measure = +2.2f;
            break;
        case 0xF9: // 11111001 s5-s7
        case 0xF8: // 11111000
        case 0xC1: // 11000001 s3-s7
            Yaw_measure = -2.2f;
            break;
        //===== 大幅度修正 ±3.7 =====
        case 0x3F: // 00111111 s1-4
        case 0x7F: // 01111111 s1-3
            Yaw_measure = +3.7f;
            break;
        case 0xFC: // 11111100 s7s8
        case 0xFE: // 11111110 仅s8
        case 0xE0: // 11100000 s4-s8
        case 0xC0: // 11000000 s3-s8
            Yaw_measure = -3.7f;
            break;
        //===== 极限修正 ±4.3 =====
        case 0x07: // 00000111 s1-s7
        case 0x03: // 00000011 s1-s6
            Yaw_measure = +4.3f;
            break;
        //===== 丢线 =====
        default:
            break;
    }

}
int yaw_PD_value(float yaw_measure,float yaw_calcu)//测量的，目标的
{
    Tpid.err = yaw_measure - yaw_calcu;    											//误差
    float out;

    if(Tpid.err> 180) Tpid.err = Tpid.err - 360;                   					//不是舍弃误差，而是转换一下 （对角度进行处理）
    if(Tpid.err<-180) Tpid.err = Tpid.err + 360;

    /* 积分抗饱和：误差过零时清积分，防止积分滞后放大振荡 */
    if ((Tpid.err > 0 && Tpid.last_err < 0) || (Tpid.err < 0 && Tpid.last_err > 0))
        Tpid.err_sum = 0;

    Tpid.err_sum += Tpid.err;														//误差累加

    /* 积分限幅：防止积分项过大导致不可控 */
    if (Tpid.err_sum >  80) Tpid.err_sum =  80;
    if (Tpid.err_sum < -80) Tpid.err_sum = -80;

    Tpid.err_difference = Tpid.err - Tpid.last_err;									//转向偏差
    Tpid.last_err = Tpid.err;                                     					//此次误差记录为"上次误差"
    out=Tpid.TKp*Tpid.err + Tpid.TKi*Tpid.err_sum + Tpid.TKd*Tpid.err_difference;	//P*（误差）+ I*误差的累加 + D*误差的差值

    if(out>400)																	//限幅
    {
        out=400;
    }else if(out<-400)
    {
        out=-400;
    }

    return out;
}

//-----------------------------------------------------------速度环------------------------------------------------------------------------------------------------
SpeedPID_t PID_left;
SpeedPID_t PID_right;

int16_t encoder_left = 0;
int16_t encoder_right = 0;

void speed_pid_init(SpeedPID_t *pid, float Kp, float Ki, float Kd, int16_t output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target_speed = 0;
    pid->current_speed = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->err_sum = 0;
    pid->output = 0;
    pid->output_max = output_max;
}

void SpeedPID_Calculate(SpeedPID_t *pid, int16_t real_speed)
{
    pid->current_speed = real_speed;
    pid->err = pid->target_speed - pid->current_speed;

    if (real_speed == 0 && pid->target_speed != 0) {
        pid->err_sum = 0;
    }
    if ((pid->err > 0 && pid->err_last < 0) || (pid->err < 0 && pid->err_last > 0))
        pid->err_sum = 0;

    int16_t sep_threshold = pid->output_max / 3;
    if (sep_threshold < 10) sep_threshold = 10;
    if (pid->err < sep_threshold && pid->err > -sep_threshold)
    {
        pid->err_sum += pid->err;
    }
    else
    {
        pid->err_sum = 0;
    }
    if (pid->err_sum >  80) pid->err_sum =  80;
    if (pid->err_sum < -80) pid->err_sum = -80;

    int16_t err_difference = pid->err - pid->err_last;
    pid->output = (int16_t)(pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * err_difference);
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }
    pid->err_last = pid->err;
}

void SpeedPID_5ms(void)
{
    /* no-op */
}
