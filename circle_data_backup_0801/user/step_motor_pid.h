#ifndef __STEP_MOTOR_PID_H
#define __STEP_MOTOR_PID_H

#include "bsp.h"
// 定义一个坐标结构体
typedef struct
{
    int x;      // X轴坐标
    int y;      // Y轴坐标
    uint8_t isValid;  // 有效标志：1=有效，0=无效
} LaserCoordTypeDef;
extern volatile LaserCoordTypeDef latest_red_laser_coord;
//pid
typedef struct{
	float kp,ki,kd;     		//三个增益
	float set,get;   				//目标值，实际值
	float err[3];    			  // err[0]上上次，err[1]上次，err[2]本次
	float pout,iout,dout;   // P/I/D各项
	float out;							//总输出
	float integral_limit;   //积分限幅
	float max_out;       		//输出限幅
}pid_t;

extern pid_t motor_pid;
float pid_calc_error(pid_t *pid, float error);        // 直接传入误差（摄像头已算好）
void pid_init(void);


#endif
