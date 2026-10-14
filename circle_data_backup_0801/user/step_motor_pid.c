#include "step_motor_pid.h"

volatile LaserCoordTypeDef latest_red_laser_coord;

pid_t motor_pid;
u32 last_pid_tick = 0;

// ===== 直接传入误差的 PID（摄像头已算好误差，无需 set - get） =====
u32 last_pid_err_tick = 0;
float pid_calc_error(pid_t *pid, float error)
{
	if(uwTick - last_pid_err_tick >= 20)
	{
		last_pid_err_tick = uwTick;

		// 1. 误差直接使用摄像头传来的值
		pid->err[2] = error;

		// 2. P, 比例
		pid->pout = pid->kp * pid->err[2];
//		// 3. I, 积分（累加）
//		pid->iout += pid->ki * pid->err[2];
//		if(pid->iout > pid->integral_limit) pid->iout = pid->integral_limit;
//		if(pid->iout < -pid->integral_limit) pid->iout = -pid->integral_limit;

//		if (fabs(pid->err[2]) < 2.0f)
//		{
//	        	 pid->iout *= 0.9f;  // 衰减积分
//		}
		// 4. D, 微分(误差变化率)
		pid->dout = pid->kd * (pid->err[2] - pid->err[1]);
		// 5. 求和 + 输出限幅
		pid->out = pid->pout + pid->iout + pid->dout;
		if(pid->out > pid->max_out) pid->out = pid->max_out;
		if(pid->out < -pid->max_out) pid->out = -pid->max_out;
		// 6. 误差历史位移
		pid->err[0] = pid->err[1];
		pid->err[1] = pid->err[2];
	}
	return pid->out;
}

//初始化
void pid_init(void)
{
	memset(&motor_pid, 0, sizeof(motor_pid));

	motor_pid.kp = 0.0f;     /* 外层P：球误差→角度偏移（越大杆越敢倾） */
	motor_pid.ki = 0.0f;     /* 暂不使能积分 */
	motor_pid.kd = 0.15f;    /* 外层D：球速→角度阻尼 */
	motor_pid.integral_limit = 0.0f;
	motor_pid.max_out = 30.0f;  /* 最大角度偏移(~30°) */
}

