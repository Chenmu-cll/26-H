#ifndef __BSP_H
#define __BSP_H
#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "gwIIC.h"
#include "jy_read.h"
#include "delay.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "oled.h"
#include "tim.h"
#include "tb6612.h"
#include "PID.h"
#include "jy_turn.h"
#include "tim.h"
extern uint8_t gray_sensor[8];
extern uint8_t analog_data[8];
extern int16_t sReg[];
extern uint8_t rx_byte;
extern float Yaw_measure;
extern int PWM0, PWML, PWMR;		   
#endif

