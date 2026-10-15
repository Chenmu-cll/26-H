//
// Created by qwe15 on 2025/10/11.
//

#ifndef CAR_GWIIC_H
#define CAR_GWIIC_H

#include "main.h"

// 0=I2C  1=serial  2=74HC165  3=IO direct (Yahboom)
#define USE_Protocol  3

#if USE_Protocol==0
/* ---- I2C ---- */
void GW_GRAY_Init(void);
int8_t GW_GRAY_DigtalDatas_Read(void);
void GW_GRAY_DigtalDatas_Split(void);
uint8_t GW_digital_Read_Bit(uint8_t Num);
void GW_GRAY_AnalogDatas_Read(void);
extern uint8_t gray_sensor[8];
extern uint8_t digital_data;
extern uint8_t analog_data[8];
extern uint8_t i2c_scan_count;
extern uint8_t i2c_addr[4];
extern uint8_t i2c_active_idx;
extern uint8_t i2c_read_ok;

#elif USE_Protocol==1
/* ---- serial ---- */
void GW_GRAY_Init(void);
void GW_GRAY_DigtalDatas_Read(void);
extern uint8_t digital_data;

#elif USE_Protocol==2
/* ---- 74HC165 ---- */
void GW_GRAY_Init(void);
void GW_GRAY_DigtalDatas_Read(void);
extern uint8_t digital_data;

#else
/* ---- IO direct (PE0-PE13) ---- */
void GW_GRAY_Init(void);
void GW_GRAY_DigtalDatas_Read(void);
extern uint8_t digital_data;

#endif
#endif /* CAR_GWIIC_H */