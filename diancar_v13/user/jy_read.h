#ifndef __JY_READ_H
#define __JY_READ_H

#include "main.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include "usart.h"
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
void jy_read_init(void);
void uart_send(uint8_t *data,uint32_t len);
void jy_Init(void);

extern float fAngle[3];
extern uint8_t rx_byte;

#endif


