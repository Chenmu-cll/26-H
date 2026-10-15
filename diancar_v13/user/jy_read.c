#include "jy_read.h"
uint8_t rx_byte;
// 全局变量：存储三个角度
float fAngle[3];

// 数据更新回调函数（SDK自动调用）
void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
	// 读取角度数据 Roll Pitch Yaw
    for(i = 0; i < 3; i++)
    {
        fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
    }
}
// 串口2发送函数（给JY901用）
void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
    HAL_UART_Transmit(&huart5, p_data, uiSize, 1000);
}

void uart_send(uint8_t *data,uint32_t len)
{
	HAL_UART_Transmit(&huart5,data,len,100);
}

void jy_Init(void)
{
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);//sdk初始化配置，普通串口协议
  WitSerialWriteRegister(SensorUartSend);//sdk自动发送
	WitRegisterCallBack(CopeSensorData);//注册获取传感器数据回调函数
}

// 串口接收中断回调（在 stm32 中断处理函数中调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        
        HAL_UART_Receive_IT(&huart5, &rx_byte, 1);  // 重新开启接收
        WitSerialDataIn(rx_byte);//传回的数据经sdk解析
    }
}

