#include "step_motor.h"

int32_t posx = 0;

/* ========== 位置模式：Emm V5 位置指令 (0xFD)，移植自视觉跟踪 ========== */

/**
  * @brief  位置模式控制（相对位置）
  * @param  dir ：方向       ，0=CW，1=CCW
  * @param  vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param  acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param  clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @note   通过 USART2 发送，raF=1(相对位置)，snF=0(不同步)
  *         协议: addr=0x02, func=0xFD, dir, vel_H, vel_L, acc, clk[4], raF=1, snF=0, 0x6B
  */
void Motor_Pos_Control_X(uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk)
{
    uint8_t cmd[13];

    cmd[0]  = 0x02;                      /* 电机地址 */
    cmd[1]  = 0xFD;                      /* 功能码：位置模式 */
    cmd[2]  = dir;                       /* 方向 */
    cmd[3]  = (uint8_t)(vel >> 8);       /* 速度(RPM)高8位 */
    cmd[4]  = (uint8_t)(vel >> 0);       /* 速度(RPM)低8位 */
    cmd[5]  = acc;                       /* 加速度 */
    cmd[6]  = (uint8_t)(clk >> 24);      /* 脉冲数(bit24 - bit31) */
    cmd[7]  = (uint8_t)(clk >> 16);      /* 脉冲数(bit16 - bit23) */
    cmd[8]  = (uint8_t)(clk >> 8);       /* 脉冲数(bit8  - bit15) */
    cmd[9]  = (uint8_t)(clk >> 0);       /* 脉冲数(bit0  - bit7 ) */
    cmd[10] = 0x01;                      /* raF=1: 相对位置运动 */
    cmd[11] = 0x00;                      /* snF=0: 不启用多机同步 */
    cmd[12] = 0x6B;                      /* 校验字节 */

    HAL_UART_Transmit(&huart2, cmd, 13, 100);
}

/* ====== 统一状态机（发速度/读位置互斥，不同时跑） ====== */
typedef enum {
    ST_IDLE = 0,
    ST_SEND_SPEED,
    ST_SEND_POS_CMD,
    ST_WAIT_POS_RESP,
} motor_state_t;

static motor_state_t state          = ST_IDLE;
static uint32_t last_speed_tick     = 0;
static uint32_t last_pos_tick       = 0;
static uint32_t resp_timeout_tick   = 0;
static int16_t  cached_speed        = 0;

/* ---------- 发送速度指令 ---------- */
static void Motor_SetSpeed_X(int16_t speed_rpm)
{
    uint8_t dir = 0;
    uint16_t abs_speed;

    if (speed_rpm > 0) {
        dir = 0x00;
        abs_speed = speed_rpm;
    } else if (speed_rpm < 0) {
        dir = 0x01;
        abs_speed = -speed_rpm;
    } else {
        abs_speed = 0;
    }

    uint8_t cmd[8];
    cmd[0] = 0x02;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (abs_speed >> 8) & 0xFF;
    cmd[4] = abs_speed & 0xFF;
    cmd[5] = 1000;
    cmd[6] = 0x00;
    cmd[7] = 0x6B;

    HAL_UART_Transmit(&huart2, cmd, 8, 100);
}

/* ---------- 解析位置响应 ---------- */
static uint8_t parse_position_response(void)
{
    uint16_t local_rx_index;
    uint8_t  local_buf[128];

    __disable_irq();
    local_rx_index = rx_index;
    memcpy(local_buf, uart_rx_buffer, local_rx_index);
    rx_index = 0;
    __enable_irq();

    uint16_t i;
    for (i = 0; i < local_rx_index; i++) {
        if (local_buf[i] == 0x02) {
            if (i + 7 >= local_rx_index) continue;
            if (local_buf[i + 7] == 0x6B) {
                uint32_t raw_pulse = 0;
                raw_pulse |= (uint32_t)local_buf[i + 3] << 24;
                raw_pulse |= (uint32_t)local_buf[i + 4] << 16;
                raw_pulse |= (uint32_t)local_buf[i + 5] << 8;
                raw_pulse |= local_buf[i + 6];

                if (local_buf[i + 2] == 0x01)
                    posx = -(int32_t)raw_pulse;
                else
                    posx = (int32_t)raw_pulse;

                return 1;
            }
        }
    }
    return 0;
}

/* ===================================================================
 * step_motor_poll() —— 统一状态机，同一时刻只做一件事
 *   IDLE → 判断下一步：读位置(优先) 或 发速度
 *   SEND_SPEED → 发完立刻回 IDLE
 *   SEND_POS_CMD → 发完读位置命令 → 等响应
 *   WAIT_POS_RESP → 收到数据解析 / 超时放弃 → 回 IDLE
 * =================================================================== */
void step_motor_poll(float speed_rpm_x)
{
    switch (state) {

    case ST_IDLE:
        /* 优先级1：读位置（20ms） */
        if (uwTick - last_pos_tick >= 20) {
            state = ST_SEND_POS_CMD;
        }
        /* 优先级2：发速度（10ms） */
        else if (uwTick - last_speed_tick >= 10) {
            cached_speed = (int16_t)(speed_rpm_x >= 0.0f
                                     ? (speed_rpm_x + 0.5f)
                                     : (speed_rpm_x - 0.5f));
            state = ST_SEND_SPEED;
        }
        break;

    /* ---- 发速度（~1ms，发完立刻走） ---- */
    case ST_SEND_SPEED:
        if (g_latest_error_x == 999) {
            Motor_SetSpeed_X(0);
        } else if (latest_red_laser_coord.isValid == 1) {
            Motor_SetSpeed_X(cached_speed);
        } else if (latest_red_laser_coord.isValid == 0) {
            if (latest_red_laser_coord.x == 0)
                Motor_SetSpeed_X(cached_speed);
            else
                Motor_SetSpeed_X(0);
        }
        last_speed_tick = uwTick;
        state = ST_IDLE;
        break;

    /* ---- 发送读位置命令 ---- */
    case ST_SEND_POS_CMD: {
        uint8_t pos_cmd[] = {0x02, 0x36, 0x6B};
        rx_index = 0;
        HAL_UART_Transmit(&huart2, pos_cmd, 3, 20);
        last_pos_tick     = uwTick;
        resp_timeout_tick = uwTick;
        state = ST_WAIT_POS_RESP;
        break;
    }

    /* ---- 等待响应 / 超时 ---- */
    case ST_WAIT_POS_RESP:
        if (rx_index >= 8) {
            parse_position_response();
            state = ST_IDLE;
        } else if (uwTick - resp_timeout_tick > 20) {
            rx_index = 0;
            state = ST_IDLE;
        }
        break;
    }
}
