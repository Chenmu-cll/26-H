#include "usart_receive.h"

/* ====== DEBUG：在 Keil 中 Watch 这三个变量 ====== */
volatile uint32_t g_dma_half_cnt = 0;   // DMA 半满中断触发次数
volatile uint32_t g_dma_full_cnt = 0;   // DMA 全满中断触发次数
volatile uint32_t g_rx_frame_cnt = 0;   // 成功解析到的帧数
volatile int32_t  g_last_rx_x = 0;      // 最后一次收到的 x 值
volatile int32_t  g_latest_error_x = 0; // ★ main.c 直接读这个，不用 latest_red_laser_coord.x
volatile int32_t  g_ball_velocity = 0;   // ★ 球的速度，从 "v:(...)" 解析
/* ============================================== */

u16 rx_index = 0;
u32 uart_rx_tick = 0;
u8 uart_rx_buffer[128] = {0};
u8 uart_rx_data;
// ===== 快速整数解析：跳过 sscanf，手动解析 =====
// 返回吃掉几个字符，失败返回 0
static int fast_parse_int(const char *s, int *out)
{
    int val = 0;
    int sign = 1;
    int i = 0;

    if (s[i] == '-') { sign = -1; i++; }
    if (s[i] < '0' || s[i] > '9') return 0;

    do {
        val = val * 10 + (s[i] - '0');
        i++;
    } while (s[i] >= '0' && s[i] <= '9');

    *out = sign * val;
    return i;
}

// ===== 解析 "origin:(%d)" 格式 =====
static void parse_line_fast(const char *line, int len)
{
    if (len < 9) return;

    const char *p = strstr(line, "origin:");
    if (!p) return;
    p += 7;

    if (*p != '(') return;
    p++;

    int x, n;
    n = fast_parse_int(p, &x);
    if (n == 0) return;
    p += n;

    if (*p != ')') return;

    latest_red_laser_coord.x = x;
    latest_red_laser_coord.y = 0;
    latest_red_laser_coord.isValid = (x == 999) ? 0 : 1;
}

// ===== 直接在 DMA 数据块中搜索 origin:(...) 帧 =====
static void process_rx_chunk(const uint8_t *data, int len)
{
    const char *buf = (const char *)data;
    int pos = 0;

    while (pos <= len - 9)  // 至少需要 "origin:(0)" = 9 字符
    {
        // 找帧头 "origin:"
        const char *p = (const char *)memchr(buf + pos, 'o', len - pos);
        if (!p) break;

        // 检查是否有足够空间存放完整帧头
        int remain = len - (int)(p - buf);
        if (remain < 9) break;

        // 验证 "rigin:" 后续字符
        if (p[1] != 'r' || p[2] != 'i' || p[3] != 'g' ||
            p[4] != 'i' || p[5] != 'n' || p[6] != ':')
        {
            pos = (int)(p - buf) + 1;
            continue;
        }

        // 找到了 "origin:"，解析后面的 (整数)
        const char *start = p + 7;  // 跳过 "origin:"
        if (*start != '(')
        {
            pos = (int)(start - buf);
            continue;
        }

        // 找右括号（限制搜索范围，防止读到缓冲区外的垃圾数据）
        const char *end = (const char *)memchr(start + 1, ')', remain - 8);
        if (!end)
        {
            // 只有一半的帧跨越了 DMA chunk 边界，跳过
            pos = (int)(start - buf);
            continue;
        }

        // 解析括号内的整数
        int x, n;
        n = fast_parse_int(start + 1, &x);
        if (n == 0)
        {
            pos = (int)(start - buf) + 1;
            continue;
        }

        // 验证右括号紧跟在数字后面
        if (start[1 + n] != ')')
        {
            pos = (int)(start - buf) + 1 + n;
            continue;
        }

        // 解析成功
        latest_red_laser_coord.x = x;
        latest_red_laser_coord.y = 0;
        latest_red_laser_coord.isValid = (x == 999) ? 0 : 1;

        g_rx_frame_cnt++;
        g_last_rx_x = x;
        g_latest_error_x = x;  // ★ 直接写 volatile int，main.c 读这个

        /* 解析速度 v:(...) */
        {
            const char *vp = end + 1;  // 跳过 origin 的 ')'
            /* 找 " v:(" */
            if (vp[0] == ' ' && vp[1] == 'v' && vp[2] == ':' && vp[3] == '(') {
                int v, vn;
                vn = fast_parse_int(vp + 4, &v);
                if (vn > 0 && vp[4 + vn] == ')') {
                    g_ball_velocity = v;
                    end = vp + 4 + vn;  // 扩展 end 到 v 的右括号
                }
            }
        }

        pos = (int)(end - buf) + 1;
    }
}

// ========== DMA 回调 ==========
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        g_dma_half_cnt++;
        process_rx_chunk(pi_rx_buf, DMA_BUF_SIZE / 2);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        g_dma_full_cnt++;
        process_rx_chunk(pi_rx_buf + DMA_BUF_SIZE / 2, DMA_BUF_SIZE / 2);
    }
	if(huart->Instance == USART2 )//接收电机的实时位置数据
	{
		/* 防止缓冲区溢出：满了就丢弃最旧的一半，为新数据腾空间 */
		if (rx_index >= sizeof(uart_rx_buffer))
		{
			uint16_t half = sizeof(uart_rx_buffer) / 2;
			memmove(uart_rx_buffer, uart_rx_buffer + half, rx_index - half);
			rx_index -= half;
		}
		uart_rx_buffer[rx_index++] = uart_rx_data;
		uart_rx_tick = uwTick;
		HAL_UART_Receive_IT(&huart2,&uart_rx_data,1);
	}
}

int pi_parse_data(char *buffer)
{
    int len = strlen(buffer);
    parse_line_fast(buffer, len);
    return (latest_red_laser_coord.isValid) ? 0 : -2;
}
