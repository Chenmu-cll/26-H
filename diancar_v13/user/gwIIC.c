#include "gwIIC.h"

//============================================ IO direct — Yahboom 8ch IR tracking ====================================//
#if USE_Protocol==3

/* PE0=s1(b7) PE1=s2(b6) PE3=s3(b5) PE7=s4(b4)
   PE8=s5(b3) PE10=s6(b2) PE12=s7(b1) PE13=s8(b0)
   0=black(line)  1=white(background) */

/* bit positions in GPIOE->IDR */
#define IO_S1  0   /* PE0  → bit7 of sensor byte */
#define IO_S2  1   /* PE1  → bit6 */
#define IO_S3  3   /* PE3  → bit5 */
#define IO_S4  7   /* PE7  → bit4 */
#define IO_S5  8   /* PE8  → bit3 */
#define IO_S6  10  /* PE10 → bit2 */
#define IO_S7  12  /* PE12 → bit1 */
#define IO_S8  13  /* PE13 → bit0 */

uint8_t digital_data;

void GW_GRAY_Init(void) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef s = {0};
    s.Mode  = GPIO_MODE_INPUT;
    s.Pull  = GPIO_PULLUP;
    s.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    s.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_7 |
              GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOE, &s);
}

/* one-shot read: read IDR once, extract all 8 bits */
void GW_GRAY_DigtalDatas_Read(void) {
    uint32_t idr = GPIOE->IDR;
    digital_data =
        ((idr >> IO_S1) & 1) << 7 |
        ((idr >> IO_S2) & 1) << 6 |
        ((idr >> IO_S3) & 1) << 5 |
        ((idr >> IO_S4) & 1) << 4 |
        ((idr >> IO_S5) & 1) << 3 |
        ((idr >> IO_S6) & 1) << 2 |
        ((idr >> IO_S7) & 1) << 1 |
        ((idr >> IO_S8) & 1) << 0;
}

////////////////////////////////////////////////////////////////////////////
#elif USE_Protocol==0
//==================== I2C (unused) =====================//
#define I2C_DELAY_US  3
#define GW_SCL_CLK()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define GW_SCL_PORT     GPIOF
#define GW_SCL_PIN      GPIO_PIN_0
#define GW_SDA_CLK()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define GW_SDA_PORT     GPIOF
#define GW_SDA_PIN      GPIO_PIN_1
#define SCL_H()   (GW_SCL_PORT->BSRR = (uint32_t)GW_SCL_PIN)
#define SCL_L()   (GW_SCL_PORT->BSRR = (uint32_t)GW_SCL_PIN << 16U)
#define SDA_H()   (GW_SDA_PORT->BSRR = (uint32_t)GW_SDA_PIN)
#define SDA_L()   (GW_SDA_PORT->BSRR = (uint32_t)GW_SDA_PIN << 16U)
#define SDA_RD()  ((GW_SDA_PORT->IDR & (uint32_t)GW_SDA_PIN) ? 1 : 0)
static void i2c_delay(void){volatile uint32_t n=I2C_DELAY_US*7;while(n--){__NOP();}}
static void i2c_start(void){SDA_H();i2c_delay();SCL_H();i2c_delay();SDA_L();i2c_delay();SCL_L();i2c_delay();}
static void i2c_stop(void){SDA_L();i2c_delay();SCL_H();i2c_delay();SDA_H();i2c_delay();}
static uint8_t i2c_write_byte(uint8_t b){for(uint8_t i=0;i<8;i++){if(b&0x80)SDA_H();else SDA_L();b<<=1;i2c_delay();SCL_H();i2c_delay();SCL_L();i2c_delay();}SDA_H();i2c_delay();SCL_H();i2c_delay();uint8_t a=SDA_RD();SCL_L();i2c_delay();return a;}
static uint8_t i2c_read_byte(uint8_t ack){uint8_t b=0;SDA_H();for(uint8_t i=0;i<8;i++){i2c_delay();SCL_H();i2c_delay();b=(b<<1)|SDA_RD();SCL_L();}if(ack)SDA_H();else SDA_L();i2c_delay();SCL_H();i2c_delay();SCL_L();i2c_delay();SDA_H();return b;}
uint8_t i2c_scan_devices(uint8_t*f,uint8_t m){uint8_t c=0;for(uint16_t a=1;a<127&&c<m;a++){i2c_start();uint8_t ack=i2c_write_byte((uint8_t)(a<<1)|0x00);i2c_stop();if(ack==0)f[c++]=(uint8_t)a;}return c;}
static int8_t i2c_dev_read(uint8_t d,uint8_t*v){i2c_start();if(i2c_write_byte((uint8_t)(d<<1)|0x01)){i2c_stop();return 1;}*v=i2c_read_byte(1);i2c_stop();return 0;}
static int8_t i2c_dev_write(uint8_t d,uint8_t c){i2c_start();if(i2c_write_byte((uint8_t)(d<<1)|0x00)){i2c_stop();return 1;}i2c_write_byte(c);i2c_stop();return 0;}
static int8_t i2c_reg_read(uint8_t d,uint8_t r,uint8_t l,uint8_t*b){i2c_start();if(i2c_write_byte((uint8_t)(d<<1)|0x00)){i2c_stop();return 1;}if(i2c_write_byte(r)){i2c_stop();return 1;}i2c_start();if(i2c_write_byte((uint8_t)(d<<1)|0x01)){i2c_stop();return 1;}for(uint8_t i=0;i<l;i++)b[i]=i2c_read_byte(i==l-1?1:0);i2c_stop();return 0;}
void GW_GRAY_PIN_Init(void){GW_SCL_CLK();GW_SDA_CLK();GPIO_InitTypeDef s={0};s.Pin=GW_SCL_PIN;s.Mode=GPIO_MODE_OUTPUT_PP;s.Pull=GPIO_PULLUP;s.Speed=GPIO_SPEED_FREQ_VERY_HIGH;HAL_GPIO_Init(GW_SCL_PORT,&s);s.Pin=GW_SDA_PIN;s.Mode=GPIO_MODE_OUTPUT_OD;s.Pull=GPIO_PULLUP;s.Speed=GPIO_SPEED_FREQ_VERY_HIGH;HAL_GPIO_Init(GW_SDA_PORT,&s);SDA_H();SCL_H();}
uint8_t digital_data;uint8_t gray_sensor[8];uint8_t analog_data[8];
uint8_t i2c_scan_count=0;uint8_t i2c_addr[4]={0};uint8_t i2c_active_idx=0;uint8_t i2c_read_ok=0;
void GW_GRAY_Init(void){uint8_t a[16];GW_GRAY_PIN_Init();for(volatile uint32_t d=0;d<500000;d++){__NOP();}i2c_scan_count=i2c_scan_devices(a,16);if(i2c_scan_count>4)i2c_scan_count=4;for(uint8_t i=0;i<i2c_scan_count;i++)i2c_addr[i]=a[i];}
void GW_GRAY_DigtalDatas_Read(void){uint8_t v=0xFF;if(i2c_scan_count>=1){uint8_t r[]={0x00,0x01,0x02,0x10,0x20,0x30};for(uint8_t x=0;x<6;x++){if(i2c_reg_read(i2c_addr[0],r[x],1,&v)==0){digital_data=v;return;}}}if(i2c_scan_count>=2){for(uint8_t c=0;c<4;c++){uint8_t cc[]={0x00,0x01,0x10,0x20};i2c_dev_write(i2c_addr[0],cc[c]);if(i2c_dev_read(i2c_addr[1],&v)==0){digital_data=v;return;}}}for(uint8_t i=0;i<i2c_scan_count;i++){if(i2c_dev_read(i2c_addr[i],&v)==0){digital_data=v;return;}}digital_data=0xFF;}
void GW_GRAY_DigtalDatas_Split(void){GW_GRAY_DigtalDatas_Read();}
uint8_t GW_digital_Read_Bit(uint8_t n){GW_GRAY_DigtalDatas_Read();return(digital_data>>(n-1))&0x01;}
void GW_GRAY_AnalogDatas_Read(void){}

#elif USE_Protocol==1
//==================== serial (Ganwei) =====================//
#define GW_GRAY_DELAY_TICK  95
static void gw_gray_delay(uint32_t d){volatile uint32_t n=d;while(n--){}}
#define GW_CLK_RCC()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define GW_CLK_GPIO   GPIOE
#define GW_CLK_PIN    GPIO_PIN_14
#define GW_DAT_RCC()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define GW_DAT_GPIO   GPIOE
#define GW_DAT_PIN    GPIO_PIN_15
#define GW_CLK_Clr()  HAL_GPIO_WritePin(GW_CLK_GPIO,GW_CLK_PIN,GPIO_PIN_RESET)
#define GW_CLK_Set()  HAL_GPIO_WritePin(GW_CLK_GPIO,GW_CLK_PIN,GPIO_PIN_SET)
#define Read_GW_DAT() HAL_GPIO_ReadPin(GW_DAT_GPIO,GW_DAT_PIN)
void GW_GRAY_Init(void){GW_CLK_RCC();GW_DAT_RCC();GPIO_InitTypeDef s={0};s.Pin=GW_CLK_PIN;s.Mode=GPIO_MODE_OUTPUT_PP;s.Pull=GPIO_NOPULL;s.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GW_CLK_GPIO,&s);s.Pin=GW_DAT_PIN;s.Mode=GPIO_MODE_INPUT;s.Pull=GPIO_PULLUP;HAL_GPIO_Init(GW_DAT_GPIO,&s);}
uint8_t gw_gray_serial_read(void){uint8_t r=0;for(uint8_t i=0;i<8;i++){GW_CLK_Clr();gw_gray_delay(GW_GRAY_DELAY_TICK);r|=(Read_GW_DAT()<<(7-i));GW_CLK_Set();gw_gray_delay(GW_GRAY_DELAY_TICK);}return r;}
uint8_t digital_data;void GW_GRAY_DigtalDatas_Read(void){digital_data=gw_gray_serial_read();}
#else
//==================== 74HC165 =====================//
#define _74HC165_PL_RCC()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define _74HC165_PL_GPIO    GPIOD
#define _74HC165_PL_PIN     GPIO_PIN_2
#define _74HC165_CLK_RCC()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define _74HC165_CLK_GPIO   GPIOC
#define _74HC165_CLK_PIN    GPIO_PIN_10
#define _74HC165_DATA_RCC() __HAL_RCC_GPIOC_CLK_ENABLE()
#define _74HC165_DATA_GPIO  GPIOC
#define _74HC165_DATA_PIN   GPIO_PIN_11
#define _74HC165_PL_Clr()  HAL_GPIO_WritePin(_74HC165_PL_GPIO,_74HC165_PL_PIN,GPIO_PIN_RESET)
#define _74HC165_PL_Set()  HAL_GPIO_WritePin(_74HC165_PL_GPIO,_74HC165_PL_PIN,GPIO_PIN_SET)
#define _74HC165_CLK_Clr() HAL_GPIO_WritePin(_74HC165_CLK_GPIO,_74HC165_CLK_PIN,GPIO_PIN_RESET)
#define _74HC165_CLK_Set() HAL_GPIO_WritePin(_74HC165_CLK_GPIO,_74HC165_CLK_PIN,GPIO_PIN_SET)
#define READ_74HC165_DATA() HAL_GPIO_ReadPin(_74HC165_DATA_GPIO,_74HC165_DATA_PIN)
#define REVERSE_BYTE(b) (((b)&0x01)<<7|((b)&0x02)<<5|((b)&0x04)<<3|((b)&0x08)<<1|((b)&0x10)>>1|((b)&0x20)>>3|((b)&0x40)>>5|((b)&0x80)>>7)
void GW_GRAY_Init(void){_74HC165_PL_RCC();_74HC165_CLK_RCC();_74HC165_DATA_RCC();GPIO_InitTypeDef s={0};s.Pin=_74HC165_PL_PIN;s.Mode=GPIO_MODE_OUTPUT_PP;s.Pull=GPIO_NOPULL;s.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(_74HC165_PL_GPIO,&s);s.Pin=_74HC165_CLK_PIN;s.Mode=GPIO_MODE_OUTPUT_PP;s.Pull=GPIO_NOPULL;s.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(_74HC165_CLK_GPIO,&s);s.Pin=_74HC165_DATA_PIN;s.Mode=GPIO_MODE_INPUT;s.Pull=GPIO_PULLUP;HAL_GPIO_Init(_74HC165_DATA_GPIO,&s);}
uint8_t _74HC165_Datas_Read(void){uint8_t d=0;_74HC165_PL_Clr();_74HC165_PL_Set();for(uint8_t i=0;i<8;i++){d<<=1;if(READ_74HC165_DATA())d|=0x01;_74HC165_CLK_Clr();_74HC165_CLK_Set();}return d;}
uint8_t digital_data;void GW_GRAY_DigtalDatas_Read(void){digital_data=_74HC165_Datas_Read();digital_data=REVERSE_BYTE(digital_data);}
#endif