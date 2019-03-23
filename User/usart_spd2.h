#ifndef __SPEED2_USART__
#define __SPEED2_USART__

#include "stm32f4xx.h"

//COM3 Define

#define USART_SPD2 UART7

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SPD2_USART_CLK RCC_APB1Periph_UART7
#define SPD2_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SPD2_USART_BAUDRATE 38400 //串口波特率

#define SPD2_USART_RX_GPIO_PORT GPIOE
#define SPD2_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPD2_USART_RX_PIN GPIO_Pin_7
#define SPD2_USART_RX_AF GPIO_AF_UART7
#define SPD2_USART_RX_SOURCE GPIO_PinSource7

#define SPD2_USART_TX_GPIO_PORT GPIOE
#define SPD2_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPD2_USART_TX_PIN GPIO_Pin_8
#define SPD2_USART_TX_AF GPIO_AF_UART7
#define SPD2_USART_TX_SOURCE GPIO_PinSource8

#define SPD2_USART_IRQ UART7_IRQn
#define SPD2_USART_IRQHandler UART7_IRQHandler

//----------------------------------------------------------------
#define SPD2_BAUDRATE wReg[111]  //2#编码器通信速度
#define SPD2_STATION wReg[112]   //2#编码器站地址
#define SPD2_START_ADR wReg[113] //2#编码器参数首地址
#define SPD2_REG_LEN wReg[114]   //2#编码器参数长度

#define SPD2_COUNTER wReg[8]   //2#编码器圈数计数器
#define SPD2_CUR_ANG wReg[20]  //2#编码器当前角度
#define SPD2_CUR_TICK wReg[21] //2#编码器当前角度
#define SPD2_CUR_DETA wReg[22] //2#编码器当前角度
#define SPD2_CUR_SPD wReg[23]  //2#编码器当前角度
#define SPD2_LST_ANG wReg[24]  //2#编码器当前角度
#define SPD2_LST_TICK wReg[25] //2#编码器当前角度
#define SPD2_LST_DETA wReg[26] //2#编码器当前角度
#define SPD2_AVG_SPD wReg[27]  //2#编码器当前角度
#define SPD2_COM_FAIL wReg[28] //2#编码器当前角度
#define SPD2_COM_SUCS wReg[29] //2#编码器当前角度

void SPD2_Init(void);
void SPD2_TxCmd(void);
void SPD2_Task(void);

void SPD2_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
