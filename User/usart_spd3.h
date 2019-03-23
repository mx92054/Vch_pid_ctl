#ifndef __SPEED3_USART__
#define __SPEED3_USART__

#include "stm32f4xx.h"

//COM4 Define

#define USART_SPD3 USART2

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SPD3_USART_CLK RCC_APB1Periph_USART2
#define SPD3_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SPD3_USART_BAUDRATE 9600 //串口波特率

#define SPD3_USART_RX_GPIO_PORT GPIOA
#define SPD3_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SPD3_USART_RX_PIN GPIO_Pin_2
#define SPD3_USART_RX_AF GPIO_AF_USART2
#define SPD3_USART_RX_SOURCE GPIO_PinSource2

#define SPD3_USART_TX_GPIO_PORT GPIOA
#define SPD3_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define SPD3_USART_TX_PIN GPIO_Pin_3
#define SPD3_USART_TX_AF GPIO_AF_USART2
#define SPD3_USART_TX_SOURCE GPIO_PinSource3

#define SPD3_USART_IRQ USART2_IRQn
#define SPD3_USART_IRQHandler USART2_IRQHandler

//----------------------------------------------------------------
#define SPD3_BAUDRATE wReg[115]  //3#编码器通信速度
#define SPD3_STATION wReg[116]   //3#编码器站地址
#define SPD3_START_ADR wReg[117] //3#编码器参数首地址
#define SPD3_REG_LEN wReg[118]   //3#编码器参数长度

#define SPD3_COUNTER wReg[9]   //3#编码器圈数计数器
#define SPD3_CUR_ANG wReg[30]  //3#编码器当前角度
#define SPD3_CUR_TICK wReg[31] //3#编码器当前角度
#define SPD3_CUR_DETA wReg[32] //3#编码器当前角度
#define SPD3_CUR_SPD wReg[33]  //3#编码器当前角度
#define SPD3_LST_ANG wReg[34]  //3#编码器当前角度
#define SPD3_LST_TICK wReg[35] //3#编码器当前角度
#define SPD3_LST_DETA wReg[36] //3#编码器当前角度
#define SPD3_AVG_SPD wReg[37]  //3#编码器当前角度
#define SPD3_COM_FAIL wReg[38] //3#编码器当前角度
#define SPD3_COM_SUCS wReg[39] //3#编码器当前角度

void SPD3_Init(void);
void SPD3_TxCmd(void);
void SPD3_Task(void);

void SPD3_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
