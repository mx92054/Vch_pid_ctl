#ifndef __DAMODULE_USART__
#define __DAMODULE_USART__

#include "stm32f4xx.h"

//COM5 Define

#define USART_DAM UART4

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define DAM_USART_CLK RCC_APB1Periph_UART4
#define DAM_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define DAM_USART_BAUDRATE 9600 //串口波特率

#define DAM_USART_RX_GPIO_PORT GPIOA
#define DAM_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define DAM_USART_RX_PIN GPIO_Pin_0
#define DAM_USART_RX_AF GPIO_AF_UART4
#define DAM_USART_RX_SOURCE GPIO_PinSource0

#define DAM_USART_TX_GPIO_PORT GPIOA
#define DAM_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define DAM_USART_TX_PIN GPIO_Pin_1
#define DAM_USART_TX_AF GPIO_AF_UART4
#define DAM_USART_TX_SOURCE GPIO_PinSource1

#define DAM_USART_IRQ UART4_IRQn
#define DAM_USART_IRQHandler UART4_IRQHandler

//----------------------------------------------------------------
#define DAM_BAUDRATE wReg[119] //模拟量输出板通信速度
#define DAM_STATION 1          //模拟量输出板站地址
#define DAM_START_ADR 0        //模拟量输出板参数首地址
#define DAM_REG_LEN 1          //模拟量输出板参数长度

#define DAM_VAL_ADR 40
#define DAM_FACT_ADR 120 

#define DAM_DA_CHN1 wReg[120]  //模拟量输出板通道1
#define DAM_DA_CHN2 wReg[121]  //模拟量输出板通道2
#define DAM_DA_CHN3 wReg[122]  //模拟量输出板通道3
#define DAM_DA_CHN4 wReg[123]  //模拟量输出板通道4
#define DAM_CUR_DETA wReg[44] //模拟量输出板当前角度
#define DAM_COM_FAIL wReg[48] //模拟量输出板当前角度
#define DAM_COM_SUCS wReg[49] //模拟量输出板当前角度

void DAM_Init(void);
void DAM_TxCmd(void);
void DAM_Task(void);

void DAM_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
