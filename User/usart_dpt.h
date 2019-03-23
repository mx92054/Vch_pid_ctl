#ifndef __DPT_USART__
#define __DPT_USART__

#include "stm32f4xx.h"

//  COM1 Define

#define USART_DPT USART6

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define DPT_USART_CLK RCC_APB2Periph_USART6
#define DPT_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define DPT_USART_BAUDRATE 9600 //串口波特率

#define DPT_USART_RX_GPIO_PORT GPIOC
#define DPT_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define DPT_USART_RX_PIN GPIO_Pin_6
#define DPT_USART_RX_AF GPIO_AF_USART6
#define DPT_USART_RX_SOURCE GPIO_PinSource6

#define DPT_USART_TX_GPIO_PORT GPIOC
#define DPT_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define DPT_USART_TX_PIN GPIO_Pin_7
#define DPT_USART_TX_AF GPIO_AF_USART6
#define DPT_USART_TX_SOURCE GPIO_PinSource7

#define DPT_USART_IRQ USART6_IRQn
#define DPT_USART_IRQHandler USART6_IRQHandler

//--------------------------------------------------------------------
#define DPT_SAVE_ADR 0         // DPTE传感器参数在wReg中的起始地址
#define DPT_BAUDRATE wReg[103] //通信波特率
#define DPT_COM_SUCS wReg[9]   //通信成功次数

#define DPT_SENSOR_ADR wReg[124] //计算角度值地址
#define DPT_SET_ADR wReg[125]    //设定角度值地址
#define DPT_ANGLE_ADR wReg[126]  //角度计算结果

void DPT_Init(void);
void DPT_Task(void);

void DPT_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
