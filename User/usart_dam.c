#include "usart_dam.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern u16 wReg[];

uint8_t DAM_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
uint8_t DMA_TxFrame[17] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x04, 0x08,
                           0x11, 0x11, 0x22, 0x22, 0x33, 0x33, 0x44, 0x44,
                           0xCC, 0xCC};
u8 DAM_buffer[256];
u8 DAM_curptr;
u8 DAM_bRecv;
u8 DAM_frame_len = 85;
u8 DAM_bFirst = 1;
u32 ulDAMTick = 0;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void DAM_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DAM_USART_IRQ;
    /* 抢断优先级为1 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    /* 子优先级为1 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	串口初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void DAM_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(DAM_USART_RX_GPIO_CLK | DAM_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    DAM_USART_APBxClkCmd(DAM_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DAM_USART_TX_PIN;
    GPIO_Init(DAM_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DAM_USART_RX_PIN;
    GPIO_Init(DAM_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(DAM_USART_RX_GPIO_PORT, DAM_USART_RX_SOURCE, DAM_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(DAM_USART_TX_GPIO_PORT, DAM_USART_TX_SOURCE, DAM_USART_TX_AF);

    /* 配置串DAM_USART 模式 */
    /* 波特率设置：DAM_USART_BAUDRATE */
    USART_InitStructure.USART_BaudRate = wBaudrate * 100;
    /* 字长(数据位+校验位)：8 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /* 停止位：1个停止位 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* 校验位选择：不使用校验 */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    /* 硬件流控制：不使用硬件流 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* USART模式控制：同时使能接收和发送 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* 完成USART初始化配置 */
    USART_Init(USART_DAM, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    DAM_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_DAM, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_DAM, ENABLE);
}

/****************************************************************
 *	@brief:	    DAM通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void DAM_Init(void)
{
    if (DAM_BAUDRATE != 96 && DAM_BAUDRATE != 192 && DAM_BAUDRATE != 384 && DAM_BAUDRATE != 1152)
    {
        DAM_BAUDRATE = 1152;
    }
    DAM_Config(DAM_BAUDRATE);

    DAM_curptr = 0;
    DAM_bRecv = 0;
    DAM_COM_FAIL = 0;
    DAM_frame_len = 2 * DAM_REG_LEN + 5;
    ulDAMTick = GetCurTick();
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DAM_TxCmd(void)
{
    u16 uCRC;

    if (DAM_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        DAM_COM_FAIL++;

    DAM_curptr = 0;
    DAM_bRecv = 1;

    DMA_TxFrame[7] = (DAM_DA_CHN1 & 0xff00) >> 8;
    DMA_TxFrame[8] = DAM_DA_CHN1 & 0x00ff;
    DMA_TxFrame[9] = (DAM_DA_CHN2 & 0xff00) >> 8;
    DMA_TxFrame[10] = DAM_DA_CHN2 & 0x00ff;
    DMA_TxFrame[11] = (DAM_DA_CHN3 & 0xff00) >> 8;
    DMA_TxFrame[12] = DAM_DA_CHN3 & 0x00ff;
    DMA_TxFrame[13] = (DAM_DA_CHN4 & 0xff00) >> 8;
    DMA_TxFrame[14] = DAM_DA_CHN4 & 0x00ff;
    uCRC = CRC16(DMA_TxFrame, 15);
    DMA_TxFrame[15] = uCRC & 0x00FF;        //CRC low
    DMA_TxFrame[16] = (uCRC & 0xFF00) >> 8; //CRC high

    Usart_SendBytes(USART_DAM, DMA_TxFrame, 17);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void DAM_Task(void)
{
    u32 tick;
		int i;

    if (DAM_curptr < DAM_frame_len)
        return;

    if (DAM_buffer[0] != DAM_STATION || DAM_buffer[1] != 0x10) //站地址判断
        return;
		
				for(i=0 ; i < 4 ; i++)
		{
			wReg[DAM_VAL_ADR + i] = wReg[DAM_FACT_ADR + i] ;
		}

    tick = GetCurTick();
    DAM_CUR_DETA = tick - ulDAMTick; //本次角度变化量
    ulDAMTick = tick;                //保存计时器

    DAM_COM_SUCS++;
    DAM_bRecv = 0;
    DAM_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DAM_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_DAM, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_DAM); //将读寄存器的数据缓存到接收缓冲区里
        DAM_buffer[DAM_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_DAM, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_DAM, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
