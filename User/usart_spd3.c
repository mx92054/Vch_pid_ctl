#include "usart_spd3.h"
#include "spd_comm.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern u16 wReg[];

uint8_t SPD3_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
u8 SPD3_buffer[256];
u8 SPD3_curptr;
u8 SPD3_bRecv;
u8 SPD3_frame_len = 85;
u8 SPD3_bFirst = 1;
u32 ulSPD3Tick = 0;

SpeedValueQueue qSPD3;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD3_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = SPD3_USART_IRQ;
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
static void SPD3_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(SPD3_USART_RX_GPIO_CLK | SPD3_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    SPD3_USART_APBxClkCmd(SPD3_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SPD3_USART_TX_PIN;
    GPIO_Init(SPD3_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SPD3_USART_RX_PIN;
    GPIO_Init(SPD3_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(SPD3_USART_RX_GPIO_PORT, SPD3_USART_RX_SOURCE, SPD3_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(SPD3_USART_TX_GPIO_PORT, SPD3_USART_TX_SOURCE, SPD3_USART_TX_AF);

    /* 配置串SPD3_USART 模式 */
    /* 波特率设置：SPD3_USART_BAUDRATE */
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
    USART_Init(USART_SPD3, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    SPD3_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_SPD3, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_SPD3, ENABLE);
}

/****************************************************************
 *	@brief:	    SPD3通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void SPD3_Init(void)
{
    if (SPD3_BAUDRATE != 96 && SPD3_BAUDRATE != 192 && SPD3_BAUDRATE != 384 && SPD3_BAUDRATE != 1152)
    {
        SPD3_BAUDRATE = 384;
    }
    SPD3_Config(SPD3_BAUDRATE);

    SPD3_curptr = 0;
    SPD3_bRecv = 0;
    SPD3_COM_FAIL = 0;
    SPD3_frame_len = 2 * SPD3_REG_LEN + 5;
    ulSPD3Tick = GetCurTick();

    SpdQueueInit(&qSPD3);
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD3_TxCmd(void)
{
    u16 uCRC;

    if (SPD3_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        SPD3_COM_FAIL++;

    SPD3_curptr = 0;
    SPD3_bRecv = 1;

    if (bChanged || SPD3_bFirst)
    {
        SPD3_frame[0] = SPD3_STATION;                   //station number
        SPD3_frame[2] = (SPD3_START_ADR & 0xff00) >> 8; //start address high
        SPD3_frame[3] = SPD3_START_ADR & 0x00ff;        //start address low
        SPD3_frame[4] = (SPD3_REG_LEN & 0xff00) >> 8;   //length high
        SPD3_frame[5] = SPD3_REG_LEN & 0x00ff;          //length low
        uCRC = CRC16(SPD3_frame, 6);
        SPD3_frame[6] = uCRC & 0x00FF;        //CRC low
        SPD3_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
        bChanged++;
        SPD3_bFirst = 0;
    }

    Usart_SendBytes(USART_SPD3, SPD3_frame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void SPD3_Task(void)
{
    u32 tick;

    if (SPD3_curptr < SPD3_frame_len)
        return;

    if (SPD3_buffer[0] != SPD3_STATION || SPD3_buffer[1] != 0x03) //站地址判断
        return;

    if (SPD3_buffer[2] != 2 * SPD3_REG_LEN) //数值长度判读
        return;

    tick = GetCurTick();
    SPD3_LST_ANG = SPD3_CUR_ANG;   //上次编码器值
    SPD3_LST_TICK = SPD3_CUR_TICK; //上次计时器值
    SPD3_LST_DETA = SPD3_CUR_DETA; //上次角度变化值

    SPD3_CUR_ANG = SPD3_buffer[3] << 0x08 | SPD3_buffer[4]; //本次编码器值
    SPD3_CUR_TICK = tick - ulSPD3Tick;                      //本次计时器值
    ulSPD3Tick = tick;                                      //保存计时器
    SPD3_CUR_DETA = SPD3_CUR_ANG - SPD3_LST_ANG;            //本次角度变化量
    if (SPD3_CUR_ANG < 1024 && SPD3_LST_ANG > 3072)
    {
        SPD3_CUR_DETA = SPD3_CUR_ANG - SPD3_LST_ANG + 4096;
        SPD3_COUNTER++;
    }
    if (SPD3_CUR_ANG > 3072 && SPD3_LST_ANG < 1024)
    {
        SPD3_CUR_DETA = SPD3_CUR_ANG - SPD3_LST_ANG - 4096;
        SPD3_COUNTER--;
    }
    if (SPD3_CUR_TICK != 0)
        SPD3_CUR_SPD = SPD3_CUR_DETA * 1000 / SPD3_CUR_TICK; //本次速度

    SpdQueueIn(&qSPD3, SPD3_CUR_DETA, SPD3_CUR_TICK);
    SPD3_AVG_SPD = SpdQueueAvgVal(&qSPD3); //10次平均速度

    SPD3_COM_SUCS++;
    SPD3_bRecv = 0;
    SPD3_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD3_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_SPD3, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_SPD3); //将读寄存器的数据缓存到接收缓冲区里
        SPD3_buffer[SPD3_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_SPD3, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_SPD3, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
