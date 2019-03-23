#include "usart_spd2.h"
#include "spd_comm.h"
#include "Modbus_svr.h"
#include "SysTick.h"
#include "stm32f4xx_conf.h"

extern u8 bChanged;
extern u16 wReg[];

uint8_t SPD2_frame[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
u8 SPD2_buffer[256];
u8 SPD2_curptr;
u8 SPD2_bRecv;
u8 SPD2_frame_len = 85;
u8 SPD2_bFirst = 1;
u32 ulSPD2Tick = 0;

SpeedValueQueue qSPD2;

//-------------------------------------------------------------------------------
//	@brief	中断初始化
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = SPD2_USART_IRQ;
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
static void SPD2_Config(u16 wBaudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(SPD2_USART_RX_GPIO_CLK | SPD2_USART_TX_GPIO_CLK, ENABLE);

    /* 使能 USART 时钟 */
    SPD2_USART_APBxClkCmd(SPD2_USART_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SPD2_USART_TX_PIN;
    GPIO_Init(SPD2_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = SPD2_USART_RX_PIN;
    GPIO_Init(SPD2_USART_RX_GPIO_PORT, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(SPD2_USART_RX_GPIO_PORT, SPD2_USART_RX_SOURCE, SPD2_USART_RX_AF);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(SPD2_USART_TX_GPIO_PORT, SPD2_USART_TX_SOURCE, SPD2_USART_TX_AF);

    /* 配置串SPD2_USART 模式 */
    /* 波特率设置：SPD2_USART_BAUDRATE */
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
    USART_Init(USART_SPD2, &USART_InitStructure);

    /* 嵌套向量中断控制器NVIC配置 */
    SPD2_NVIC_Configuration();

    /* 使能串口接收中断 */
    USART_ITConfig(USART_SPD2, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART_SPD2, ENABLE);
}

/****************************************************************
 *	@brief:	    SPD2通信初始化程序
 *	@param:	    None
 *	@retval:	None
 ****************************************************************/
void SPD2_Init(void)
{
    if (SPD2_BAUDRATE != 96 && SPD2_BAUDRATE != 192 && SPD2_BAUDRATE != 384 && SPD2_BAUDRATE != 1152)
    {
        SPD2_BAUDRATE = 384;
    }
    SPD2_Config(SPD2_BAUDRATE);

    SPD2_curptr = 0;
    SPD2_bRecv = 0;
    SPD2_COM_FAIL = 0;
    SPD2_frame_len = 2 * SPD2_REG_LEN + 5;
    ulSPD2Tick = GetCurTick();

    SpdQueueInit(&qSPD2);
}

//-------------------------------------------------------------------------------
//	@brief	发送命令帧
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD2_TxCmd(void)
{
    u16 uCRC;

    if (SPD2_bRecv == 1) //如果当前未完成接收，则通信错误计数器递增
        SPD2_COM_FAIL++;

    SPD2_curptr = 0;
    SPD2_bRecv = 1;

    if (bChanged || SPD2_bFirst)
    {
        SPD2_frame[0] = SPD2_STATION;                   //station number
        SPD2_frame[2] = (SPD2_START_ADR & 0xff00) >> 8; //start address high
        SPD2_frame[3] = SPD2_START_ADR & 0x00ff;        //start address low
        SPD2_frame[4] = (SPD2_REG_LEN & 0xff00) >> 8;   //length high
        SPD2_frame[5] = SPD2_REG_LEN & 0x00ff;          //length low
        uCRC = CRC16(SPD2_frame, 6);
        SPD2_frame[6] = uCRC & 0x00FF;        //CRC low
        SPD2_frame[7] = (uCRC & 0xFF00) >> 8; //CRC high
        bChanged++;
        SPD2_bFirst = 0;
    }

    Usart_SendBytes(USART_SPD2, SPD2_frame, 8);
}

/*
 *	@brief	接收数据处理
 *	@param	None
 *	@retval	None
 */
void SPD2_Task(void)
{
    u32 tick;

    if (SPD2_curptr < SPD2_frame_len)
        return;

    if (SPD2_buffer[0] != SPD2_STATION || SPD2_buffer[1] != 0x03) //站地址判断
        return;

    if (SPD2_buffer[2] != 2 * SPD2_REG_LEN) //数值长度判读
        return;

    tick = GetCurTick();
    SPD2_LST_ANG = SPD2_CUR_ANG;   //上次编码器值
    SPD2_LST_TICK = SPD2_CUR_TICK; //上次计时器值
    SPD2_LST_DETA = SPD2_CUR_DETA; //上次角度变化值

    SPD2_CUR_ANG = SPD2_buffer[3] << 0x08 | SPD2_buffer[4]; //本次编码器值
    SPD2_CUR_TICK = tick - ulSPD2Tick;                      //本次计时器值
    ulSPD2Tick = tick;                                      //保存计时器
    SPD2_CUR_DETA = SPD2_CUR_ANG - SPD2_LST_ANG;            //本次角度变化量
    if (SPD2_CUR_ANG < 1024 && SPD2_LST_ANG > 3072)
    {
        SPD2_CUR_DETA = SPD2_CUR_ANG - SPD2_LST_ANG + 4096;
        SPD2_COUNTER++;
    }
    if (SPD2_CUR_ANG > 3072 && SPD2_LST_ANG < 1024)
    {
        SPD2_CUR_DETA = SPD2_CUR_ANG - SPD2_LST_ANG - 4096;
        SPD2_COUNTER--;
    }
    if (SPD2_CUR_TICK != 0)
        SPD2_CUR_SPD = SPD2_CUR_DETA * 1000 / SPD2_CUR_TICK; //本次速度

    SpdQueueIn(&qSPD2, SPD2_CUR_DETA, SPD2_CUR_TICK);
    SPD2_AVG_SPD = SpdQueueAvgVal(&qSPD2); //10次平均速度

    SPD2_COM_SUCS++;
    SPD2_bRecv = 0;
    SPD2_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	串口中断服务程序
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD2_USART_IRQHandler(void)
{
    u8 ch;

    if (USART_GetITStatus(USART_SPD2, USART_IT_RXNE) != RESET) //判断读寄存器是否非空
    {
        ch = USART_ReceiveData(USART_SPD2); //将读寄存器的数据缓存到接收缓冲区里
        SPD2_buffer[SPD2_curptr++] = ch;
    }

    if (USART_GetITStatus(USART_SPD2, USART_IT_TXE) != RESET)
    {
        USART_ITConfig(USART_SPD2, USART_IT_TXE, DISABLE);
    }
}

//-----------------------------End of file--------------------------------------------------
