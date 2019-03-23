#include "usart_dpt.h"
#include "Modbus_svr.h"
#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "SysTick.h"

extern u16 wReg[];

char DPT_buffer[256];
u8 DPT_curptr;
u8 DPT_bRecv;

float fDepth;
float fHead, fPitch, fRoll;

u32 ulLastDptTicks, ulLastHprTicks;

//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void DPT_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Ƕ�������жϿ�������ѡ�� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ����USARTΪ�ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = DPT_USART_IRQ;
	/* �������ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* �����ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	/* ʹ���ж� */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ��ʼ������NVIC */
	NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void DPT_Config(short baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(DPT_USART_RX_GPIO_CLK | DPT_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	RCC_APB2PeriphClockCmd(DPT_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DPT_USART_TX_PIN;
	GPIO_Init(DPT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DPT_USART_RX_PIN;
	GPIO_Init(DPT_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(DPT_USART_RX_GPIO_PORT, DPT_USART_RX_SOURCE, DPT_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(DPT_USART_TX_GPIO_PORT, DPT_USART_TX_SOURCE, DPT_USART_TX_AF);

	/* ���ô�DPT_USART ģʽ */
	/* ���������ã�DPT_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = baud * 100;
	/* �ֳ�(����λ+У��λ)��8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* ���USART��ʼ������ */
	USART_Init(USART_DPT, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	DPT_NVIC_Configuration();

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART_DPT, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(USART_DPT, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DPT_Init(void)
{
	if (DPT_BAUDRATE != 96 && DPT_BAUDRATE != 192 && DPT_BAUDRATE != 384 && DPT_BAUDRATE != 1152)
	{
		DPT_BAUDRATE = 384;
	}
	DPT_Config(DPT_BAUDRATE);

	DPT_curptr = 0;
	DPT_bRecv = 0;
}

//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//   Frame format 1: $ISDPT,-000.485,M,000.9689,B,30.81,C*28
//   Frame format 2: $ISHPR,060.8,-89.5,-081.1*60
//-------------------------------------------------------------------------------
void DPT_Task(void)
{
	int bErr;
	short tmp;
	u32 tick;

	if (DPT_bRecv == 0) // δ�յ���ȼ���Ϣ֡������������ӣ�
	{
		return;
	}

	wReg[9]++;
	if (DPT_buffer[1] == 'P') //���������֡
	{
		wReg[9]++;
		bErr = sscanf(DPT_buffer, "$PIPS,%f,M*", &fDepth);
		if (bErr == 1)
		{
			tick = GetCurTick();
			wReg[DPT_SAVE_ADR] = (u16)(fDepth * 10);
			wReg[DPT_SAVE_ADR + 1] = tick - ulLastDptTicks;
			ulLastDptTicks = tick;
		}
	}

	if (DPT_buffer[1] == 'C') //��������̬֡
	{
		bErr = sscanf(DPT_buffer, "$C%fP%fR%f*", &fHead, &fPitch, &fRoll);
		if (bErr == 3)
		{
			tick = GetCurTick();
			wReg[DPT_SAVE_ADR + 2] = (u16)(fHead * 10);
			wReg[DPT_SAVE_ADR + 3] = (short)(fPitch * 10);
			wReg[DPT_SAVE_ADR + 4] = (short)(fRoll * 10);
			wReg[DPT_SAVE_ADR + 5] = tick - ulLastHprTicks;
			ulLastHprTicks = tick;

			if (DPT_SENSOR_ADR <= 0 || DPT_SENSOR_ADR > 10)
				DPT_SENSOR_ADR = 0;
			if (DPT_SET_ADR < 100 || DPT_SET_ADR >= 200)
				DPT_SET_ADR = 199;

			DPT_ANGLE_ADR = wReg[DPT_SENSOR_ADR]; //�Ƕ�ת��
			tmp = wReg[DPT_SENSOR_ADR] - wReg[DPT_SET_ADR];
			if (tmp > 1800)
			{
				DPT_ANGLE_ADR = wReg[DPT_SENSOR_ADR] - 3600;
			}
			if (tmp < -1800)
			{
				DPT_ANGLE_ADR = wReg[DPT_SENSOR_ADR] + 3600;
			}
		}
	}

	DPT_COM_SUCS++;
	DPT_curptr = 0;
	DPT_bRecv = 0;
}

/**-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
*/
void DPT_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(USART_DPT, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
	{
		ch = USART_ReceiveData(USART_DPT); //�����Ĵ��������ݻ��浽���ջ�������

		if (ch == '*' && DPT_curptr > 10 && DPT_bRecv == 0) // Is tail of frame?
		{
			DPT_buffer[DPT_curptr++] = ch;
			DPT_buffer[DPT_curptr] = 0;
			DPT_bRecv = 1;
		}

		if (DPT_curptr > 0 && DPT_bRecv == 0) // Is middle of frame ?
			DPT_buffer[DPT_curptr++] = ch;

		if (ch == '$' && DPT_curptr == 0 && DPT_bRecv == 0) // need receive frame
		{
			DPT_buffer[DPT_curptr++] = ch;
			wReg[7]++;
		}
	}

	if (USART_GetITStatus(USART_DPT, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(USART_DPT, USART_IT_TXE, DISABLE);
	}
}

//-----------------------------End of file--------------------------------------------------
