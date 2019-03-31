/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ��1.5.1�汾�⽨�Ĺ���ģ��
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "SysTick.h"
#include "Modbus_svr.h"
#include "usart_dpt.h"
#include "usart_spd1.h"
#include "usart_spd2.h"
#include "usart_spd3.h"
#include "usart_dam.h"
#include "gpio.h"
#include "bsp_innerflash.h"
#include "plant.h"

#include "spd_comm.h"
#include "BP_comm.h"

extern short wReg[];
extern u8 bChanged;

PID_Module pid1;
PID_Module pid2;
PID_Module pid3;

Plant pt;

int main(void)
{
	int nTestMode = 0; //��ǰ����ģʽ
	int nTestVol;	  //���������ѹ��ֵ
	int nTestCur;	  //��ǰ�����ѹֵ
	short sTestSw = 0; //�ϴβ��԰�ť״̬

	SysTick_Init();												  //tick��ʱ����ʼ
	GPIO_Config();												  //GPIO��ʼ��
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //ͨ�żĴ�����ʼ��
	wReg[102]++;												  //����������1
	bSaved = 1;													  //���浽EPROM

	Modbus_init(); //��λ��ͨ�ų�ʼ��
	DPT_Init();	//��ȼ�ͨ�ų�ʼ��
	SPD1_Init();   //1#������ͨ�ų�ʼ��
	SPD2_Init();   //2#������ͨ�ų�ʼ��
	SPD3_Init();   //3#������ͨ�ų�ʼ��
	DAM_Init();	//ģ����������ʼ��

	wReg[139] = 0;
	wReg[149] = 0;
	wReg[159] = 0;
	wReg[PID_ZERO_ZONE] = 0;
	wReg[120] = 0x7FFF;
	wReg[121] = 0x7FFF;
	wReg[122] = 0x7FFF;
	wReg[123] = 0x7FFF;

	wReg[124] = 2;
	wReg[125] = 153;

	wReg[130] = 27;
	wReg[131] = 121;
	wReg[132] = 6604;
	wReg[134] = 800;
	wReg[135] = 500;
	wReg[136] = 5;
	wReg[137] = 100;
	wReg[138] = 0;

	wReg[140] = 17;
	wReg[141] = 122;
	wReg[142] = 6604;
	wReg[144] = 800;
	wReg[145] = 500;
	wReg[146] = 5;
	wReg[147] = 100;
	wReg[148] = 1;

	wReg[150] = 126;
	wReg[151] = 121;
	wReg[152] = 1;
	wReg[154] = 500;
	wReg[155] = 10;
	wReg[156] = 5000;
	wReg[157] = 70;
	wReg[158] = 1;

	wReg[161] = 0;
	wReg[163] = 20;
	wReg[164] = 20;
	wReg[165] = 60;

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);
	SetTimer(3, 100);
	SetTimer(4, 200);

	IWDG_Configuration(); //���Ź���ʼ
	PIDMod_initialize(&pid1, 130);
	PIDMod_initialize(&pid2, 140);
	PIDMod_initialize(&pid3, 150);

	plant_init(&plant);
	plant_water_set(&plant, 1.0f, 0);

	//bp_plant_init(&pt);

	while (1)
	{
		Modbus_task(); //ͨ�ų�������
		DPT_Task();
		SPD1_Task();
		SPD2_Task();
		SPD3_Task();
		DAM_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //���Ź���λ
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //�����޸Ĺ��ļĴ���
			PIDMod_update_para(&pid1);
			PIDMod_update_para(&pid2);
			PIDMod_update_para(&pid3);
			bSaved = 0;
		}

		if (GetTimer(2)) //100ms cycle
		{
			PIDMod_step(&pid1);
			PIDMod_step(&pid2);
			Thruster_step(&pid3);

			plant_step(&plant, wReg[161] * 10);
			wReg[50] = (int)(plant.angle * 1800.0f / 3.14f);
			wReg[51] = (int)(plant.dangle * 1800.0f / 3.14f);
		}

		if (GetTimer(3)) //100ms cycle
		{
			SPD1_TxCmd(); //��1#����������ȡָ��
			SPD2_TxCmd(); //��2#����������ȡָ��
			SPD3_TxCmd(); //��3#����������ȡָ��
		}

		if (GetTimer(4)) //200ms
		{
			DAM_TxCmd(); //��ģ��������巢��ָ��

			//ģ�������Ի���
			if (wReg[129] == 1)
			{
				if (wReg[127] > 0 && wReg[127] < 5 && nTestMode == 0) //��������
				{
					nTestMode = 1; //����
					nTestVol = (wReg[128] > 0) ? (3276 * wReg[128]) : (-3276 * wReg[128]);
					nTestCur = 0;
				}

				if (nTestMode == 1) //����ģʽ
				{
					wReg[119 + wReg[127]] = (wReg[128] > 0) ? (0x8000 + nTestCur) : (0x8000 - nTestCur);
					nTestCur += 655; //ÿ1s����1V
					if (nTestCur >= nTestVol)
					{
						nTestMode = 2;
						nTestCur = 0;
					}
				}

				if (nTestMode == 2) //�̶��������2����
				{
					nTestCur++;
					if (nTestCur > 10)
					{
						nTestMode = 3;
						nTestCur = nTestVol;
					}
				}

				if (nTestMode == 3) //�½�ģʽ
				{
					wReg[119 + wReg[127]] = (wReg[128] > 0) ? (0x8000 + nTestCur) : (0x8000 - nTestCur);
					nTestCur -= 655;   //ÿ1s����1V
					if (nTestCur <= 0) //ģʽ����ж�
					{
						nTestMode = 0;
						wReg[129] = 0;
					}
				}
			}

			if (wReg[129] == 0 && sTestSw != 0) //��ť�˳�
			{
				nTestMode = 0;
				wReg[119 + wReg[127]] = 0x8000;
			}
			sTestSw = wReg[129];
		}
	}
}

/*********************************************END OF FILE**********************/
