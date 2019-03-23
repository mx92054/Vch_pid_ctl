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


	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);
	SetTimer(3, 100);
	SetTimer(4, 200);

	IWDG_Configuration(); //���Ź���ʼ
	PIDMod_initialize(&pid1, 130);
	PIDMod_initialize(&pid2, 140);
	PIDMod_initialize(&pid3, 150);

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

		if (GetTimer(2))
		{
			PIDMod_step(&pid1);
			PIDMod_step(&pid2);
			Thruster_step(&pid3);

			/*bp_plant_step(&pt, (float)(wReg[170] - 0x8000));
			wReg[50] = (short)(pt.out - 0x8000);
			wReg[171] = (short)(pt.out - 0x8000);*/
		}

		if (GetTimer(3))
		{
			SPD1_TxCmd(); //��1#����������ȡָ��
			SPD2_TxCmd(); //��2#����������ȡָ��
			SPD3_TxCmd(); //��3#����������ȡָ��
		}

		if (GetTimer(4))
		{
			DAM_TxCmd(); //��ģ��������巢��ָ��
		}
	}
}

/*********************************************END OF FILE**********************/
