/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   用1.5.1版本库建的工程模板
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

#define ushort unsigned short

extern short wReg[];
extern u8 bChanged;

PID_Module pid1;
PID_Module pid2;
PID_Module pid3;

Plant pt;

int main(void)
{
	int nTestMode = 0; //当前测试模式
	int nTestVol;	  //最终输出电压数值
	int nTestCur;	  //当前输出电压值
	short sTestSw = 0; //上次测试按钮状态

	ushort lastDAVal[4]; //上一次DA输出值
	ushort currDAVal; //本次DA输出值
	ushort DAVal[4];	 //本次DA输出值保存
	int bMaxDAVal[4];	//当前是否是最大输出状态
	int i, DAChn;

	SysTick_Init();												  //tick定时器初始
	GPIO_Config();												  //GPIO初始化
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //通信寄存器初始化
	wReg[102]++;												  //启动次数加1
	bSaved = 1;													  //保存到EPROM

	Modbus_init(); //上位机通信初始化
	DPT_Init();	//深度计通信初始化
	SPD1_Init();   //1#编码器通信初始化
	SPD2_Init();   //2#编码器通信初始化
	SPD3_Init();   //3#编码器通信初始化
	DAM_Init();	//模拟量输出板初始化

	wReg[139] = 0;
	wReg[149] = 0;
	wReg[159] = 0;
	wReg[PID_ZERO_ZONE] = 0;
	wReg[120] = 0x8000;
	wReg[121] = 0x8000;
	wReg[122] = 0x8000;
	wReg[123] = 0x8000;

	wReg[127] = 1;
	wReg[128] = 5;
	wReg[129] = 0;

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
	wReg[154] = 800;
	wReg[155] = 3;
	wReg[156] = 8000;
	wReg[157] = 100;
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

	IWDG_Configuration(); //看门狗初始
	PIDMod_initialize(&pid1, 130);
	PIDMod_initialize(&pid2, 140);
	PIDMod_initialize(&pid3, 150);

	//plant_init(&plant);
	//plant_water_set(&plant, 1.0f, 0);

	//bp_plant_init(&pt);
	for (i = 0; i < 4; i++)
	{
		lastDAVal[i] = (ushort)wReg[120 + i];
		bMaxDAVal[i] = 0;
	}

	while (1)
	{
		Modbus_task(); //通信出来进程
		DPT_Task();
		SPD1_Task();
		SPD2_Task();
		SPD3_Task();
		DAM_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //看门狗复位
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //保存修改过的寄存器
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

			//plant_step(&plant, wReg[161] * 10);
			//wReg[50] = (int)(plant.angle * 1800.0f / 3.14f);
			//wReg[51] = (int)(plant.dangle * 1800.0f / 3.14f);
			//每次模拟量输出值过零时，输出一个最大的值100ms时间
			for (i = 0; i < 4; i++)
			{
				DAChn = 120 + i;
				currDAVal = (ushort)wReg[DAChn];

				//情形1 当前处于最大或最小输出状态
				if (bMaxDAVal[i])
				{
					bMaxDAVal[i]--;
					if (bMaxDAVal[i] == 0)
					{
						wReg[DAChn] = (short)DAVal[i];
					}
				}

				//情形2 输出值从零或负变为正输出
				if (lastDAVal[i] <= 0x8000 && currDAVal > 0x8000 && !bMaxDAVal[i])
				{
					DAVal[i] = (ushort)wReg[DAChn];
					wReg[DAChn] = 0xFFFF; //真输出最大
					bMaxDAVal[i] = 2;
				}
				//情形3 输出值从零或正变为负输出
				if (lastDAVal[i] >= 0x8000 && currDAVal < 0x8000 && !bMaxDAVal[i])
				{
					DAVal[i] = (ushort)wReg[DAChn];
					wReg[DAChn] = 0x0000; //真输出最大
					bMaxDAVal[i] = 2;
				}

				lastDAVal[i] = (ushort)wReg[DAChn];
			}
		}

		if (GetTimer(3)) //100ms cycle
		{
			SPD1_TxCmd(); //向1#编码器发读取指令
			SPD2_TxCmd(); //向2#编码器发读取指令
			SPD3_TxCmd(); //向3#编码器发读取指令
		}

		if (GetTimer(4)) //200ms
		{
			DAM_TxCmd(); //向模拟量输出板发出指令

			//模拟量测试环节
			if (wReg[129] == 1)
			{
				if (wReg[127] > 0 && wReg[127] < 5 && nTestMode == 0) //启动测试
				{
					if (wReg[128] < -10)
						wReg[128] = -10;
					if (wReg[128] > 10)
						wReg[128] = 10;
					nTestMode = 1; //上升
					nTestVol = (wReg[128] > 0) ? (3270 * wReg[128]) : (-3270 * wReg[128]);
					nTestCur = 0;
				}

				if (nTestMode == 1) //上升模式
				{
					wReg[119 + wReg[127]] = (wReg[128] > 0) ? (0x8000 + nTestCur) : (0x8000 - nTestCur);
					nTestCur += 655; //每1s增加1V
					if (nTestCur >= nTestVol)
					{
						nTestMode = 2;
						nTestCur = 0;
					}

					if ((wReg[127] == 0 && (wReg[37] > 5 || wReg[37] < -5)) ||
						(wReg[127] == 1 && (wReg[27] > 5 || wReg[27] < -5)) ||
						(wReg[127] == 2 && (wReg[17] > 5 || wReg[17] < -5)))
					{
						nTestMode = 0;
						wReg[129] = 0;
					}
				}

				if (nTestMode == 2) //固定输出保持2秒钟
				{
					nTestCur++;
					if (nTestCur > 10)
					{
						nTestMode = 3;
						nTestCur = nTestVol;
					}
				}

				if (nTestMode == 3) //下降模式
				{
					wReg[119 + wReg[127]] = (wReg[128] > 0) ? (0x8000 + nTestCur) : (0x8000 - nTestCur);
					nTestCur -= 655;   //每1s减少1V
					if (nTestCur <= 0) //模式完成判断
					{
						nTestMode = 0;
						wReg[129] = 0;
					}
				}
			}

			if (wReg[129] == 0 && sTestSw != 0) //按钮退出
			{
				nTestMode = 0;
				wReg[119 + wReg[127]] = 0x8000;
			}
			sTestSw = wReg[129];
		}
	}
}

/*********************************************END OF FILE**********************/
