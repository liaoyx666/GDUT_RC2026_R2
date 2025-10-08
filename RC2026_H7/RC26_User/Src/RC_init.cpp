#include "RC_init.h"

/*--------------------------------外设初始化------------------------------------------*/
tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

cdc::CDC CDC_HS(cdc::USB_CDC_HS);// 虚拟串口



/*----------------------------------电机初始化----------------------------------------*/
motor::M6020 m6020_1(1, can1, tim7_1khz);
motor::Go go_1(0, 3, can2, tim7_1khz);



/*-------------------------------软件模块初始化---------------------------------------*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳



/*--------------------------------硬件模块初始化--------------------------------------*/
ros::Radar radar(CDC_HS, 1);// 雷达数据接收

//chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 2, 2);

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控



/*---------------------------————-----DeBug------------------------------------------*/
SquareWave wave(1000, 3000);// 用于调pid
SinWave sin_wave(1000, 3000);

float target = 0;
float a = 0;

void test(void *argument)
{
	sin_wave.Init();
	wave.Init();
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		
		//uint8_t aaa[8] = {1,2,3};
		//omni_chassis.Set_Chassis_Spd(remote_ctrl.left_x / 100.f, remote_ctrl.left_y / 100.f, remote_ctrl.right_x / 100.f);
		//CDC_HS.CDC_AddToBuf(aaa, 8, 1);
		
		uart_printf("%f,%f\n", go_1.pos, target);
		
		go_1.Set_Pos(target);
		
		osDelay(1);
	}
}

task::TaskCreator test_task("test", 20, 256, test, NULL);
    


/*---------------------------————-----初始化函数—————------------------------------------------*/
void All_Init()
{
	can1.Can_Filter_Init(FDCAN_STANDARD_ID, 1, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can1.Can_Filter_Init(FDCAN_EXTENDED_ID, 2, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can1.Can_Start();

	can2.Can_Filter_Init(FDCAN_STANDARD_ID, 3, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can2.Can_Filter_Init(FDCAN_EXTENDED_ID, 4, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can2.Can_Start();
	
	can3.Can_Filter_Init(FDCAN_STANDARD_ID, 5, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can3.Can_Filter_Init(FDCAN_EXTENDED_ID, 6, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can3.Can_Start();

	tim4_timer.Tim_It_Start();
	tim7_1khz.Tim_It_Start();
}



