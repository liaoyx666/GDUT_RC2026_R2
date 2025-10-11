#include "RC_init.h"

/*--------------------------------外设初始化------------------------------------------*/
tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

cdc::CDC CDC_HS(cdc::USB_CDC_HS);// 虚拟串口



/*----------------------------------电机初始化----------------------------------------*/
//motor::M6020 m6020_1(1, can1, tim7_1khz);
//motor::Go go_1(0, 3, can2, tim7_1khz);
motor::M3508 m3508_1(1, can1, tim7_1khz);
motor::M3508 m3508_2(2, can1, tim7_1khz);
motor::M3508 m3508_3(3, can1, tim7_1khz);



/*-------------------------------软件模块初始化---------------------------------------*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳

path::PathPlan path_plan(1, 1, 1);

/*--------------------------------硬件模块初始化--------------------------------------*/
ros::Radar radar(CDC_HS, 1);// 雷达数据接收

chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 1.5, 2);

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控



/*---------------------------————-----DeBug------------------------------------------*/
//SquareWave wave(1000, 3000);// 用于调pid
//SinWave sin_wave(1000, 3000);


//float target = 0;
//float a = 0;

void test(void *argument)
{
	//sin_wave.Init();
	//wave.Init();
	for (;;)
	{
		//wave.Set_Amplitude(a);
		//target = wave.Get_Signal();
		
		//uint8_t aaa[8] = {1,2,3};
		//omni_chassis.Set_Chassis_World_Spd(remote_ctrl.left_x / 100.f, remote_ctrl.left_y / 100.f, remote_ctrl.right_x / 100.f, -radar.yaw / 180.f * PI);
		
		//CDC_HS.CDC_AddToBuf(aaa, 8, 1);
		
		//uart_printf("%f,%f\n", go_1.pos, target);
		
		//go_1.Set_Pos(target);
		
		
		
		
		osDelay(1);
	}
}

task::TaskCreator test_task("test", 20, 128, test, NULL);
    


vector2d::Vector2D location(0, 0);
float yaw = 0;
float sx, sy, sa;

void path_teat(void *argument)
{
	path_plan.Add_Path_Point(vector2d::Vector2D(0, 0), false);// 起点
	
	path_plan.Add_Path_Point(vector2d::Vector2D(0, 1), false, 0);
	
	path_plan.Add_Path_Point(vector2d::Vector2D(1, 1), false, 0.4);
	
	path_plan.Add_Path_Point(vector2d::Vector2D(1, 0), false, 0.5);
	
	path_plan.Add_Path_Point(vector2d::Vector2D(2, 0), false, 0.5);
	
	path_plan.Add_Path_Point(vector2d::Vector2D(2, 1), true);
	
	
	//osDelay(10000);
	
	for (;;)
	{
	
		path_plan.Get_Speed(
			vector2d::Vector2D(radar.x, radar.y),
			-radar.yaw / 180.f * PI,
			&sx,
			&sy,
			&sa
		);
		
		
		if (remote_ctrl.swb == 0)
		{
			omni_chassis.Set_Chassis_World_Spd(0, 0, 0, -radar.yaw / 180.f * PI);
		}
		else if (remote_ctrl.swb == 1)
		{
			omni_chassis.Set_Chassis_World_Spd(remote_ctrl.left_x / 100.f, remote_ctrl.left_y / 100.f, remote_ctrl.right_x / 100.f, -radar.yaw / 180.f * PI);
		}
		else
		{
			omni_chassis.Set_Chassis_World_Spd(sx, sy, remote_ctrl.right_x / 100.f, -radar.yaw / 180.f * PI);
		}
		
		osDelay(1);
	}
}


task::TaskCreator path_task("test", 27, 512, path_teat, NULL);

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



