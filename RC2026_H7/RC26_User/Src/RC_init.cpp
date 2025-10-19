#include "RC_init.h"

/*--------------------------------外设初始化------------------------------------------*/
tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

cdc::CDC CDC_HS(cdc::USB_CDC_HS);// 虚拟串口



/*----------------------------------电机初始化----------------------------------------*/
//motor::M6020 m6020_1(1, can2, tim7_1khz);
//motor::Go go_1(0, 3, can3, tim7_1khz);
motor::M3508 m3508_1(1, can1, tim7_1khz);
motor::M3508 m3508_2(2, can1, tim7_1khz);
motor::M3508 m3508_3(3, can1, tim7_1khz);

motor::M2006 m2006_4(4, can1, tim7_1khz);

//motor::DM4310 dm4310(1, can3, tim7_1khz);

motor::J60 j60_1(1, can3, tim7_1khz);


/*-------------------------------软件模块初始化---------------------------------------*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳

path::PathPlan path_plan(2, 1.f);



/*--------------------------------硬件模块初始化--------------------------------------*/
ros::Radar radar(CDC_HS, 1);// 雷达数据接收
ros::BestPath MF_path(CDC_HS, 3);
ros::Map map(CDC_HS, 2);


chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 3, 3);

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控



/*---------------------------————-----DeBug------------------------------------------*/
SquareWave wave(1000, 3000);// 用于调pid
//SinWave sin_wave(1000, 3000);


float target = 0;
float a = 0;
float w = 70;

void test(void *argument)
{
	//sin_wave.Init();
	wave.Init();
	j60_1.Reset_Out_Pos(0);
	
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		

		uart_printf("%f,%f\n", j60_1.Get_Out_Pos(), target);
		
		j60_1.Set_Out_Pos(a);
		
		
		
		
		osDelay(1);
	}
}

task::TaskCreator test_task("test", 20, 512, test, NULL);
   


vector2d::Vector2D location(0, 0);
float yaw = 0;
float sx, sy, sa;

void path_teat(void *argument)
{
	MF_path.MF_Best_Path_Plan(map, path_plan);

	for (;;)
	{
		
		
		
		if (remote_ctrl.swb == 0)
		{
			omni_chassis.Set_Chassis_World_Spd(0, 0, 0, -radar.yaw / 180.f * PI);
		}
		else if (remote_ctrl.swb == 1)
		{
			omni_chassis.Set_Chassis_World_Spd(remote_ctrl.left_x / 300.f, remote_ctrl.left_y / 300.f, remote_ctrl.right_x / 300.f, -radar.yaw / 180.f * PI);
		}
		else
		{
			path_plan.Get_Speed(
				vector2d::Vector2D(radar.x, radar.y),
				-radar.yaw / 180.f * PI,
				&sx,
				&sy,
				&sa
			);

			omni_chassis.Set_Chassis_World_Spd(sx, sy, sa, -radar.yaw / 180.f * PI);
		}
		
		
		static uint8_t flag = 0;
		static uint32_t last_time = 0;
		
		if (path_plan.Is_End() == true && flag == 0)
		{
			last_time = timer_us.Get_TimeStamp();
			flag = 1;
		}
		else if (flag == 1 && timer_us.Get_DeltaTime(last_time) >= 3000000)// 延时3s
		{
			path_plan.Next_Path();
			flag = 0;
		}
		
		
		
		osDelay(1);
	}
}


//task::TaskCreator path_task("test", 27, 512, path_teat, NULL);

/*---------------------------————-----初始化函数—————------------------------------------------*/
void All_Init()
{
	
	
	/*------------------------------------外设初始化------------------------------------------*/
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
	
	
	/*------------------------------------------------------------------------------*/
	
	
	
	
}



