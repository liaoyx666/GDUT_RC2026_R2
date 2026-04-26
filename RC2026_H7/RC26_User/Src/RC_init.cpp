#include "RC_init.h"
/*====================================外设初始化====================================*/
// 定时中断
tim::Tim tim7_1khz(htim7);
tim::Tim tim13_500hz(htim13);
tim::Tim tim4_timer(htim4);

// can通讯
can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

// 虚拟串口上位机通讯
cdc::CDC CDC_HS(cdc::USB_CDC_HS);

// 用于获取us级时间戳
timer::Timer timer_us(&tim4_timer);
/*====================================电机初始化====================================*/



motor::M2006D m2006d_can3_3_4(
	3, can3, &tim13_500hz, 
	4, can3, &tim13_500hz, 
	36, motor::POL_REV, true
);

motor::M3508D m3508d_can3_1_2(
	1, can3, &tim7_1khz, 
	2, can3, &tim7_1khz, 
	3591.f / 187.f, motor::POL_REV, true
);

motor::M2006 m2006_can3_5(5, can3, &tim13_500hz);
	
motor::DM4310 dm4310_can3_0x13(0x12, can3, &tim7_1khz);

/*====================================模块====================================*/
data::RobotPose robot_pose;// 机器人位姿

ros::Radar 		radar(CDC_HS, 1, robot_pose);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收

lidar::LiDAR lidar_1(huart3);// 激光测距

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控


//path::HeadCtrl head_ctrl(
//	robot_pose,
//	swerve_4_chassis,
//	0
//);

//path::TrajTrack3 track(
//	robot_pose,
//	swerve_4_chassis,
//	head_ctrl,
//	0.02
//);

//path::PathPlan3 pp(
//	path::LonConstr3(2.5, 2.3),
//	path::HeadConstr3(0, 3, 4, false),
//	track
//);


//path::GraphPlan gp(robot_pose, pp);

//path::HeadCheck headcheck(1, PI, track, robot_pose);

/*====================================DeBug====================================*/
SquareWave wave(1000, 3000);// 用于调pid

vector2d::Vector2D pc;
float target = 0;
float a = 0;

uint8_t s = 0, e = 0;

void test(void *argument)
{
	wave.Init();
	
	
	remote_ctrl.signal_swa();
	remote_ctrl.signal_swd();

	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
   
	
		

//		if (remote_ctrl.swa == 1)
//		{
//			pp.Enable();
//		}
//		else
//		{
//			pp.Disable();
//			
//			swerve_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 150.f, -remote_ctrl.left_x / 150.f), -remote_ctrl.right_x / 100.f);
//		}

//		if (remote_ctrl.swc == 0)
//		{
//			if (remote_ctrl.signal_swd())
//			{
//				swerve_4_chassis.Chassis_Re_Init(); /*舵轮校准*/
//			}
//		}
//		else if (remote_ctrl.swc == 1)
//		{
//			if (remote_ctrl.signal_swd())
//			{
//				radar.Reposition(); /*雷达重定位*/
//			}
//		}
		

		
		osDelay(1);
	}
}

task::TaskCreator test_task("test", 20, 512, test, NULL);
 
/*====================================初始化函数====================================*/
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
	tim13_500hz.Tim_It_Start();
	
	lidar_1.Uart_Rx_Start();

	data::Init_Side(true); /*初始化场地位置*/
	
	/*------------------------------------------------------------------------------*/
}
