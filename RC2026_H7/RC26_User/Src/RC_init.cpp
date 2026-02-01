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
timer::Timer timer_us(tim4_timer);
/*====================================电机初始化====================================*/
// 3全向轮底盘电机-------------------------------------------
//motor::M3508 m3508_1(1, can3, tim7_1khz);
//motor::M3508 m3508_2(2, can3, tim7_1khz);
//motor::M3508 m3508_3(3, can3, tim7_1khz);
// ----------------------------------------------------------


// 4舵轮底盘电机---------------------------------------------
motor::M2006 m2006_1_can3(1, can3, tim13_500hz, 4.f * 36.f);
motor::M2006 m2006_2_can3(2, can3, tim13_500hz, 4.f * 36.f);
motor::M2006 m2006_3_can3(3, can3, tim13_500hz, 4.f * 36.f);
motor::M2006 m2006_4_can3(4, can3, tim13_500hz, 4.f * 36.f);

motor::Vesc vesc_101_can3(101, can3, tim13_500hz, 21);
motor::Vesc vesc_102_can3(102, can3, tim13_500hz, 21);
motor::Vesc vesc_103_can3(103, can3, tim13_500hz, 21);
motor::Vesc vesc_104_can3(104, can3, tim13_500hz, 21);
// -----------------------------------------------------------
//otor::Vesc vesc_101_can1(101, can3, tim13_500hz, 21, true);
//motor::M6020 m6020_5_can1(6, can1, tim7_1khz, true, motor::M6020CtrlType::VOLTAGE);

// 机械臂电机-------------------------------------------------
//motor::M2006 	m2006_4		(3, 	can1, tim7_1khz, 36.f * 2.f);
//motor::M3508 	m3508_2_c1	(2, 	can1, tim7_1khz, 51.f * 1.2f);
//motor::J60 		j60_1		(1, 	can1, tim7_1khz);
//motor::Go 		go_0_3		(0, 3, 	can1, tim7_1khz);
// -----------------------------------------------------------


// 轮腿电机---------------------------------------------------
//motor::RS04 	rs04_120(120, can3, tim7_1khz, true, 0, 0);
//motor::RS04 	rs04_127(127, can3, tim7_1khz, true, 0, 0);
// -----------------------------------------------------------


// 4撑杆电机-------------------------------------------------
motor::M3508 m3508_left_front_can2(1, can2, tim7_1khz, 10 * 3591.f / 187.f, true);
motor::M3508 m3508_left_behind_can2(2, can2, tim7_1khz, 99.506f, true);
motor::M3508 m3508_right_behind_can2(3, can2, tim7_1khz, 99.506f, true);
motor::M3508 m3508_right_front_can2(4, can2, tim7_1khz, 10 * 3591.f / 187.f, true);
// -----------------------------------------------------------

// 主动轮电机-------------------------------------------------
motor::M2006 	m2006_5_can2(5, can2, tim7_1khz, 36.f);
motor::M2006 	m2006_6_can2(6, can2, tim7_1khz, 36.f);
// -----------------------------------------------------------


/*====================================模块====================================*/
data::RobotPose robot_pose;// 机器人位姿

ros::Radar 		radar(CDC_HS, 1, robot_pose);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收

lidar::LiDAR lidar_1(huart3);

// 4舵轮底盘
chassis::Swerve4Chassis swerve_4_chassis(
	m2006_1_can3, m2006_2_can3, m2006_3_can3, m2006_4_can3,
	vesc_101_can3, vesc_102_can3, vesc_103_can3, vesc_104_can3,
	2.5, 5, 5,
	4, 7, 7,
	GPIO_PIN_2, GPIO_PIN_9, GPIO_PIN_14, GPIO_PIN_15
);


path::PathPlan2 path_plan(
	robot_pose, swerve_4_chassis,
	1, 2.5, 2.5,
	2, 4.5, 4.5,
	0.01, 0.01// 死区
);


// 4撑杆
chassis_jack::Chassis_jack chassis_jack_test(
	1, 2, 3,
	path_plan,
	m3508_left_front_can2,  m3508_left_behind_can2,
	m3508_right_front_can2, m3508_right_behind_can2,
	m2006_5_can2, m2006_6_can2,
	1.f,
	lidar_1,
	swerve_4_chassis,
	2.5, 0.6, 0.8, 0.7,
	GPIOA, GPIO_PIN_8,
	GPIOG, GPIO_PIN_1,
	GPIOA, GPIO_PIN_9,
	GPIOG, GPIO_PIN_0
);

//Arm_task arm_task(tim7_1khz);

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控

imu::JY901S jy901s(huart1);// 陀螺仪  


/*====================================模块====================================*/

//path::PathPlan path_plan(2, 1.f);// 路径规划

/*====================================DeBug====================================*/
SquareWave wave(1000, 3000);// 用于调pid

float target = 0;
float a = 0;

float wl1 = 0, wl2 = 0;


void test(void *argument)
{
//	sin_wave.Init();
	wave.Init();
	
//	m3508_2_c1.pid_spd.Pid_Mode_Init(true, false, 0.01);
//	m3508_2_c1.pid_spd.Pid_Param_Init(10, 0.54, 0, 0, 0.001, 0, 16384, 10000, 5000, 5000, 5000);// 1ms

//	m3508_2_c1.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
//	m3508_2_c1.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 2000, 1000, 500, 500, 500, 1000, 1800);// 1ms

//	j60_1.pid_pos.Pid_Mode_Init(true, true, 0.1);
//	j60_1.pid_spd.Pid_Param_Init(0.1, 0.0006, 0, 0, 0.001, 0, 10, 5, 5, 5, 5);

//	j60_1.pid_pos.Pid_Mode_Init(false, false, 0.1, true);
//	j60_1.pid_pos.Pid_Param_Init(250, 0, 15, 0, 0.001, 0, 100, 5, 5, 5, 5, 50, 80);

//	go_0_3.Set_Out_Pos(0);
//	m2006_4.Set_Out_Pos(0);
//	m3508_2_c1.Set_Out_Pos(0);
//	j60_1.Set_Out_Pos(0);

	m3508_left_front_can2.Set_Out_Angle(0);
	m3508_left_behind_can2.Set_Out_Pos(0);
	m3508_right_behind_can2.Set_Out_Pos(0);
	m3508_right_front_can2.Set_Out_Angle(0);

	remote_ctrl.signal_swa();
	

	
//	path_plan.Add_Point(
//		vector2d::Vector2D(1.147, -0.276),					
//		0,
//		0,									
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(7.628, -0.231),					
//		0,
//		0,							
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(8.273, -0.211),					
//		0.3,
//		0,							
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(8.298, -1.025),					
//		0,
//		0,									
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(8.421, -4.372),					
//		0,
//		0,									
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(8.447, -5.051),					
//		0.3,
//		0,									
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(7.421, -5.099),					
//		0,
//		0,									
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	path_plan.Add_Point(
//		vector2d::Vector2D(2.147, -5.093),					
//		0,
//		0,									
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,																									
//		0
//	);
//	
//	
//	path_plan.Add_End_Point(
//		vector2d::Vector2D(1.040, -2.251),					// 坐标  
//		0,				// 到达前目标yaw					
//		0,		// 离开前目标yaw					
//		PATH_MAX_PARAM,									
//		PATH_MAX_PARAM,											
//		PATH_MAX_PARAM,												
//		PATH_MAX_PARAM,										
//		PATH_MAX_PARAM,														
//		PATH_MAX_PARAM,		
//		false,				// 是否停止																				
//		0				// 事件id
//	);

////////////////////////////////////////////////

	path_plan.Add_Point(
		vector2d::Vector2D(1.396, 1.567),					
		0,
		0,									
		PATH_MAX_PARAM,									
		PATH_MAX_PARAM,											
		PATH_MAX_PARAM,												
		PATH_MAX_PARAM,										
		PATH_MAX_PARAM,														
		PATH_MAX_PARAM,																									
		0
	);
	
	
	path_plan.Add_Point(
		vector2d::Vector2D(1.921, 1.601),					
		0,
		0,				
		1,
		PATH_MAX_PARAM,										
		PATH_MAX_PARAM,												
		PATH_MAX_PARAM,										
		PATH_MAX_PARAM,														
		PATH_MAX_PARAM,																									
		1
	);
	
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(3.346, 1.616),// 坐标  
		PATH_NO_TARGET_YAW,// 到达前目标yaw					
		0,// 离开前目标yaw					
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		1// 事件id
	);
		

		
		
	path_plan.Add_End_Point(
		vector2d::Vector2D(4.587, 1.673),// 坐标  
		PATH_NO_TARGET_YAW,// 到达前目标yaw					
		0,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		1// 事件id
	);
	
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(5.779, 1.695),// 坐标  
		PATH_NO_TARGET_YAW,// 到达前目标yaw					
		0,// 离开前目标yaw				
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		2// 事件id
	);
	
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(5.829, 1.695),// 坐标  
		PATH_NO_TARGET_YAW,// 到达前目标yaw					
		-PI / 2,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		0// 事件id
	);
	
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(5.789, 2.866),// 坐标  
		-PI / 2,// 到达前目标yaw					
		-PI / 2,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		2// 事件id
	);
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(5.789, 2.916),// 坐标  
		-PI / 2,// 到达前目标yaw					
		-PI,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		0// 事件id
	);
	
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(6.987, 2.995),// 坐标  
		-PI,// 到达前目标yaw					
		-PI,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		2// 事件id
	);
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(7.037, 2.995),// 坐标  
		-PI,// 到达前目标yaw					
		-PI,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		0// 事件id
	);
	
	
	path_plan.Add_End_Point(
		vector2d::Vector2D(8.306, 3.011),// 坐标  
		-PI,// 到达前目标yaw					
		-PI,// 离开前目标yaw					
		1,
		1,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		PATH_MAX_PARAM,
		false,// 是否停止																				
		0// 事件id
	);
		

	
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();

		//m6020_5_can1.Set_Out_Pos(target);
		
//		vesc_101_can1.Set_Out_Rpm(a);

		//path_plan.Enable();
//		uart_printf("%f,%f\n", vesc_101_can1.Get_Out_Rpm(), a);
		
		if (remote_ctrl.swa == 0)
		{
			path_plan.Disable();
			
			chassis_jack_test.Up_Or_Down_Steps(remote_ctrl.signal_swd(), remote_ctrl.swc);
			
			swerve_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 200.f, -remote_ctrl.left_x / 200.f), -remote_ctrl.right_x / 100.f, *robot_pose.Get_pYaw());
		}
		else
		{
			chassis_jack_test.Up_Or_Down_Event();
			
			path_plan.Enable();
		}
		
		
		chassis_jack_test.Set_Vel(swerve_4_chassis.Get_Vel().x());
		
		osDelay(1);
	}
}


task::TaskCreator test_task("test", 20, 1024, test, NULL);




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
	
	jy901s.Uart_Rx_Start();
	lidar_1.Uart_Rx_Start();

	/*------------------------------------------------------------------------------*/
	
}
