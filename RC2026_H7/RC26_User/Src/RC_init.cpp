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

<<<<<<< Updated upstream
=======
// 底盘
//motor::M3508 m3508_1(1, can3, tim7_1khz);
//motor::M3508 m3508_2(2, can3, tim7_1khz);
//motor::M3508 m3508_3(3, can3, tim7_1khz);
// motor::M2006 m2006_1_can3(1, can3, tim7_1khz, 4.f * 36.f);
// motor::M2006 m2006_2_can3(2, can3, tim7_1khz, 4.f * 36.f);
// motor::M2006 m2006_3_can3(3, can3, tim7_1khz, 4.f * 36.f);
// motor::M2006 m2006_4_can3(4, can3, tim7_1khz, 4.f * 36.f);


//jack test
motor::M3508 m3508_left_front_can2(1, can2, tim7_1khz, 10 * 3591.f / 187.f);
motor::M3508 m3508_left_behind_can2(2, can2, tim7_1khz, 99.506f);
motor::M3508 m3508_right_behind_can2(3, can2, tim7_1khz, 99.506f);
motor::M3508 m3508_right_front_can2(4, can2, tim7_1khz, 10 * 3591.f / 187.f);
<<<<<<< Updated upstream
//
=======
// -----------------------------------------------------------

lidar::LiDAR LiDAR_1(huart3);
>>>>>>> Stashed changes

>>>>>>> Stashed changes


/*-------------------------------软件模块初始化---------------------------------------*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳

path::PathPlan path_plan(1, 1, 1);

/*--------------------------------硬件模块初始化--------------------------------------*/
<<<<<<< Updated upstream
ros::Radar radar(CDC_HS, 1);// 雷达数据接收

<<<<<<< Updated upstream
chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 1.5, 2);

=======
ros::Radar 		radar(CDC_HS, 1);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收


// 3全向轮底盘
//chassis::OmniChassis omni_chassis(
//	m3508_3, m3508_1, m3508_2, 3, 3
//);


// 4舵轮底盘
chassis::Swerve4Chassis swerve_4_chassis(
	m2006_1_can3, m2006_2_can3, m2006_3_can3, m2006_4_can3,
	vesc_101_can3, vesc_102_can3, vesc_103_can3, vesc_104_can3,
	0.5, 6, 6,
	4, 8, 8,
	GPIO_PIN_2, GPIO_PIN_9, GPIO_PIN_14, GPIO_PIN_15
);

// 4撑杆
chassis_jack::Chassis_jack chassis_jack_test(
	m3508_left_front_can2, 
	m3508_left_behind_can2, 
	m3508_right_front_can2,
	m3508_right_behind_can2,
	LiDAR_1
);

>>>>>>> Stashed changes
flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控


=======
//chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 3, 3);// 三全向轮底盘

// chassis::Swerve4Chassis swerve_4_chassis(
// 	m2006_1_can3, m2006_2_can3, m2006_3_can3, m2006_4_can3,
// 	vesc_101_can3, vesc_102_can3, vesc_103_can3, vesc_104_can3,
// 	3, 1, 1,
// 	1, 1, 1,
// 	GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
// );

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控

imu::JY901S jy901s(huart1);
liDAR::LiDAR LiDAR_1(huart3);

//jack test
chassis_jack::Chassis_jack chassis_jack_test(m3508_left_front_can2, m3508_left_behind_can2, m3508_right_front_can2, m3508_right_behind_can2, LiDAR_1);
>>>>>>> Stashed changes

/*---------------------------————-----DeBug------------------------------------------*/
//SquareWave wave(1000, 3000);// 用于调pid
//SinWave sin_wave(1000, 3000);

<<<<<<< Updated upstream

//float target = 0;
//float a = 0;
=======
float target = 0;
float a = 0;
uint8_t c = 0;

	
float wl1 = 0, wl2 = 0;

float a1 = 0, a2 = 0, a3 = 0, a4 = 0;
>>>>>>> Stashed changes

void test(void *argument)
{
	//sin_wave.Init();
<<<<<<< Updated upstream
	//wave.Init();
=======
	wave.Init();
	
	j60_1_can1.Reset_Out_Pos(0);
	m2006_4_can1.Reset_Out_Pos(0);
	go_0_3_can2.Reset_Out_Pos(0);
	m3508_2_can1.Reset_Out_Pos(0);
<<<<<<< Updated upstream
	
	//
	m3508_left_front_can2.Reset_Out_Pos(0);
	m3508_left_front_can2.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	m3508_left_front_can2.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 3000, 7000);// 1ms
	
	m3508_right_front_can2.Reset_Out_Pos(0);
	m3508_right_front_can2.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	m3508_right_front_can2.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000, 4000, 2000, 2000, 2000, 3000, 7000);
	
	m3508_right_behind_can2.Reset_Out_Pos(0);
	m3508_right_behind_can2.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	m3508_right_behind_can2.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 3000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
	
	m3508_left_behind_can2.Reset_Out_Pos(0);
	m3508_left_behind_can2.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	m3508_left_behind_can2.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 8000 / ((10 * 3591.f / 187.f) / 99.506f), 4000, 2000, 2000, 2000, 3000 / ((10 * 3591.f / 187.f) / 99.506f), 7000 / ((10 * 3591.f / 187.f) / 99.506f));
	//
	
	//go_0_0.Reset_Out_Pos(0);
	
=======

	m3508_left_front_can2.Reset_Out_Pos(0);
	m3508_left_front_can2.Reset_Out_Angle(0);
		
	m3508_right_front_can2.Reset_Out_Pos(0);
	m3508_right_front_can2.Reset_Out_Angle(0);
		
	m3508_right_behind_can2.Reset_Out_Pos(0);
	m3508_right_behind_can2.Reset_Out_Angle(0);

	m3508_left_behind_can2.Reset_Out_Pos(0);
	m3508_left_behind_can2.Reset_Out_Angle(0);

>>>>>>> Stashed changes
//	rs04_120.Set_ZeroPos();
//	rs04_120.Set_K_Pos(50);
//	rs04_120.Set_K_Spd(10);
//	
//	rs04_127.Set_ZeroPos();
//	rs04_127.Set_K_Pos(50);
//	rs04_127.Set_K_Spd(10);
<<<<<<< Updated upstream
//	m2006_4_can3.pid_pos.Pid_Mode_Init(false, false, 0, false);
//	m2006_4_can3.pid_pos.Pid_Param_Init(200, 0, 0, 0, 0.001, 0, 12000, 10000, 10000, 10000, 10000);

>>>>>>> Stashed changes
=======
	osDelay(10);
>>>>>>> Stashed changes
	for (;;)
	{
		//wave.Set_Amplitude(a);
		//target = wave.Get_Signal();
		
		//uint8_t aaa[8] = {1,2,3};
		//omni_chassis.Set_Chassis_World_Spd(remote_ctrl.left_x / 100.f, remote_ctrl.left_y / 100.f, remote_ctrl.right_x / 100.f, -radar.yaw / 180.f * PI);
		
		//CDC_HS.CDC_AddToBuf(aaa, 8, 1);
		
<<<<<<< Updated upstream
		//uart_printf("%f,%f\n", go_1.pos, target);
		
		//go_1.Set_Pos(target);
		
<<<<<<< Updated upstream
=======
//		swerve_4_chassis.Set_Robot_Vel(vector2d::Vector2D(remote_ctrl.left_y / 400.f, -remote_ctrl.left_x / 400.f), remote_ctrl.right_x / 400.f);
=======


		chassis_jack_test.chassis_test(remote_ctrl.signal_swd());

>>>>>>> Stashed changes
		
		//uart_printf("%f,%f\n", m6020_1.Get_Rpm(), target);
//		m6020_1.Set_Rpm(target);

		
		go_0_3_can2.Set_Out_Pos(a1);
		j60_1_can1.Set_Out_Pos(a2);
		m3508_2_can1.Set_Out_Pos(a3);
		m2006_4_can1.Set_Out_Pos(a4);
		
		//m3508_5_can1.Set_Out_Pos(a);
		//m3508_left_front_can2.Set_Out_Pos(a);
		//m3508_right_front_can2.Set_Out_Pos(-a);
		//m3508_left_behind_can2.Set_Out_Pos(a);
		//m3508_right_behind_can2.Set_Out_Pos(-a);
		//chassis_jack_test.chassis_up();
		
		if(remote_ctrl.signal_swd() == true)
		{
			chassis_jack_test.chassis_up_test();
		}
		
		
		
//		arm_gravity.motor_angle.theta1 = j60_1.Get_Out_Pos();
//		arm_gravity.motor_angle.theta2 = dm4310_1.Get_Out_Pos();
//		arm_gravity.motor_angle.theta3 = m2006_4.Get_Out_Pos();
>>>>>>> Stashed changes
		
		
<<<<<<< Updated upstream
		
=======
//		j60_1.Set_Feedforward(-arm_gravity.joint_gravity_compensation.joint1);
//		dm4310_1.Set_Feedforward(arm_gravity.joint_gravity_compensation.joint2);

		//go_0_3.Set_Out_Pos(wl1);
		//go_0_0.Set_Out_Pos(wl2);
		c = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
=======
	
	jy901s.Uart_Rx_Start();
	LiDAR_1.Uart_Rx_Start();
<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
	/*------------------------------------------------------------------------------*/
	
>>>>>>> Stashed changes
}



