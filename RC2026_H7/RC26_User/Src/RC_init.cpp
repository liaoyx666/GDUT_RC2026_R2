#include "RC_init.h"
/*--------------------------------外设初始化------------------------------------------*/
// 定时中断
tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

// can通讯
can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

// 虚拟串口上位机通讯
cdc::CDC CDC_HS(cdc::USB_CDC_HS);

/*----------------------------------电机初始化----------------------------------------*/

// 3全向轮底盘电机-------------------------------------------
//motor::M3508 m3508_1(1, can3, tim7_1khz);
//motor::M3508 m3508_2(2, can3, tim7_1khz);
//motor::M3508 m3508_3(3, can3, tim7_1khz);
// ----------------------------------------------------------


// 4舵轮底盘电机---------------------------------------------
motor::M2006 m2006_1_can3(1, can3, tim7_1khz, 4.f * 36.f);
motor::M2006 m2006_2_can3(2, can3, tim7_1khz, 4.f * 36.f);
motor::M2006 m2006_3_can3(3, can3, tim7_1khz, 4.f * 36.f);
motor::M2006 m2006_4_can3(4, can3, tim7_1khz, 4.f * 36.f);

motor::Vesc vesc_101_can3(101, can3, tim7_1khz, 21);
motor::Vesc vesc_102_can3(102, can3, tim7_1khz, 21);
motor::Vesc vesc_103_can3(103, can3, tim7_1khz, 21);
motor::Vesc vesc_104_can3(104, can3, tim7_1khz, 21);
// -----------------------------------------------------------


// 机械臂电机-------------------------------------------------
motor::M2006 	m2006_4_can1(4, can1, tim7_1khz);
motor::M3508 	m3508_2_can1(2, can1, tim7_1khz, 51.f * 1.2f);
motor::J60 		j60_1_can1(1, can1, tim7_1khz);
motor::Go 		go_0_3_can2(0, 3, can2, tim7_1khz);
// -----------------------------------------------------------


// 轮腿电机---------------------------------------------------
//motor::RS04 	rs04_120(120, can3, tim7_1khz, true, 0, 0);
//motor::RS04 	rs04_127(127, can3, tim7_1khz, true, 0, 0);
// -----------------------------------------------------------


// 4撑杆电机-------------------------------------------------
motor::M3508 m3508_left_front_can2(1, can2, tim7_1khz, 10 * 3591.f / 187.f);
motor::M3508 m3508_left_behind_can2(2, can2, tim7_1khz, 99.506f);
motor::M3508 m3508_right_behind_can2(3, can2, tim7_1khz, 99.506f);
motor::M3508 m3508_right_front_can2(4, can2, tim7_1khz, 10 * 3591.f / 187.f);
// -----------------------------------------------------------

// 主动轮电机-------------------------------------------------
motor::M2006 	m2006_5_can2(5, can2, tim7_1khz, 36.f);
motor::M2006 	m2006_6_can2(6, can2, tim7_1khz, 36.f);
// -----------------------------------------------------------


/*-------------------------------软件模块初始化---------------------------------------*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳

path::PathPlan path_plan(2, 1.f);// 路径规划

arm::ArmDynamics arm_gravity;// 机械臂重力补偿




/*--------------------------------硬件模块初始化--------------------------------------*/
ros::Radar 		radar(CDC_HS, 1);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收

lidar::LiDAR lidar_1(huart3);

// 3全向轮底盘
//chassis::OmniChassis omni_chassis(
//	m3508_3, m3508_1, m3508_2, 3, 3
//);


// 4舵轮底盘
chassis::Swerve4Chassis swerve_4_chassis(
	m2006_1_can3, m2006_2_can3, m2006_3_can3, m2006_4_can3,
	vesc_101_can3, vesc_102_can3, vesc_103_can3, vesc_104_can3,
	0.5, 5, 5,
	4, 8, 8,
	GPIO_PIN_2, GPIO_PIN_9, GPIO_PIN_14, GPIO_PIN_15
);

// 4撑杆
chassis_jack::Chassis_jack chassis_jack_test(
	m3508_left_front_can2, 
	m3508_left_behind_can2, 
	m3508_right_front_can2,
	m3508_right_behind_can2,
	m2006_5_can2,
	m2006_6_can2,
	0.5f,
	lidar_1
);


flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控

imu::JY901S jy901s(huart1);// 陀螺仪

/*---------------------------————-----DeBug------------------------------------------*/
SquareWave wave(1000, 3000);// 用于调pid
//SinWave sin_wave(1000, 3000);

float target = 0;
float a = 0;

float wl1 = 0, wl2 = 0;

float a1 = 0, a2 = 0, a3 = 0, a4 = 0;

void test(void *argument)
{
//	sin_wave.Init();
	wave.Init();
	
	j60_1_can1.Reset_Out_Pos(0);
	m2006_4_can1.Reset_Out_Pos(0);
	go_0_3_can2.Reset_Out_Pos(0);
	m3508_2_can1.Reset_Out_Pos(0);

//	rs04_120.Set_ZeroPos();
//	rs04_120.Set_K_Pos(50);
//	rs04_120.Set_K_Spd(10);

//	rs04_127.Set_ZeroPos();
//	rs04_127.Set_K_Pos(50);
//	rs04_127.Set_K_Spd(10);
	
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		
//		m2006_3_can3.Set_Out_Pos(a);
//		m2006_4_can3.Set_Out_Pos(a);
		
//		rs04_120.Set_Torque(a);
//		rs04_127.Set_Torque(a);
		
		swerve_4_chassis.Set_Robot_Vel(vector2d::Vector2D(remote_ctrl.left_y / 200.f, -remote_ctrl.left_x / 200.f), -remote_ctrl.right_x / 100.f);
		
		

		chassis_jack_test.chassis_test(remote_ctrl.signal_swd(), remote_ctrl.swa);
		chassis_jack_test.Set_Vel(swerve_4_chassis.Get_Vel().x());
		
		
//		uart_printf("%f,%f\n", m6020_1.Get_Rpm(), target);
//		m6020_1.Set_Rpm(target);
//		uart_printf("%d,%d,%d,%d\n", remote_ctrl.channel_list[0] - 1500, remote_ctrl.channel_list[1] - 1500, remote_ctrl.channel_list[2] - 1500, remote_ctrl.channel_list[3] - 1500);
		go_0_3_can2.Set_Out_Pos(a1);
		j60_1_can1.Set_Out_Pos(a2);
		m3508_2_can1.Set_Out_Pos(a3);
		m2006_4_can1.Set_Out_Pos(a4);
		
//		arm_gravity.motor_angle.theta1 = j60_1.Get_Out_Pos();
//		arm_gravity.motor_angle.theta2 = dm4310_1.Get_Out_Pos();
//		arm_gravity.motor_angle.theta3 = m2006_4.Get_Out_Pos();
		
		arm_gravity.gravity_compensation();
		
//		j60_1.Set_Feedforward(-arm_gravity.joint_gravity_compensation.joint1);
//		dm4310_1.Set_Feedforward(arm_gravity.joint_gravity_compensation.joint2);

//		go_0_3.Set_Out_Pos(wl1);
//		go_0_0.Set_Out_Pos(wl2);
		
		osDelay(1);
	} 
}

task::TaskCreator test_task("test", 20, 512, test, NULL);




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
	
	jy901s.Uart_Rx_Start();
	lidar_1.Uart_Rx_Start();

	/*------------------------------------------------------------------------------*/
	
}
