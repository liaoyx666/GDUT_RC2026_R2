#include "RC_init.h"
/*====================================外设初始化====================================*/
// 定时中断
tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

// can通讯
can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

// 虚拟串口上位机通讯
cdc::CDC CDC_HS(cdc::USB_CDC_HS);

/*====================================电机初始化====================================*/
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
motor::M2006 	m2006_4(3, can1, tim7_1khz, 36.f * 2.f);
motor::M3508 	m3508_2_c1(2, can1, tim7_1khz, 51.f * 1.2f);
motor::J60 		j60_1(1, can1, tim7_1khz);
motor::Go 		go_0_3(0, 3, can1, tim7_1khz);
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


/*====================================软件模块初始化====================================*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳

path::PathPlan path_plan(2, 1.f);// 路径规划

arm::ArmDynamics arm_gravity;// 机械臂重力补偿




/*====================================硬件模块初始化====================================*/
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
	2.5, 5, 5,
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
	1.f,
	lidar_1,
	swerve_4_chassis
);


Arm_task arm_task(tim7_1khz);


flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控

imu::JY901S jy901s(huart1);// 陀螺仪  

/*====================================DeBug====================================*/
SquareWave wave(1000, 3000);// 用于调pid
//SinWave sin_wave(1000, 3000);

float target = 0;
float a = 0;

float wl1 = 0, wl2 = 0;


void test(void *argument)
{
	

//	sin_wave.Init();
	wave.Init();
	
	m3508_2_c1.pid_spd.Pid_Mode_Init(true, false, 0.01);
	m3508_2_c1.pid_spd.Pid_Param_Init(10, 0.54, 0, 0, 0.001, 0, 15000, 10000, 5000, 5000, 5000);// 1ms

	m3508_2_c1.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	m3508_2_c1.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 2000, 1000, 500, 500, 500, 1000, 1800);// 1ms

	j60_1.pid_pos.Pid_Mode_Init(true, true, 0.1);
	j60_1.pid_spd.Pid_Param_Init(0.1, 0.0006, 0, 0, 0.001, 0, 10, 5, 5, 5, 5);

	j60_1.pid_pos.Pid_Mode_Init(false, false, 0.1, true);
	j60_1.pid_pos.Pid_Param_Init(250, 0, 15, 0, 0.001, 0, 100, 5, 5, 5, 5, 50, 80);
	
	HAL_Delay(10);


//	rs04_120.Set_ZeroPos();
//	rs04_120.Set_K_Pos(50);
//	rs04_120.Set_K_Spd(10);

//	rs04_127.Set_ZeroPos();
//	rs04_127.Set_K_Pos(50);
//	rs04_127.Set_K_Spd(10);
	
	go_0_3.Set_Out_Pos(0);
	m2006_4.Set_Out_Pos(0);
	m3508_2_c1.Set_Out_Pos(0);
	j60_1.Set_Out_Pos(0);
	
	remote_ctrl.signal_swa();
	
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		
//		m2006_3_can3.Set_Out_Pos(a);
//		m2006_4_can3.Set_Out_Pos(a);
		
//		rs04_120.Set_Torque(a);
//		rs04_127.Set_Torque(a);
		
		swerve_4_chassis.Set_Robot_Vel(vector2d::Vector2D(remote_ctrl.left_y / 200.f, -remote_ctrl.left_x / 200.f), -remote_ctrl.right_x / 100.f);
		
		if (remote_ctrl.signal_swa() == true)
		{
			if (arm_task.Arm_IsBusy() == false)
			{

				if (remote_ctrl.swb == 0)
				{
					arm_task.Arm_Control(ARM_TASK::PICK_FRONT_UP_CUBE);
				}
				else if (remote_ctrl.swb == 1)
				{
					arm_task.Arm_Control(ARM_TASK::PICK_FRONT_UP_CUBE);
				}
				
				else if (remote_ctrl.swb == 2)
				{
					arm_task.Arm_Control(ARM_TASK::PLACE_LEFT_CUBE);
				}
			}
		}
		
		chassis_jack_test.chassis_test(remote_ctrl.signal_swd(), remote_ctrl.swc, 2.5, GPIOA, GPIO_PIN_8,
																				  0.3, GPIOA, GPIO_PIN_9,
																				  0.8, GPIOG, GPIO_PIN_1,
																				  0.7, GPIOG, GPIO_PIN_0);
		chassis_jack_test.Set_Vel(swerve_4_chassis.Get_Vel().x());
		
		
		
//		chassis_jack_test.chassis_test(remote_ctrl.signal_swd(), remote_ctrl.swa);
//		chassis_jack_test.Set_Vel(swerve_4_chassis.Get_Vel().x());
//		
		
//		uart_printf("%f,%f\n", m6020_1.Get_Rpm(), target);


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
	
	jy901s.Uart_Rx_Start();
	lidar_1.Uart_Rx_Start();

	/*------------------------------------------------------------------------------*/
	
}
