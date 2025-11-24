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
//motor::M6020 m6020_1(1, can2, tim7_1khz);

// 底盘
motor::M3508 m3508_1(1, can3, tim7_1khz);
motor::M3508 m3508_2(2, can3, tim7_1khz);
motor::M3508 m3508_3(3, can3, tim7_1khz);

// 机械臂
motor::M2006 	m2006_4(4, can1, tim7_1khz);
//motor::DM4310 	dm4310_1(1, can2, tim7_1khz);
motor::M3508 m3508_2_c1(2, can1, tim7_1khz, 51.f * 1.2f);

motor::J60 		j60_1(1, can1, tim7_1khz);
//motor::Go 		go_0_3(0, 3, can2, tim7_1khz);

//轮腿
motor::Go 		go_0_0(0, 0, can2, tim7_1khz, true, 0.15, 5);
motor::Go 		go_0_3(0, 3, can2, tim7_1khz, true, 0.15, 5);

motor::RS04 	rs04_120(120, can3, tim7_1khz, true, 0, 0);
motor::RS04 	rs04_127(127, can3, tim7_1khz, true, 0, 0);

/*-------------------------------软件模块初始化---------------------------------------*/
timer::Timer timer_us(tim4_timer);// 用于获取us级时间戳

path::PathPlan path_plan(2, 1.f);// 路径规划

arm::ArmDynamics arm_gravity;// 机械臂重力补偿

/*--------------------------------硬件模块初始化--------------------------------------*/
ros::Radar 		radar(CDC_HS, 1);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收

chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 3, 3);// 三全向轮底盘

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控

imu::JY901S jy901s(huart1);

/*---------------------------————-----DeBug------------------------------------------*/
SquareWave wave(1000, 3000);// 用于调pid
//SinWave sin_wave(1000, 3000);

float target = 0;
float a = 0;
float w = 70;

float wl1 = 0, wl2 = 0;

float a1 = 0, a2 = 0, a3 = 0, a4 = 0;

void test(void *argument)
{
	//sin_wave.Init();
	wave.Init();
	
	j60_1.Reset_Out_Pos(0);
	//dm4310_1.Reset_Out_Pos(0);
	m2006_4.Reset_Out_Pos(0);
	//go_0_3.Reset_Out_Pos(0);
	m3508_2_c1.Reset_Out_Pos(0);
	
	//go_0_0.Reset_Out_Pos(0);
	
	rs04_120.Set_ZeroPos();
	rs04_120.Set_K_Pos(50);
	rs04_120.Set_K_Spd(10);
	
	rs04_127.Set_ZeroPos();
	rs04_127.Set_K_Pos(50);
	rs04_127.Set_K_Spd(10);

	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		
//		m3508_2_c1.Set_Out_Angle(a);
		
		
		
		rs04_120.Set_Torque(a);
		rs04_127.Set_Torque(a);
		
		
		//uart_printf("%f,%f\n", m6020_1.Get_Rpm(), target);
//		m6020_1.Set_Rpm(target);

//		go_0_3.Set_Out_Pos(a1);
		j60_1.Set_Out_Pos(a2);
		m3508_2_c1.Set_Out_Pos(a3);
		m2006_4.Set_Out_Pos(a4);
		
//		arm_gravity.motor_angle.theta1 = j60_1.Get_Out_Pos();
//		arm_gravity.motor_angle.theta2 = dm4310_1.Get_Out_Pos();
//		arm_gravity.motor_angle.theta3 = m2006_4.Get_Out_Pos();
		
		arm_gravity.gravity_compensation();
		
//		j60_1.Set_Feedforward(-arm_gravity.joint_gravity_compensation.joint1);
//		dm4310_1.Set_Feedforward(arm_gravity.joint_gravity_compensation.joint2);

		//go_0_3.Set_Out_Pos(wl1);
		//go_0_0.Set_Out_Pos(wl2);
		
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

	/*------------------------------------------------------------------------------*/
	
}
