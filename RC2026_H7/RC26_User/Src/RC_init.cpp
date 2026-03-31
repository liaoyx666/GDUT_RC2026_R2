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

// 4舵轮底盘电机>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
motor::M2006 m2006_1_can3(1, can3, &tim13_500hz, 4.f * 36.f);
motor::M2006 m2006_2_can3(2, can3, &tim13_500hz, 4.f * 36.f);
motor::M2006 m2006_3_can3(3, can3, &tim13_500hz, 4.f * 36.f);
motor::M2006 m2006_4_can3(4, can3, &tim13_500hz, 4.f * 36.f);

motor::Vesc vesc_101_can3(101, can3, &tim13_500hz, 21);
motor::Vesc vesc_102_can3(102, can3, &tim13_500hz, 21);
motor::Vesc vesc_103_can3(103, can3, &tim13_500hz, 21);
motor::Vesc vesc_104_can3(104, can3, &tim13_500hz, 21);
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// 机械臂电机>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
motor::DM4310 	dm4310_0x13_can2(0x13, can2, &tim7_1khz, false ,0 , 0, true);
motor::M3508 	m3508_2_can1    (2,    can1, &tim7_1khz, 51.f, true);
motor::J60 		j60_1_can1	    (1,    can1, &tim7_1khz, false ,0 , 0, true);
motor::Go 		go_0_0_can1	    (0, 0, can1, &tim7_1khz, false ,0 , 0, true);
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// 4撑杆电机>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
motor::M3508 m3508_left_front_can2  (1, can2, &tim7_1khz, 10 * 3591.f / 187.f, true);
motor::M3508 m3508_left_behind_can2 (2, can2, &tim7_1khz, 99.506f,             true);
motor::M3508 m3508_right_behind_can2(3, can2, &tim7_1khz, 99.506f,             true);
motor::M3508 m3508_right_front_can2 (4, can2, &tim7_1khz, 10 * 3591.f / 187.f, true);
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// 主动轮电机>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
motor::M2006 	m2006_5_can2(5, can2, &tim7_1khz, 36.f);
motor::M2006 	m2006_6_can2(6, can2, &tim7_1khz, 36.f);
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/*====================================模块====================================*/
data::RobotPose robot_pose;// 机器人位姿

ros::Radar 		radar(CDC_HS, 1, robot_pose);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收

lidar::LiDAR lidar_1(huart3);// 激光测距

//Arm_task arm_task(tim13_500hz, go_0_0_can2, j60_1_can1, m3508_2_can1, dm4310_0x13_can1);

// 4舵轮底盘
chassis::Swerve4Chassis swerve_4_chassis(
	m2006_1_can3, m2006_2_can3, m2006_3_can3, m2006_4_can3,
	vesc_101_can3, vesc_102_can3, vesc_103_can3, vesc_104_can3,
	2.5, 5, 5,
	4, 7, 7,
	GPIO_PIN_2, GPIO_PIN_9, GPIO_PIN_14, GPIO_PIN_15
);

//// 路径规划
//path::PathPlan2 path_plan(
//	robot_pose, swerve_4_chassis,
//	1, 2.5, 2.5,
//	2, 4.5, 4.5,
//	0.01, 0.01// 死区
//);

// 4撑杆
//chassis_jack::Chassis_jack chassis_jack_test(
//	1, 2, 3,
//	path_plan,
//	m3508_left_front_can2,  m3508_left_behind_can2,
//	m3508_right_front_can2, m3508_right_behind_can2,
//	m2006_5_can2, m2006_6_can2,
//	1.f,
//	lidar_1,
//	swerve_4_chassis,
//	2.5, 0.6, 0.8, 0.7,
//	GPIOA, GPIO_PIN_8,
//	GPIOG, GPIO_PIN_1,
//	GPIOA, GPIO_PIN_9,
//	GPIOG, GPIO_PIN_0
//);

path::Path3 p_test;

path::TrajPlan3 tp(
	{2.5, 3},
	{0, 4, 7, false}
);

path::TrajTrack3 tt(robot_pose);

//arm::ArmDynamics arm_dynamics;

//arm::AutoArm auto_arm(arm_task, path_plan, 4, 5);

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控

imu::JY901S jy901s(huart1);// 陀螺仪  

/*====================================DeBug====================================*/
SquareWave wave(1000, 3000);// 用于调pid

float a1 = 0, a2 = 0, a3 = 0, a4 = 0;

vector2d::Vector2D pc;

float target = 0;
float a = 0;

void test(void *argument)
{
	wave.Init();

	//j60_1_can1.pid_pos. Pid_Param_Init(250, 0, 20, 0, 0.001, 0, 150, 5, 5, 5, 5, 70, 80);
	//go_0_0_can2.pid_pos.Pid_Param_Init(200, 0, 7, 0, 0.001, 0, 80, 5, 5, 5, 5, 8, 3);

	dm4310_0x13_can2.pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	//m3508_2_can1    .pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	j60_1_can1	    .pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	go_0_0_can1	    .pid_pos.Pid_Mode_Init(false, false, 0.01, true);
	
	dm4310_0x13_can2.pid_pos.Pid_Param_Init( 20, 0,  0.15, 0, 0.001, 0,   5,    5,   5,   5,   5,  20,   7);
	//m3508_2_can1    .pid_pos.Pid_Param_Init(  0, 0,     0, 0, 0.001, 0,   5,    5, 500, 500, 500, 150, 200);
	j60_1_can1	    .pid_pos.Pid_Param_Init(  7, 0,     0.06, 0, 0.001, 0,  30,    5,   5,   5,   5,  5,  30);
	go_0_0_can1	    .pid_pos.Pid_Param_Init(0.7, 0, 0.007, 0, 0.001, 0,  10,  0.1,   5,   2,   2,  40,  10);
	
	go_0_0_can1.     Set_Out_Mit_Pos(0);
	j60_1_can1.      Set_Out_Mit_Pos(0);
	dm4310_0x13_can2.Set_Out_Mit_Pos(0);
	m3508_2_can1.    Set_Out_Pos(0);
	
	//dm4310_0x13_can1.Set_Pos_Offset(-1.88506126f);  
	//j60_1_can1.      Set_Pos_Offset(0.228691101f);
	//go_0_0_can2.     Set_Pos_Offset(-1.69178915f);
	
	m3508_left_front_can2.  Set_Out_Angle(0);
	m3508_left_behind_can2. Set_Out_Pos  (0);
	m3508_right_behind_can2.Set_Out_Pos  (0);
	m3508_right_front_can2. Set_Out_Angle(0);

	remote_ctrl.signal_swa();
	remote_ctrl.signal_swd();

//	MF_path.MF_Best_Path_Plan(map, path_plan);

	tp.Load_Path(&p_test);

	path::Point3 p;
	
	p.point = vector2d::Vector2D(0, 0);
	p.blend_dis = 0;
	tp.Add_Point(p);

	p.point = vector2d::Vector2D(2, 0);
	p.blend_dis = 0;
	tp.Add_Point(p);

	p.point = vector2d::Vector2D(2, 2);
	p.blend_dis = 0;
	tp.Add_Point(p);

	p.point = vector2d::Vector2D(1, 2);
	p.blend_dis = 0;
	tp.Add_Point(p);

	p.point = vector2d::Vector2D(1, 0);
	p.blend_dis = 0;
	p.Set_Is_End();
	tp.Add_Point(p);
	
	tt.Load_Path(&p_test);
	
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
   
//		if (a >= 1)
//		{
//			a = 0;
//		}
//		
//		a += 0.0001;
//		
//		
//		
//		vector2d::Vector2D pp;
		
		
//		p_test.Get_Point_On_T(a, &pp);
		
//		robot_pose.Update_Position(&pp.x(), &pp.y(), NULL);
		
/// 		vector2d::Vector2D tan;
		
		vector2d::Vector2D v;
		
		float w;
		
		if (remote_ctrl.swa == 1)
		{
			tt.Calc_Vel(&v, &w);
			
			swerve_4_chassis.Set_World_Vel(v, w, *robot_pose.Get_pYaw());
		}
		else
		{
			swerve_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 200.f, -remote_ctrl.left_x / 200.f), -remote_ctrl.right_x / 100.f, *robot_pose.Get_pYaw());
		}

		
		if (remote_ctrl.signal_swd())
		{
			swerve_4_chassis.Chassis_Re_Init(); /*舵轮校准*/
		}
		
//		
//		uart_printf("%f,%f,", pp.x(), pp.y());
		
//		uart_printf("%f,%f,%f\n", v.x(), v.y(), v.length());
		
		//swerve_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 200.f, -remote_ctrl.left_x / 200.f), -remote_ctrl.right_x / 100.f, *robot_pose.Get_pYaw());
		
		
		//go_0_0_can1.Set_Out_Pos(a);
		
		//uart_printf("%f,%f,%f,%f\n", a, go_0_0_can1.Get_Out_Pos(), go_0_0_can1.Get_Rpm());
		
		
//		if (remote_ctrl.swa == 0)
//		{
//			// 手操
//			path_plan.Disable();
//			
//			chassis_jack_test.Up_Or_Down_Steps(remote_ctrl.signal_swd(), remote_ctrl.swc);
//			
//			swerve_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 200.f, -remote_ctrl.left_x / 200.f), -remote_ctrl.right_x / 100.f, *robot_pose.Get_pYaw());
//		}
//		else
//		{
//			// 自动
//			chassis_jack_test.Up_Or_Down_Event();
//			
//			auto_arm.Auto_Arm();
//			
//			path_plan.Enable();
//		}
//		
//		// 辅助轮
//		chassis_jack_test.Set_Vel(swerve_4_chassis.Get_Vel().x());
		
//		arm_dynamics.Calc_Torque(
//			-j60_1_can1.      Get_Out_Pos() + (9.99f / 180.f * PI), 
//			-m3508_2_can1.    Get_Out_Pos() + (167.22f / 180.f * PI), 
//			-dm4310_0x13_can2.Get_Out_Pos() + (-102.22f / 180.f * PI), 
//			0, 0, 0, 0,
//			-go_0_0_can1.     Get_Out_Torque(),
//			-j60_1_can1.      Get_Out_Torque(),
//			-m3508_2_can1.    Get_Out_Torque(),
//		    -dm4310_0x13_can2.Get_Out_Torque()
//		);
//		
//		dm4310_0x13_can2.Set_Out_Mit_Tor(-arm_dynamics.tor[3]);
//		//m3508_2_can1.Set_Out_Mit_Tor(-arm_dynamics.tor[2]);
//		j60_1_can1.Set_Out_Mit_Tor(-arm_dynamics.tor[1]);
//		go_0_0_can1.Set_Out_Mit_Tor(-arm_dynamics.tor[0]);
//		
//		dm4310_0x13_can2.Set_Out_Mit_Pos(a4);
//		m3508_2_can1.Set_Out_Pos(a3);
//		j60_1_can1.Set_Out_Mit_Pos(a2);
//		
		
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
