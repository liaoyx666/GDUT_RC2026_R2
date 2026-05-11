#include "RC_init.h"
/*====================================外设初始化====================================*/
// 定时中断
tim::Tim tim7_1khz(htim7);
tim::Tim tim13_500hz(htim13);
tim::Tim tim4_500hz(htim4);

// can通讯
can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);

// 虚拟串口上位机通讯
cdc::CDC CDC_HS(cdc::USB_CDC_HS);
/*====================================电机初始化====================================*/
// 底盘电机---------------------------------------------
motor::M3508 m3508_1_can1(1, can2, &tim13_500hz);
motor::M3508 m3508_2_can1(2, can2, &tim13_500hz);
motor::M3508 m3508_3_can1(3, can2, &tim13_500hz);
motor::M3508 m3508_4_can1(4, can2, &tim13_500hz);

// 龙门架电机---------------------------------------------
motor::M2006D m2006d_can1_3_4(
	3, can1, &tim13_500hz, 
	4, can1, &tim13_500hz, 
	36, motor::POL_REV, true
);
motor::M3508D m3508d_can1_1_2(
	1, can1, &tim13_500hz, 
	2, can1, &tim13_500hz, 
	3591.f / 187.f, motor::POL_REV, true
);
motor::M2006 m2006_can1_5(5, can1, &tim13_500hz);
motor::DM4310 dm4310_can1_0x12(0x12, can1, &tim7_1khz);

// 抬升电机---------------------------------------------
motor::M3508 m3508_can3_5(5, can3, &tim13_500hz, 51, true);
motor::M3508 m3508_can3_6(6, can3, &tim13_500hz, 51, true);	

// 辅助轮电机---------------------------------------------
motor::M2006 m2006_can3_7(7, can3, &tim13_500hz);
motor::M2006 m2006_can3_8(8, can3, &tim13_500hz);

/*====================================模块====================================*/

// 机器人位姿
data::RobotPose robot_pose;

ros::Radar 		radar(CDC_HS, 1, robot_pose);// 雷达数据接收
ros::Map 		map(CDC_HS, 2);// 地图数据接收
ros::BestPath 	MF_path(CDC_HS, 3);// 路径数据接收

// 激光测距
lidar::LiDAR lidar_1(huart3);

// 遥控
flysky::FlySky remote_ctrl(GPIO_PIN_8);

// 全向轮底盘
chassis::Omni4Chassis omni_4_chassis(
	m3508_1_can1, m3508_2_can1,
	m3508_3_can1, m3508_4_can1,
	2.5, 4, 4.5,
	5, 5, 6,
	robot_pose
);

// 抬升
chassis::LiftChassis lift(
	m3508_can3_5, m3508_can3_6,
	m2006_can3_7, m2006_can3_8,
	&omni_4_chassis, NULL
);

// 航向控制
path::HeadCtrl head_ctrl(
	robot_pose,
	omni_4_chassis,
	0
);

// 轨迹跟踪
path::TrajTrack3 track(
	robot_pose,
	omni_4_chassis,
	head_ctrl,
	0.01
);

// 路径规划
path::PathPlan3 path_plan(
	path::LonConstr3(1.5, 2.3),
	path::HeadConstr3(0, 3, 4, false),
	track
);

// 图规划
path::GraphPlan graph_plan(
	robot_pose, 
	path_plan
);

// 抬升自动上下台阶
chassis::AutoLift auto_lift(
	lift,
	track,
	robot_pose
);

gantry::Gantry gan(
	m2006d_can1_3_4,
	m2006_can1_5,
	m3508d_can1_1_2,
	dm4310_can1_0x12
);

/*====================================DeBug====================================*/
// 方波发生
//SquareWave wave(1000, 3000);// 用于调pid

float target = 0;
float a = 0;

float x = 0;
float y = 0;
float z = 0;
float p = 0;

volatile float x_1 = 0;
volatile float y_1 = 0;
volatile float z_1 = 0;
volatile float p_1 = 0;

void Main_Task(void *argument)
{
	remote_ctrl.signal_swa();
	remote_ctrl.signal_swd();
//	wave.Init();
	
	gan.Set_Defualt_Td();
	gan.Set_Reset_Pos();
	
	path::MapGraph::Set_MF_Valid(6, false);
	path::MapGraph::Set_MF_Valid(10, false);
	path::MapGraph::Set_MF_Valid(11, false);

	path_plan.Add_Point(
		vector2d::Vector2D(0.42, -4.53),
		0,
		NULL,
		NULL,
		EVENT3_NULL,
		false
	);
	
	float x_ = 0.42;
	float y_ = -4.53;
	robot_pose.Update_Position(&x_, &y_, NULL);
	
	path::Destination dst;
	dst.nav.p = vector2d::Vector2D(10.6, -4.53);//vector2d::Vector2D(8.79124641, -1.73101103);//path::MapGraph::Get_MF_Center(4);
	dst.nav.yaw = -PI / 2.f;
	dst.type = path::DST_END;
	dst.event = EVENT3_NULL;
	
	graph_plan.Plan(dst);
	
	for (;;)
	{
//		wave.Set_Amplitude(a);
//		target = wave.Get_Signal();

		gan.Set_X(x);
		gan.Set_Y(y);
		gan.Set_Z(z);
		gan.Set_P(p);
		
		x_1 = gan.Get_X();
		y_1 = gan.Get_Y();
		z_1 = gan.Get_Z();
		p_1 = gan.Get_P();
	
		if (remote_ctrl.swc == 0)
		{
			
		}
		else if (remote_ctrl.swc == 1)
		{
			if (remote_ctrl.signal_swd())
			{
				radar.Reposition(); /*雷达重定位*/
			}
		}
		
		if (remote_ctrl.swa == 1)
		{
			path_plan.Enable();
		}
		else
		{
			path_plan.Disable();
			
			chassis::LiftAction la;
		
//			if (remote_ctrl.swb == 0)
//				la = chassis::LIFT_LOCK;
//			else if (remote_ctrl.swb == 1)
				la = chassis::LIFT_UP;
//			else
//				la = chassis::LIFT_DOWN;
			
			chassis::LiftHeigth lh;
			
			if (remote_ctrl.swa == 0)
				lh = chassis::LIFT_20;
			else
				lh = chassis::LIFT_40;
			
			chassis::LiftDir ld;
			
//			if (remote_ctrl.swc == 0)
				ld = chassis::LIFT_L;
//			else
//				ld = chassis::LIFT_R;
			
			//lift.Lift(la, lh, ld, remote_ctrl.signal_swd());
			
			omni_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 150.f, -remote_ctrl.left_x / 150.f), -remote_ctrl.right_x / 100.f);
		}
		
		osDelay(1);
	}
}

task::TaskCreator main_task("Main_Task", 20, 512, Main_Task, NULL);

/*====================================初始化函数====================================*/

void Motor_Config()
{
	// 撑杆电机
	m3508_can3_5.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.002, 0, 8500, 1000, 500, 500, 500, 150, 8500);
	m3508_can3_6.pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.002, 0, 8500, 1000, 500, 500, 500, 150, 8500);
	m3508_can3_5.Set_Pos_limit(620.f, -600.f);
	m3508_can3_6.Set_Pos_limit(620.f, -600.f);
	
	
	m2006d_can1_3_4.	pid_pos.Pid_Param_Init(200, 0, 3, 0, 0.002, 0, 8000, 500, 500, 500, 500, 2000, 8000.f);
	m3508d_can1_1_2.	pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.002, 0, 3000, 1000, 500, 500, 500, 1000, 2000);
	m2006_can1_5.		pid_pos.Pid_Param_Init(200, 0, 3, 0, 0.002, 0, 8000, 500, 500, 500, 500, 2000, 8000.f);
	dm4310_can1_0x12.	pid_pos.Pid_Param_Init(20, 0, 0.05, 0, 0.001, 0, 5, 5, 5, 5, 5, 20, 7);
	
	m2006d_can1_3_4 .Set_Pos_limit(940.14f, 0.f);
	m3508d_can1_1_2 .Set_Pos_limit(524.95f, 0.f);
	m2006_can1_5    .Set_Pos_limit(486.15f, 0.f);
	dm4310_can1_0x12.Set_Pos_limit(0, -4.9324f);
}


void vApplicationIdleHook(void)
{
	
}

void All_Init()
{
	// CAN初始化
	can1.Can_Filter_Init(FDCAN_STANDARD_ID, 1, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can1.Can_Filter_Init(FDCAN_EXTENDED_ID, 2, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can1.Can_Start();

	can2.Can_Filter_Init(FDCAN_STANDARD_ID, 3, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can2.Can_Filter_Init(FDCAN_EXTENDED_ID, 4, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can2.Can_Start();
	
	can3.Can_Filter_Init(FDCAN_STANDARD_ID, 5, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can3.Can_Filter_Init(FDCAN_EXTENDED_ID, 6, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can3.Can_Start();

	// 定时中断初始化
	tim4_500hz.Tim_It_Start();
	tim7_1khz.Tim_It_Start();
	tim13_500hz.Tim_It_Start();

	// 时间戳初始化
	timer::Timer::Timer_Start();
	
	
	// 串口接收初始化
	lidar_1.Uart_Rx_Start();

	// 场地位置初始化
	data::Init_Side(true);
	
	
	Motor_Config();
}
