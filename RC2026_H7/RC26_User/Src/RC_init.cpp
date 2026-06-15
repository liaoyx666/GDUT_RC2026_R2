#include "RC_init.h"
/*==================外设==================*/
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

/*===================Motor=================*/
// 底盘电机
motor::M3508 m3508_1_can1(1, can1, &tim13_500hz);
motor::M3508 m3508_2_can1(2, can1, &tim13_500hz);
motor::M3508 m3508_3_can1(3, can1, &tim13_500hz);
motor::M3508 m3508_4_can1(4, can1, &tim13_500hz);

// 龙门架电机
motor::M2006D m2006d_can1_3_4(
	3, can2, &tim13_500hz, 
	4, can2, &tim13_500hz, 
	36, motor::POL_REV, true
);
motor::M3508D m3508d_can1_1_2(
	1, can2, &tim13_500hz, 
	2, can2, &tim13_500hz, 
	3591.f / 187.f, motor::POL_REV, true
);
motor::M2006 m2006_can1_5(5, can2, &tim13_500hz);
motor::DM4340 dm4310_can1_0x12(0x12, can2, &tim7_1khz);
motor::M2006 m2006_can1_7(7, can2, &tim13_500hz);

// 抬升电机
motor::M3508 m3508_can3_5(5, can3, &tim13_500hz, 51, true);
motor::M3508 m3508_can3_6(6, can3, &tim13_500hz, 51, true);	

// 辅助轮电机
motor::M2006 m2006_can3_7(7, can3, &tim13_500hz);
motor::M2006 m2006_can3_8(8, can3, &tim13_500hz);

/*====================数据池====================*/
// 机器人位姿
data::RobotPose robot_pose;

/*==================底盘=======================*/

// 全向轮底盘
chassis::Omni4Chassis omni_4_chassis(
	m3508_1_can1, m3508_2_can1,
	m3508_3_can1, m3508_4_can1,
	2.5, 4, 4,
	5, 6, 7,
	robot_pose
);


/*=====================路径规划==================*/
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
	0.008
);

// 路径规划
path::PathPlan3 path_plan(
	path::LonConstr3(2.5, 2.0),
	path::HeadConstr3(0, 4, 5, false),
	track
);

// 图规划
path::GraphPlan graph_plan(path_plan);

// 全图导航
path::Navigation navigation(graph_plan);

// 航向检查
check::HeadCheck head_check(
	omni_4_chassis,
	robot_pose
);

/*===================上位机接口===================*/

// 雷达数据接收
ros::Radar radar(CDC_HS, 1, robot_pose);

// 梅林路径接收，生成
ros::BestPath best_path(CDC_HS, 7, navigation);

// 等待清障
ros::WaitR1 wait_R1(CDC_HS, 8, omni_4_chassis, head_ctrl);

ros::Camera camera_aim(CDC_HS, 6);

/*===================外置模块=================*/

// 激光测距校准
uint8_t cali_laser_buffer[MINI_LASER_RX_BUFFER_SIZE] __attribute__((section(".D2RAM"))) ;
mini_laser::MiniLaser cali_laser(huart3, cali_laser_buffer);


// 激光测距检查
uint8_t check_laser_buffer[MINI_LASER_RX_BUFFER_SIZE] __attribute__((section(".D2RAM"))) ;
mini_laser::MiniLaser check_laser(huart2, check_laser_buffer);

// imu
uint8_t hwt101ct_buffer[HWT101CT_RX_BUFFER_SIZE] __attribute__((section(".D2RAM"))) ;
HWT101CT hwt101ct(huart8, hwt101ct_buffer);

uint8_t ir_buffer[IR_COM_RX_BUFFER_SIZE] __attribute__((section(".D2RAM"))) ;
IR::IRCom ir_com(huart6, ir_buffer);

// 遥控
flysky::FlySky remote_ctrl(GPIO_PIN_8);

// imu雷达融合
fusion::ImuFusion imu_fusion(radar, hwt101ct);

// 底盘里程计
fusion::QEO chassis_qeo(
	m3508_1_can1, m3508_2_can1,
	m3508_3_can1, m3508_4_can1,
	robot_pose,
	tim4_500hz,
	radar
);

// 抬升
chassis::LiftChassis lift(
	m3508_can3_5, m3508_can3_6,
	m2006_can3_7, m2006_can3_8,
	&omni_4_chassis, chassis_qeo
);




/*==================上层龙门架====================*/
// 龙门架
gantry::Gantry gan(
	m2006d_can1_3_4,
	m2006_can1_5,
	m3508d_can1_1_2,
	dm4310_can1_0x12
);

// 吸盘 
gantry::Suction suck(GPIOG, GPIO_PIN_7);

// 取KFS
gantry::GetKFS getKFS(gan, suck, cali_laser);

// 放KFS
gantry::PutKFS putKFS(gan, suck, check_laser, navigation);

// 夹爪
gantry::Gripper gripper(m2006_can1_7);

// 夹取武器头
gantry::GetWeaponHead get_weapon_head(
	omni_4_chassis,
	robot_pose,
	gan,
	check_laser,
	gripper,
	path_plan,
	head_ctrl
);

// 对接
gantry::Aim_Ctrl aim(
	camera_aim, 
	gan,
	omni_4_chassis,
	gripper
);

// 合体
combine::Combine com(
	omni_4_chassis, 
	lift,
	path_plan
);

/*==================Main_Task==================*/
// 方波发生
//SquareWave wave(1000, 3000);// 用于调pid
//float target = 0;
//float a = 0;

//float x_1 = 0;
//float y_1 = 0;
//float z_1 = 0;
//float p_1 = 0;

void Main_Task(void *argument)
{
	remote_ctrl.signal_swa();
	remote_ctrl.signal_swd();
//	wave.Init();
	
	for (;;)
	{
		imu_fusion.Fusion();
		float fusion_yaw = hwt101ct.Yaw();
		robot_pose.Update_Orientation(&fusion_yaw, NULL, NULL);
		

		path_plan.Plan();
		
		robot_pose.Robot_Pose_Check();
		
		getKFS.Auto_Get_KFS();
		
		gan.Gantry_Base();
		
		putKFS.Auto_Put_KFS();
		
		get_weapon_head.Auto_Get_Weapon_Head();

		wait_R1.Wait_R1();
		
		aim.Auto_Aim();
		
		com.Auto_Combine();
		

		
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
			omni_4_chassis.Set_World_Vel(vector2d::Vector2D(remote_ctrl.left_y / 150.f, -remote_ctrl.left_x / 150.f), -remote_ctrl.right_x / 100.f);
		}
		
		osDelay(1);
	}
}

task::TaskCreator main_task("Main_Task", 22, 512, Main_Task, NULL);









void Path_Task(void *argument)
{
	for (;;)
	{
		track.Traj_Track();
		head_ctrl.Head_Ctrl();
		lift.Auto_Lift();
		head_check.Head_Check();

		osDelay(1);
	}
}

task::TaskCreator path_task("Path_Task", 31, 256, Path_Task, NULL);





void Plan_Task(void *argument)
{
	// 全局起点
	navigation.Add_Start(vector2d::Vector2D(0.42, -4.53), 0);
	
	get_weapon_head.Set_Pick_Num(1); /*夹第4个武器（靠内小）*/
	
	navigation.Go_To_Get_Weapon_Head();

	navigation.Go_To_Dock();
	
	best_path.Generate_Path();

	navigation.Go_To_Put_KFS_2L(2);

	for (;;)
	{
		putKFS.Put_Fail_Navi();
		
		
		
		
		if (remote_ctrl.swb == 1 && remote_ctrl.signal_swd() && !com.Is_Combine())
		{
			navigation.Go_To_Combine_Ready();
		}
		else if (remote_ctrl.swb == 2 && remote_ctrl.signal_swd() && !com.Is_Combine())
		{
			navigation.Go_To_Combine();
		}
		else if (remote_ctrl.swb == 0 && remote_ctrl.signal_swd() && com.Is_Combine())
		{
			navigation.Uncombine(vector2d::Vector2D(robot_pose.X(), robot_pose.Y()), robot_pose.Yaw());
		}
		
		
		osDelay(1);
	}
}

task::TaskCreator plan_task("Plan_Task", 19, 256, Plan_Task, NULL);














/*===================初始化函数=================*/

void Motor_Config()
{
	m3508_can3_5.pid_pos.Pid_Param_Init(100, 0, 0.005, 	0, 0.002, 0, 8500, 1000, 500, 500, 500, 150, 890.12); /* (rad / s^2), (rad / s) */
	m3508_can3_6.pid_pos.Pid_Param_Init(100, 0, 0.005, 	0, 0.002, 0, 8500, 1000, 500, 500, 500, 150, 890.12);
	m3508_can3_5.Set_Pos_limit(620.f, -600.f);
	m3508_can3_6.Set_Pos_limit(620.f, -600.f);
	
	m2006d_can1_3_4.	pid_pos.Pid_Param_Init(200, 0, 3, 		0, 0.002, 0, 8000, 500, 500, 500, 500, 	2000, 837.76f);
	m3508d_can1_1_2.	pid_pos.Pid_Param_Init(100, 0, 0.005, 	0, 0.002, 0, 3000, 1000, 500, 500, 500, 1000, 314.16);
	m2006_can1_5.		pid_pos.Pid_Param_Init(200, 0, 3, 		0, 0.002, 0, 8000, 500, 500, 500, 500, 	2000, 837.76f);
	//dm4310_can1_0x12.	pid_pos.Pid_Param_Init(15, 0, 0.055, 0, 0.001, 0, 7, 5, 5, 5, 5, 20, 7);
	dm4310_can1_0x12.	pid_pos.Pid_Param_Init(20, 0, 1.4, 		0, 0.001, 0, 27, 5, 5, 5, 5, 20, 5);
	
	m2006d_can1_3_4 .Set_Pos_limit(940.14f, 0.f);
	m3508d_can1_1_2 .Set_Pos_limit(524.95f, 0.f);
	m2006_can1_5    .Set_Pos_limit(486.15f, 0.f);
	dm4310_can1_0x12.Set_Pos_limit(0, -4.9324f);
}

void All_Init()
{
	// 电机配置
	Motor_Config();
	
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
	cali_laser.Uart_Rx_Start();
	check_laser.Uart_Rx_Start();
	hwt101ct.Uart_Rx_Start();
	ir_com.Uart_Rx_Start();

	// 场地位置初始化
	data::Init_Side(true);
	
	gan.Init();
	

}
