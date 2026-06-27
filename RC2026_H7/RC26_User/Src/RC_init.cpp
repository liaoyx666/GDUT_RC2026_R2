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
	1, can3, &tim13_500hz, 
	2, can3, &tim13_500hz, 
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
	2.8, 4, 4,
	5, 8, 8,
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
	path::LonConstr3(2.8, 2.5),
	path::HeadConstr3(0, 5, 7, false),
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

ros::GetData get_data(CDC_HS, 9);

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
	omni_4_chassis, chassis_qeo
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


StickEdge stick_edge(
	omni_4_chassis, 
	path_plan, 
	gan,
	gripper
);


/*==================IR====================*/
IR::IRCmd combine_ready_cmd(2);
IR::IRCmd combine_cmd(3);
IR::IRCmd put_3L_cmd(4);

/*==================Main_Task==================*/
// 方波发生
//SquareWave wave(1000, 3000);// 用于调pid
//float target = 0;
//float a = 0;

//bool auto_flag = 0;



bool en_flag = false;
bool dis_flag = false;

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
		
//		m2006d_can1_3_4.Set_Current(0);
//		m3508d_can1_1_2.Set_Current(0);
//		m2006_can1_5.Set_Current(0);
//		
		
		
		
		get_weapon_head.Auto_Get_Weapon_Head();
		
		stick_edge.Stick_Edge();
		
		path_plan.Plan();
		
		robot_pose.Robot_Pose_Check();
		
		getKFS.Auto_Get_KFS();
		
		putKFS.Auto_Put_KFS();
		
		wait_R1.Wait_R1();
		
		aim.Auto_Aim();
		
		com.Auto_Combine();
		
		gan.Gantry_Base();
		
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
		
		if (remote_ctrl.swa)
		{
			dis_flag = false;
			
			if (!en_flag)
			{
				path_plan.Enable();
				en_flag = true;
			}
		}
		else
		{
			en_flag = false;
			
			if (!dis_flag)
			{
				path_plan.Disable();
				dis_flag = true;
			}
			
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



// 自动状态机
enum AutoState : uint8_t
{
	STATE_PUT_2L = 0,
	STATE_AVOID_R1,
	STATE_COMBINE_READY,
	STATE_COMBINE,
	STATE_PUT_3L,
	STATE_END
};


AutoState state = STATE_PUT_2L;


void Plan_Task(void *argument)
{
	while (!data::AllData::Is_All_Init())
	{
		osDelay(10);
	}
	
	osDelay(200);
	ir_com.Clear_All_Cmd();
	
	get_weapon_head.Set_Side(data::Side::Is_Blue_Left_Side());
	get_weapon_head.Set_Pick_Num(data::PickWeaponNum::Get_Pick_Num()); /*夹第4个武器（靠内小）*/
	
	
	// 初始化全局起点
	if (data::BootArea::Is_Boot_At_Mc())
	{
		// 默认启动区
		if (data::Side::Is_Blue_Left_Side())
		{
			navigation.Add_Start(vector2d::Vector2D(0.42, -4.53), 0);// 蓝
		}
		else
		{
			navigation.Add_Start(vector2d::Vector2D(0.42, 4.53), 0);// 红
		}
	}
	else
	{
		// 对抗区启动区
		if (data::Side::Is_Blue_Left_Side())
		{
			navigation.Add_Start(vector2d::Vector2D(0.42, -4.53), 0);// 蓝
		}
		else
		{
			navigation.Add_Start(vector2d::Vector2D(0.42, 4.53), 0);// 红
		}
	}
	
	
	
	// 规划夹武器和对接
	if (data::IsDock::Is_Dock())
	{
		if (!data::HaveWeapon::Have_Weapon())
		{
			navigation.Go_To_Get_Weapon_Head();// 取武器头
		}
		
		navigation.Go_To_Stick_Edge();// 贴边对接
	}
	

	// 规划梅林
	if (data::BootArea::Is_Boot_At_Mc())
	{
		best_path.Generate_Path();
	}

	// 放中间
	navigation.Go_To_Put_KFS_2L(2);

	for (;;)
	{
		
		switch (state)
		{
			case STATE_PUT_2L:
			{
				// 放中间失败重试
				if (putKFS.Put_First_Fail_Navi())
				{
					state = STATE_AVOID_R1;
				}
				break;
			}
			
			case STATE_AVOID_R1:
			{
				// 避开r1
				if (navigation.Go_To_Avoid_R1_In_ARENA())
				{
					state = STATE_COMBINE_READY;
				}
				break;
			}
			
			case STATE_COMBINE_READY:
			{
				// 准备合体
				if (combine_ready_cmd.Get_Cmd() && !com.Is_Combine())
				{
					navigation.Go_To_Combine_Ready();
					
					state = STATE_COMBINE;
				}
				break;
			}
			
			
			case STATE_COMBINE:
			{
				// 合体
				if (combine_cmd.Get_Cmd() && !com.Is_Combine())
				{
					navigation.Go_To_Combine();
					
					state = STATE_PUT_3L;
				}
				break;
			}
			
			
			case STATE_PUT_3L:
			{
				// 放第三层命令
				if (put_3L_cmd.Get_Cmd())
				{
					path::Event3::Trig_Event(EVENT_PUT_KFS_3L_READY | EVENT_PUT_KFS_PUT);
					
					state = STATE_END;
				}
				break;
			}
			
			
			default:
			{
				state = STATE_END;
				break;
			}
			
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
	
	m2006d_can1_3_4.	pid_pos.Pid_Param_Init(200, 0, 3, 		0, 0.002, 0, 9000, 500, 500, 500, 500, 	2000, 837.76f);
	m3508d_can1_1_2.	pid_pos.Pid_Param_Init(100, 0, 0.005, 	0, 0.002, 0, 4000, 1000, 500, 500, 500, 1000, 314.16);
	m2006_can1_5.		pid_pos.Pid_Param_Init(170, 0, 2.5, 		0, 0.002, 0, 9000, 500, 500, 500, 500, 	2000, 837.76f);
	//dm4310_can1_0x12.	pid_pos.Pid_Param_Init(15, 0, 0.055, 0, 0.001, 0, 7, 5, 5, 5, 5, 20, 7);
	dm4310_can1_0x12.	pid_pos.Pid_Param_Init(20, 0, 1.4, 		0, 0.001, 0, 27, 5, 5, 5, 5, 20, 5);
	
	m2006d_can1_3_4 .Set_Pos_limit(874.770203f, 0.f);
	m3508d_can1_1_2 .Set_Pos_limit(0.f, -541.696167f);
	m2006_can1_5    .Set_Pos_limit(511.869476f, 0.f);
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

	// 初始化数据
//	data::Side::Init_Is_Blue_Left_Side(false);
//	data::KFSNum::Init_KFS_Num(0);
//	data::HaveWeapon::Init_Have_Weapon(false);
//	data::IsDock::Init_Is_Dock(true);
//	data::BootArea::Init_Is_Boot_At_Mc(true);
//	data::PickWeaponNum::Init_Pick_Num(1);
	
	gan.Init();
}
