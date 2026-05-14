//#include "RC_arm_task.h"
//using namespace path;
////using path::Pose3D;

///*================= 硬件对象 =================*/
//extern motor::M2006  m2006_4;
//extern motor::M3508  m3508_2_c1;
//extern motor::J60    j60_1;
//extern motor::Go     go_0_3;

///*================= 轨迹对象 =================*/
//CartesianPlanner planner;
//TrajectoryExecutor executor;
//JointArray current_joint_angles = {0,0,0,0};

///*================= 状态变量 =================*/
//bool is_reseted = false;
//ArmState  g_state = ARM_STATE_IDLE;
//uint32_t g_hold_end_tick = 0;
//bool g_task_lock = false; // 全局任务锁

// ARM_TASK g_task_mode      = ARM_TASK::IDLE;
// ARM_TASK g_last_task_mode = ARM_TASK::IDLE;

///*================= 动作序列 =================*/
//ArmAction* g_action_seq = nullptr;
//uint32_t g_action_index = 0;

///*================= 关键：记录最近一次位姿 =================*/
//Pose3D g_last_pose;

///*================= IO =================*/
//void Gripper_Close(){}
//void Gripper_Open(){}
//void Air_Pump_On(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);}
//void Air_Pump_Off(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);}
///*================= 条件判断 =================*/
//bool IS_CUBE_GRIPED(){return 1;}
//bool IS_WEAPON_GRIPED(){return 1;}; // 用作起点，自动记录上一个动作的终点

///*================= 位姿定义 =================*/
//// Home / Rest
//Pose3D home_pose  = POS(0.12090f, 0.0f, 0.15580f,  0.0f);
//Pose3D end_pose  = POS(0.025f,   0.0f, 0.284f,    1.2826f);
///* ===== 上方块 ===== */
//Pose3D pick_front_cube0  = POS(0.3f,  0.0f,   0.3f,    -0.22f);

//Pose3D pick_front_cube2  = POS(0.73f,  0.0f,  0.22f,   -0.22f);
////Pose3D pick_front_cube3  = POS(0.4f,  0.0f,   0.4f,    -0.2f);

///* ===== 下方块 ===== */
////Pose3D pick_front_cube5  = POS(0.3f,  0.0f,    0.3f,    -0.2f);
//Pose3D pick_front_cube5  = POS(0.4f,  0.0f,    0.1f,    -0.24f);
//Pose3D pick_front_cube6  = POS(0.60f,  0.0f,   -0.16f,    -0.24f);
//Pose3D pick_front_cube7  = POS(0.7f,  0.0f,   -0.16f,    -0.24f);
////Pose3D pick_front_cube8  = POS(0.35f,  0.0f,    0.35f,    -0.2f);

///* =======放置======= */
//Pose3D place_left_cube1  = POS(0.15f,  0.0f,  0.4f, 0.2f);
////Pose3D place_left_cube2  = POS(0.2f,  0.0f,  0.5f, -0.2f);
//Pose3D place_left_cube3  = POS(0.3f,  0.0f,  0.56f, -0.16f);
//Pose3D place_left_cube4  = POS(0.5f,  0.0f,  0.56f, -0.16f);
////Pose3D place_left_cube4  = POS(0.35f,  0.0f,  0.45f, -0.2f);



////Pose3D place_right_cube = POS(0.2f,  0.0f,  0.25f, -0.2f);

/////* ===== 武器 ===== */
////Pose3D via_weapon   = POS(0.2f, 0.0f, 0.35f, -0.2f);
////Pose3D pick_weapon  = POS(0.45f,0.0f, 0.40f, -0.2f);
////Pose3D place_weapon = POS(0.2f, 0.0f, 0.20f, -0.2f);

///*================= 动作序列 =================*/
///* --- PICK FRONT --- */
//ArmAction PICK_FRONT_UP_CUBE_SEQ[9] =
//{
//	{ ARM_ACTION_IO,   {},{},0,0,0,0,Air_Pump_On,nullptr },
//	{ ARM_ACTION_MOVE, {}, pick_front_cube0,0.0f,1.0f,1.2f,0,nullptr,nullptr },
//	//{ ARM_ACTION_MOVE, {}, pick_front_cube1,0.0f, 1.0f,1.2f,0,nullptr,nullptr },
//	{ ARM_ACTION_MOVE, {}, pick_front_cube2,0.0f, 0.3f,0.2f,0,nullptr,nullptr },
//	//{ ARM_ACTION_MOVE, {}, pick_front_cube3,0.0f, 0.4f,1.2f,0,nullptr,nullptr },
//	//{ ARM_ACTION_HOLD, {},{},{},{},{}, 1000,nullptr,nullptr},
//	{ ARM_ACTION_MOVE, {}, end_pose,0.0f, 0.3f,0.7f,0,nullptr,nullptr },
//	{ ARM_ACTION_HOLD, {},{},{},{},{}, 1000,nullptr,nullptr},
//	{ ARM_ACTION_END }
//};



//ArmAction PICK_FRONT_DOWN_CUBE_SEQ[9] =
//{
//	{ ARM_ACTION_IO,   {},{},0,0,0,0,Air_Pump_On,nullptr },
//	{ ARM_ACTION_MOVE, {}, pick_front_cube5,0.0f,1.0f,1.0f,0,nullptr,nullptr },
//	{ ARM_ACTION_MOVE, {}, pick_front_cube6,0.0f, 0.8f,1.2f,0,nullptr,nullptr },
//	{ ARM_ACTION_MOVE, {}, pick_front_cube7,0.0f, 0.5f,0.1f,0,nullptr,nullptr },
//	//{ ARM_ACTION_MOVE, {}, pick_front_cube8,0.0f, 0.8f,0.5f,0,nullptr,nullptr },
//	//{ ARM_ACTION_HOLD, {},{},{},{},{}, 1000,nullptr,nullptr},
//	{ ARM_ACTION_MOVE, {}, end_pose,0.0f, 0.8f,0.8f,0,nullptr,nullptr },
//	{ ARM_ACTION_HOLD, {},{},{},{},{}, 1000,nullptr,nullptr},
//	{ ARM_ACTION_END }
//};



///* --- PLACE LEFT--- */
//ArmAction PLACE_LEFT_CUBE_SEQ[9] =
//{
//	{ ARM_ACTION_MOVE, {}, place_left_cube1,0,0.5f,1.0f,0,nullptr,nullptr },
//	//{ ARM_ACTION_MOVE, {}, place_left_cube2,0,0.5f,1.0f,0,nullptr,nullptr },

//	{ ARM_ACTION_MOVE, {}, place_left_cube3,0,0.2f,0.2f,0,nullptr,nullptr },
//	{ ARM_ACTION_MOVE, {}, place_left_cube4,0,0.2f,0.2f,0,nullptr,nullptr },
//	{ ARM_ACTION_IO,   {},{},0,0,0,0,Air_Pump_Off,nullptr },
//	{ ARM_ACTION_HOLD, {},{},{},{},{}, 1000,nullptr,nullptr},
//	//	{ ARM_ACTION_MOVE, {}, place_left_cube4,0,0.5f,0.1f,0,nullptr,nullptr },
//	{ ARM_ACTION_MOVE, {}, end_pose,0,0.5f,1.0f,0,nullptr,nullptr },
//	{ ARM_ACTION_HOLD, {},{},{},{},{}, 1000,nullptr,nullptr},

//	{ ARM_ACTION_END }
//};

///*================= 外部统一接口 =================*/
//bool Arm_task::Arm_IsBusy(void)
//{
//    return (g_state != ARM_STATE_IDLE);
//}

//bool Arm_task::Arm_Control(ARM_TASK task)
//{
//    if (task <= ARM_TASK::IDLE || task > ARM_TASK::HOME)
//        return false;

//    if (Arm_task::Arm_IsBusy() || g_task_lock)  // 如果忙或者锁住，不触发
//        return false;

//    g_task_mode = task;
//    g_task_lock = true; // 上锁
//    return true;
//}


///*================= 任务绑定 =================*/
//void Bind_Action_Sequence(void)
//{
//    switch (g_task_mode)
//    {
//    case ARM_TASK::PICK_FRONT_UP_CUBE:
//        g_action_seq = PICK_FRONT_UP_CUBE_SEQ;
//        break;

//    case ARM_TASK::PICK_FRONT_DOWN_CUBE:
//        g_action_seq = PICK_FRONT_DOWN_CUBE_SEQ;
//        break;

//		case ARM_TASK::PLACE_LEFT_CUBE:
//				g_action_seq = PLACE_LEFT_CUBE_SEQ;
//        break;
//		
////    case ARM_TASK::PLACE_RIGHT_CUBE:
////        g_action_seq = PLACE_RIGHT_CUBE_SEQ;
////        break;

////    case ARM_TASK::PICK_FRONT_WEAPON:
////        g_action_seq = PICK_WEAPON_SEQ;
////        break;

////    case ARM_TASK::PLACE_FRONT_WEAPON:
////        g_action_seq = PLACE_WEAPON_SEQ;
////        break;

////    case ARM_TASK::HOME:
////        g_action_seq = HOME_SEQ;
////        break;

//    default:
//        g_action_seq = nullptr;
//        break;
//    }

//    g_action_index = 0;
//}


///*================= HOLD =================*/
//void Arm_Hold(uint32_t ms)
//{
//    g_hold_end_tick = osKernelGetTickCount() + ms;
//    g_state = ARM_STATE_WAITING;
//}

///*================= 构造 =================*/
//Arm_task::Arm_task(tim::Tim& tim_)
//    : tim::TimHandler(tim_)
//    , ManagedTask("arm_task", 15, 1024, task::TASK_PERIOD, 1)
//{
//}

///*================= 主任务 =================*/
//void Arm_Path_Manager(void *)
//{
//    if (!is_reseted)
//    {
//        go_0_3.Reset_Out_Pos(0);
//        j60_1.Reset_Out_Pos(0);
//        m3508_2_c1.Reset_Out_Pos(0);
//        m2006_4.Reset_Out_Pos(0);
//        is_reseted = true;
//		g_last_pose = home_pose; // 初始化为 home
//    }

//    for (;;)
//    {
//        if (g_task_mode != g_last_task_mode)
//		{

//			planner.clear();
//			executor.reset();
//			Bind_Action_Sequence();
//	
//			g_state = ARM_STATE_IDLE;
//			g_last_task_mode = g_task_mode;
//		}

//        if (g_state == ARM_STATE_WAITING)
//        {
//            if (osKernelGetTickCount() >= g_hold_end_tick)
//                g_state = ARM_STATE_IDLE;
//            osDelay(2);
//            continue;
//        }

//			if (g_state == ARM_STATE_FINISHED) {
//				// 到达后更新起点记录
//				const ArmAction &last_act = g_action_seq[g_action_index];
//				if (last_act.type == ARM_ACTION_MOVE)
//							{
//					g_last_pose = last_act.target;
//				}
//				g_action_index++; 
//				g_state = ARM_STATE_IDLE;
//				continue;
//			}


//        if (!g_action_seq)
//        {
//            osDelay(5);
//            continue;
//        }

//        const ArmAction &act = g_action_seq[g_action_index];
//        switch (act.type)
//        {
//					
//			case ARM_ACTION_MOVE:
//				if (g_state == ARM_STATE_IDLE) {

//				executor.reset();

//				planner.addPoint(g_last_pose, act.smooth, act.speed, act.acc);
//				planner.addPoint(act.target, act.smooth, act.speed, act.acc);

//				if (planner.build()) {
//						executor.bind(&planner);
//						executor.reset();
//						g_state = ARM_STATE_MOVING; 
//					} else {
//						g_task_mode = ARM_TASK::IDLE;
//						g_state = ARM_STATE_IDLE;
//					}
//				}
//				break;
//	

//        case ARM_ACTION_HOLD:
//            Arm_Hold(act.hold_ms);
//            g_action_index++;
//            break;

//        case ARM_ACTION_IO:
//            if (act.io_func) act.io_func();
//            g_action_index++;
////			   	i = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
//            break;

//        case ARM_ACTION_COND:
//            if (act.cond_func && act.cond_func())
//                g_action_index++;
//            break;

//        case ARM_ACTION_END:
//					g_task_mode = ARM_TASK::IDLE;
//					g_action_seq = nullptr;
//					g_action_index = 0;
//					g_task_lock = false;  // 解锁
//					//g_last_pose = end_pose;
//					break;
//        }

//        osDelay(1);
//    }
//}

///*================= TIM =================*/
//void Arm_task::Tim_It_Process()
//{
//	if (g_state != ARM_STATE_MOVING)
//    return;
//	if(!g_task_lock) return;
//    current_joint_angles =
//    {
//        go_0_3.Get_Out_Pos(),
//        -j60_1.Get_Out_Pos(),
//        -m3508_2_c1.Get_Out_Pos(),
//        -m2006_4.Get_Out_Pos()
//    };

//    
//		

//    if (executor.hasTrajectory())
//    {
//        const JointArray &cmd = executor.getCurrentCmd();
//        go_0_3.Set_Out_Pos(cmd.j[0]);
//        j60_1.Set_Out_Pos(-cmd.j[1]);
//        m3508_2_c1.Set_Out_Pos(-cmd.j[2]);
//        m2006_4.Set_Out_Pos(-cmd.j[3]);
//    }
//		if (g_state == ARM_STATE_MOVING) 
//		{
//			
//			
//    if (executor.run(current_joint_angles)) {
//        g_state = ARM_STATE_FINISHED; 
//    }
//		
//}
//		
//		


//}

//void Arm_task::Task_Process()
//{
//    Arm_Path_Manager(NULL);
//}
