#include "RC_arm_task.h"
using namespace path;
using path::Pose3D;

/*================= 硬件对象 =================*/
extern motor::M2006  m2006_4;
extern motor::M3508  m3508_2_c1;
extern motor::J60    j60_1;
extern motor::Go     go_0_3;

/*================= 轨迹对象 =================*/
CartesianPlanner planner;
TrajectoryExecutor executor;
JointArray current_joint_angles = {0,0,0,0};

/*================= 状态变量 =================*/
static bool is_reseted = false;
static ArmState volatile  g_state = ARM_STATE_IDLE;
static uint32_t g_hold_end_tick = 0;

volatile ARM_TASK g_task_mode      = ARM_TASK::IDLE;
volatile ARM_TASK g_last_task_mode = ARM_TASK::IDLE;

/*================= 动作序列 =================*/
static const ArmAction* g_action_seq = nullptr;
static uint32_t g_action_index = 0;

/*================= 关键：记录最近一次位姿 =================*/
static Pose3D g_last_pose; // 用作起点，自动记录上一个动作的终点

/*================= 位姿定义 =================*/
// Home / Rest
Pose3D home_pose  = POS(0.3f,    0.0f, 0.2f,     0.0f);
Pose3D safe_pose  = POS(0.3f,   0.0f, 0.3f,    0.0f);

/* ===== 方块 ===== */
Pose3D pick_front_cube0  = POS(0.3f,  0.0f,   0.3f,    -0.2f);
Pose3D pick_front_cube1  = POS(0.60f,  0.0f,  0.22f,   -0.2f);
Pose3D pick_front_cube2  = POS(0.73f,  0.0f,  0.22f,   -0.2f);
Pose3D pick_front_cube3  = POS(0.4f,  0.0f,   0.4f,    -0.2f);



Pose3D pick_front_cube5  = POS(0.3f,  0.0f,    0.3f,    -0.2f);
Pose3D pick_front_cube6  = POS(0.60f,  0.0f,   -0.16f,    -0.22f);
Pose3D pick_front_cube7  = POS(0.7f,  0.0f,   -0.16f,    -0.22f);
Pose3D pick_front_cube8  = POS(0.35f,  0.0f,    0.35f,    -0.2f);


Pose3D place_left_cube  = POS(0.2f,  0.5f,  0.25f, -0.2f);
Pose3D place_right_cube = POS(0.2f, -0.5f,  0.25f, -0.2f);

/* ===== 武器 ===== */
Pose3D via_weapon   = POS(0.2f, 0.0f, 0.35f, -0.2f);
Pose3D pick_weapon  = POS(0.45f,0.0f, 0.40f, -0.2f);
Pose3D place_weapon = POS(0.2f, 0.0f, 0.20f, -0.2f);

/*================= IO =================*/
static void Gripper_Close(){}
static void Gripper_Open(){}
static void Air_Pump_On(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);}
static void Air_Pump_Off(){HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);}
/*================= 条件判断 =================*/
static bool IS_CUBE_GRIPED(){return 1;}
static bool IS_WEAPON_GRIPED(){return 1;}

/*================= 动作序列 =================*/
/* --- PICK FRONT --- */
static const ArmAction PICK_FRONT_UP_CUBE_SEQ[] =
{
	  { ARM_ACTION_IO,   {},{},0,0,0,0,Air_Pump_On,nullptr },
    { ARM_ACTION_MOVE, {}, pick_front_cube0,0.0f,1.0f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_MOVE, {}, pick_front_cube1,0.0f, 1.0f,1.0f,0,nullptr,nullptr },
	
		{ ARM_ACTION_MOVE, {}, pick_front_cube2,0.0f, 0.5f,0.1f,0,nullptr,nullptr },
		{ ARM_ACTION_MOVE, {}, pick_front_cube3,0.0f, 0.4f,1.0f,0,nullptr,nullptr },
		{ ARM_ACTION_MOVE, {}, home_pose,0.0f, 0.5f,0.5f,0,nullptr,nullptr },
		{ ARM_ACTION_HOLD, {},{},{},{},{},2000,nullptr,nullptr},
    { ARM_ACTION_END }
};



static const ArmAction PICK_FRONT_DOWN_CUBE_SEQ[] =
{
		{ ARM_ACTION_IO,   {},{},0,0,0,0,Air_Pump_On,nullptr },
    { ARM_ACTION_MOVE, {}, pick_front_cube5,0.0f,1.0f,0.5f,0,nullptr,nullptr },
    //{ ARM_ACTION_IO,   {},{},0,0,0,0,Air_Pump_On,nullptr },

		{ ARM_ACTION_MOVE, {}, pick_front_cube6,0.0f, 0.8f,0.8f,0,nullptr,nullptr },
		{ ARM_ACTION_MOVE, {}, pick_front_cube7,0.0f, 0.5f,0.1f,0,nullptr,nullptr },
		{ ARM_ACTION_MOVE, {}, pick_front_cube8,0.0f, 0.8f,0.8f,0,nullptr,nullptr },
		{ ARM_ACTION_MOVE, {}, home_pose,0.0f, 0.5f,0.3f,0,nullptr,nullptr },
    { ARM_ACTION_END }
};
/* --- PICK LEFT --- */


/* --- PLACE LEFT--- */
static const ArmAction PLACE_LEFT_CUBE_SEQ[] =
{
    { ARM_ACTION_MOVE, {}, place_left_cube,0,0.5f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_IO,   {},{},0,0,0,0,Gripper_Open,nullptr },
    { ARM_ACTION_END }
};
/* --- PLACE RIGHT--- */
static const ArmAction PLACE_RIGHT_CUBE_SEQ[] =
{
    { ARM_ACTION_MOVE, {}, place_right_cube,0,0.5f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_IO,   {},{},0,0,0,0,Gripper_Open,nullptr },
    { ARM_ACTION_END }
};

/* --- PICK WEAPON --- */
static const ArmAction PICK_WEAPON_SEQ[] =
{
    { ARM_ACTION_MOVE, {}, via_weapon,0,0.5f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_MOVE, {}, pick_weapon,0,0.5f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_IO,   {},{},0,0,0,0,Gripper_Close,nullptr },
    { ARM_ACTION_END }
};
/* --- PLACE WEAPON --- */
static const ArmAction PLACE_WEAPON_SEQ[] =
{
    { ARM_ACTION_MOVE, {}, place_weapon,0,0.5f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_IO,   {},{},0,0,0,0,Gripper_Open,nullptr },
    { ARM_ACTION_END }
};

/* --- HOME --- */
static const ArmAction HOME_SEQ[] =
{
    { ARM_ACTION_MOVE, {}, home_pose,0,0.5f,1.0f,0,nullptr,nullptr },
    { ARM_ACTION_END }
};

/*================= 外部统一接口 =================*/
bool Arm_task::Arm_IsBusy(void)
{
    return (g_state != ARM_STATE_IDLE);
}

bool Arm_task::Arm_Control(ARM_TASK task)
{
	if (task <= ARM_TASK::IDLE || task > ARM_TASK::HOME)
        return false;

	if (Arm_task::Arm_IsBusy())
        return false;

    g_task_mode = task;
    return true;
}


/*================= 任务绑定 =================*/
static void Bind_Action_Sequence(void)
{
    switch (g_task_mode)
    {
    case ARM_TASK::PICK_FRONT_UP_CUBE:
        g_action_seq = PICK_FRONT_UP_CUBE_SEQ;
        break;

    case ARM_TASK::PICK_FRONT_DOWN_CUBE:
        g_action_seq = PICK_FRONT_DOWN_CUBE_SEQ;
        break;

    case ARM_TASK::PLACE_RIGHT_CUBE:
        g_action_seq = PLACE_RIGHT_CUBE_SEQ;
        break;

    case ARM_TASK::PICK_FRONT_WEAPON:
        g_action_seq = PICK_WEAPON_SEQ;
        break;

    case ARM_TASK::PLACE_FRONT_WEAPON:
        g_action_seq = PLACE_WEAPON_SEQ;
        break;

    case ARM_TASK::HOME:
        g_action_seq = HOME_SEQ;
        break;

    default:
        g_action_seq = nullptr;
        break;
    }

    g_action_index = 0;
}


/*================= HOLD =================*/
static void Arm_Hold(uint32_t ms)
{
    g_hold_end_tick = osKernelGetTickCount() + ms;
    g_state = ARM_STATE_WAITING;
}

/*================= 构造 =================*/
Arm_task::Arm_task(tim::Tim& tim_)
    : tim::TimHandler(tim_)
    , ManagedTask("arm_task", 15, 512, task::TASK_PERIOD, 1)
{
}

/*================= 主任务 =================*/
void Arm_Path_Manager(void *)
{
    if (!is_reseted)
    {
        go_0_3.Reset_Out_Pos(0);
        j60_1.Reset_Out_Pos(0);
        m3508_2_c1.Reset_Out_Pos(0);
        m2006_4.Reset_Out_Pos(0);
        is_reseted = true;
				g_last_pose = home_pose; // 初始化为 home
    }

    for (;;)
    {
        if (g_task_mode != g_last_task_mode)
        {
            planner.clear();
            executor.reset();
            Bind_Action_Sequence();
            g_state = ARM_STATE_IDLE;
            g_last_task_mode = g_task_mode;
        }

        if (g_state == ARM_STATE_WAITING)
        {
            if (osKernelGetTickCount() >= g_hold_end_tick)
                g_state = ARM_STATE_IDLE;
            osDelay(2);
            continue;
        }

				if (g_state == ARM_STATE_FINISHED) {
            // 到达后更新起点记录
            const ArmAction &last_act = g_action_seq[g_action_index];
            if (last_act.type == ARM_ACTION_MOVE) {
                g_last_pose = last_act.target;
            }
            g_action_index++; 
            g_state = ARM_STATE_IDLE;
            continue;
        }


        if (!g_action_seq)
        {
            osDelay(5);
            continue;
        }

        const ArmAction &act = g_action_seq[g_action_index];
        switch (act.type)
        {
								case ARM_ACTION_MOVE:
                if (g_state == ARM_STATE_IDLE) {
                    planner.clear();
                    planner.addPoint(g_last_pose, act.smooth, act.speed, act.acc);
                    planner.addPoint(act.target, act.smooth, act.speed, act.acc);
                
                    if (planner.build()) {
                        executor.bind(&planner);
                        executor.reset();
                        g_state = ARM_STATE_MOVING; 
                    } else {
                        // 规划失败处理
                        g_task_mode = ARM_TASK::IDLE;
                        g_state = ARM_STATE_IDLE;
                    }
                }
                break;

        case ARM_ACTION_HOLD:
            Arm_Hold(act.hold_ms);
            g_action_index++;
            break;

        case ARM_ACTION_IO:
            if (act.io_func) act.io_func();
            g_action_index++;
            break;

        case ARM_ACTION_COND:
            if (act.cond_func && act.cond_func())
                g_action_index++;
            break;

        case ARM_ACTION_END:
					  g_task_mode = ARM_TASK::IDLE;
            g_action_seq = nullptr;
            g_action_index = 0;
            break;
        }

        osDelay(2);
    }
}

/*================= TIM =================*/
void Arm_task::Tim_It_Process()
{
    current_joint_angles =
    {
        go_0_3.Get_Out_Pos(),
        -j60_1.Get_Out_Pos(),
        -m3508_2_c1.Get_Out_Pos(),
        -m2006_4.Get_Out_Pos()
    };

    
		

    if (executor.hasTrajectory())
    {
        const JointArray &cmd = executor.getCurrentCmd();
        go_0_3.Set_Out_Pos(cmd.j[0]);
        j60_1.Set_Out_Pos(-cmd.j[1]);
        m3508_2_c1.Set_Out_Pos(-cmd.j[2]);
        m2006_4.Set_Out_Pos(-cmd.j[3]);
    }
		if (g_state == ARM_STATE_MOVING) 
		{
    if (executor.run(current_joint_angles)) {
        g_state = ARM_STATE_FINISHED; 
    }
}
		


}

void Arm_task::Task_Process()
{
    Arm_Path_Manager(nullptr);
}
