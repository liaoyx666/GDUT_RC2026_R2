#include "RC_arm.h"
	// ===== 任务表接口 =====
#define Z_radius  3.18f//cm
#define X_radius  2.483f//cm
#define Y_radius  1.60f//cm
#define Z_offset_angle 0.0f
#define X_offset_angle 0.0f
#define Y_offset_angle 0.0f
#define A_offset_angle -PI/2
#define DM_POS_OFFSET 0.0f
#define Suction_length  12.6f//cm
#define Gripper_length  14.0f//cm
#define Y_OFFSET  -0.1345f  //m
#define Z_BASE    0.234f//m
#define open_current  -2000
#define close_current  2000
//X_motor_pos>0，向前   //0-21rad
//Y_motor_pos>0, 向右   //0-18.5
//Z_motor_pos>0, 向上   //0-25
//A_motor_pos>0, 顺时针，注意！！反了//

/*
使用指南：
1.任务表：一个命令+8个参数
2.8个参数：
			X
			Y
			Z
			ANGLE
			VEL
			ACC
			WAITTIME
			FUNC
TIPS:
1.切换到吸盘的时候，X要大于0.25，否则会与机构冲突
2.任务表的开头建议先选定要使用的末端，否则可能有冲突
3.Z要大于0.3

*/
const float pos_eps = 0.005f;
static ArmAction PICK_UP_KFS_SEQ[13] =
{
		{ACTION_SWITCH_TO_SUCTION,										0.0,	0.0,	0.5,	1.57,	3.0,	3.0,	0,	0},
		{ACTION_MOVE,																	0.3,	0.0,	0.65,	0.0,	1.0,	3.0,	0,	0},
    {ACTION_MOVE,																	0.6,	0.0,	0.65,	-1.57,	1.0,	3.0,	0,	0},
		{ACTION_MOVE,																	0.6,	0.0,	0.55,	-1.57,	1.0,	3.0,	0,	0},
		
    {ACTION_SUCTION,															0,		0,		0,		0,	0,	0,	0,    	1},
		
		
		{ACTION_MOVE,																	0.6,	0.0,	0.65,	-1.57,	1.0,	3.0,	0,	0},
		{ACTION_WAIT,           										  0,    0,    0,    0,  0,  0,  1000,  0},
		{ACTION_MOVE,																	0.35,	0.0,	0.55,	1.57,	3,	4,	0,	0},
		{ACTION_MOVE,																	0.2,	0,	0.55,   2.0,	2,	4,	0,	0},
		{ACTION_MOVE,																	-Suction_length/100.0f,	0,	0.55,   3.14,	2,	4,	0,	0},
		{ACTION_SWITCH_TO_SUCTION,										-Suction_length/100.0f,	0,	Z_BASE,	3.14,	1.0,	1.5,	0,	0},
		{ACTION_SUCTION,															0,		0,		0,		0,	0,	0,	0,	0},
    {ACTION_END}
};
		


static ArmAction PICK_DOWN_KFS_SEQ[11] =
{
		{ACTION_SWITCH_TO_SUCTION,										0.25,	0.0,	0.4,			1.57,	3.0,	3.0,	0,	0},
    {ACTION_MOVE,																	0.4,	0.0,	0.3,		 	0,		2.0,	4.0,		0,	0},
		{ACTION_MOVE,																	0.61,	0.0,	0.3,		 	-1.57,		2.0,	4.0,		0,	0},			
		{ACTION_MOVE,																	0.61,	0.0,	0.15,		 	-1.57,		2.0,	4.0,		0,	0},
		{ACTION_SUCTION,															0,		0,		0,		0,	0,	0,	0,    	1},
		{ACTION_MOVE,																	0.6,	0.0,	0.4,		 	-1.57,		2.0,	4.0,		0,	0},
		{ACTION_MOVE,																	0.35,	0.0,	0.4,		 	0.0,		2.0,	4.0,		0,	0},
		{ACTION_MOVE,																	0.20,	0.0,	0.4,		 	1.7,		2.0,	4.0,		0,	0},
		{ACTION_MOVE,										 -Suction_length/100.0f,	0,	0.4,   3.14,	2,	4,	0,	0},
		{ACTION_SWITCH_TO_SUCTION,			 -Suction_length/100.0f,	0,	Z_BASE,	3.14,	1.0,	1.5,	0,	0},
    {ACTION_END}
};

static ArmAction PUT_KFS_SEQ[5] =
{
    {ACTION_MOVE,0.3,0.4,0.2,0,3,10,0,0},
    {ACTION_MOVE,0.3,0.4,0.05,0,2,8,0,0},
    {ACTION_GRIPPER,0,0,0,0,0,0,0,0},
    {ACTION_MOVE,0.3,0.4,0.2,0,3,10,0,0},
    {ACTION_END}
};


static ArmAction PICK_WEAPON_SEQ[9] =
{
		 {ACTION_GRIPPER,					0,0,0,0,0,0,0,0},
    {ACTION_SWITCH_TO_GRIPPER,Gripper_length/100,	0.0,0.55,0,5,12,0,0},
    {ACTION_MOVE,							0.33,	0.0,0.55,0,2,8, 0,0},
		
    {ACTION_GRIPPER,					0,0,0,0,0,0,0,1},
		{ACTION_WAIT,           	0,    0,    0,    0,  0,  0,  1000,  0},
		{ACTION_MOVE,							0,	0.0,0.7,1.57,2,8, 0,0},
		{ACTION_MOVE,							0,	0.0,0.5,1.57,2,8, 0,0},
    {ACTION_END}
};

static ArmAction CONNECT_WEAPON_SEQ[6] =
{
    {ACTION_MOVE,0.55,0.15,0.2,0,3,10,0,0},
    {ACTION_MOVE,0.55,0.15,0.05,0,2,8,0,0},
    {ACTION_GRIPPER,0,0,0,0,0,0,0,1},
    {ACTION_MOVE,0.55,0.15,0.2,0,3,10,0,0},
    {ACTION_END}
};

static ArmAction REALSE_WEAPON_SEQ[5] =
{
    {ACTION_MOVE,0.6,0.1,0.2,0,3,10,0,0},
    {ACTION_MOVE,0.6,0.1,0.05,0,2,8,0,0},
    {ACTION_GRIPPER,0,0,0,0,0,0,0,0},
    {ACTION_MOVE,0.6,0.1,0.2,0,3,10,0,0},
    {ACTION_END}
};

static ArmAction HOME_SEQ[3 ] =
{
		{ACTION_SWITCH_TO_SUCTION,-Suction_length/100.0f,	Y_OFFSET,	Z_BASE,	-A_offset_angle,	1.0,	1.5,	0,	0},
    {ACTION_END}
};


ArmAction* GetTaskSequence(ARM_TASK task)
{
    switch(task)
    {
    case ARM_TASK::PICK_UP_KFS:
        return PICK_UP_KFS_SEQ;
    case ARM_TASK::PICK_DOWN_KFS:
        return PICK_DOWN_KFS_SEQ;
    case ARM_TASK::PUT_KFS:
        return PUT_KFS_SEQ;
    case ARM_TASK::PICK_WEAPON:
        return PICK_WEAPON_SEQ;
    case ARM_TASK::CONNECT_WEAPON:
        return CONNECT_WEAPON_SEQ;
    case ARM_TASK::REALSE_WEAPON:
        return REALSE_WEAPON_SEQ;
    case ARM_TASK::HOME:
        return HOME_SEQ;
    default:
        return nullptr;
    }
}
// ==============  规划  ========================
TrapezoidalPlanner::Profile 
TrapezoidalPlanner::compute(float L, float v_max, float a_max)
{
    Profile pf{};
    if(L < 1e-6f) L = 1e-6f;

    float La = 0.5f * v_max * v_max / a_max;

    if(L <= 2*La)
    {
        pf.v_max = sqrtf(L*a_max);
        pf.Ta = pf.v_max / a_max;
        pf.Tv = 0;
    }
    else
    {
        pf.v_max = v_max;
        pf.Ta = v_max / a_max;
        pf.Tv = (L-2*La)/v_max;
    }

    pf.Td = pf.Ta;
    pf.t_total = pf.Ta + pf.Tv + pf.Td;
    pf.L = L;
    pf.a_max = a_max;

    return pf;
}

TrapezoidalPlanner::State
TrapezoidalPlanner::get_state(const Profile& pf, float t)
{
    State st{0};

    if(t<=0) return st;
    if(t>=pf.t_total){ st.s=1; return st;}

    if(t < pf.Ta)
        st.s = 0.5f*pf.a_max*t*t/pf.L;
    else if(t < pf.Ta+pf.Tv)
    {
        float t2 = t-pf.Ta;
        float s_acc = 0.5f*pf.a_max*pf.Ta*pf.Ta;
        st.s = (s_acc + pf.v_max*t2)/pf.L;
    }
    else
    {
        float t3 = t-(pf.Ta+pf.Tv);
        float s1 = pf.L - 0.5f*pf.a_max*pf.Ta*pf.Ta;
        st.s = (s1 + pf.v_max*t3 - 0.5f*pf.a_max*t3*t3)/pf.L;
    }

    return st;
}

// ================= Motion =================
void ArmMotion::SetEndType(EndType type)
{
    end_type_ = type;
}

ArmMotion::ArmMotion(motor::DjiMotor& mx,
                     motor::DjiMotor& my,
                     motor::DjiMotor& mz,
										 motor::JointM& ma)
{
    motor_[0]=&mx;
    motor_[1]=&my;
    motor_[2]=&mz;
    dm_motor[0] =&ma;
}

void ArmMotion::MoveTo(float x,float y,float z,float a,
                       float speed,float acc)
{
    move_time_ = 0;
    busy_ = true;

    float theta = a;
	//末端为吸盘的时候
		if(end_type_ == END_SUCTION)
		{
		//解算行程
		length[0] =  (x * 100.00f - (Suction_length * cosf(theta)));
		length[1] = -(y - Y_OFFSET) * 100.00f;
		length[2] = ((z - Z_BASE) * 100.00f - (Suction_length * sinf(theta)));
		//解算弧度
    target_[0] = length[0]/ X_radius ;
    target_[1] = length[1] / Y_radius ;
    target_[2] =  length[2] / Z_radius ;
    target_[3] = a;
		//补偿
    tgt_[0] = target_[0] + X_offset_angle;
    tgt_[1] = target_[1] + Y_offset_angle;
    tgt_[2] = target_[2] + Z_offset_angle;
    tgt_[3] = target_[3] + A_offset_angle;
		}
		//末端是夹爪的时候
		else
		{
		//解算行程
		length[0] =  (x * 100.00f - (Gripper_length * cosf(theta)));
		length[1] = -(y - Y_OFFSET) * 100.00f;
		length[2] = ((z - Z_BASE) * 100.00f - (Gripper_length * sinf(theta)));
		//解算弧度
    target_[0] = length[0]/ X_radius ;
    target_[1] = length[1] / Y_radius ;
    target_[2] =  length[2] / Z_radius ;
    target_[3] = a;
		//补偿
    tgt_[0] = target_[0] + X_offset_angle;
    tgt_[1] = target_[1] + Y_offset_angle;
    tgt_[2] = target_[2] + Z_offset_angle;
    tgt_[3] = target_[3] + A_offset_angle + PI/2;
		}
    // ===== 统一时间轴 =====
    float Lx = fabsf(tgt_[0] - cur_[0]);
    float Ly = fabsf(tgt_[1] - cur_[1]);
    float Lz = fabsf(tgt_[2] - cur_[2]);
    float La = fabsf(tgt_[3] - cur_[3]);

    float Lmax = fmaxf(fmaxf(Lx, Ly), fmaxf(Lz, La));

    pf_ = TrapezoidalPlanner::compute(Lmax, speed, acc);

    // 记录起点终点
    for(int i=0;i<4;i++)
    {
        axis_[i].start  = cur_[i];
        axis_[i].target = tgt_[i];
    }
}

void ArmMotion::Update(float dt)
{
    if(!busy_) return;

    move_time_ += dt;

    auto st = TrapezoidalPlanner::get_state(pf_, move_time_);
    float s = st.s;

    // ===== 插补 =====
    for(int i = 0; i < 4; i++)
    {
        cmd_[i] = axis_[i].start +
                  (axis_[i].target - axis_[i].start) * s;
    }

    // ===== 下发 =====
    motor_[0]->Set_Out_Pos(cmd_[0]);
    motor_[1]->Set_Out_Pos(cmd_[1]);
    motor_[2]->Set_Out_Pos(cmd_[2]);
    dm_motor[0]->Set_Out_Pos(cmd_[3]);

    // ===== 更新反馈 =====
    cur_[0] = motor_[0]->Get_Out_Pos() ;
    cur_[1] = motor_[1]->Get_Out_Pos();
    cur_[2] = motor_[2]->Get_Out_Pos() ;
    cur_[3] = dm_motor[0]->Get_Out_Pos() ;

    // ===== 误差 =====
    error_[0] = fabsf(cur_[0] - tgt_[0]);
    error_[1] = fabsf(cur_[1] - tgt_[1]);
    error_[2] = fabsf(cur_[2] - tgt_[2]);
    error_[3] = fabsf(cur_[3] - tgt_[3]);

    // ===== 结束判断（统一时间轴）=====
    

    bool all_close = true;
    for(int i = 0; i < 4; i++)
    {
        if(error_[i] > pos_eps)
        {
            all_close = false;
            break;
        }
    }

    if(move_time_ >= pf_.t_total && all_close)
    {
        busy_ = false;
        move_time_ = 0;
    }
}








// ================= Gripper =================

Gripper::Gripper(motor::DjiMotor& m)
{
    motor_ = &m;
}

void Gripper::Open()
{
    motor_->Set_Current(open_current);
}

void Gripper::Close()
{
    motor_->Set_Current(close_current);
}


// ================= Suction  =================
Suction::Suction(GPIO_TypeDef* port, uint16_t pin)
{
		port_ = port;
    pin_ = pin;
}

void Suction::On()
{
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    state_ = true;
}

void Suction::Off()
{
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    state_ = false;
}
		
// ================= System =================



ArmSystem::ArmSystem(tim::Tim* tim,
                     ArmMotion* motion,
                     Gripper* gripper,
                     Suction* suction)
: TimHandler(tim),
  ManagedTask("arm",25,256,task::TASK_DELAY,2),
  motion_(motion),
  gripper_(gripper),
  suction_(suction)
{}

void ArmSystem::SetTask(ARM_TASK task)
{
    if(task == current_task_) return;

    current_task_ = task;
    seq_ = GetTaskSequence(task);
    index_ = 0;
}

void ArmSystem::Tim_It_Process()
{
    if(motion_) motion_->Update(0.002f);
}

void ArmSystem::Task_Process()
{
    if(!motion_ || !seq_) return;
    if(!motion_->IsBusy())
    {
        auto& act = seq_[index_];

        switch(act.type)
        {
        case ACTION_MOVE:
            motion_->MoveTo(act.x,act.y,act.z,act.a,act.speed,act.acc);
						index_++;
            break;
				case ACTION_SWITCH_TO_GRIPPER:
						motion_->SetEndType(END_GRIPPER);
						motion_->MoveTo(act.x,act.y,act.z,act.a,act.speed,act.acc);
						index_++;
						break;
				
				case ACTION_SWITCH_TO_SUCTION:
						motion_->SetEndType(END_SUCTION);
						motion_->MoveTo(act.x,act.y,act.z,act.a,act.speed,act.acc);
						index_++;
						break;
				
        case ACTION_GRIPPER:
            if(act.cmd) gripper_->Close();
            else        gripper_->Open();
            index_++;
            break;

        case ACTION_SUCTION:
            if(act.cmd) suction_->On();
            else        suction_->Off();
            index_++;
            break;

        case ACTION_WAIT:
            if(wait_start_==0)
            {
                wait_start_ = xTaskGetTickCount();
                return;
            }
            if(xTaskGetTickCount()-wait_start_ < act.wait_ms)
                return;
            wait_start_=0;
            index_++;
            break;

        case ACTION_END:
            seq_ = nullptr;
            current_task_ = ARM_TASK::IDLE;
            break;
        }
    }
}



