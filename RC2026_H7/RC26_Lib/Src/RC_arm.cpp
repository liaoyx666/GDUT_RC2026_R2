#include "RC_arm.h"

/*======================================底层代码=================================*/
namespace arm {

    ArmDynamics::ArmDynamics() {}

    void ArmDynamics::gravity_compensation()
    {
        joint_angle_now.theta1 = -motor_angle.theta1;
        joint_angle_now.theta2 = -motor_angle.theta1 + motor_angle.theta2 + PI - 0.146084f;
        joint_angle_now.theta3 = -motor_angle.theta3 + motor_angle.theta2 - motor_angle.theta1 - 0.146084f;

        joint_gravity_compensation.joint1 =
            L1_gravity * L1_Particle_LENGTH * cosf(joint_angle_now.theta1) +
            L2_gravity * (L2_Particle_LENGTH * cosf(joint_angle_now.theta2) + L1_LENGTH * cosf(joint_angle_now.theta1)) +
            L3_gravity * (L3_Particle_LENGTH * cosf(joint_angle_now.theta3) + L2_LENGTH * cosf(joint_angle_now.theta2) + L1_LENGTH * cos(joint_angle_now.theta1));

        joint_gravity_compensation.joint2 =
            L2_gravity * L2_Particle_LENGTH * cosf(joint_angle_now.theta2) +
            L3_gravity * (L3_Particle_LENGTH * cosf(joint_angle_now.theta3) + L2_LENGTH * cosf(joint_angle_now.theta2));

        joint_gravity_compensation.joint3 =
            L3_gravity * L3_Particle_LENGTH * cosf(joint_angle_now.theta3);
    }

    float ArmKinematics::unwrapAngle(float now, float last)
    {
        while (now - last > PI) now -= 2.0f * PI;
        while (now - last < -PI) now += 2.0f * PI;
        return now;
    }

    // ------------------ 静态成员变量 ------------------
    ArmMatrix<4, 4> ArmKinematics::T01;
    ArmMatrix<4, 4> ArmKinematics::T12;
    ArmMatrix<4, 4> ArmKinematics::T23;
    ArmMatrix<4, 4> ArmKinematics::T34;
    ArmMatrix<4, 4> ArmKinematics::T45;
    ArmMatrix<4, 4> ArmKinematics::T05;

    // ------------------ 工具函数 ------------------
    float ArmKinematics::normalizeAngle(float angle)
    {
        while (angle <= -PI) angle += 2 * PI;
        while (angle > PI) angle -= 2 * PI;
        return angle;
    }

    float ArmKinematics::constrainValue(float value, float min, float max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    ArmMatrix<4, 4> ArmKinematics::buildDHTable(float theta, float alpha, float a, float d, float offset)
    {
        ArmMatrix<4, 4> dh;
        dh.setZero();

        float ct = cosf(theta + offset);
        float st = sinf(theta + offset);
        float ca = cosf(alpha);
        float sa = sinf(alpha);

        dh(0,0) = ct;
        dh(0,1) = -st*ca;
        dh(0,2) = st*sa;
        dh(0,3) = a*ct;

        dh(1,0) = st;
        dh(1,1) = ct*ca;
        dh(1,2) = -ct*sa;
        dh(1,3) = a*st;

        dh(2,1) = sa;
        dh(2,2) = ca;
        dh(2,3) = d;

        dh(3,3) = 1.0f;
        return dh;
    }

    // ------------------ 正运动学 ------------------
    void ArmKinematics::forward(const JointAngles& angles, EndEffectorPos& end_pos)
    {
        float theta1 = angles.theta1;
        float theta2 = angles.theta2;
        float theta3 = angles.theta3;
        float theta4 = angles.theta4;

        T01 = buildDHTable(theta1, PI/2, 0.0f, BASE_HEIGHT, THETA1_OFFSET);
        T12 = buildDHTable(theta2, 0.0f, L1_LENGTH, 0.0f, THETA2_OFFSET);
        T23 = buildDHTable(theta3, 0.0f, L2_LENGTH, 0.0f, THETA3_OFFSET);
        T34 = buildDHTable(0.0f, 0.0f, L3_LENGTH, 0.0f, THETA4_OFFSET);
        T45 = buildDHTable(theta4, 0.0f, L4_LENGTH, 0.0f, THETA5_OFFSET);

        T05 = T01 * T12 * T23 * T34 * T45;

        end_pos.x = T05(0,3);
        end_pos.y = T05(1,3);
        end_pos.z = T05(2,3);

        // angle = q1 + q2 + q3 + π (小臂 offset)
        end_pos.angle = normalizeAngle(theta2 + theta3 + THETA4_OFFSET + theta4 + THETA1_OFFSET + THETA2_OFFSET + THETA3_OFFSET + THETA5_OFFSET);
    }

    bool ArmKinematics::inverse(const EndEffectorPos& target, JointAngles& result)
    {
        float px = target.x;
        float py = target.y;
        float pz = target.z;
        float pitch = target.angle;

        /* ---------- 常量 ---------- */
        const float L1_d = BASE_HEIGHT;
        const float L2 = L1_LENGTH;
        const float L3 = L2_LENGTH;
        const float L4 = L3_LENGTH;
        const float L5 = L4_LENGTH;
        const float off2 = THETA2_OFFSET;
        const float off3 = THETA3_OFFSET;
        const float off4 = THETA4_OFFSET;
        const float off5 = THETA5_OFFSET;

        /* ---------- q1 候选 ---------- */
        float q1_base = atan2f(py, px);
        float q1_candidates[2] = {q1_base, q1_base + PI};

        struct Cand { float q1, q2, q3, q5, score; } cands[2];
        int cand_count = 0;

        for (int i = 0; i < 2; ++i)
        {
            float q1 = q1_candidates[i];
            q1 = normalizeAngle(q1); // [-pi, pi]

            /* ---------- 基座平面投影 ---------- */
            float R = px * cosf(q1) + py * sinf(q1);
            float Z = pz - L1_d;

            /* ---------- Wrist Center ---------- */
            float WR = R - L5 * cosf(pitch);
            float WZ = Z - L5 * sinf(pitch);
            float D = sqrtf(WR * WR + WZ * WZ);

            /* ---------- 等效连杆 ---------- */
            float Vx = L3 + L4 * cosf(off4);
            float Vy = L4 * sinf(off4);
            float Lv = sqrtf(Vx * Vx + Vy * Vy);
            float beta = atan2f(Vy, Vx);

            /* ---------- 检查可达性 ---------- */
            if (D > L2 + Lv || D < fabsf(L2 - Lv)) continue;

            float cos_arg = (L2 * L2 + D * D - Lv * Lv) / (2.0f * L2 * D);
            cos_arg = constrainValue(cos_arg, -1.0f, 1.0f);

            float alpha = acosf(cos_arg);
            float theta_w = atan2f(WZ, WR);

            /* ---------- 肘上解 ---------- */
            float elbow = 1.0f;
            float theta_L2 = theta_w + elbow * alpha;
            float q2 = theta_L2 - off2;
            q2 = normalizeAngle(q2);

            float ER = L2 * cosf(theta_L2);
            float EZ = L2 * sinf(theta_L2);

            float theta_v = atan2f(WZ - EZ, WR - ER);
            float q3 = theta_v - theta_L2 - off3 - beta;
            q3 = normalizeAngle(q3);

            float q5 = pitch - ((q2 + off2) + (q3 + off3) + off4 + off5);
            q5 = normalizeAngle(q5);

            /* ---------- 评分 ---------- */
            float cost_base = fabsf(q1) + fabsf(normalizeAngle(q1 - last_joint.theta1));
            float cost_arm_flat = fabsf(theta_L2);
            float cost_elbow = fabsf(fabsf(theta_v - theta_L2) - PI / 2.0f);

            float score = 10.0f * cost_base + 4.0f * cost_arm_flat + 2.0f * cost_elbow;

            cands[cand_count++] = {q1, q2, q3, q5, score};
        }

        if (cand_count == 0) return false;

        /* ---------- 选择最佳候选 ---------- */
        int best_idx = 0;
        float best_score = cands[0].score;
        for (int i = 1; i < cand_count; ++i)
        {
            if (cands[i].score < best_score)
            {
                best_score = cands[i].score;
                best_idx = i;
            }
        }

        /* ---------- 限幅 ---------- */
        float rq1 = constrainValue(cands[best_idx].q1, THETA1_MIN, THETA1_MAX);
        float rq2 = constrainValue(cands[best_idx].q2, THETA2_MIN, THETA2_MAX);
        float rq3 = constrainValue(cands[best_idx].q3, THETA3_MIN, THETA3_MAX);
        float rq4 = constrainValue(cands[best_idx].q5, THETA4_MIN, THETA4_MAX);

        result.theta1 = rq1;
        result.theta2 = rq2;
        result.theta3 = rq3;
        result.theta4 = rq4;

        last_joint = result;
        return true;
    }

}

/*=======================================规划层代码====================================*/
namespace path {

    // =============================================
    // 速度规划
    // =============================================
    TimePlanner::Profile TimePlanner::compute(float L, float v_max, float a_max)
    {
        Profile pf{};
        pf.L = L;
        pf.v_max = v_max;
        pf.a_max = a_max;

        float t_acc = v_max / a_max;
        float d_acc = 0.5f * a_max * t_acc * t_acc;

        if (2*d_acc >= L)
        {
            t_acc = sqrtf(L / a_max);
            pf.t_acc = t_acc;
            pf.t_cruise = 0;
            pf.t_total = 2*t_acc;
        }
        else
        {
            float d_cruise = L - 2*d_acc;
            float t_cruise = d_cruise / v_max;
            pf.t_acc = t_acc;
            pf.t_cruise = t_cruise;
            pf.t_total = 2*t_acc + t_cruise;
        }

        return pf;
    }

    float TimePlanner::timeToS(const Profile& pf, float t)
    {
        float a = pf.a_max;
        float v = pf.v_max;
        float L = pf.L;

        if (t <= 0) return 0;
        if (t >= pf.t_total) return 1;

        if (t < pf.t_acc)
        {
            float d = 0.5f*a*t*t;
            return d / L;
        }

        if (t < pf.t_acc + pf.t_cruise)
        {
            float d_acc = 0.5f*a*pf.t_acc*pf.t_acc;
            float d = d_acc + v*(t - pf.t_acc);
            return d / L;
        }

        float td = t - pf.t_acc - pf.t_cruise;
        float d_acc = 0.5f*a*pf.t_acc*pf.t_acc;
        float d_cruise = v*pf.t_cruise;
        float d_dec = v*td - 0.5f*a*td*td;

        return (d_acc + d_cruise + d_dec) / L;
    }

} // namespace path

/*================================应用层==========================================*/
using namespace path;
/*================= 静态 IO 接口实现 =================*/
void Arm_task::Front_Air_Pump_On()		{HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);}
void Arm_task::Front_Air_Pump_Off()		{HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);}
const float pos_tol = 0.005f;  //允许的误差
/*================= 静态成员初始化 =================*/
Pose3D Arm_task::home_pose = 			POS(-0.103f, 0.0f, 0.290, 2.0071f); //取前上
Pose3D Arm_task::safe_pos = POS(0.011f, 0.0f, 0.314f,  1.392f);

Pose3D Arm_task::pick_front_cube0 = POS(0.50f, 0.0f, 0.24f,  -0.22f);
Pose3D Arm_task::pick_front_cube2 = POS(0.70f, 0.0f, 0.19f,  -0.32f);
//放左
Pose3D Arm_task::pick_front_cube_test2 = POS(0.153f,   -0.111f, 0.418, 0.0f);
Pose3D Arm_task::pick_front_cube_test3 = POS(-0.103f,  0.075f, 0.502f, 1.57f);
//放右
Pose3D Arm_task::pick_front_cube_test0 = POS(-0.153f,   0.111f, 0.418, -0.1f);
Pose3D Arm_task::pick_front_cube_test1 = POS(0.103f,  -0.075f, 0.502f, 1.57f);

//取前下
Pose3D Arm_task::pick_front_cube5 = POS(0.55f, 0.0f, -0.15f, -0.28f);
Pose3D Arm_task::pick_front_cube6 = POS(0.67f, 0.0f, -0.15f, -0.34f);
Pose3D Arm_task::pick_front_cube7 = POS(0.278f, 0.0f, 0.158f, 0.0f);
//Pose3D Arm_task::pick_front_cube8 = POS(0.2f, 0.0f, 0.3f, 0.0f);

/*================= 动作序列定义 =================*/
ArmAction Arm_task::PICK_FRONT_UP_CUBE_SEQ[5] = {
    { ARM_ACTION_IO, {}, {}, 0, 0, 0, false, 0, Arm_task::Front_Air_Pump_On, nullptr },
    { ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube0, 0.0f, 2.0f, 3.0f, true, 0, nullptr, nullptr },
    { ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube2, 0.0f, 1.0f, 2.0f, true, 0, nullptr, nullptr },
    { ARM_ACTION_END }
};

ArmAction Arm_task::PICK_FRONT_DOWN_CUBE_SEQ[5] = {

    { ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube5, 0.0f, 2.0f, 2.5f, true, 0, nullptr, nullptr },
    { ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube6, 0.0f, 1.0f, 2.0f, true, 0, nullptr, nullptr },
		{ ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube7, 0.0f, 0.8f, 3.0f, true, 0, nullptr, nullptr },
		//{ ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube7, 0.0f, 3.0f, 3.5f, true, 0, nullptr, nullptr },
		//{ ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube8, 0.0f, 1.5f, 1.5f, true, 0, nullptr, nullptr },
		//{ ARM_ACTION_MOVE, {}, Arm_task::home_pose, 0.0f, 2.0f, 2.0f, true, 0, nullptr, nullptr },
    { ARM_ACTION_END }
};

ArmAction Arm_task::PLACE_RIGHT_CUBE_SEQ[7] = 
{
		{ ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube_test0, 0.0f, 2.0f, 3.0f, true, 0, nullptr, nullptr },
    { ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube_test1, 0.0f, 2.0f, 3.0f, true, 0, nullptr, nullptr },
		{ ARM_ACTION_IO, {}, {}, 0, 0, 0, false, 0, Arm_task::Front_Air_Pump_Off, nullptr },
    { ARM_ACTION_HOLD, {}, {}, 0, 0, 0, false, 3000, nullptr, nullptr },
		{ ARM_ACTION_MOVE, {}, Arm_task::safe_pos, 0.0f, 1.5f, 2.0f, true, 0, nullptr, nullptr },
    { ARM_ACTION_END }
};

ArmAction Arm_task::PLACE_LEFT_CUBE_SEQ[8] = {

    { ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube_test2, 0.0f, 2.0f, 2.5f, true, 0, nullptr, nullptr },
		{ ARM_ACTION_MOVE, {}, Arm_task::pick_front_cube_test3, 0.0f, 1.0f, 2.0f, true, 0, nullptr, nullptr },
		{ ARM_ACTION_IO, {}, {}, 0, 0, 0, false, 0, Arm_task::Front_Air_Pump_Off, nullptr },
    { ARM_ACTION_HOLD, {}, {}, 0, 0, 0, false, 3000, nullptr, nullptr },
    { ARM_ACTION_MOVE, {}, Arm_task::safe_pos, 0.0f, 1.0f, 2.0f, true, 0, nullptr, nullptr },
    { ARM_ACTION_END }
};

/*================= 构造函数 =================*/
Arm_task::Arm_task(tim::Tim& tim_, motor::Motor& motor_1_, motor::Motor& motor_2_, motor::Motor& motor_3_, motor::Motor& motor_4_) :
    tim::TimHandler(tim_),
    ManagedTask("arm_task", 25, 256, task::TASK_DELAY, 2),
    g_state(ARM_STATE_IDLE),
    g_task_mode(ARM_TASK::IDLE),
    g_last_task_mode(ARM_TASK::IDLE),
    g_task_lock(false),
    g_hold_end_tick(0),
    g_action_seq(nullptr),
    g_action_index(0),
    is_reseted(false),
    g_last_pose(home_pose),
    current_target_pose_(),
    current_target_joints_(),
    current_is_joint_space_(false),
    current_speed_(0.0f),
    current_acc_(0.0f),
    move_start_time_(0.0f),
    start_joints_(),
    start_pose_(),
    last_tick_(0),
		is_finished(0)
{
    Arm_motor[0] = &motor_1_;
    Arm_motor[1] = &motor_2_;
    Arm_motor[2] = &motor_3_;
    Arm_motor[3] = &motor_4_;
}



/*================= 成员函数实现 =================*/
void Arm_task::startTask(ARM_TASK task)
{
    g_task_mode = task;
    g_task_lock = true;
    is_finished = false;
}

bool Arm_task::isTaskFinished() const
{
    return is_finished;
}

bool Arm_task::Arm_IsBusy(void)
{
    return (g_state != ARM_STATE_IDLE);
}

bool Arm_task::Arm_Control(ARM_TASK task)
{
    if (task <= ARM_TASK::IDLE || task > ARM_TASK::HOME) return false;
    if (this->Arm_IsBusy() || g_task_lock) return false;
    if (!task_started_ || active_task_ != task)
    {
        startTask(task);
        active_task_ = task;
        task_started_ = true;
        return false;   
    }
  
    if (isTaskFinished())
    {
        task_started_ = false;      
        active_task_ = ARM_TASK::IDLE;
        return true;                
    }
    return false;                   
}


void Arm_task::Bind_Action_Sequence(void)
{
    switch (g_task_mode)
    {
        case ARM_TASK::PICK_FRONT_UP_CUBE:				g_action_seq = PICK_FRONT_UP_CUBE_SEQ;
            break;
        case ARM_TASK::PICK_FRONT_DOWN_CUBE:			g_action_seq = PICK_FRONT_DOWN_CUBE_SEQ;
            break;
        case ARM_TASK::PLACE_LEFT_CUBE:						g_action_seq = PLACE_LEFT_CUBE_SEQ;
            break;
        case ARM_TASK::PLACE_RIGHT_CUBE:					g_action_seq = PLACE_RIGHT_CUBE_SEQ;
            break;
        default:
            g_action_seq = nullptr;
            break;
    }
    g_action_index = 0;
}

void Arm_task::Arm_Hold(uint32_t ms)
{
    g_hold_end_tick = osKernelGetTickCount() + ms;
    g_state = ARM_STATE_WAITING;
}

void Arm_task::Arm_Path_Manager(void *)
{
    if (!is_reseted)
    {
        is_reseted = true;
        g_last_pose = home_pose;
    }

    
    if (g_task_mode != g_last_task_mode)
    {
        Bind_Action_Sequence();
        g_state = ARM_STATE_IDLE;
        g_last_task_mode = g_task_mode;
    }

    if (g_state == ARM_STATE_WAITING)
    {
        if (osKernelGetTickCount() >= g_hold_end_tick) g_state = ARM_STATE_IDLE;
        return;
    }

    if (g_state == ARM_STATE_FINISHED)
    {
        const ArmAction &last_act = g_action_seq[g_action_index];
        if (last_act.type == ARM_ACTION_MOVE) g_last_pose = last_act.target;
        g_action_index++;
        g_state = ARM_STATE_IDLE;
				is_finished = 1;
        return;
    }

    if (!g_action_seq) return;

    const ArmAction &act = g_action_seq[g_action_index];

    switch (act.type)
    {
			is_finished = 0;
        case ARM_ACTION_MOVE:
            if (g_state == ARM_STATE_IDLE)
            {
                move_start_time_ = 0;
                last_tick_ = osKernelGetTickCount();
                g_state = ARM_STATE_MOVING;

                if (act.joint_space)
                {
                    arm::ArmKinematics ik;
                    arm::JointAngles sol;
                    arm::EndEffectorPos tgt{act.target.pos.x(), act.target.pos.y(), act.target.pos.z(), act.target.pitch};

                    if (!ik.inverse(tgt, sol))
                    {
                        g_task_mode = ARM_TASK::IDLE;
                        g_state = ARM_STATE_IDLE;
                        break;
                    }

                    cmd_joints_ = current_joint_angles;
                    start_joints_ = current_joint_angles;
                    current_target_joints_ = {sol.theta1, sol.theta2, sol.theta3, sol.theta4};
                    current_is_joint_space_ = true;
                    current_speed_ = act.speed;
                    current_acc_ = act.acc;

                    // 计算全局 profile
                    float maxL = 0;
                    for (int i = 0; i < ARM_JOINTS; i++)
                    {
                        float L = fabsf(current_target_joints_.j[i] - start_joints_.j[i]);
                        if (L > maxL) maxL = L;
                    }
                    global_profile_ = path::TimePlanner::compute(maxL, current_speed_, current_acc_);
                }
                else
                {
                    // 笛卡尔空间
                    start_pose_ = g_last_pose;     

										current_target_pose_ = act.target;
										current_is_joint_space_ = false;
										current_speed_ = act.speed;
										current_acc_ = act.acc;
								
										float L = (current_target_pose_.pos - start_pose_.pos).length() 
															+ fabsf(current_target_pose_.pitch - start_pose_.pitch) * 0.1f;
										global_profile_ = path::TimePlanner::compute(L, current_speed_, current_acc_);
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
            if (act.cond_func && act.cond_func()) g_action_index++;
            break;

        case ARM_ACTION_END:
					cmd_joints_ = current_joint_angles;
				
					g_state = ARM_STATE_IDLE;   
					g_task_mode = ARM_TASK::IDLE;
					g_last_task_mode = ARM_TASK::IDLE;  
					g_action_seq = nullptr;
					g_action_index = 0;
					g_task_lock = false;
					         
					return;                             
			
    }
	
}
void Arm_task::executeMove()
{
    // 关节空间规划
    if (current_is_joint_space_)
    {
        move_s_ = path::TimePlanner::timeToS(global_profile_, move_start_time_);

        for (int i = 0; i < ARM_JOINTS; i++)
        {
            float delta = current_target_joints_.j[i] - start_joints_.j[i];
            cmd_joints_.j[i] = start_joints_.j[i] + delta * move_s_;
        }
    }
    else
    {
        move_s_ = path::TimePlanner::timeToS(global_profile_, move_start_time_);

        if (move_s_ < 1.0f)
        {
            arm::ArmKinematics ik;
            vector3d::Vector3D delta_pos(
						current_target_pose_.pos.x() - start_pose_.pos.x(),
						current_target_pose_.pos.y() - start_pose_.pos.y(), 
						current_target_pose_.pos.z() - start_pose_.pos.z()
				);
            float delta_pitch = current_target_pose_.pitch - start_pose_.pitch;

						vector3d::Vector3D next_pos(
							start_pose_.pos.x() + (current_target_pose_.pos.x() - start_pose_.pos.x()) * move_s_,
							start_pose_.pos.y() + (current_target_pose_.pos.y() - start_pose_.pos.y()) * move_s_,
							start_pose_.pos.z() + (current_target_pose_.pos.z() - start_pose_.pos.z()) * move_s_
					);

            float next_pitch = start_pose_.pitch + delta_pitch * move_s_;

            arm::EndEffectorPos next{next_pos.x(), next_pos.y(), next_pos.z(), next_pitch};
            arm::JointAngles q;
            if (ik.inverse(next, q))
            {
                cmd_joints_ = {q.theta1, q.theta2, q.theta3, q.theta4};
            }
        }
        else
        {
            arm::ArmKinematics ik;
            arm::JointAngles q;
            arm::EndEffectorPos tgt{
                current_target_pose_.pos.x(),
                current_target_pose_.pos.y(),
                current_target_pose_.pos.z(),
                current_target_pose_.pitch
            };
            if (ik.inverse(tgt, q))
            {
                cmd_joints_ = {q.theta1, q.theta2, q.theta3, q.theta4};
            }
						else 
							{
							cmd_joints_ = current_joint_angles;
							}
        }
    }
}

bool Arm_task::isAllReached()
{
    

    for (int i = 0; i < ARM_JOINTS; i++)
    {
        if (fabsf(current_joint_angles.j[i] - cmd_joints_.j[i]) > pos_tol) return false;
    }
    return true;
}

void Arm_task::Tim_It_Process()
{
    // 只有 MOVING 才执行
    if (g_state != ARM_STATE_MOVING || !g_task_lock) return;

    // 1. 读取电机当前位置
    current_joint_angles = {
        Arm_motor[0]->Get_Out_Pos(),
        Arm_motor[1]->Get_Out_Pos(),
        Arm_motor[2]->Get_Out_Pos(),
        Arm_motor[3]->Get_Out_Pos()
    };

    move_start_time_ += 0.002;

    // 3. 计算下一步 cmd
    executeMove();

    if (g_state == ARM_STATE_MOVING)
    {
        if (move_s_ >= 1.0f && isAllReached())  
        {
            g_state = ARM_STATE_FINISHED;
					return;
        }
    }

    // 4. 下发 cmd
    Arm_motor[0]->Set_Out_Pos(cmd_joints_.j[0]);
    Arm_motor[1]->Set_Out_Pos(cmd_joints_.j[1]);
    Arm_motor[2]->Set_Out_Pos(cmd_joints_.j[2]);
    Arm_motor[3]->Set_Out_Pos(cmd_joints_.j[3]);
}

void Arm_task::Task_Process()
{
    Arm_Path_Manager(nullptr);
}
