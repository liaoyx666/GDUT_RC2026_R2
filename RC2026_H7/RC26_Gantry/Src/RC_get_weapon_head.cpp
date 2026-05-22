// 底盘移动速度 
// 夹爪反馈

#include "RC_get_weapon_head.h"
 float test_val1 = 0;
 float test_val2 = 0;
 bool test_flag1 = 0;
 bool test_flag2 = 0;

 bool test_flag3 = 0;
 bool test_flag4 = 0;
 bool test_flag5 = 0;


namespace gantry
{
GetWeaponHead::GetWeaponHead(
    chassis::Chassis& omni4chassis_,
    data::RobotPose& pose_,
    Gantry& gantry_,
    Gripper& gripper_,
    path::PathPlan3& path_plan_
    ):
        // task::ManagedTask("GetWeaponHead", 34, 128, task::TASK_DELAY, 1),
        weapon_event(20, 0.1f, true, false),
        path_plan(path_plan_), 
        //   trigger_event(1, true, 0.1f), 
        pose(pose_), omni4chassis(omni4chassis_),
        user(gantry_),
        gripper(gripper_),
        // --- 新增：初始化三轴PID参数 (Kp, accel, delta, max_out, deadzone) ---
        chassis_npid_x(1.62f, 0.0f, 2.0f, 0.05f, 1.5f, 0.01f),
        chassis_npid_y(1.62f, 0.0f, 2.0f, 0.05f, 1.5f, 0.01f),
        head_ctrl(pose_, omni4chassis_, 0.01f)
    {
        state = State::IDLE;
        picked = false; 
        current_target_idx = 0;
        grab_start_time = 0;
        // 删除原来的 distance_npid.Init(...)


    }

    void GetWeaponHead::Pick(uint8_t num) {
        if (picked) return;
        if (num >= 1 && num <= WEAPON_NUM) {
            current_target_idx = num - 1;
            state = State::ALIGN_CHASSIS;
        }
    }

    void GetWeaponHead::Pick_Nearest() {
        if (picked) return; 
        float min_dist = 999.0f;
        uint8_t nearest = 0;
        float curr_x;
        float curr_y;


        curr_x = pose.X();
        curr_y = pose.Y();
        

        for (uint8_t i = 0; i < WEAPON_NUM; ++i) {
            float dist = fabsf(curr_x - weapon_heads_x[i]);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = i;
            }
        }
        current_target_idx = nearest;
        state = State::ALIGN_CHASSIS;
    }

void GetWeaponHead::Reset_Task() {
        state = State::IDLE;
        picked = false;
    }

    void GetWeaponHead::Auto_Get_Weapon_Head() {
        //head_ctrl.Head_Ctrl();  

        // ================= 新增：全局航向托管 =================
        // 只要不是空闲或结束状态，就开启 head_ctrl 死死锁住目标角度
        // 这样即使底盘线速度为0，受到撞击也会自动纠正角度
        if (state != State::IDLE && state != State::CHECK_RESULT) {
            head_ctrl.Enable();
            head_ctrl.Set_Yaw(TARGET_YAW);
        } else {
            head_ctrl.Disable();
        }


        switch (state) {
            case State::IDLE:
                // ===== 修复：空闲时底盘强制停 =====
                // omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(0, 0));
                if (weapon_event.Is_Trig()) {
                    Pick(pick_num); 
                    path_plan.Disable();
                    head_ctrl.Enable();
                }
                break;

            case State::ALIGN_CHASSIS: {
                if(!user.Take_Control()) {
                break; // 拿不到权限就不执行
                }
                float curr_x = pose.X();
                float curr_y = pose.Y();
                
                float target_gantry_y = 0.0f;

                // 计算 X 轴分配逻辑
                if (curr_x + HALF_CHASSIS_X < weapon_heads_x[current_target_idx]) {
                    target_x = weapon_heads_x[current_target_idx] - HALF_CHASSIS_X;
                    target_gantry_y = HALF_CHASSIS_X;
                } else if (curr_x - HALF_CHASSIS_X > weapon_heads_x[current_target_idx]) {
                    target_x = weapon_heads_x[current_target_idx] + HALF_CHASSIS_X;
                    target_gantry_y = -HALF_CHASSIS_X;
                } else {
                    target_x = curr_x;
                    target_gantry_y = weapon_heads_x[current_target_idx] - curr_x;
                }

                user.Set_Y(target_gantry_y);
                target_y = READY_POINT_Y;
                target_yaw = TARGET_YAW;

                // 预设云台高度和夹爪
                user.Set_X(GANTRY_RETRACT_X);
                user.Set_Z(GET_Z);
                user.Set_P(PI / 2.f);
                gripper.Open();

                bool chassis_ready = Chassis3AxisPosControl(target_x, target_y, target_yaw);

                bool gantry_ready = (fabsf(user.Get_Y() - target_gantry_y) < GANTRY_POS_TOLERANCE) &&
                                    (fabsf(user.Get_Z() - GET_Z) < GANTRY_POS_TOLERANCE);
test_val1 = target_gantry_y;
test_val2 = user.Get_Y();
test_flag1 = chassis_ready;
test_flag2 = gantry_ready;
                if(gantry_ready)
                {
                    user.Set_X(READY_DIST);
                }
                if (chassis_ready && gantry_ready) {
                    state = State::ACTION_GRAB_1;
                    StopChassis(); // 双重保险：进入下一状态前确保底盘完全停止
                }

                break;
            }

            case State::ACTION_GRAB_1:
                // 底盘停止，只动云台
                user.Set_X(READY_DIST);
                StopChassis();
                if (fabsf(user.Get_X() - READY_DIST) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_GRAB_2;
                }
                break;

            case State::ACTION_GRAB_2:
                StopChassis();
                gripper.Close();
//                 if (gripper.IsPickSuccess()) {
//     // 夹到了！
// }
                grab_start_time = timer::Timer::Get_TimeStamp();
                state = State::ACTION_GRAB_2_1;
                break;

            case State::ACTION_GRAB_2_1:
                StopChassis();
                if (timer::Timer::Get_DeltaTime(grab_start_time) >= 500000) {
                    state = State::ACTION_LIFT_Z;
                }
                break;

            case State::ACTION_LIFT_Z:
                StopChassis();
                user.Set_Z((GET_Z) + LIFT_UP_Z);
                if (fabsf(user.Get_Z() - (GET_Z + LIFT_UP_Z)) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_RETRACT_X;
                }
                break;

            case State::ACTION_RETRACT_X:
                StopChassis();
                user.Set_X(GANTRY_RETRACT_X);
                user.Set_P(0.0f);
                if (fabsf(user.Get_X() - GANTRY_RETRACT_X) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_RETRACT_YZ;
                }
                break;

            case State::ACTION_RETRACT_YZ:
                StopChassis();
                user.Set_Reset_Pos();
                if (fabsf(user.Get_Y() - 0.0f) < GANTRY_POS_TOLERANCE && fabsf(user.Get_Z() - 0.0f) < GANTRY_POS_TOLERANCE) {
                    state = State::CHECK_RESULT;
                }
                break;

            case State::CHECK_RESULT:
                picked = true;
                weapon_event.Finish();
                path_plan.Enable();
                user.Give_Control();  // 归还权限
                state = State::IDLE;
                break;
        }

    }

void GetWeaponHead::StopChassis() {
        omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(0.0f, 0.0f));
    }

// =========================================================================
    bool GetWeaponHead::Chassis3AxisPosControl(float t_x, float t_y, float t_yaw) {
        float real_yaw = pose.Yaw(); // 请确保此获取方式与你的代码库匹配
        float real_x = pose.X();
        float real_y = pose.Y();

        // 1. 判断是否到达目标位置
        bool x_ok = fabsf(real_x - t_x) <= POS_TOLERANCE;
        bool y_ok = fabsf(real_y - t_y) <= POS_TOLERANCE;
        
        // 角度误差归一化 [-PI, PI]
        float yaw_error = t_yaw - real_yaw;
        if(yaw_error > PI) yaw_error -= 2 * PI;
        if(yaw_error < -PI) yaw_error += 2 * PI;
        bool yaw_ok = fabsf(yaw_error) <= YAW_TOLERANCE;
test_flag3 = x_ok;
test_flag4 = y_ok;
test_flag5 = yaw_ok;
        // 2. 到达目标 → 强制下发 0 速度并退出
        if (x_ok && y_ok && yaw_ok) {
            StopChassis();
            return true; 
        }

        // 3. 未到目标，使用非线性PID计算三轴速度
        float target_vx = chassis_npid_x.NPid_Calculate(t_x, real_x, false, 0.0f);
        float target_vy = chassis_npid_y.NPid_Calculate(t_y, real_y, false, 0.0f);

        // 4. 下发三轴速度 (X线速度, Y线速度, 旋转角速度, 当前yaw用于底盘解算)
omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(target_vx, target_vy));        
        return false;
    }
}