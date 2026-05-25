// 底盘移动速度 
// 夹爪反馈

#include "RC_get_weapon_head.h"
float test_z = 0.02;
float test_interval = 0.22;

float test_vel = 2.5f;
float Kp = 2.9f; 


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
        weapon_event(20, 0.8f, true, false),
        path_plan(path_plan_), 
        //   trigger_event(1, true, 0.1f), 
        pose(pose_), omni4chassis(omni4chassis_),
        user(gantry_),
        gripper(gripper_),
        // --- 新增：初始化三轴PID参数 (Kp, accel, delta, max_out, deadzone) ---
        chassis_npid_y(1.62f, 0.0f, 2.0f, 0.05f, 1.5f, 0.01f),
        head_ctrl(pose_, omni4chassis_, 0.01f)
    {
        state = State::IDLE;
        picked = false; 
        current_target_idx = 0;
        grab_start_time = 0;


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

        // ================= 全局航向托管 =================
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

            case State::ALIGN_CHASSIS:  {
                if(!user.Take_Control()) {
                    break;
                }
                
                curr_x = pose.X();
                curr_y = pose.Y();
                
                // --- 底盘 X 轴控制 ---
                float error_x = weapon_heads_x[current_target_idx] - curr_x;
                float abs_error_x = fabsf(error_x);
                float target_vx = 0.0f;

                bool x_in_range = (
                curr_x + HALF_CHASSIS_X > weapon_heads_x[current_target_idx]) && 
                (curr_x - HALF_CHASSIS_X < weapon_heads_x[current_target_idx]);
                
                if (!x_in_range) {

                    float dist_to_boundary = abs_error_x - HALF_CHASSIS_X;
                    float calc_speed = dist_to_boundary * Kp;

                    if (calc_speed > test_vel) {
                        calc_speed = test_vel;
                    }

                    target_vx = (error_x > 0) ? calc_speed : -calc_speed;

                    user.Set_Y(0.0f);
                } else {
                    target_vx = 0.0f;
                }

                // --- 底盘 Y 轴控制 ---
                float target_vy = chassis_npid_y.NPid_Calculate(READY_POINT_Y, curr_y, false, 0.0f);
                bool y_ok = fabsf(curr_y - READY_POINT_Y) <= POS_TOLERANCE;

                omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(target_vx, target_vy));

                // --- 预设云台高度和夹爪 ---
                user.Set_X(GANTRY_RETRACT_X);
                user.Set_Z((GET_Z + test_z));
                user.Set_P(PI / 2.f);
                gripper.Open();

                // --- 龙门架 Y 轴补偿 ---
                float target_gantry_y = 0.0f;
                bool gantry_ready = false;

                if (x_in_range) {
                    target_gantry_y = weapon_heads_x[current_target_idx] - curr_x;
                    user.Set_Y(target_gantry_y);
                    gantry_ready = (fabsf(user.Get_Y() - target_gantry_y) < GANTRY_POS_TOLERANCE);
                }

                // --- 状态跳转判定 ---
                if (x_in_range && gantry_ready) {
                    user.Set_X(READY_DIST);
                }
                if(user.Get_X() >= test_interval){
                    user.Set_Z(GET_Z);
                }
                if (x_in_range && y_ok && gantry_ready) {
                    state = State::ACTION_GRAB_1;
                    StopChassis(); 
                }
                break;
            }

            case State::ACTION_GRAB_1:
                // 底盘停止，只动云台
                if(user.Get_X() >= test_interval){
                    user.Set_Z(GET_Z);
                }
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
					weapon_event.Finish();
					path_plan.Enable();
                }
                break;

            case State::ACTION_LIFT_Z:
                user.Set_Z((GET_Z) + LIFT_UP_Z);
                if (fabsf(user.Get_Z() - (GET_Z + LIFT_UP_Z)) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_RETRACT_X;
                }
                break;

            case State::ACTION_RETRACT_X:
                user.Set_X(GANTRY_RETRACT_X);
                user.Set_P(0.0f);
                if (fabsf(user.Get_X() - GANTRY_RETRACT_X) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_RETRACT_YZ;
                }
                break;

            case State::ACTION_RETRACT_YZ:
                user.Set_Reset_Pos();
                if (fabsf(user.Get_Y() - 0.0f) < GANTRY_POS_TOLERANCE && fabsf(user.Get_Z() - 0.0f) < GANTRY_POS_TOLERANCE) {
                    state = State::CHECK_RESULT;
                }
                break;

            case State::CHECK_RESULT:
                picked = true;
                user.Give_Control();  // 归还权限
                state = State::IDLE;
                break;
        }

    }

void GetWeaponHead::StopChassis() {
        omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(0.0f, 0.0f));
    }
}
