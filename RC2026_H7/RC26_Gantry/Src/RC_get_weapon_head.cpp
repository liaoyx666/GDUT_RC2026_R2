// 底盘移动速度 
// 夹爪反馈

#include "RC_get_weapon_head.h"

float test_z = 0.035;//龙门架z轴多抬高高度
float down_z_dist = 0.13;//龙门架下降时距离夹爪的距离

float GET_Z_ER = -0.02;//龙门架下降时距离夹爪的距离

float test_vel = 2.5f;// 底盘接近武器头时的最大速度（在夹爪触发前）
float Kp = 5.0f; // 底盘接近武器头时的速度控制增益（在夹爪触发前）

float chassis_y_error_threshold = 0.05f; // 底盘y轴误差阈值，当误差小于该值时认为y轴到位

namespace gantry
{
GetWeaponHead::GetWeaponHead(
    chassis::Chassis& omni4chassis_,
    data::RobotPose& pose_,
    Gantry& gantry_,
    Gripper& gripper_,
    path::PathPlan3& path_plan_,
	path::HeadCtrl& head_ctrl_,
    bool blue_side_
    ):
        weapon_event(20, 0.8f, true, false),
        path_plan(path_plan_), 
		head_ctrl(head_ctrl_),
        pose(pose_), omni4chassis(omni4chassis_),
        user(gantry_),  
        gripper(gripper_),
        chassis_npid_y(1.6f, 0.0f, 2.0f, 0.05f, 1.5f, 0.01f),
        blue_side(blue_side_)
    {
        state = State::IDLE;
        is_picked = false; 
        current_target_idx = 0;
        grab_start_time = 0;

        if(blue_side) {
            target_yaw = - PI / 2.f;
            TARGET_WEAPON_Y = -6.0f;
            READY_POINT_Y = TARGET_WEAPON_Y + READY_DIST + HALF_CHASSIS_Y;
        }
        else {// 红区
            target_yaw = PI / 2.f;
            TARGET_WEAPON_Y = 0.0f;
            READY_POINT_Y = TARGET_WEAPON_Y - READY_DIST - HALF_CHASSIS_Y;
        }

    }

    void GetWeaponHead::Auto_Get_Weapon_Head() {

        if (state != State::IDLE && state != State::CHECK_RESULT) {
            head_ctrl.Enable();
            head_ctrl.Set_Yaw(target_yaw);
        } else {
            head_ctrl.Disable();
        }

        switch (state) {
            case State::IDLE:
                if (weapon_event.Is_Trig()) {
                    Pick(pick_num); 
                    path_plan.Disable();
                    head_ctrl.Enable();
                }
                break;
            //底盘x轴到位 夹爪z轴和y轴到位 才可以夹爪x轴往前伸 
            //底盘y轴快到位时停下 直接夹爪x轴往前伸补偿
            //算出夹爪x轴的实时位置 距离武器头5cm时下降一点z轴
            case State::ALIGN_CHASSIS:  {
                if(!user.Take_Control()) {
                    break;
                }

                gripper.Open();
                
                curr_x = pose.X() + RADAR_ERROR_X;
                curr_y = pose.Y() + RADAR_ERROR_Y;
                
                // --- 底盘 X 轴控制 ---
                float error_x = WEAPON_X_RAW[current_target_idx] - curr_x;
                float abs_error_x = fabsf(error_x);
                float target_vx = 0.0f;

                bool x_in_range = (
                curr_x + HALF_CHASSIS_X > WEAPON_X_RAW[current_target_idx]) && 
                (curr_x - HALF_CHASSIS_X < WEAPON_X_RAW[current_target_idx]);
                
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
                bool y_ok = false;
                if(fabsf(curr_y - READY_POINT_Y) < chassis_y_error_threshold) {
                    target_vy = 0.0f;
                    y_ok = true;
                }

                omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(target_vx, target_vy));

                // --- 预设云台高度和夹爪 ---
                if(!sig1)
                {
                    user.Set_X(GANTRY_RETRACT_X);
                    sig1 = true;
                }
                user.Set_Z((GET_Z + test_z + GET_Z_ER));
                user.Set_P(PI / 2.f);
                gripper.Open();

                // --- 龙门架 Y 轴补偿 ---
                float target_gantry_y = 0.0f;

                if(!sig3)
                {
                    if (x_in_range) {
						
                        target_gantry_y = WEAPON_X_RAW[current_target_idx] - curr_x;
                        user.Set_Y(target_gantry_y);
                        gantry_yz_ready = (fabsf(user.Get_Y() - target_gantry_y) < GANTRY_POS_TOLERANCE)
                                        && (fabsf(user.Get_Z() - (GET_Z + test_z + GET_Z_ER)) < GANTRY_POS_TOLERANCE);
                        if(gantry_yz_ready) {
                            sig3 = true;
                        }
                    }
                }



                // --- 状态跳转判定 ---
                if (x_in_range && gantry_yz_ready) {
                    if(!sig2)
                    {
                        user.Set_X(READY_DIST);
                        sig2 = true;
                    }
                }

                // --- 龙门架 X 轴补偿 ---
                float target_gantry_x = 0.0f;
                target_gantry_x = fabs(TARGET_WEAPON_Y - curr_y) - HALF_CHASSIS_Y;
                if(y_ok && x_in_range && gantry_yz_ready) {
                    user.Set_X(target_gantry_x);
                }

                // if(fabsf(TARGET_WEAPON_Y - curr_y) - (user.Get_X() + HALF_CHASSIS_Y) < down_z_dist) {
                //     user.Set_Z(GET_Z);
                // }

                bool all_ready = 
                x_in_range && 
                gantry_yz_ready && 
                y_ok 
                //&& (fabsf(TARGET_WEAPON_Y - curr_y) - (user.Get_X() + HALF_CHASSIS_Y) < down_z_dist)
                ;

                if (all_ready) {
                    state = State::ACTION_GRAB_1;
                    StopChassis(); 
                }
                break;
            }

            case State::ACTION_GRAB_1:{
                // 底盘停止，只动云台
                user.Set_Z(GET_Z + GET_Z_ER);
                
                StopChassis();
                float current_target_gantry_x = fabs(TARGET_WEAPON_Y - curr_y) - HALF_CHASSIS_Y;
                if (fabsf(user.Get_X() - current_target_gantry_x) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_GRAB_2;
                }
                break;
            }

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
                if (timer::Timer::Get_DeltaTime(grab_start_time) >= 1000000) {
                    state = State::ACTION_LIFT_Z;
                }
                break;

            case State::ACTION_LIFT_Z:
                user.Set_Z((GET_Z + GET_Z_ER) + LIFT_UP_Z);
				user.Set_P(0.0f);
                if (fabsf(user.Get_Z() - (GET_Z + GET_Z_ER + LIFT_UP_Z)) < GANTRY_POS_TOLERANCE) {
                    state = State::ACTION_RETRACT_X;
					weapon_event.Finish();
					path_plan.Enable();
                    head_ctrl.Disable();
                }
                break;

            case State::ACTION_RETRACT_X:
                user.Set_X(GANTRY_RETRACT_X);
                //user.Set_P(0.0f);
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
                // is_picked = true;
                user.Give_Control();  // 归还权限
                state = State::IDLE;
                break;
        }
        		//head_ctrl.Head_Ctrl();
    }

void GetWeaponHead::StopChassis() {
        omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(0.0f, 0.0f));
    }

void GetWeaponHead::Pick(uint8_t num) {
    if (is_picked) return;
    sig1 = false;
    sig2 = false;
    sig3 = false;
    gantry_yz_ready = false;

    if (num >= 1 && num <= WEAPON_NUM) {
        current_target_idx = num - 1;
        state = State::ALIGN_CHASSIS;
    }
}

void GetWeaponHead::Pick_Nearest() {
    if (is_picked) return; 
    float min_dist = 999.0f;
    uint8_t nearest = 0;

    curr_x = pose.X() + RADAR_ERROR_X;
    curr_y = pose.Y() + RADAR_ERROR_Y;

    sig1 = false;
    sig2 = false;
    sig3 = false;
    gantry_yz_ready = false;

    for (uint8_t i = 0; i < WEAPON_NUM; ++i) {
        float dist = fabsf(curr_x - WEAPON_X_RAW[i]);
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
    is_picked = false;
}

}