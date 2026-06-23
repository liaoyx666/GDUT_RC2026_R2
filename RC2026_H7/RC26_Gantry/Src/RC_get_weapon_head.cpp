#include "RC_get_weapon_head.h"

namespace gantry
{
GetWeaponHead::GetWeaponHead(
    chassis::Chassis& omni4chassis_,
    data::RobotPose& pose_,
    Gantry& gantry_, 
    mini_laser::MiniLaser& laser_,
    Gripper& gripper_, 
    path::PathPlan3& path_plan_,
    path::HeadCtrl& head_ctrl_,
    Computer_Side computer_side_
    ):
    weapon_event(20, 0.8f, true, false),
    path_plan(path_plan_), 
    head_ctrl(head_ctrl_),
    pose(pose_), 
    omni4chassis(omni4chassis_),
    user(gantry_),  
    laser(laser_),
    gripper(gripper_),
    computer_side(computer_side_),
    chassis_npid_(
        1.5f,// kp
        0.0f,// kd
        2.0f,// acc
        0.3f,// delta
        1.5f,// max_out
        0.02// deadzone 小于CHASSIS_POS_TOLERANCE
            )
{
    chassis_state = CHASSIS_STATE::Chassis_Idle;
    gantry_state = GANTRY_STATE::Gantry_Idle;
    gripper.Stop();

    grab_start_time = 0;
    pick_num = 1;

    chassis_target_x = 0.0f;
    chassis_target_y = 0.0f;
    chassis_target_yaw = 0.0f; 

    UpdateSideParam();

}

void GetWeaponHead::Auto_Get_Weapon_Head() {

    if (weapon_event.Is_Trig() && gantry_state == GANTRY_STATE::Gantry_Idle) {
        Pick(pick_num); 
        path_plan.Disable();
        head_ctrl.Enable();
        Set_Yaw(chassis_target_yaw);
    }

    // 更新curr_x和curr_y
    curr_x = pose.X() + RADAR_ERROR_X;
    curr_y = pose.Y() + RADAR_ERROR_Y;

    switch (chassis_state)
    {
    case CHASSIS_STATE::Chassis_Idle:
        break;

    case CHASSIS_STATE::Chassis_Align:
        if(MoveChassis(chassis_target_x, chassis_target_y, CHASSIS_POS_TOLERANCE)) {
            chassis_state = CHASSIS_STATE::Gantry_Grab_Y;
        }
        break;

    case CHASSIS_STATE::Gantry_Grab_Y:
        MoveChassis(chassis_target_x, chassis_target_y, CHASSIS_POS_TOLERANCE);

        if(computer_side == Computer_Side::RED_SIDE) {
            user.Set_Y(curr_x - WEAPON_X_RAW[pick_num]);
            chassis_state = CHASSIS_STATE::Chassis_Ready;
        } else {
            user.Set_Y(WEAPON_X_RAW[pick_num] - curr_x);
                            chassis_state = CHASSIS_STATE::Chassis_Ready;

        }

        break;

    case CHASSIS_STATE::Chassis_Ready:
        StopChassis();
        if(!user.Take_Control()) {
            break;
        }

        if(computer_side == Computer_Side::RED_SIDE) {
            user.Set_Y(curr_x - WEAPON_X_RAW[pick_num]);
        } else {
            user.Set_Y(WEAPON_X_RAW[pick_num] - curr_x);
        }

        
        break;

    default:
        break;
    }


    switch (gantry_state)
    {
    case GANTRY_STATE::Gantry_Idle:
        break;

    case GANTRY_STATE::Gantry_Ready_Z_Pitch:
        gripper.Open();
        Set_Gantry_Pitch(PI * 0.5f);
        if( Set_Gantry_Z( GET_Z + LIFT_UP_Z) )
        {
            gantry_state = GANTRY_STATE::Gantry_Ready_X;
        }
        break;

    case GANTRY_STATE::Gantry_Ready_X:
        {
            float gantry_ready_x = fabs(TARGET_WEAPON_Y - curr_y) - HALF_CHASSIS_Y - READY_GANTRY_DIST;
            
            if( Set_Gantry_X(gantry_ready_x) ) {
                gantry_state = GANTRY_STATE::Gantry_Down_Z;
            }
            break;
        }
    case GANTRY_STATE::Gantry_Down_Z:
        if(chassis_state == CHASSIS_STATE::Chassis_Ready)
        {
            if( Set_Gantry_Z( GET_Z ) ) {
                gantry_state = GANTRY_STATE::Gantry_Grab_X;
            }
        }
        break;
    
    case GANTRY_STATE::Gantry_Grab_X:
        {
        float gantry_ready_x = fabs(TARGET_WEAPON_Y - curr_y) - HALF_CHASSIS_Y;
        if( Set_Gantry_X( gantry_ready_x ) ) {
            gantry_state = GANTRY_STATE::Gantry_Grab;
        }
        break;
        }

    case GANTRY_STATE::Gantry_Grab:
        gripper.Close();
        grab_start_time = timer::Timer::Get_TimeStamp();
        gantry_state = GANTRY_STATE::Wait_Gantry_Grab;
        break;

    case GANTRY_STATE::Wait_Gantry_Grab:
        if (timer::Timer::Get_DeltaTime(grab_start_time) >= WAIT_GANTRY_GRAB_TIME ) {
            gantry_state = GANTRY_STATE::Gantry_Up_Z_Pitch;
        }
        break;
    
    case GANTRY_STATE::Gantry_Up_Z_Pitch:
        Set_Gantry_Pitch(0.0f);
        if( Set_Gantry_Z( GET_Z + LIFT_UP_Z ) ) {
            gantry_state = GANTRY_STATE::Gantry_Re_Ready_X;
        }
        break;

    case GANTRY_STATE::Gantry_Re_Ready_X:
        {
            float gantry_ready_x = fabs(TARGET_WEAPON_Y - curr_y) - HALF_CHASSIS_Y - READY_GANTRY_DIST;
            
            if( Set_Gantry_X(gantry_ready_x) ) {
                gantry_state = GANTRY_STATE::Judge_Grab;
            }
            break;
        }
    case GANTRY_STATE::Judge_Grab:

        if(laser.distance < GRAB_DETECT_THRESH)
        {
            detect_cnt ++;
            if(detect_cnt >= 3) 
            {
                PICK_SUCCESS_FLAG = true;
                detect_cnt = 0; // 判定成功后清零
                gantry_state = GANTRY_STATE::Gantry_Result;
            }
        }
        else
        {
            detect_cnt = 0;
            PICK_SUCCESS_FLAG = false;
            gantry_state = GANTRY_STATE::Gantry_Result;
        }
        break;

    case GANTRY_STATE::Gantry_Result:
      
        if(PICK_SUCCESS_FLAG) {
            gantry_state = GANTRY_STATE::Gantry_Restoration_X;
            weapon_event.Finish();
            path_plan.Enable();
            // head_ctrl.Disable();
            chassis_state = CHASSIS_STATE::Chassis_Idle;
        }
        else {
            gantry_state = GANTRY_STATE::Gantry_Retry;
        }
        
        break;
    
    case GANTRY_STATE::Gantry_Retry: // 失败进入这
        Pick_Next();
        gripper.Open();
        Set_Gantry_Pitch(PI * 0.5f);
        break;

    case GANTRY_STATE::Gantry_Restoration_X://成功进入这
        if( Set_Gantry_X( GANTRY_RETRACT_X ) ) {
            gantry_state = GANTRY_STATE::Gantry_Restoration_YZ;
        }
        break;

    case GANTRY_STATE::Gantry_Restoration_YZ:
        Set_Gantry_Pitch(0.0f);
        if( Set_Gantry_Y(0.0f) && Set_Gantry_Z(0.0f)   ) {
            user.Give_Control();
            gantry_state = GANTRY_STATE::Gantry_Idle;
        }
        break;

    default:
        break;
    }

}

void GetWeaponHead::StopChassis() {
    omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(0.0f, 0.0f));
}

void GetWeaponHead::Pick(uint8_t num) {
    chassis_state = CHASSIS_STATE::Chassis_Align;
    gantry_state = GANTRY_STATE::Gantry_Ready_Z_Pitch;

    chassis_target_x = WEAPON_X_RAW[num];
    chassis_target_y = READY_POINT_Y;
    chassis_target_yaw = target_yaw; 

}

void GetWeaponHead::Pick_Next() {
    pick_num++;
    if(pick_num > WEAPON_NUM) {
        pick_num = 1;
    }      
    Pick(pick_num);
    gantry_state = GANTRY_STATE::Gantry_Down_Z;
}

bool GetWeaponHead::MoveChassis(float world_x, float world_y, float deadzone) {

    float error_x = world_x - curr_x;
    float error_y = world_y - curr_y;
    float target_vx = 0.0f;
    float target_vy = 0.0f;

    float error_dist = sqrtf(error_x * error_x + error_y * error_y);
    if (error_dist < deadzone) {
        target_vx = 0.0f;
        target_vy = 0.0f;
        omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(target_vx, target_vy));
        return true;
    } else {
        float chassis_vx = chassis_npid_.NPid_Calculate(error_dist, 0.0f, false, 0.0f);
        float angle_to_target = atan2f(error_y, error_x);
        target_vx = chassis_vx * cosf(angle_to_target);
        target_vy = chassis_vx * sinf(angle_to_target);
        omni4chassis.Set_World_Lin_Vel(vector2d::Vector2D(target_vx, target_vy));
    }

    return false;
}


void GetWeaponHead::Set_Yaw(float yaw) {
    head_ctrl.Set_Yaw(yaw);
}

void GetWeaponHead::Cal_Current_Pos() {
}

bool GetWeaponHead::Set_Gantry_X(float target_x){
    if(!user.Take_Control()) {
        return false;
    }
    user.Set_X(target_x);
    if(fabsf(user.Get_X() - target_x) < GANTRY_POS_TOLERANCE) {
        return true;
    }
    return false;
}

bool GetWeaponHead::Set_Gantry_Y(float target_y){
    if(!user.Take_Control()) {
        return false;
    }
    user.Set_Y(target_y);
    if(fabsf(user.Get_Y() - target_y) < GANTRY_POS_TOLERANCE) {
        return true;
    }
    return false;
} 

bool GetWeaponHead::Set_Gantry_Z(float target_z){
    if(!user.Take_Control()) {
        return false;
    }
    user.Set_Z(target_z);
    if(fabsf(user.Get_Z() - target_z) < GANTRY_POS_TOLERANCE) {
        return true;
    }
    return false;
}

void GetWeaponHead::Set_Gantry_Pitch(float target_pitch){
    if(!user.Take_Control()) {
        return ;
    }
    user.Set_P(target_pitch);

    return ;
}

void GetWeaponHead::Set_Side(bool side) { 
	if(side == true)
	{
		this->computer_side = Computer_Side::BLUE_SIDE; 
	}
	else
	{
		this->computer_side = Computer_Side::RED_SIDE; 
	}
    UpdateSideParam(); 
}

void GetWeaponHead::UpdateSideParam()
{
    if(computer_side == Computer_Side::BLUE_SIDE) {
        target_yaw = - PI / 2.f;
        TARGET_WEAPON_Y = -6.0f;
        READY_POINT_Y = TARGET_WEAPON_Y + READY_CHASSIS_DIST + HALF_CHASSIS_Y;
    } else {
        target_yaw = PI / 2.f;
        TARGET_WEAPON_Y = 6.0f;
        READY_POINT_Y = TARGET_WEAPON_Y - READY_CHASSIS_DIST - HALF_CHASSIS_Y;
    }
}


}