#pragma once
#include <stdint.h>
#include "RC_event3.h"
#include "RC_task.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"
#include "RC_gantry.h"
#include "RC_head_ctrl.h"
#include "RC_nonlinear_pid.h"
#include "RC_gripper.h"
#include "RC_path_plan3.h"
#include "RC_mini_laser.h"
#ifndef PI
#define PI 3.1415926535f
#endif

#ifdef __cplusplus
namespace gantry
{
    class GetWeaponHead
    {
    public:

        enum class Computer_Side : uint8_t {
            BLUE_SIDE = 1,
            RED_SIDE = 0
        };

        enum class SpeedMode : uint8_t
        {
            SLOW = 0,
            NORMAL,
            FAST
        };

        enum class CHASSIS_STATE : uint8_t {
            Chassis_Idle,    // 等待触发
            Chassis_Align,  // 调整底盘位置，到达死区即停(x轴在范围内即可)
            Chassis_Ready,  

            Gantry_Grab_Y, 

        };

        enum class GANTRY_STATE : uint8_t {
            Gantry_Idle,           // 等待触发
            Gantry_Ready_Z_Pitch,  // z_pitch写死，y根据底盘位置调整，只要z到达预设高度，x可以提前伸出到Gantry_Ready_X
            Gantry_Ready_X,  // x轴伸出到预设位置
            Gantry_Down_Z,   // Chssis_Ready以及Gantry_Y就位后，下降z轴到预设位置
            Gantry_Grab_X,
            Gantry_Grab,
            Wait_Gantry_Grab, 

            Gantry_Up_Z_Pitch,
            Gantry_Re_Ready_X,
            Judge_Grab, 

            Gantry_Result,// 根据Judge_Grab判断是否夹取成功，成功则直接结束，失败则进入Pick_Next回到上面的状态机
            
            // 成功 释放底盘
            Gantry_Restoration_X,
            Gantry_Restoration_YZ,

            // 失败 底盘y和龙门架y都就位时 重新进入Gantry_Down_Z
            Gantry_Retry, 

        };

        GetWeaponHead(
        chassis::Chassis& omni4chassis_,
        data::RobotPose& pose_,
        Gantry& gantry_, 
        mini_laser::MiniLaser& laser_,
        Gripper& gripper_, 
        path::PathPlan3& path_plan_,
	    path::HeadCtrl& head_ctrl_,
        Computer_Side computer_side_ = Computer_Side::BLUE_SIDE
    );        
        ~GetWeaponHead() = default;
        void Set_Side(bool side);
        void Set_Pick_Num(int num) { pick_num = num; }
        void Auto_Get_Weapon_Head();
        
    private:
        void StopChassis();
        void Pick(uint8_t num);      
        void Pick_Next(); 

        bool MoveChassis(float world_x, float world_y, float deadzone);
        
        void Set_Yaw(float yaw);

        void Cal_Current_Pos();// 注意激光数据跳变

        bool Set_Gantry_X(float target_x);
        bool Set_Gantry_Y(float target_y);
        bool Set_Gantry_Z(float target_z);
        void Set_Gantry_Pitch(float target_pitch);

        void UpdateSideParam();
        void Set_Ctrl_Mode(SpeedMode mode_);


        data::RobotPose& pose;
        chassis::Chassis& omni4chassis;
        GantryUser user;
        Gripper& gripper; 
        mini_laser::MiniLaser& laser;
        path::Event3 weapon_event;
        path::PathPlan3& path_plan;
		path::HeadCtrl& head_ctrl;
        pid::NonlinearPid chassis_npid_;

        CHASSIS_STATE chassis_state;
        GANTRY_STATE  gantry_state;

        bool PICK_SUCCESS_FLAG = false;

        // （从底盘中心到夹爪的距离）即夹爪x轴方向
        static constexpr float HALF_CHASSIS_Y = 0.41227f; 
        // 武器头数量
        static constexpr uint8_t WEAPON_NUM = 6; 

        // 武器头坐标
        float TARGET_WEAPON_Y = -6.0f;
        // 先定义原始坐标
        static constexpr float WEAPON_X_RAW[7] = {
            0.0f, 0.45f, 0.65f, 0.85f, 1.05f, 1.25f, 1.45f //跟pick_num对齐，第一个0.0不用
        };

        //取武器头底盘需要的位姿
        float chassis_target_x ;
        float chassis_target_y ;
        float target_yaw ;
        float chassis_target_yaw ; 

		float curr_x;
		float curr_y;

        uint32_t grab_start_time;

        int pick_num;
        Computer_Side computer_side; // 蓝区/红区标志

        // 龙门架三轴位置
        static constexpr float GET_Z = (0.317129f - 0.028f + 0.03f); // 取武器头z轴位置
        static constexpr float LIFT_UP_Z = 0.01f;//取到武器头后上升距离
        static constexpr float LIFT_UP_Z_ = 0.03f;//取到武器头后上升距离

        static constexpr float GANTRY_RETRACT_X = 0.03f;//龙门架复位后X轴位置

        static constexpr float READY_GANTRY_DIST = 0.16f;
        static constexpr float READY_CHASSIS_DIST = 0.22f;

        // 停止阈值
        static constexpr float GANTRY_POS_TOLERANCE = 0.01f;
        static constexpr float CHASSIS_POS_TOLERANCE = 0.1f;
        static constexpr uint32_t WAIT_GANTRY_GRAB_TIME =  350000U ; // 单位：us

        uint8_t detect_cnt = 0;
        static constexpr float GRAB_DETECT_THRESH = 100.0f;
        bool rst_y_done = false;
        bool rst_z_done = false;

        // Y轴触发基准
        float READY_POINT_Y;

        // 雷达偏移
        static constexpr float RADAR_ERROR_X = 0.0f;
        static constexpr float RADAR_ERROR_Y = 0.0f;

    };
}
#endif