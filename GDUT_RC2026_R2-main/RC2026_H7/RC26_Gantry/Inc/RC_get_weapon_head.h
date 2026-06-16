#pragma once
#include "RC_event3.h"
#include "RC_task.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"
#include "RC_gantry.h"
#include "RC_head_ctrl.h"
#include "RC_nonlinear_pid.h"
#include "RC_gripper.h"
#include "RC_path_plan3.h"

#ifdef __cplusplus
namespace gantry
{
    class GetWeaponHead
    {
    public:
        GetWeaponHead(
        chassis::Chassis& omni4chassis_,
        data::RobotPose& pose_,
        Gantry& gantry_, 
        Gripper& gripper_, 
        path::PathPlan3& path_plan_,
	    path::HeadCtrl& head_ctrl_,
        bool blue_side = true
    );        
        ~GetWeaponHead() = default;

        void Set_Pick_Num(int num) { pick_num = num; }
        void Auto_Get_Weapon_Head();


    private:
        
        enum class State : uint8_t {
            IDLE,           // 待机：监听触发事件并检查 picked 标志
            ALIGN_CHASSIS,  // 准备：底盘对齐目标角度
            ACTION_GRAB_1,  // 执行：夹爪前伸
            ACTION_GRAB_2,  // 执行：夹爪夹紧
            ACTION_GRAB_2_1, // 执行：夹紧后等待，确保夹爪动作完成
            ACTION_LIFT_Z,  // 夹紧后 Z 轴先向上抬升 15cm
            ACTION_RETRACT_X,  // 夹紧后 X 轴单动收回，防止干涉
            ACTION_RETRACT_YZ, // X 轴安全收回后，Y 轴和 Z 轴归零
            CHECK_RESULT    // 反馈：检查结果，失败则迭代下一个
        } state;

        data::RobotPose& pose;
        chassis::Chassis& omni4chassis;
        GantryUser user;
        Gripper& gripper; 

        path::Event3 weapon_event;
        path::PathPlan3& path_plan;
		path::HeadCtrl& head_ctrl;

        pid::NonlinearPid chassis_npid_y;


        // （夹爪横移的距离）即夹爪y轴方向
        static constexpr float HALF_CHASSIS_X = 0.10f;   
        // （从底盘中心到夹爪的距离）即夹爪x轴方向
        static constexpr float HALF_CHASSIS_Y = 0.41227f; 
        // 武器头数量
        static constexpr uint8_t WEAPON_NUM = 6; 

        // 龙门架三轴位置
        static constexpr float GET_Z = 0.327129f;  
        static constexpr float LIFT_UP_Z = 0.05f;//取到武器头后上升距离
        static constexpr float GANTRY_RETRACT_X = 0.03f;//龙门架复位后X轴位置

        // 武器头坐标
        float TARGET_WEAPON_Y = -6.0f;
        // 先定义原始坐标
        static constexpr float WEAPON_X_RAW[WEAPON_NUM] = {
            0.45f, 0.65f, 0.85f, 1.05f, 1.25f, 1.45f 
        };

        // 雷达偏移
        static constexpr float RADAR_ERROR_X = -0.017f;
        static constexpr float RADAR_ERROR_Y = 0.009f;
        static constexpr float RADAR_ERROR_YAW = 0.0f * PI / 180.f;

        // 夹取准备位置(即夹爪初始状态距离武器头的y轴距离小于等于READY_DIST时才往前伸夹爪，在这之前调整地盘靠近夹爪)
        static constexpr float READY_DIST = 0.3f;
	   
        // 底盘停止的阈值
        static constexpr float YAW_TOLERANCE = 1.0f * PI / 180.f; 
        static constexpr float GANTRY_POS_TOLERANCE = 0.018f;
        static constexpr float POS_TOLERANCE = 0.01f;

        // Y轴触发基准
        float READY_POINT_Y;

        float target_x = 0.0f;
        float target_y = 0.0f;
        float target_yaw = - PI / 2.f; 

		float curr_x;
		float curr_y;
		
        uint32_t init_time;
        uint32_t grab_start_time;

        int pick_num;
        uint8_t current_target_idx; 
        bool is_picked;    // 是否已成功夹取
        bool blue_side; // 蓝区/红区标志

        void StopChassis();
        void Reset_Task();

        void Pick(uint8_t num);      
        void Pick_Nearest(); 

        bool sig1 = false;
        bool sig2 = false;
        bool sig3 = false;
				
        bool time_sig1 = true;

		
		bool sig4 = false;
		
		float pose_buf[5] = { 0 };
		int length = 0;
        
        bool gantry_yz_ready = false;

    };
}
#endif