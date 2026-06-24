#pragma once
#include "RC_gantry.h"
#include "RC_event3.h"
#include "RC_task.h"
#include "RC_suction.h"
#include "RC_LiDAR.h"
#include "RC_timer.h"
#include <math.h>
#include "RC_data_pool.h"
#include "RC_mini_laser.h"
#include "RC_filter.h"

#ifdef __cplusplus
namespace gantry
{

enum  ARM_TASK
{
		/*			STEP1			*/
    PICK_UP_KFS_20CM_1_step1,
		PICK_UP_KFS_20CM_2_step1,
		PICK_UP_KFS_20CM_3_step1,
		PICK_UP_KFS_20CM_4_step1,
	
		PICK_UP_KFS_40CM_1_step1,
		PICK_UP_KFS_40CM_2_step1,
		PICK_DOWN_KFS_1_step1,
    PICK_DOWN_KFS_2_step1,
		PICK_DOWN_KFS_3_step1,
		PICK_DOWN_KFS_4_step1,
		
		PICK_UP_KFS_0CM_1_step1,
		PICK_UP_KFS_0CM_2_step1,	
		PICK_UP_KFS_0CM_3_step1,
		PICK_UP_KFS_0CM_4_step1,

			/*		STEP2 				*/
		PICK_UP_KFS_20CM_1_step2,
		PICK_UP_KFS_20CM_2_step2,
		PICK_UP_KFS_20CM_3_step2,
		PICK_UP_KFS_20CM_4_step2,
	
		PICK_UP_KFS_40CM_1_step2,
		PICK_UP_KFS_40CM_2_step2,
		
		PICK_DOWN_KFS_1_step2,
    PICK_DOWN_KFS_2_step2,
		PICK_DOWN_KFS_3_step2,
		PICK_DOWN_KFS_4_step2,
		
		PICK_UP_KFS_0CM_1_step2,
		PICK_UP_KFS_0CM_2_step2,	
		PICK_UP_KFS_0CM_3_step2,
		PICK_UP_KFS_0CM_4_step2,
		
	
    HOME
};

class GetKFS
    {
    public:
			enum class SpeedMode : uint8_t
			{
				SLOW = 0,
				NORMAL,
				FAST
			};
			GetKFS(Gantry& gantry_, Suction& suction_,mini_laser::MiniLaser& laser_);
			~GetKFS() = default;
			void Auto_Get_KFS();
    private:
			
		uint8_t step;
			enum class CtrlMode
			{
					IDLE,
					OPEN_LOOP,
					CLOSE_LOOP_LASER,
					Y_LOCK,
					Check,
			};
		
			void Set_Task(ARM_TASK task_);
			void Set_OpenLoop_Target(float x_m, float y_m, float z_m, float p_rad);
			void Update_Laser_Distance();
			void Update_Vision_Offset(float dx_m, float dy_m, float dz_m, float dp_m, bool valid);
			void Set_Ctrl_Mode(SpeedMode mode_);
			void Set_Step_Delay(uint32_t delay_ms);
			void Cancel();
			bool Is_Busy() const;
			void Finish_Event_Early();
			void Update_Camera_Distance();
			void Trigger_Task_By_Event();
			bool Configure_Current_Step();
			void Set_Step_Target(float x, float y, float z, float p, CtrlMode mode_);
			void Set_Step_Act(uint8_t suction);

			bool Reached_Target(float cmd_x, float cmd_y, float cmd_z, float cmd_p) ;
			void Go_Next_Step();
			void Finish_Current_Task();
			void Restart_Current_Task();
			void Do_Suction_Action(uint8_t action_id);
			void Lock_Current_Y();
			void Unlock_Y();
			float kfs_num;
			float locked_y;
			bool y_locked;
			GantryUser user;
			gantry::Gantry& gantry;
			path::Event3 gantry_event[5];
			path::Event3* active_event;
			Suction&  suction_;
			mini_laser::MiniLaser& laser_;
			CtrlMode mode;
			ARM_TASK cur_task;
			bool busy;
			uint16_t stable_cnt;
			uint32_t step_delay_ms;
			uint32_t step_reached_ts;
			bool wait_step_delay;

			float target_x;
			float target_y;
			float target_z;
			float target_p;
			bool mech_reached;
			 float current_x;
			 float current_y;
			 float current_z;
			 float current_p;
			float base_target_x;
			float base_target_y;
			float base_target_z;
			float base_target_p;
			uint8_t step_suction;
			uint8_t seq_idx;
			float err;
			uint8_t cor_cnt ;
			float laser_distance_m;
			float laser_target_m;
			bool laser_valid;
			float laser_err_i ;

		float cam_err_i;         // 摄像头 I 积分项
		float cam_target_pixel;  // 视觉期望中心像素位置
		float camera_distance_m; // 当前视觉检测到的像素位置
		bool  camera_valid;
		uint32_t laser_lost_ts;
		bool laser_lost_start;
		uint8_t laser_retry_cnt;
		
		filter::SecondOrderLPF filter;

		//float laser_err_lpf ;
    };

}
#endif