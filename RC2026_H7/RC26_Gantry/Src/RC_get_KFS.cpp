#include "RC_get_KFS.h"
namespace gantry
{
	// ========================= 到位判定阈值 =========================
	constexpr float KFS_X_REACH_TH = 0.05f;
	constexpr float KFS_Y_REACH_TH = 0.05f;
	constexpr float KFS_Z_REACH_TH = 0.05f;
	constexpr float KFS_P_REACH_TH = 0.07f;
	
	// ========================= 激光闭环参数 =========================
	constexpr float KFS_LASER_P = 0.5f;
	constexpr float KFS_LASER_I = 0.002f;
	constexpr float LASER_LPF_ALPHA = 0.08f;
	constexpr float KFS_LASER_ERR_TH = 0.0061f;   // 激光允许误差

	// 激光数据有效范围
	constexpr float raw_data_low_limit  = 0.00f;
	constexpr float raw_data_high_limit = 0.20f;

	// pitch 判定阈值
	constexpr float pitch_detech_upper = 0.4f;

	// 激光修正 Y 轴最大偏移限制
	constexpr float lidar_offset_up_limit   = 0.5f;
	constexpr float lidar_offset_down_limit = -0.5f;
	
	constexpr float KFS_CAM_P = 0.002f;           // 视觉 P 增益 (米/像素)
	constexpr float KFS_CAM_I = 0.00002f;         // 视觉 I 增益
	constexpr float KFS_CAM_ERR_TH = 2.0f;        // 像素允许误差值 (pixel)
	constexpr float cam_offset_up_limit   = 0.5f; // Y轴最大修正限制
	constexpr float cam_offset_down_limit = -0.5f;
	
	// ========================= 激光丢失重试参数 =========================
	constexpr uint32_t LASER_LOST_TIMEOUT_US = 500000; // 1s
	constexpr uint8_t  LASER_RETRY_MAX       = 1;       // 最大重试次数
	float retry_x = 0;
	float cnt = 0;
	GetKFS::GetKFS(gantry::Gantry& gantry_ , gantry::Suction& suction_, mini_laser::MiniLaser& laser_)
		: gantry(gantry_),
		  gantry_event
		{
				path::Event3(13, 0.50f, true, true, 2.9f),//20
				path::Event3(14, 0.70f, true, true, 2.9f),//40
				path::Event3(15, 0.50f, true, true, 2.9f),//-20
				path::Event3(16, 0.50f, true, true, 2.9f),//0
				path::Event3(25, 0.02f, true, true)//pick
		  },
		  suction_(suction_),
		  laser_(laser_),
		  user(gantry_)
	{
		laser_err_i = 0.f;
		//laser_err_lpf  = 0.f;
		active_event = nullptr;
		mode = CtrlMode::IDLE;
		cur_task = ARM_TASK::HOME;
		busy = false;
		stable_cnt = 0;
		step_delay_ms = 0;
		step_reached_ts = 0;
		wait_step_delay = false;
		seq_idx = 0;
		target_x = 0.03f;
		target_y = 0.00f;
		target_z = 0.00f;
		target_p = 0.0f;
		base_target_x = target_x;
		base_target_y = target_y;
		base_target_z = target_z;
		base_target_p = target_p;
		current_x = 0.0f;
		current_y = 0.0f;
		current_z = 0.0f;
		current_p = 0.0f;
	
		kfs_num = 0;
		locked_y = 0.f;
		y_locked = false;
		laser_lost_ts = 0;
		laser_lost_start = false;
		laser_retry_cnt = 0;
		step_suction = 0;
		step = 0;
		laser_target_m = 0.073f;
		laser_distance_m = 0;
		laser_valid = false;
	}
// ============================================================
	// 设置控制速度模式
	// ============================================================
		void GetKFS::Set_Ctrl_Mode(SpeedMode mode_)
	{
		float scale = 1.0f;
		switch (mode_)
		{
			case SpeedMode::SLOW:   scale = 0.45f; break;
			case SpeedMode::FAST:   scale = 1.5f; break;
			case SpeedMode::NORMAL:	scale = 1.0f;
			default:                scale = 0.55f; break;
		}

		user.Set_X_Td(3000.f * scale, 8000.f * scale);
		user.Set_Y_Td(3000.f * scale, 8000.f * scale);
		user.Set_Z_Td(2000.f * scale, 8000.f * scale);
		user.Set_P_Td(20.f   * scale, 12.f    * scale);
	}
	
void GetKFS::Finish_Event_Early()
{
    if (active_event != nullptr)
    {
        active_event->Finish();
        active_event = nullptr;
    }
}


bool GetKFS::Configure_Current_Step()
{
    switch (cur_task)
    {
             case ARM_TASK::PICK_UP_KFS_20CM_1_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.14f,  0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.35f, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
			default: return false;
            }
        
        case ARM_TASK::PICK_UP_KFS_20CM_1_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);       	    Set_Step_Target(0.35f, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);     Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(300000);   		  Set_Step_Target(0.52f+retry_x, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);          		Set_Step_Target(0.2f,  0.00f, 0.14f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);          		Set_Step_Target(0.2f,  0.00f, 0.14f, 1.57f, CtrlMode::Check);        Set_Step_Act(1); return true;
                case 4: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);      Set_Step_Target(0.2f,  0.00f, 0.14f, 1.57f, CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
                case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.20f,  0.00f, 0.14f, 1.57f, CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.10f, 0.00f, 0.07f, 0.3f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(100000);        	  Set_Step_Target(0.02f, 0.00f, 0.02f ,0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
				case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(500000);        	  Set_Step_Target(0.01f, 0.00f, 0.01f ,0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
default: return false;
            }
						
						
        case ARM_TASK::PICK_UP_KFS_20CM_2_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.14f,  0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.35f, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
            
        case ARM_TASK::PICK_UP_KFS_20CM_2_step2:
            switch (seq_idx) 
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.35f, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(300000);     	Set_Step_Target(0.52f+retry_x, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);         	  Set_Step_Target(0.2f,  0.00f, 0.40f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);         	  Set_Step_Target(0.2f,  0.00f, 0.40f, 1.57f, CtrlMode::Check);        Set_Step_Act(1); return true;
                case 4: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    Set_Step_Target(0.2f,  0.00f, 0.4f, 1.57f, CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
                case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);        Set_Step_Target(0.10f,  0.00f, 0.40f, 0.3f, CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(500000);          Set_Step_Target(0.00f, 0.00f, 0.40f, 0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
				case 7: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(500000);          Set_Step_Target(0.07f, 0.00f, 0.40f, 0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
                default: return false;
            }

        case ARM_TASK::PICK_UP_KFS_20CM_3_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.07f, 0.00f, 0.14f,  0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.35f, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
        
        case ARM_TASK::PICK_UP_KFS_20CM_3_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);       		     Set_Step_Target(0.35f, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(300000);     		 Set_Step_Target(0.52f+retry_x, 0.00f, 0.14f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);     				 Set_Step_Target(0.20f,  0.00f, 0.4f, 2.0f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);     				 Set_Step_Target(0.20f,  0.00f, 0.4f, 2.0f, CtrlMode::Check);        Set_Step_Act(1); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);         	 	 Set_Step_Target(0.20f,  0.00f, 0.4f, 2.0f, CtrlMode::CLOSE_LOOP_LASER);        Set_Step_Act(1);data::HaveOutKFS::Set_Have_Out_KFS(true); return true;							
				
				case 5: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(500000);         	  Set_Step_Target(0.08f,  0.00f, 0.4f, 2.6f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
				default: return false;
        
            }
            
        case ARM_TASK::PICK_UP_KFS_20CM_4_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.08f, 0.00f, 0.4f,  2.6f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
				case 1: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.2f, 0.00f, 0.4f, 1.57f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.2f, 0.00f, 0.75f, 1.57f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.0f, 0.00f, 0.75f, 0.0f, CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(500000);          Set_Step_Target(0.02f, 0.00f, 0.75f, 0.0f, CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
				case 5: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.35f, 0.00f, 0.16f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
        
        case ARM_TASK::PICK_UP_KFS_20CM_4_step2:
            switch (seq_idx)
            {
				//case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.35f, 0.00f, 0.16f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 0: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);     	 Set_Step_Target(0.52f+retry_x, 0.00f, 0.16f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 1: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);         	  Set_Step_Target(0.13f,  0.00f, 0.4f, 2.6f, CtrlMode::CLOSE_LOOP_LASER);        Set_Step_Act(1);data::HaveOutKFS::Set_Have_Out_KFS(true); return true;
				//
				case 2: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);         	  Set_Step_Target(0.08f,  0.00f, 0.4f, 2.6f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
				default: return false;
        
            }
						
						
        case ARM_TASK::PICK_UP_KFS_40CM_1_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.35f, 0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);     	 		 Set_Step_Target(0.35f, 0.00f, 0.35f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }

        case ARM_TASK::PICK_UP_KFS_40CM_1_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);      		 Set_Step_Target(0.35f, 0.00f, 0.35f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(300000);     Set_Step_Target(0.52f+retry_x, 0.00f, 0.35f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 2: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    		  Set_Step_Target(0.2f, 0.00f, 0.35f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    		  Set_Step_Target(0.2f, 0.00f, 0.35f, 1.57f, CtrlMode::Check);        Set_Step_Act(1); return true;
                case 4: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    		  Set_Step_Target(0.20f, 0.00f, 0.35f, 1.57f, CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
                case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.20f,  0.00f, 0.35f, 1.57f, CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 6: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.06f, 0.3f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(100000);          Set_Step_Target(0.00f, 0.00f, 0.01f, 0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
				case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(500000);          Set_Step_Target(0.02f, 0.00f, 0.00f, 0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
				default: return false;
            }

			case ARM_TASK::PICK_UP_KFS_40CM_2_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.1f, 0.00f, 0.36f, 0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);     	 		 Set_Step_Target(0.35f, 0.00f, 0.36f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }

        case ARM_TASK::PICK_UP_KFS_40CM_2_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);      		 Set_Step_Target(0.35f, 0.00f, 0.36f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(300000);     Set_Step_Target(0.52f+retry_x, 0.00f, 0.36f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 2: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    		  Set_Step_Target(0.20f, 0.00f, 0.36f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    		  Set_Step_Target(0.20f, 0.00f, 0.36f, 1.57f, CtrlMode::Check);        Set_Step_Act(1); return true;
                case 4: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);    		  Set_Step_Target(0.20f, 0.00f, 0.36f, 1.57f, CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
                case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.20f,  0.00f, 0.36f, 1.57f, CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.36f, 0.3f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(500000);          Set_Step_Target(0.00f, 0.00f, 0.36f, 0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
				case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(000000);          Set_Step_Target(0.02f, 0.00f, 0.36f, 0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
				default: return false;
            }
			
        case ARM_TASK::PICK_DOWN_KFS_1_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(10000);      Set_Step_Target(0.60f, 0.00f, 0.1f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
            
        case ARM_TASK::PICK_DOWN_KFS_1_step2:
            switch (seq_idx)
            {
			    case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);      Set_Step_Target(0.60f, 0.00f, 0.1f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(300000);     Set_Step_Target(0.60f, 0.00f, 0.025f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 2: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);          Set_Step_Target(0.60f, 0.00f, 0.10f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 3: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.30f, 0.00f, 0.35f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.30f, 0.00f, 0.35f, 1.57f, CtrlMode::Check);        Set_Step_Act(1); return true;
                case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.30f, 0.00f, 0.35f, 1.57f, CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
                case 6: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.30f, 0.00f, 0.30f, 1.57f, CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.15f, 0.00f, 0.30f, 1.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 8: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.08f, 0.00f, 0.30f, 0.6f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 9: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.00f, 0.00f, 0.2f,  0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 10: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(600000);     Set_Step_Target(0.0f,  0.00f, 0.13f,  0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
                case 11: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(500000);          Set_Step_Target(0.04f, 0.00f, 0.0f,  0.0f,  CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
                default: return false;
            }
            
        case ARM_TASK::PICK_DOWN_KFS_2_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.60f, 0.00f, 0.1f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
                
                default: return false;
            }

        case ARM_TASK::PICK_DOWN_KFS_2_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.60f, 0.00f, 0.10f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(300000);     Set_Step_Target(0.60f, 0.00f, 0.025f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 2: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);        Set_Step_Target(0.60f, 0.00f, 0.10f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 3: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);          Set_Step_Target(0.40f, 0.00f, 0.35f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);          Set_Step_Target(0.40f, 0.00f, 0.35f, 1.57f, CtrlMode::Check);        Set_Step_Act(1); return true;
                case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);      Set_Step_Target(0.40f,  0.00f, 0.35f, 1.57f, CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
                case 6: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.3f,  0.00f, 0.50f, 1.57f, CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.15f, 0.00f, 0.50f, 1.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 8: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.08f, 0.00f, 0.50f, 0.6f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 9: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.03f, 0.00f, 0.50f, 0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
                case 10: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(100000);     Set_Step_Target(0.02f, 0.00f, 0.50f, 0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
                case 11: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(500000);          Set_Step_Target(0.04f, 0.00f, 0.40f,  0.0f,  CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
                default: return false;    
            }


        case ARM_TASK::PICK_DOWN_KFS_3_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.60f, 0.00f, 0.10f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
           
                default: return false;
            }

        case ARM_TASK::PICK_DOWN_KFS_3_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.60f, 0.00f, 0.10f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 1: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(300000);     Set_Step_Target(0.60f, 0.00f, 0.025f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);        Set_Step_Target(0.60f, 0.00f, 0.10f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 3: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);        Set_Step_Target(0.20f, 0.00f, 0.40f, 2.0f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);        Set_Step_Target(0.20f, 0.00f, 0.40f, 2.0f, CtrlMode::Check);        Set_Step_Act(1); return true;
				//
				case 5: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);        Set_Step_Target(0.20f, 0.00f, 0.40f, 2.0f, CtrlMode::CLOSE_LOOP_LASER);        Set_Step_Act(1); data::HaveOutKFS::Set_Have_Out_KFS(true);return true;
                case 6: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(500000);          Set_Step_Target(0.08f, 0.00f, 0.40f, 2.6f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;

                default: return false;    
            }

        case ARM_TASK::PICK_DOWN_KFS_4_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.08f, 0.00f, 0.4f,  2.6f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.08f, 0.00f, 0.60f, 2.6f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.2f, 0.00f, 0.80f, 1.57f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
			    case 3: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.80f, 0.0f, CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(500000);          Set_Step_Target(0.00f, 0.00f, 0.80f, 0.0f, CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
				case 5: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);          Set_Step_Target(0.03f, 0.00f, 0.80f, 0.0f, CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
				
				default: return false;
            }

        case ARM_TASK::PICK_DOWN_KFS_4_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.60f, 0.00f, 0.10f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(300000);     Set_Step_Target(0.60f, 0.00f, 0.05f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				//
				case 2: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);        Set_Step_Target(0.20f, 0.00f, 0.30f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); data::HaveOutKFS::Set_Have_Out_KFS(true);return true;
                case 3: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(500000);          Set_Step_Target(0.08f, 0.00f, 0.40f, 2.6f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;    
            }
						
						
		case ARM_TASK::PICK_UP_KFS_0CM_1_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.0f,  0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.30f, 0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
        
        case ARM_TASK::PICK_UP_KFS_0CM_1_step2:
            switch (seq_idx)
            {
			//	case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);       	    Set_Step_Target(0.3f, 0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
            //    case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(300000);   		  Set_Step_Target(0.45f, 0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 0: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);          		Set_Step_Target(0.40f,  0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(500000);      Set_Step_Target(0.40f,  0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP); Set_Step_Act(1); return true;
                case 2: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);          Set_Step_Target(0.40f,  0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);           Set_Step_Act(1); return true;
                case 3: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);         Set_Step_Target(0.20f, 0.00f, 0.0f, 1.57f,  CtrlMode::OPEN_LOOP);           Set_Step_Act(1); return true;
                
				case 4: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.20f, 0.00f, 0.0f, 1.57f,  CtrlMode:: Check);           Set_Step_Act(1); return true;
				case 5: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.20f, 0.00f, 0.05f, 1.57f,  CtrlMode::CLOSE_LOOP_LASER);           Set_Step_Act(1); return true;
				case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.20f, 0.00f, 0.05f, 1.57f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(100000);        	  Set_Step_Target(0.08f, 0.00f, 0.05f ,0.6f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				
				case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(100000);        	  Set_Step_Target(0.00f, 0.00f, 0.00f ,0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
                case 9: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(100000);        	  Set_Step_Target(0.03f, 0.00f, 0.00f ,0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
                
				default: return false;
            }
						case ARM_TASK::PICK_UP_KFS_0CM_2_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.0f,  0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.3f, 0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
        
        case ARM_TASK::PICK_UP_KFS_0CM_2_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);          		Set_Step_Target(0.3f,  0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(500000);      Set_Step_Target(0.4f,  0.00f, 0.0f, 3.14f, CtrlMode::OPEN_LOOP); Set_Step_Act(1); return true;				
                case 2: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);      Set_Step_Target(0.4f,  0.00f, 0.3f, 1.57f, CtrlMode::OPEN_LOOP); Set_Step_Act(1); return true;				
                
				case 3: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.20f, 0.00f, 0.3f, 1.57f,  CtrlMode:: Check);           Set_Step_Act(1); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.20f, 0.00f, 0.3f, 1.57f,  CtrlMode::CLOSE_LOOP_LASER);           Set_Step_Act(1); return true;
				case 5: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);        	  Set_Step_Target(0.20f, 0.00f, 0.3f, 1.57f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(100000);        	  Set_Step_Target(0.08f, 0.00f, 0.4f ,0.6f,  CtrlMode::Y_LOCK);           Set_Step_Act(1); return true;
				case 7: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(500000);        	  Set_Step_Target(0.00f, 0.00f, 0.3f ,0.1f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
              case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);        	  Set_Step_Target(0.03f, 0.00f, 0.30f ,0.0f,  CtrlMode::Y_LOCK);           Set_Step_Act(0); return true;
              
				default: return false;
            }
		case ARM_TASK::PICK_UP_KFS_0CM_3_step1:
            switch (seq_idx)
            {
                case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.05f, 0.00f, 0.1f,  0.00f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);          Set_Step_Target(0.51f, 0.00f, 0.31f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                default: return false;
            }
        
        case ARM_TASK::PICK_UP_KFS_0CM_3_step2:
            switch (seq_idx)
            {
				case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);       			    Set_Step_Target(0.51f, 0.00f, 0.31f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
                case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(500000);   		    	Set_Step_Target(0.51f, 0.00f, 0.26f, 4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 2: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);          		Set_Step_Target(0.51f,  0.00f, 0.35f,4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
               case 3: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);          		Set_Step_Target(0.20f,  0.00f, 0.40f,2.0f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
				case 4: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0);          		Set_Step_Target(0.10f,  0.00f, 0.40f,2.0f, CtrlMode::Check);        Set_Step_Act(1); return true;
               case 5: Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(500000);          		Set_Step_Target(0.08f,  0.00f, 0.40f,2.3f, CtrlMode::CLOSE_LOOP_LASER);        Set_Step_Act(1); data::HaveOutKFS::Set_Have_Out_KFS(true);return true;
				default: return false;
            }
				
        case ARM_TASK::HOME:
        default:
            if (seq_idx == 0)
            {
                Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0); Set_Step_Target(0.03f, 0.00f, 0.3f, 0.00f, CtrlMode::OPEN_LOOP); Set_Step_Act(0);
                return true;
            }
            return false;
    }
}

	void GetKFS::Go_Next_Step()
{
    seq_idx++;
    switch (cur_task)
    {
        case ARM_TASK::PICK_UP_KFS_20CM_1_step2:
           if (seq_idx == 4) Finish_Event_Early();
            break;

       case ARM_TASK::PICK_UP_KFS_20CM_2_step2:
            if (seq_idx == 4) Finish_Event_Early();
            break;

	   case ARM_TASK::PICK_UP_KFS_20CM_3_step2:
            if (seq_idx == 4) Finish_Event_Early();
            break;
			  
        case ARM_TASK::PICK_UP_KFS_40CM_1_step2:
            if (seq_idx == 4) Finish_Event_Early();
            break;

		 case ARM_TASK::PICK_UP_KFS_40CM_2_step2:
            if (seq_idx == 4) Finish_Event_Early();
            break;
		 
        case ARM_TASK::PICK_DOWN_KFS_1_step2:
            if (seq_idx == 5) Finish_Event_Early();
            break;

        case ARM_TASK::PICK_DOWN_KFS_2_step2:
            if (seq_idx == 5) Finish_Event_Early();
            break;
				
		case ARM_TASK::PICK_DOWN_KFS_3_step2:
            if (seq_idx == 4) Finish_Event_Early();
            break;				
				
		case ARM_TASK::PICK_UP_KFS_0CM_1_step2:
            if (seq_idx == 4) Finish_Event_Early();
            break;
		
		
								
        default:
            break;
    }

    stable_cnt = 0;
    wait_step_delay = false;

    if (!Configure_Current_Step())
        Finish_Current_Task();
}


void GetKFS::Trigger_Task_By_Event()
{
    uint8_t kfs_count = data::KFSNum::Get_KFS_Num();
    // =========================================================
    // Event0 : 20cm Step1 (根据当前数量，依次执行第 1, 2, 3 组任务的 step1)
    // =========================================================
    if (gantry_event[0].Is_Trig())
    {
        active_event = &gantry_event[0];

        switch (kfs_count)
        {
            case 0:  Set_Task(ARM_TASK::PICK_UP_KFS_20CM_1_step1); break;
            case 1:  Set_Task(ARM_TASK::PICK_UP_KFS_20CM_2_step1); break;
            case 2:  Set_Task(ARM_TASK::PICK_UP_KFS_20CM_3_step1); break;
						case 3:  Set_Task(ARM_TASK::PICK_UP_KFS_20CM_4_step1); break;
            default: break; 
        }
    }
	
    // =========================================================
    // Event1 : 40cm Step1
    // =========================================================
    else if (gantry_event[1].Is_Trig())
    {
        active_event = &gantry_event[1];
		    switch (kfs_count)
        {
            case 0:  Set_Task(ARM_TASK::PICK_UP_KFS_40CM_1_step1); break;
            case 1:  Set_Task(ARM_TASK::PICK_UP_KFS_40CM_2_step1); break;
            default: break; 
        }
    }

    // =========================================================
    // Event2 : Down Step1 (放下动作，根据当前数量，依次执行第 1, 2, 3 组的 step1)
    // =========================================================
    else if (gantry_event[2].Is_Trig())
    {
        active_event = &gantry_event[2];

        switch (kfs_count)
        {
            case 0:  Set_Task(ARM_TASK::PICK_DOWN_KFS_1_step1); break;
            case 1:  Set_Task(ARM_TASK::PICK_DOWN_KFS_2_step1); break;
            case 2:  Set_Task(ARM_TASK::PICK_DOWN_KFS_3_step1); break;
						case 3:  Set_Task(ARM_TASK::PICK_DOWN_KFS_4_step1); break;
            default: break;
        }
    }
	  else if (gantry_event[3].Is_Trig())
    {
        active_event = &gantry_event[3];

        switch (kfs_count)
        {
						case 0:  Set_Task(ARM_TASK::PICK_UP_KFS_0CM_1_step1); break;
            case 1:  Set_Task(ARM_TASK::PICK_UP_KFS_0CM_2_step1); break;
            case 2:  Set_Task(ARM_TASK::PICK_UP_KFS_0CM_3_step1); break;
						case 3:  Set_Task(ARM_TASK::PICK_UP_KFS_0CM_4_step1); break;
            default: break; 
        }
    }
		
    // =========================================================
    // Event3 : 执行所有 Step2 (从 step1 转换到对应的 step2)
    // =========================================================
    else if (gantry_event[4].Is_Trig())
    {
        active_event = &gantry_event[4];

        switch (cur_task)
        {
            // ------ 20CM 夹取系列 ------
            case ARM_TASK::PICK_UP_KFS_20CM_1_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_20CM_1_step2);
                break;
            case ARM_TASK::PICK_UP_KFS_20CM_2_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_20CM_2_step2);
                break;
            case ARM_TASK::PICK_UP_KFS_20CM_3_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_20CM_3_step2);
                break;
						case ARM_TASK::PICK_UP_KFS_20CM_4_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_20CM_4_step2);
                break;
            // ------ 40CM 夹取系列 ------
            case ARM_TASK::PICK_UP_KFS_40CM_1_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_40CM_1_step2);
                break;

			case ARM_TASK::PICK_UP_KFS_40CM_2_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_40CM_2_step2);
                break;
            // ------ -20CM系列 ------
            case ARM_TASK::PICK_DOWN_KFS_1_step1:
                Set_Task(ARM_TASK::PICK_DOWN_KFS_1_step2);
                break;
            case ARM_TASK::PICK_DOWN_KFS_2_step1:
                Set_Task(ARM_TASK::PICK_DOWN_KFS_2_step2);
                break;
            case ARM_TASK::PICK_DOWN_KFS_3_step1:
                Set_Task(ARM_TASK::PICK_DOWN_KFS_3_step2); 
                break;
						case ARM_TASK::PICK_DOWN_KFS_4_step1:
                Set_Task(ARM_TASK::PICK_DOWN_KFS_4_step2); 
                break;
						// ------ 0CM --------
						case ARM_TASK::PICK_UP_KFS_0CM_1_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_0CM_1_step2);
                break;
            case ARM_TASK::PICK_UP_KFS_0CM_2_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_0CM_2_step2);
                break;
            case ARM_TASK::PICK_UP_KFS_0CM_3_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_0CM_3_step2);
                break;
						case ARM_TASK::PICK_UP_KFS_0CM_4_step1:
                Set_Task(ARM_TASK::PICK_UP_KFS_0CM_4_step2);
                break;
            default:
                break;
        }

        data::KFSNum::KFS_Add_One();
    }
}
	
		void GetKFS::Set_Step_Act( uint8_t suction)
	{
		step_suction = suction;
	}
	
		void GetKFS::Set_Step_Target(float x, float y, float z, float p, CtrlMode mode_)
	{
		base_target_x = x;
		base_target_y = y;
		base_target_z = z;
		base_target_p = p;
		target_x = x;
		target_y = y;
		target_z = z;
		target_p = p;
		mode = mode_;
	}
	
void GetKFS::Set_Task(ARM_TASK task_)
{
    cur_task = task_;

    seq_idx = 0;

    stable_cnt = 0;

    y_locked = false;

    wait_step_delay = false;

    laser_retry_cnt = 0;

    laser_lost_start = false;

    busy = Configure_Current_Step();

    if (!busy)
        mode = CtrlMode::IDLE;
}

void GetKFS::Restart_Current_Task()
{
    seq_idx = 0;
	retry_x = 0.1f;
	//data::HaveOutKFS::Set_Have_Out_KFS(false);
    stable_cnt = 0;
    wait_step_delay = false;
    y_locked = false;
    laser_lost_start = false;
    laser_retry_cnt++;
    Configure_Current_Step();
}

	void GetKFS::Set_OpenLoop_Target(float x_m, float y_m, float z_m, float p_rad)
	{
		cur_task = ARM_TASK::HOME;
		seq_idx = 0;
		stable_cnt = 0;
		busy = true;
		Set_Ctrl_Mode(SpeedMode::NORMAL);
		
		Set_Step_Target(x_m, y_m, z_m, p_rad, CtrlMode::OPEN_LOOP);
		Set_Step_Act(0);
		Set_Step_Delay(0);
	}

	// ============================================================
	// 更新激光距离
	// ============================================================
	void GetKFS::Update_Laser_Distance()
	{
		float raw_data = (float)laser_.distance / 1000.00f; //单位转化成米
		
		if(raw_data > raw_data_low_limit&& raw_data < raw_data_high_limit)
		{
		laser_distance_m = raw_data;
			laser_valid = true ;
		}
		else
		{
			laser_valid = false ;
		}
		
	}
	


		// ============================================================
	// 锁定当前 Y
	// ============================================================
	void GetKFS::Lock_Current_Y()
{
    locked_y = current_y;
    y_locked = true;
}

void GetKFS::Unlock_Y()
{
    y_locked = false;
}

	// ============================================================
	// 设置步骤延时
	// ============================================================
	void GetKFS::Set_Step_Delay(uint32_t delay_ms)
	{
		step_delay_ms = delay_ms ;
	}

	void GetKFS::Cancel()
	{
		busy = false;
		mode = CtrlMode::IDLE;
		active_event = nullptr;
		stable_cnt = 0;
		wait_step_delay = false;
		user.Give_Control();
	}

	bool GetKFS::Is_Busy() const
	{
		return busy;
	}
	// ============================================================
	// 判断是否到达目标点
	// ============================================================
	bool GetKFS::Reached_Target(float cmd_x, float cmd_y, float cmd_z, float cmd_p)
	{
		bool mech_reached = false;
		if (cmd_x < pitch_detech_upper)
		{
			mech_reached =
				fabsf(current_x - cmd_x) < KFS_X_REACH_TH &&
				fabsf(current_y - cmd_y) < KFS_Y_REACH_TH &&
				fabsf(current_z - cmd_z) < KFS_Z_REACH_TH;
		}
		else
		{
			mech_reached =
				fabsf(current_x - cmd_x) < KFS_X_REACH_TH &&
				fabsf(current_y - cmd_y) < KFS_Y_REACH_TH &&
				fabsf(current_z - cmd_z) < KFS_Z_REACH_TH &&
				fabsf(current_p - cmd_p) < KFS_P_REACH_TH;
		}
		if (!mech_reached) return false;
		  if (mode == CtrlMode::Check)
		{
			return laser_valid;
		}
		if (mode == CtrlMode::CLOSE_LOOP_LASER)
		{
			//return 0;
			return laser_valid && fabsf(laser_target_m - laser_distance_m) < KFS_LASER_ERR_TH;
		}
		if (mode == CtrlMode::Y_LOCK && y_locked)
		{
				cmd_y = locked_y;
		}

		return true;
	}

	void GetKFS::Finish_Current_Task()
	{
		busy = false;
		mode = CtrlMode::IDLE;
		stable_cnt = 0;
		wait_step_delay = false;
		user.Give_Control();
		cnt = 0;
		seq_idx = 0;
		retry_x = 0;
		if (active_event != nullptr) { active_event->Finish(); active_event = nullptr; }
	}




	void GetKFS::Do_Suction_Action(uint8_t action_id)
	{
		if (action_id == 0)
		{
			suction_.Off();
		}
		if (action_id == 1)
		{
			suction_.On();
		}
	}

	// ============================================================
	// 自动抓取主逻辑
	// ============================================================
void GetKFS::Auto_Get_KFS()
{
	  kfs_num = data::KFSNum::Get_KFS_Num();
		current_x = gantry.Get_X();
		current_y = gantry.Get_Y();
		current_z = gantry.Get_Z();
		current_p = gantry.Get_P();
		
    if (!busy)  Trigger_Task_By_Event();
    if (mode == CtrlMode::IDLE || !busy)  return;
	if (!user.Take_Control()) return;
    Update_Laser_Distance();

    float cmd_x = target_x;
    float cmd_y = target_y;
    float cmd_z = target_z;
    float cmd_p = target_p;
	
	if (mode == CtrlMode::Check)
	{
	if (!laser_valid)
    {
        if (!laser_lost_start)
        {
            laser_lost_ts = timer::Timer::Get_TimeStamp();

            laser_lost_start = true;
        }
        else
        {
            if (timer::Timer::Get_DeltaTime(laser_lost_ts) >= LASER_LOST_TIMEOUT_US)
            {
                if (laser_retry_cnt >= LASER_RETRY_MAX)
                {
                    Finish_Current_Task();
					Set_Task(ARM_TASK::HOME);
					data::KFSNum::KFS_Sub_One();
                    return;
                }

                Restart_Current_Task();

                return;
            }
        }
    }
    else
    {
        laser_lost_start = false;
	}
	}
if (mode == CtrlMode::CLOSE_LOOP_LASER)
{
       float err = laser_target_m - laser_distance_m;	
		if(cnt == 0)
		{
			cmd_y = err;
			cnt = 1;
		}

}
    if (mode == CtrlMode::Y_LOCK && y_locked)
    {
        cmd_y = locked_y;
    }
		
    target_x = cmd_x;
    target_y = cmd_y;
    target_z = cmd_z;
    target_p = cmd_p;
		
		
    user.Set_X(cmd_x);
    user.Set_Y(cmd_y);
    user.Set_Z(cmd_z);
    user.Set_P(cmd_p);


    if (Reached_Target(cmd_x, cmd_y, cmd_z, cmd_p))
    {                            
        if (mode == CtrlMode::CLOSE_LOOP_LASER && !y_locked)
        {
            locked_y = cmd_y;
            y_locked = true;
        }

        if (!wait_step_delay)
        {
            step_reached_ts = timer::Timer::Get_TimeStamp();
            wait_step_delay = true;
        }
        else if (timer::Timer::Get_DeltaTime(step_reached_ts) >= step_delay_ms)
        {
            Do_Suction_Action(step_suction);
            Go_Next_Step();
        }
    }
    else
    {
        stable_cnt = 0;
        wait_step_delay = false;
    }
}
}