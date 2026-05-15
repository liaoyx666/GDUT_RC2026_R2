#include "RC_get_KFS.h"
namespace gantry
{
	// ========================= 到位判定阈值 =========================
	constexpr float KFS_X_REACH_TH = 0.03f;
	constexpr float KFS_Y_REACH_TH = 0.03f;
	constexpr float KFS_Z_REACH_TH = 0.03f;
	constexpr float KFS_P_REACH_TH = 0.03f;

	// ========================= 激光闭环参数 =========================
	constexpr float KFS_LASER_P = 0.10f;          // 激光闭环比例系数
	constexpr float KFS_LASER_ERR_TH = 0.001f;   // 激光允许误差

	// 激光数据有效范围
	constexpr float raw_data_low_limit  = 0.02f;
	constexpr float raw_data_high_limit = 0.08f;

	// pitch 判定阈值
	constexpr float pitch_detech_upper = 0.4f;

	// 激光修正 Y 轴最大偏移限制
	constexpr float lidar_offset_up_limit   = 0.2f;
	constexpr float lidar_offset_down_limit = -0.2f;

	GetKFS::GetKFS(gantry::Gantry& gantry_ , gantry::Suction& suction_, lidar::LiDAR& lidar_)
		: gantry(gantry_),
		  gantry_event
			{
			  path::Event3(13, true, 0.1f),//20
			  path::Event3(14, true, 0.1f),//40
			  path::Event3(15, true, 0.1f)//-20
		  },
		  suction_(suction_),
		  lidar_(lidar_)
	{
		active_event = nullptr;
		mode = CtrlMode::IDLE;
		cur_task = ARM_TASK::HOME;
		busy = false;
		stable_cnt = 0;
		step_delay_ms = 0;
		step_reached_ts = 0;
		wait_step_delay = false;
		seq_idx = 0;
		target_x = 0.10f;
		target_y = 0.00f;
		target_z = 0.10f;
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

		step_suction = 0;
		
		laser_target_m = 0.053f;
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
			case SpeedMode::SLOW:   scale = 0.2f; break;
			case SpeedMode::FAST:   scale = 1.00f; break;
			case SpeedMode::NORMAL:	scale = 0.5f;
			default:                scale = 0.5f; break;
		}

		gantry.Set_X_Td(2000.f * scale, 8000.f * scale);
		gantry.Set_Y_Td(1000.f * scale, 2000.f * scale);
		gantry.Set_Z_Td(2000.f * scale, 8000.f * scale);
		gantry.Set_P_Td(20.f   * scale, 7.f    * scale);
	}
	


	bool GetKFS::Configure_Current_Step()
	{
		switch (cur_task)
		{
			case ARM_TASK::PICK_UP_KFS_20CM_1:
				switch (seq_idx)
				{
					case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0); 		   Set_Step_Target(0.03f, 0.00f, 0.2f, 0.00f,  CtrlMode::OPEN_LOOP);					Set_Step_Act(1); return true;
					case 1: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0);	  Set_Step_Target(0.62f, 0.00f, 0.52f, 4.71f, CtrlMode::OPEN_LOOP);				 Set_Step_Act(1); return true;
					case 2: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(10000);   		Set_Step_Target(0.62f, 0.00f, 0.43f, 4.71f, CtrlMode::OPEN_LOOP);					Set_Step_Act(1); return true;
					case 3: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0); 		   Set_Step_Target(0.62f, 0.00f, 0.60f, 4.71f, CtrlMode::OPEN_LOOP);					Set_Step_Act(1); return true;
					case 4: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.3f, 0.00f, 0.30f, 1.57f,  CtrlMode::OPEN_LOOP);				Set_Step_Act(1); return true;
					case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);  		 Set_Step_Target(0.3f, 0.00f, 0.35f, 1.57f,  CtrlMode::CLOSE_LOOP_LASER);  Set_Step_Act(1); return true;
					case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.16f, 0.00f, 0.30f, 1.2f,  CtrlMode::Y_LOCK); 					 Set_Step_Act(1); return true;
					case 7: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);  		 Set_Step_Target(0.0f, 0.00f, 0.30f, 0.0f,   CtrlMode::Y_LOCK); 					 Set_Step_Act(1); return true;
					case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);  		 Set_Step_Target(0.0f, 0.00f, 0.25f, 0.0f,   CtrlMode::Y_LOCK); 					 Set_Step_Act(0); return true;
					case 9: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);  		 Set_Step_Target(0.03f, 0.00f, 0.0f, 0.0f,   CtrlMode::OPEN_LOOP); 					 Set_Step_Act(0); return true;
					default: return false;
				}

				case ARM_TASK::PICK_UP_KFS_20CM_2:
				switch (seq_idx)
				{
					case 0: Set_Ctrl_Mode(SpeedMode::FAST);   Set_Step_Delay(0); 		   Set_Step_Target(0.03f, 0.00f, 0.2f,	 0.00f, CtrlMode::OPEN_LOOP);				Set_Step_Act(0); return true;
					case 1: Set_Ctrl_Mode(SpeedMode::FAST); 	Set_Step_Delay(0);		   Set_Step_Target(0.62f, 0.00f, 0.52f,  4.71f, CtrlMode::OPEN_LOOP);				Set_Step_Act(1); return true;
					case 2: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(10000);   Set_Step_Target(0.62f, 0.00f, 0.43f,  4.71f, CtrlMode::OPEN_LOOP); 			Set_Step_Act(1); return true;
					case 3: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0); 		   Set_Step_Target(0.62f, 0.00f, 0.60f,  4.71f, CtrlMode::OPEN_LOOP);				Set_Step_Act(1); return true;
					case 4: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.3f,  0.00f, 0.55f,  1.57f, CtrlMode::OPEN_LOOP); 			Set_Step_Act(1); return true;
					case 5: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);  		 Set_Step_Target(0.3f,  0.00f, 0.55f,  1.57f, CtrlMode::CLOSE_LOOP_LASER);Set_Step_Act(1); return true;
					case 6: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.16f, 0.00f, 0.55f,  1.2f,  CtrlMode::Y_LOCK); 					Set_Step_Act(1); return true;
					case 7: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.1f,  0.00f, 0.55f,  1.0f,  CtrlMode::Y_LOCK); 			 		Set_Step_Act(1); return true;
					case 8: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(10000);  		 Set_Step_Target(0.3f,  0.00f, 0.50f,  0.0f,  CtrlMode::Y_LOCK);  				Set_Step_Act(0); return true;
					case 9: Set_Ctrl_Mode(SpeedMode::NORMAL);   Set_Step_Delay(0);  		 Set_Step_Target(0.03f, 0.00f, 0.0f, 0.0f,   CtrlMode::OPEN_LOOP); 					 Set_Step_Act(0); return true;
					default: return false;
				}
				
			case ARM_TASK::PICK_UP_KFS_40CM_1:
				switch (seq_idx)
				{
					case 0: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(0); 						Set_Step_Target(0.03f, 0.00f, 0.30f,  0.00f,	 CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
					case 1: Set_Ctrl_Mode(SpeedMode::FAST); Set_Step_Delay(1000); 				Set_Step_Target(0.63f, 0.00f, 0.70f,  4.71f,	 CtrlMode::OPEN_LOOP);		  	Set_Step_Act(1); return true;
					case 2: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(100000); 			Set_Step_Target(0.63f, 0.00f, 0.57f,  4.71f,	 CtrlMode::OPEN_LOOP); 				Set_Step_Act(1); return true;
					case 3: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(1000); 				Set_Step_Target(0.63f, 0.00f, 0.75f,  4.71f,	 CtrlMode::OPEN_LOOP);    	  Set_Step_Act(1); return true;
					case 4: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(1000); 				Set_Step_Target(0.30f, 0.00f, 0.50f,  1.57f,	 CtrlMode::CLOSE_LOOP_LASER); Set_Step_Act(1); return true;
					case 5: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 						Set_Step_Target(0.16f, 0.00f, 0.40f,  1.4f,		 CtrlMode::Y_LOCK);        		Set_Step_Act(1); return true;
					case 6: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 						Set_Step_Target(0.1f, 0.00f,  0.30f	, 1.0f, 	 CtrlMode::Y_LOCK);      		  Set_Step_Act(1); return true;
					case 7: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 						Set_Step_Target(0.00f, 0.00f, 0.25f,  0.0f,		 CtrlMode::Y_LOCK);        		Set_Step_Act(1); return true;
					case 8: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);  		  		Set_Step_Target(0.0f, 0.00f, 0.20f, 0.0f,			 CtrlMode::Y_LOCK); 					Set_Step_Act(0); return true;
					case 9: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.03f, 0.00f, 0.0f, 0.0f,   CtrlMode::OPEN_LOOP); 					 Set_Step_Act(0); return true;
					default: return false;
				}

				case ARM_TASK::PICK_DOWN_KFS_1:
				switch (seq_idx)
				{
					case 0: Set_Ctrl_Mode(SpeedMode::FAST);	Set_Step_Delay(0);	  						Set_Step_Target(0.63f, 0.00f,  0.08f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
					case 1: Set_Ctrl_Mode(SpeedMode::SLOW);	Set_Step_Delay(1000000);	  			Set_Step_Target(0.63f, 0.00f,  0.03f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
					case 2: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(100);							Set_Step_Target(0.63f, 0.00f,  0.25f, 	4.71f, CtrlMode::OPEN_LOOP);				Set_Step_Act(1); return true;
					case 3: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.30f, 0.00f,	 0.35f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
					case 4: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.30f, 0.00f,  0.50f, 1.57f, CtrlMode::CLOSE_LOOP_LASER);        Set_Step_Act(1); return true;
					case 5: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.16f, 0.00f, 0.40f,  1.4f,		 CtrlMode::Y_LOCK);        		Set_Step_Act(1); return true;
					case 6: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.1f, 0.00f,  0.30f	, 1.0f, 	 CtrlMode::Y_LOCK);      		  Set_Step_Act(1); return true;
					case 7: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 						Set_Step_Target(0.00f, 0.00f, 0.25f,  0.0f,		 CtrlMode::Y_LOCK);        		Set_Step_Act(1); return true;
					case 8: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0);  		  		Set_Step_Target(0.0f, 0.00f, 0.20f, 0.0f,			 CtrlMode::Y_LOCK); 					Set_Step_Act(0); return true;
					case 9: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.03f, 0.00f, 0.0f, 0.0f,   CtrlMode::OPEN_LOOP); 					 Set_Step_Act(0); return true;
					default: return false;
				}
				
			case ARM_TASK::PICK_DOWN_KFS_2:
				switch (seq_idx)
				{
			
					case 0: Set_Ctrl_Mode(SpeedMode::FAST);	Set_Step_Delay(0);	  						Set_Step_Target(0.64f, 0.00f,  0.08f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(0); return true;
					case 1: Set_Ctrl_Mode(SpeedMode::SLOW);	Set_Step_Delay(1000000);	  			Set_Step_Target(0.64f, 0.00f,  0.03f,  4.71f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
					case 2: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(100);							Set_Step_Target(0.64f, 0.00f,  0.25f, 	4.71f, CtrlMode::OPEN_LOOP);				Set_Step_Act(1); return true;
					case 3: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.30f, 0.00f, 0.35f, 1.57f, CtrlMode::OPEN_LOOP);        Set_Step_Act(1); return true;
					case 4: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.30f, 0.00f, 0.50f, 1.57f, CtrlMode::CLOSE_LOOP_LASER);        Set_Step_Act(1); return true;
					case 5: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.16f, 0.00f, 0.50f, 1.2f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
					case 6: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.1f, 0.00f, 0.50f, 1.0f, CtrlMode::Y_LOCK);        Set_Step_Act(1); return true;
					case 7: Set_Ctrl_Mode(SpeedMode::SLOW); Set_Step_Delay(0); 								Set_Step_Target(0.03f, 0.00f, 0.50f, 0.0f, CtrlMode::Y_LOCK);        Set_Step_Act(0); return true;
					case 8: Set_Ctrl_Mode(SpeedMode::SLOW);   Set_Step_Delay(0);  		 Set_Step_Target(0.03f, 0.00f, 0.0f, 0.0f,   CtrlMode::OPEN_LOOP); 					 Set_Step_Act(0); return true;
					default: return false;
				}

			

			case ARM_TASK::HOME:
			default:
				if (seq_idx == 0)
				{
					Set_Ctrl_Mode(SpeedMode::NORMAL); Set_Step_Delay(0); Set_Step_Target(0.03f, 0.00f, 0.0f, 0.00f, CtrlMode::OPEN_LOOP);Set_Step_Act(0);
					return true;
				}
				return false;
		}
	}
	
		void GetKFS::Trigger_Task_By_Event()
	{
				 if (gantry_event[0].Is_Trig())
				{ 
				 active_event = &gantry_event[0]; 
					if(kfs_num == 0)
					{
						Set_Task(ARM_TASK::PICK_UP_KFS_20CM_1);
						kfs_num++;
					}
					else 
					{
						Set_Task(ARM_TASK::PICK_UP_KFS_20CM_2);
					}
					
					
				}
					else if (gantry_event[1].Is_Trig())
					{
							active_event = &gantry_event[1]; 
							if (kfs_num == 0)
							{
								Set_Task(ARM_TASK::PICK_UP_KFS_40CM_1);
								kfs_num ++;
							}
					}
					
					else if (gantry_event[2].Is_Trig())
					{ 
						active_event = &gantry_event[2]; 
						if(kfs_num == 0)
						{
							Set_Task(ARM_TASK::PICK_DOWN_KFS_1);
							kfs_num ++;
						}
						else
						{
							Set_Task(ARM_TASK::PICK_DOWN_KFS_2);
						}
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
		busy = Configure_Current_Step();
		if (!busy) mode = CtrlMode::IDLE;
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
		float raw_data = (float)lidar_.distance / 1000.00f; //单位转化成米
		
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
		if (mode == CtrlMode::CLOSE_LOOP_LASER)
		{
			return laser_valid &&
				fabsf(laser_target_m - laser_distance_m) < KFS_LASER_ERR_TH;
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
		seq_idx = 0;
		if (active_event != nullptr) { active_event->Finish(); active_event = nullptr; }
	}

	void GetKFS::Go_Next_Step()
	{
		seq_idx++;
		stable_cnt = 0;
		wait_step_delay = false;
		if (!Configure_Current_Step()) Finish_Current_Task();
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
		current_x = gantry.Get_X();
		current_y = gantry.Get_Y();
		current_z = gantry.Get_Z();
		current_p = gantry.Get_P();
	
    if (!busy)
        Trigger_Task_By_Event();

    if (mode == CtrlMode::IDLE || !busy)
        return;

    Update_Laser_Distance();

    float cmd_x = target_x;
    float cmd_y = target_y;
    float cmd_z = target_z;
    float cmd_p = target_p;

    if (mode == CtrlMode::CLOSE_LOOP_LASER && laser_valid)
    {
        float err = laser_target_m - laser_distance_m;

        cmd_y += KFS_LASER_P * err * 0.01f;

        if (cmd_y > lidar_offset_up_limit)
            cmd_y =lidar_offset_up_limit;

        if (cmd_y < lidar_offset_down_limit)
            cmd_y = lidar_offset_down_limit;
    }
    if (mode == CtrlMode::Y_LOCK && y_locked)
    {
        cmd_y = locked_y;
    }

    target_x = cmd_x;
    target_y = cmd_y;
    target_z = cmd_z;
    target_p = cmd_p;

    gantry.Set_X(cmd_x);
    gantry.Set_Y(cmd_y);
    gantry.Set_Z(cmd_z);
    gantry.Set_P(cmd_p);


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