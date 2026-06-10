#pragma once
#include "RC_gantry.h"
#include "RC_suction.h"
#include "RC_timer.h"
#include "RC_event3.h"
#include "RC_mini_laser.h"
#include "RC_navigation.h"

#ifdef __cplusplus
namespace gantry
{
	
	enum PutKFSHeight : uint8_t
	{
		PUTKFS_2L = 0,
		PUTKFS_3L,
	};
	
	enum PutKFSGetState : uint8_t 
	{
		PUTKFS_GET_STRETCH = 0,
		PUTKFS_GET_STRETCH_CHECK,
		
		PUTKFS_GET_TO_KFS,
		PUTKFS_GET_TO_KFS_CHECK,
		
		PUTKFS_GET_WITHDRAW_SUCK,
		PUTKFS_GET_WITHDRAW_SUCK_CHECK,
		
		PUTKFS_GET_KFS_OUT,
		PUTKFS_GET_KFS_OUT_CHECK,
	};
	
	
	
	enum PutKFSPhase : uint8_t
	{
		PUTKFS_RESET = 0,
		PUTKFS_GET_PHASE,
		PUTKFS_PUT_PHASE,
	};
	
	
	enum PutKFSPutState : uint8_t 
	{
		//PUTKFS_PUT_CLOSE_TO = 0,
		//PUTKFS_PUT_CLOSE_TO_CHECK,
		PUTKFS_PUT_CHECK_SUDOKU = 0,
		PUTKFS_PUT_CHECK_SUDOKU_CHECK,
		
		
		PUTKFS_PUT_CHECK_SUDOKU_FAIL,
		
		
		
		PUTKFS_PUT_DROP,
		PUTKFS_PUT_DROP_CHECK,
		
		
		
		PUTKFS_PUT_IN,
		PUTKFS_PUT_IN_CHECK,
		
		PUTKFS_PUT_DOWN,
		PUTKFS_PUT_DOWN_CHECK,
		
		PUTKFS_PUT_RELESE,
		PUTKFS_PUT_RELESE_CHECK,
	};
	

	class PutKFS
    {
    public:
		PutKFS(Gantry& gan_, Suction& suck_, mini_laser::MiniLaser& laser_, path::Navigation& navi_);
		~PutKFS() = default;
	
		void Auto_Put_KFS();
	
		void Put_Fail_Navi()
		{
			if (is_fail)
			{
				if (fail_num == 1)
				{
					navi.Go_To_Put_KFS_2L(1);
				}
				else if (fail_num == 2)
				{
					navi.Go_To_Put_KFS_2L(3);
				}
				
				is_fail = true;
			}
		}
    private:
		
		bool Get_KFS_Phase();
		bool Put_KFS_Phase();
	
		
	
		
		PutKFSPhase phase;
		PutKFSGetState get_state;
		PutKFSPutState put_state;
		
		Suction& suck;
	
		uint32_t last_time;
	
		uint32_t last_check_time;
	
		float get_z;
		float put_z;
		GantryUser user;
		
		bool ready_trig;
		
		path::Event3 put_event[3];
		path::Navigation& navi;
		
		bool is_fail;
		uint8_t fail_num;
	
		mini_laser::MiniLaser& laser;
    };
}
#endif
