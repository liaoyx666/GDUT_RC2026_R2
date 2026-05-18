#pragma once
#include "RC_gantry.h"
#include "RC_suction.h"
#include "RC_timer.h"
#include "RC_event3.h"

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
		PUTKFS_GET_TO_KFS,
		PUTKFS_GET_WITHDRAW_SUCK,
		PUTKFS_GET_KFS_OUT,
	};
	
	
	
	enum PutKFSPhase : uint8_t
	{
		PUTKFS_RESET = 0,
		PUTKFS_GET_PHASE,
		PUTKFS_PUT_PHASE,
	};
	
	
	enum PutKFSPutState : uint8_t 
	{
		PUTKFS_PUT_CLOSE_TO = 0,
		PUTKFS_PUT_IN,
		PUTKFS_PUT_DOWN,
		PUTKFS_PUT_RELESE,
		PUTKFS_PUT_OUT,
	};
	

	class PutKFS
    {
    public:
		PutKFS(Gantry& gan_, Suction& suck_);
		~PutKFS() = default;
		
		void Put_KFS(PutKFSHeight h_, bool trig_);
		void Auto_Put_KFS();
    private:
		
		bool Get_KFS_Phase();
		bool Put_KFS_Phase();
		
		PutKFSPhase phase;
		PutKFSGetState get_state;
		PutKFSPutState put_state;
		
		Suction& suck;
	
		uint32_t last_time;
	
		float get_z;
		float put_z;
		GantryUser user;
	
	
		PutKFSHeight h;
		bool put_trig;
	
	
		path::Event3 put_event[2];
    };
}
#endif
