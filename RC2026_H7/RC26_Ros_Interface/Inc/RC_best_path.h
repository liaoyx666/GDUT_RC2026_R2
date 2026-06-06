#pragma once
#include "RC_cdc.h"
#include <string.h>
#include "RC_navigation.h"
#include "RC_map_graph.h"

#ifdef __cplusplus

constexpr uint8_t BESTPATH_MAX_PATH_LEN = 20;

namespace ros
{
	struct PickKfs
	{
		uint8_t kfs_pos;
		path::Direction dir;
	};
	
	enum PickKFSState : uint8_t
	{
		PICK_KFS_WAIT_PATH = 0,
		PICK_KFS_WAIT_KFS_POS,
		PICK_KFS_WAIT_R1_KFS,
	};
	
	
	
	class BestPath : cdc::CDCHandler
    {
    public:
		BestPath(cdc::CDC &cdc_, uint8_t rx_id_, path::Navigation& navi_);
		~BestPath() = default;
		
		constexpr bool Is_Init() {return is_init;}
		
		void Generate_Path();
		
    private:
		uint8_t path[BESTPATH_MAX_PATH_LEN];
		uint8_t path_len;
		uint8_t kfs_pick_num;
	
		PickKfs pick_kfs[5];

	
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
		path::Navigation& navi;
		bool is_init;
	
		PickKFSState state;
		uint8_t current_path;
    };
}
#endif
