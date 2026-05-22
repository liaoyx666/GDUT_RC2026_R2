#pragma once
#include "RC_cdc.h"
#include <string.h>

#ifdef __cplusplus

constexpr uint8_t BESTPATH_MAX_PATH_LEN = 20;

namespace ros
{
	class BestPath : cdc::CDCHandler
    {
    public:
		BestPath(cdc::CDC &cdc_, uint8_t rx_id_);
		~BestPath() = default;
		
		constexpr bool Is_Init() {return is_init;}
		
    private:
		uint8_t path[BESTPATH_MAX_PATH_LEN];
		uint8_t path_len;
	
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
	
		bool is_init;
    };
}
#endif
