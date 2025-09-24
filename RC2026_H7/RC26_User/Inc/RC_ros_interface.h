#pragma once
#include "RC_cdc.h"



#ifdef __cplusplus
namespace ros
{
	class Radar : cdc::CDCHandler
    {
    public:
		Radar(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~Radar() {}
		
		float x = 0, y = 0, z = 0, yaw = 0;
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
	
    private:
    
    };
	
	
	
	// -1：未知
	// 0：无效
	// 1：r1
	// 2：r2
	// 3：fake
	// 4：空位
	class Map : cdc::CDCHandler
    {
    public:
		Map(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~Map() {}
		
		int8_t map[12] = {0};
		
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
		
		
    private:
		
    };
	
	
	
	
	
	
	
}
#endif
