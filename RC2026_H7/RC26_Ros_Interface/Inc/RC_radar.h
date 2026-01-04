#pragma once
#include "RC_cdc.h"
#include "RC_timer.h"


#ifdef __cplusplus
namespace ros
{
	class Radar : cdc::CDCHandler
    {
    public:
		Radar(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~Radar() {}
		
		bool Is_Valid();
		bool Get_Radar_Data(float* x, float* y, float* yaw);
		float Get_X() const {return x;};
		float Get_Y() const {return y;};
		float Get_Yaw() const {return yaw;};
		
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
		
    private:
		uint32_t latest_receive_time = 0;// 最新接收时间戳
		float x = 0, y = 0, z = 0, yaw = 0;
		
    };

	
}
#endif
