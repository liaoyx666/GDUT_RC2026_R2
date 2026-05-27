#pragma once
#ifdef __cplusplus
#include "RC_cdc.h"

namespace ros
{
	class KfsPose : public cdc::CDCHandler
    {
    public:
		KfsPose(cdc::CDC &cdc_, uint8_t rx_id_);
		~KfsPose() = default;
		
		bool Enable(); /*成功返回true*/
		bool Disable(); /*成功返回true*/
		constexpr bool Is_Enable() const { return is_enable; }
		constexpr float X() const { return x; }
	
    private:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
	
		uint8_t id;
		float x;
		bool is_enable;
    };
}
#endif
