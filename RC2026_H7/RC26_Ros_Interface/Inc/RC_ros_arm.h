#pragma once
#include "RC_cdc.h"

#ifdef __cplusplus
namespace ros
{
	class RosArm : cdc::CDCHandler
    {
    public:
		RosArm(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~RosArm() {}
		
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;
	
    private:
    
    };
}
#endif
