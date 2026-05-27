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
		
    private:
		
    };
}
#endif
