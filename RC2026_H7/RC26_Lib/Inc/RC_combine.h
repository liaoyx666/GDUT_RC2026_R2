#pragma once
#ifdef __cplusplus

#include "RC_event3.h"

namespace combine
{
	
	class Combine
    {
    public:
		Combine();
		~Combine() = default;
	
		void Auto_Combine();
    private:
		path::Event3 combine_event;
	
		bool is_combine;
    };

}
#endif
