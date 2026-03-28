#pragma once
#include "stdint.h"

#ifdef __cplusplus

using Event3_t = uint8_t;

constexpr Event3_t EVENT3_ID_1 = (1 << 0);
constexpr Event3_t EVENT3_ID_2 = (1 << 1);
constexpr Event3_t EVENT3_ID_3 = (1 << 2);
constexpr Event3_t EVENT3_ID_4 = (1 << 3);
constexpr Event3_t EVENT3_ID_5 = (1 << 4);
constexpr Event3_t EVENT3_ID_6 = (1 << 5);
constexpr Event3_t EVENT3_ID_7 = (1 << 6);
constexpr Event3_t EVENT3_ID_8 = (1 << 7);

namespace path
{
	constexpr uint8_t EVENT3_MAX_EVENT_NUM = sizeof(Event3_t) * 8;/*事件最大定义数*/
	
	class Event3
    {
    public:
		Event3(uint8_t id_);/*id_: 1 ~ EVENT3_MAX_EVENT_NUM*/
		virtual ~Event3() {}
		
    protected:
		
    private:
		static Event3* list[EVENT3_MAX_EVENT_NUM];
		uint8_t id;
    };
}
#endif
