#pragma once
#include "stdint.h"
#include <main.h>
#ifdef __cplusplus

using Event3_t = uint8_t;



constexpr Event3_t EVENT3_NULL = 0;
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
	constexpr float EVENT3_TRIG_MAX_THRESHOLD = 0.4f; /*触发事件最大阈值 m*/
	constexpr float EVENT3_TRIG_MIN_THRESHOLD = 0.02f; /*触发事件最小阈值 m*/
	
	constexpr uint8_t EVENT3_MAX_EVENT_NUM = sizeof(Event3_t) * 8;/*事件最大定义数*/
	
	class Event3
    {
    public:
		Event3(uint8_t id_, bool wait_finish_, float trig_threshold_ = 0);/*id_: 1 ~ EVENT3_MAX_EVENT_NUM*/
		~Event3() = default;
		
		bool Is_Trig();
		void Finish();
			
		bool Wait_Finish() const {return wait_finish;}
		
    protected:
		
    private:
		bool Is_Finish();
		void Trig_Once();
		
		static Event3* list[EVENT3_MAX_EVENT_NUM];
		uint8_t id;
	
		bool trig_signal;
		bool finish_signal;
	
		bool wait_finish; /*路径结束时等待完成*/
	
		float trig_threshold; /*触发阈值*/
	
		friend class Path3;
		friend class TrajTrack3;
    };
}
#endif
