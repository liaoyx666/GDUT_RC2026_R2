#pragma once
#include "stdint.h"
#include "arm_math.h"

#include <main.h>
#ifdef __cplusplus

using Event3_t = uint32_t;


constexpr Event3_t EVENT3_NULL  = 0;
constexpr Event3_t EVENT3_ID_1  = (1 << 0);
constexpr Event3_t EVENT3_ID_2  = (1 << 1);
constexpr Event3_t EVENT3_ID_3  = (1 << 2);
constexpr Event3_t EVENT3_ID_4  = (1 << 3);
constexpr Event3_t EVENT3_ID_5  = (1 << 4);
constexpr Event3_t EVENT3_ID_6  = (1 << 5);
constexpr Event3_t EVENT3_ID_7  = (1 << 6);
constexpr Event3_t EVENT3_ID_8  = (1 << 7);
constexpr Event3_t EVENT3_ID_9  = (1 << 8);
constexpr Event3_t EVENT3_ID_10 = (1 << 9);
constexpr Event3_t EVENT3_ID_11 = (1 << 10);
constexpr Event3_t EVENT3_ID_12 = (1 << 11);
constexpr Event3_t EVENT3_ID_13 = (1 << 12);
constexpr Event3_t EVENT3_ID_14 = (1 << 13);
constexpr Event3_t EVENT3_ID_15 = (1 << 14);
constexpr Event3_t EVENT3_ID_16 = (1 << 15);
constexpr Event3_t EVENT3_ID_17 = (1 << 16);
constexpr Event3_t EVENT3_ID_18 = (1 << 17);
constexpr Event3_t EVENT3_ID_19 = (1 << 18);






constexpr Event3_t EVENT_HEAD_CHECK_F = EVENT3_ID_1;
constexpr Event3_t EVENT_HEAD_CHECK_B = EVENT3_ID_2;
constexpr Event3_t EVENT_HEAD_CHECK_L = EVENT3_ID_3;
constexpr Event3_t EVENT_HEAD_CHECK_R = EVENT3_ID_4;

constexpr Event3_t EVENT_UP_2_READY_L   = EVENT3_ID_5;
constexpr Event3_t EVENT_UP_4_READY_L   = EVENT3_ID_6;
constexpr Event3_t EVENT_UP_2_READY_R   = EVENT3_ID_7;
constexpr Event3_t EVENT_UP_4_READY_R   = EVENT3_ID_8;
constexpr Event3_t EVENT_DOWN_2_READY_L = EVENT3_ID_9;
constexpr Event3_t EVENT_DOWN_4_READY_L = EVENT3_ID_10;
constexpr Event3_t EVENT_DOWN_2_READY_R = EVENT3_ID_11;
constexpr Event3_t EVENT_DOWN_4_READY_R = EVENT3_ID_12;

constexpr Event3_t GET_HIGH_20_KFS_READY_EVENT 	= EVENT3_ID_13;
constexpr Event3_t GET_HIGH_40_KFS_READY_EVENT 	= EVENT3_ID_14;
constexpr Event3_t GET_LOW_20_KFS_READY_EVENT 	= EVENT3_ID_15;
constexpr Event3_t GET_PICK_KFS_EVENT 			= EVENT3_ID_16;

constexpr Event3_t EVENT_PUT_KFS_2L = EVENT3_ID_17;
constexpr Event3_t EVENT_PUT_KFS_3L = EVENT3_ID_18;
	
namespace path
{
	constexpr float EVENT3_TRIG_MAX_THRESHOLD = 0.4f; /*触发事件最大阈值 m*/
	constexpr float EVENT3_TRIG_MIN_THRESHOLD = 0.02f; /*触发事件最小阈值 m*/
	
	constexpr float EVENT3_YAW_ALIGN_MIN_THRESHOLD = 2.f / 180.f * PI; /*2度*/
	
	constexpr uint8_t EVENT3_MAX_EVENT_NUM = sizeof(Event3_t) * 8;/*事件最大定义数*/
	
	class Event3
    {
    public:
		Event3(uint8_t id_, float trig_threshold_, bool wait_finish_, bool yaw_aligh_, float yaw_align_threshold_ = 0);/*id_: 1 ~ EVENT3_MAX_EVENT_NUM*/
		~Event3() = default;
		
		bool Is_Trig();
		void Finish();
		
		constexpr bool Wait_Finish() const { return wait_finish; }
		constexpr bool Yaw_Align() const { return yaw_align; }
		constexpr bool Yaw_Align_Threshold() const { return yaw_align_threshold; }
    private:
		bool Is_Finish();
		void Trig_Once();
		
		static Event3* list[EVENT3_MAX_EVENT_NUM];
		uint8_t id;
	
		bool trig_signal;
		bool finish_signal;
	
		bool wait_finish; /*路径结束时等待完成*/
		bool yaw_align;
	
		float yaw_align_threshold;
	
		float trig_threshold; /*触发阈值*/
	
		friend class Path3;
		friend class TrajTrack3;
    };
}
#endif
