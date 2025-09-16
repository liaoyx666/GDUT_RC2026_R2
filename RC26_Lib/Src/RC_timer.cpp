#include "RC_timer.h"

namespace timer
{
	volatile uint32_t Timer::cycle = 0;
	tim::Tim *Timer::timer_tim = nullptr;
	
	Timer::Timer(tim::Tim &tim_) : tim::TimHandler(tim_)
	{
		timer_tim = &tim_;
	}

	
	void Timer::Tim_It_Process()
	{
		cycle++;
		if (cycle > 0x10000) cycle = 0;// 防止溢出
	}
	
	uint32_t Timer::Get_TimeStamp()
	{
		return timer_tim->htim->Instance->CNT + cycle * 0xffff;
	}
	
	uint32_t Timer::Get_DeltaTime(uint32_t last_time_stamp)
	{
		return Get_TimeStamp() - last_time_stamp;
	}
	
}