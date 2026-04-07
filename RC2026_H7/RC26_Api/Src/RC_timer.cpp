#include "RC_timer.h"

namespace timer
{
	volatile uint32_t Timer::cycle = 0;
	TIM_HandleTypeDef *Timer::htim = nullptr;
	
	Timer::Timer(tim::Tim *tim_) : tim::TimHandler(tim_)
	{
		if (tim_ != nullptr)
		{
			htim = tim_->htim;
		}
		else
		{
			Error_Handler();
		}
	}

	void Timer::Tim_It_Process()
	{
		cycle++;
	}
	
	uint32_t Timer::Get_TimeStamp()
	{
		if(htim == nullptr) return 0;
		
		uint32_t current_cycle;
        uint32_t current_cnt;

		__disable_irq(); // 关闭全局中断
		current_cycle = cycle;
		current_cnt = htim->Instance->CNT;
		__enable_irq(); // 开启中断
		
		return current_cnt + current_cycle * 0x10000;
	}
	
	uint32_t Timer::Get_DeltaTime(uint32_t last_time_stamp)
	{
		return Get_TimeStamp() - last_time_stamp;
	}
}