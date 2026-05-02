#pragma once
#include "tim.h"

#ifdef __cplusplus
namespace timer
{
	// 使用用32bit定时器
	class Timer
    {
    public:
		~Timer() = delete;

		static inline uint32_t Get_TimeStamp()
		{
			return htim2.Instance->CNT;
		}
		
		static inline uint32_t Get_DeltaTime(uint32_t last_time_stamp)
		{
			return htim2.Instance->CNT - last_time_stamp;
		}
		
		static inline void Timer_Start()
		{
			HAL_TIM_Base_Start(&htim2);
		}

    private:
		Timer() = delete;
    };
}
#endif
