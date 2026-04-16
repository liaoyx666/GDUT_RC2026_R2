#pragma once
#include "RC_tim.h"

#ifdef __cplusplus
namespace timer
{
	// 用1hz的定时器初始化
	// 只能实例化一次
	class Timer : tim::TimHandler
    {
    public:
		Timer(tim::Tim *tim_);
		~Timer() = default;
		
		volatile static uint32_t cycle;
		
		static uint32_t Get_TimeStamp();
		
		static uint32_t Get_DeltaTime(uint32_t last_time_stamp);
		
		static TIM_HandleTypeDef *htim;
    protected:
		void Tim_It_Process() override;
	
    private:
    
    };
}
#endif
