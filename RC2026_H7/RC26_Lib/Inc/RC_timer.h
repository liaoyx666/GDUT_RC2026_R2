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
		Timer(tim::Tim &tim_);
		virtual ~Timer() {}
		
		
		volatile static uint32_t cycle;
		
		static uint32_t Get_TimeStamp();
		
		static uint32_t Get_DeltaTime(uint32_t last_time_stamp);
	 
		static tim::Tim *timer_tim;
    protected:
		void Tim_It_Process() override;
	
    private:
    
    };

	
	

}
#endif
