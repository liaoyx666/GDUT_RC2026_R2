#pragma once
#include "tim.h"
#include "cmsis_os.h"

#ifdef __cplusplus


#define MAX_TIM_HANDLE_NUM 16
#define MAX_TIM_NUM 13


namespace tim
{
	
	class TimHandler;
	
	class Tim
	{
	public:
		Tim(TIM_HandleTypeDef &htim_);
		virtual ~Tim() {}
			
		void Tim_It_Start();
	
		void Add_TimHandle(TimHandler *hd);
		
		static void All_Tim_It_Process(TIM_HandleTypeDef *htim);
		
		TIM_HandleTypeDef *htim;
		
	protected:
		
	private:
		static uint8_t tim_num;// 定时器数量
		static Tim *tim_list[MAX_TIM_NUM];// 所有定时器
	
		
	
		uint16_t hd_num = 0;// 处理函数数量
		TimHandler *hd_list[MAX_TIM_HANDLE_NUM] = {nullptr};// 处理对象
	};




	class TimHandler
	{
	public:
		TimHandler(Tim &tim_);
		virtual ~TimHandler() {}
		
		virtual void Tim_It_Process() = 0;
		
	protected:
		Tim *tim;
	
	private:

	};


}

#endif
#ifdef __cplusplus
extern "C" {
#endif


void All_Tim_It_Process(TIM_HandleTypeDef *htim);// 放在main.c中的定时中断函数


#ifdef __cplusplus
}
#endif
