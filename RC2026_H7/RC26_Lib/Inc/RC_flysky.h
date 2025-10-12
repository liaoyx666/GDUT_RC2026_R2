#pragma once
#include "RC_timer.h"
#include "gpio.h"
#include "RC_task.h"


#include "RC_serial.h"


#ifdef __cplusplus


#define CHANNEL_NUM 8// 8通道




namespace flysky
{

	// 只能实例化一次
	class FlySky : public task::ManagedTask
    {
    public:
		FlySky(uint16_t GPIO_Pin_);
		virtual ~FlySky() {}
		
		volatile static uint16_t channel_list[CHANNEL_NUM];// 取值范围1000~2000
		static void EXTI_Prosess(uint16_t GPIO_Pin);
		
		volatile static uint8_t swa, swb, swc, swd;
		volatile static int16_t left_x, left_y, right_x, right_y;
		volatile static uint16_t data_buf[CHANNEL_NUM];
			
		static bool signal_swa();
		static bool signal_swd();
		
		
    protected:
		void Task_Process() override;
	
		static uint16_t GPIO_Pin;
		static bool is_init;
	
    private:
		static uint32_t last_time;
	
		static uint8_t last_swa, last_swd;
		static bool is_last_init;
    };

}

#endif
