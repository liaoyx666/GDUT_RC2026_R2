#pragma once
#include "RC_timer.h"
#include "gpio.h"
#include "RC_task.h"
#include "RC_gpio_exti.h"

#include "RC_serial.h"


#ifdef __cplusplus


#define CHANNEL_NUM 8// 8通道




namespace flysky
{
	class FlySky : public task::ManagedTask, gpio::GpioExti
    {
    public:
		FlySky(uint16_t gpio_pin_);
		virtual ~FlySky() {}
		
		volatile uint16_t channel_list[CHANNEL_NUM];// 取值范围1000~2000
		
		volatile uint8_t swa, swb, swc, swd;
		volatile int16_t left_x, left_y, right_x, right_y;
		volatile uint16_t data_buf[CHANNEL_NUM];
		
		bool signal_swa();
		bool signal_swd();
		
    protected:
		void Task_Process() override;
		void EXTI_Prosess() override;
		bool is_init;
	
    private:
		uint32_t last_time;
	
		uint8_t last_swa, last_swd;
		bool is_last_init;
    };

}

#endif
