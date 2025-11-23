#include "RC_flysky.h"


namespace flysky
{
	FlySky::FlySky(uint16_t gpio_pin_) : task::ManagedTask("Flysky", 39, 128, task::TASK_DELAY, 1), gpio::GpioExti(gpio_pin_)
	{
		is_init = true;
	}

	// 上升沿触发
	void FlySky::EXTI_Prosess()
	{
		if (is_init == true)
		{
			static uint8_t flag = 0;
			
			uint32_t delta_time = timer::Timer::Get_DeltaTime(last_time);
			
			last_time = timer::Timer::Get_TimeStamp();
			
			if (delta_time > 2100)
			{
				flag = 0;
			}
			else if (delta_time > 950 && delta_time < 2050)
			{
				if (flag < CHANNEL_NUM)
				{
					if (delta_time < 1000) delta_time = 1000;
					else if (delta_time > 2000) delta_time = 2000;
					
					channel_list[flag] = delta_time;// 1000~2000
					flag++;
				}
			}
			else
			{
				flag = 0;
			}
			
			if (flag == CHANNEL_NUM)
			{
				for (uint8_t i = 0; i < CHANNEL_NUM; i++)
				{
					data_buf[i] = channel_list[i];
				}
			}
		}
	}
	
	void FlySky::Task_Process()
	{
		// 检测断连
		if (timer::Timer::Get_DeltaTime(last_time) < 80000)
		{
			int16_t temp_left_x = data_buf[0] - 1500;
			int16_t temp_left_y = data_buf[2] - 1500;
			
			int16_t temp_right_x = data_buf[3] - 1500;
			int16_t temp_right_y = data_buf[1] - 1500;
			
			// 消抖
			if (temp_left_y > -50 && temp_left_y < 50) left_y = 0;
			else if (temp_left_y >= 50) left_y = temp_left_y - 50;
			else left_y = temp_left_y + 50;
			
			if (temp_left_x > -50 && temp_left_x < 50) left_x = 0;
			else if (temp_left_x >= 50) left_x = temp_left_x - 50;
			else left_x = temp_left_x + 50;
			
			if (temp_right_x > -50 && temp_right_x < 50) right_x = 0;
			else if (temp_right_x >= 50) right_x = temp_right_x - 50;
			else right_x = temp_right_x + 50;
			
			if (temp_right_y > -50 && temp_right_y < 50) right_y = 0;
			else if (temp_right_y >= 50) right_y = temp_right_y - 50;
			else right_y = temp_right_y + 50;
			
			swa = data_buf[4] <= 1500 ? 0 : 1;
		
			swb = data_buf[5] <= 1250 ? 0 : data_buf[5] <= 1750 ? 1 : 2;
			swc = data_buf[6] <= 1250 ? 0 : data_buf[6] <= 1750 ? 1 : 2;
			
			swd = data_buf[7] <= 1500 ? 0 : 1;
			
			if (is_last_init == false)
			{
				last_swa = swa;
				last_swd = swd;
				is_last_init = true;
			}
		}
		else
		{
			left_x = 0;
			left_y = 0;
			
			right_x = 0;
			right_y = 0;
			
			swa = 0;
			swb = 0;
			swc = 0;
			swd = 0;
		}
	}
	
	bool FlySky::signal_swa()
	{
		if (swa != last_swa)
		{
			last_swa = swa;
			return true;
		}
		return false;
	}
	
	bool FlySky::signal_swd()
	{
		if (swd != last_swd)
		{
			last_swd = swd;
			return true;
		}
		return false;
	}
}