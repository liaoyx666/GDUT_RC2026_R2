#include "RC_cdc.h"


#define MAX_RETRY_COUNT 10


namespace cdc
{

	CDC::CDC() : task::ManagedTask("Cdc", 30, 128, task::TASK_DELAY, 1)
	{
		xMutex = xSemaphoreCreateMutex();
		if (xMutex == NULL)
		{
            Error_Handler();
        }
	}
	
	
	void CDC::Task_Process()
	{
		uint8_t sending_buf_dx = (writing_buf_dx == 0) ? 1 : 0;
		
		if (send_buf_used[sending_buf_dx] != 0)
		{
			uint8_t retry_count = 0;
			uint8_t result;
			do
			{
				result = CDC_Transmit_HS(send_buf[sending_buf_dx], send_buf_used[sending_buf_dx]);// 发送
				retry_count++;
			} while (result != USBD_OK && retry_count < MAX_RETRY_COUNT);
			
			send_buf_used[sending_buf_dx] = 0;// 清空已发送的缓冲区
			
			
		}
		
		
		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)// 获取互斥锁
		{
			writing_buf_dx = sending_buf_dx;// 切换缓冲区
			xSemaphoreGive(xMutex); // 释放互斥锁
		}
	}
	
	
	
	
	bool CDC::CDC_AddToBuf(uint8_t *data, uint16_t len, uint16_t max_wait_time)
	{
		if (data == NULL || len == 0 || len > MAX_BUF_SIZE)
		{
            return false;
        }
		
		if (xSemaphoreTake(xMutex, max_wait_time) == pdTRUE)// 获取互斥锁
		{
			if (len + send_buf_used[writing_buf_dx] > MAX_BUF_SIZE)// 缓冲区溢出
			{
				xSemaphoreGive(xMutex); // 释放互斥锁
				return false;
			}
			else
			{
				memcpy(&send_buf[writing_buf_dx][send_buf_used[writing_buf_dx]], data, len);// 存入数据
				send_buf_used[writing_buf_dx] += len;
				xSemaphoreGive(xMutex); // 释放互斥锁
				return true;
			}
		}
		else
		{
			return false;
		}
	}
	
	
	
	
	
}