#include "RC_cdc.h"


#define MAX_CDC_RETRY_COUNT 10


extern "C" void CDC_It_Receive_HS(uint8_t* buf, uint32_t len)// C语言接口函数
{
	cdc::CDC::CDC_It_Receive(buf, len, cdc::USB_CDC_HS);
}


extern "C" void CDC_It_Receive_FS(uint8_t* buf, uint32_t len)// C语言接口函数
{
	cdc::CDC::CDC_It_Receive(buf, len, cdc::USB_CDC_FS);
}



namespace cdc
{
	
	CDC *CDC::cdc_list[2] = {nullptr};
	
	

	CDC::CDC(CDCType cdc_type_) : 
		cdc_type(cdc_type_), 
		task::ManagedTask("Cdc", 30, 128, task::TASK_PERIOD, 1), 
		receive_task("cdc_receive_task", 30, 128, CDC_All_Task_Receive_Process, &cdc_type)
	{
		if (cdc_type == USB_CDC_HS) cdc_list_dx = 0;
		else cdc_list_dx = 1;

		cdc_list[cdc_list_dx] = this;
		
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
				if (cdc_type == USB_CDC_HS)
				{
					result = CDC_Transmit_HS(send_buf[sending_buf_dx], send_buf_used[sending_buf_dx]);// 发送
				}
				retry_count++;
			} while (result != USBD_OK && retry_count < MAX_CDC_RETRY_COUNT);
			
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
		if (data == NULL || len == 0 || len > MAX_SEND_BUF_SIZE)
		{
            return false;
        }
		
		if (xSemaphoreTake(xMutex, max_wait_time) == pdTRUE)// 获取互斥锁
		{
			if (len + send_buf_used[writing_buf_dx] > MAX_SEND_BUF_SIZE)// 缓冲区溢出
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
	
	bool CDC::CDC_Send_Pkg(uint8_t id_, uint8_t *data, uint16_t len, uint16_t max_wait_time)
	{
		if (len <= 59)
		{
			send_pkg_buf[0] = 0xaa;
			send_pkg_buf[1] = 0x55;
			send_pkg_buf[2] = id_;
			send_pkg_buf[3] = len;
			memcpy(&send_pkg_buf[4], data, len);
			send_pkg_buf[len + 4] = xor_check(data, len);
			send_pkg_buf[len + 5] = 0xee;
			return CDC_AddToBuf(send_pkg_buf, len + 6, max_wait_time);
		}
		return false;
	}
	
	/*-------------------------------------接收数据-------------------------------------------------*/
	
	void CDC::CDC_It_Receive(uint8_t* buf, uint32_t len, CDCType cdc_type_)
	{
		uint8_t dx = (cdc_type_ == USB_CDC_HS) ? 0 : 1;
    
		// 检查指针是否有效
		if (cdc_list[dx] == nullptr) return;
		
		if (len <= 64)
		{
			uint16_t free_space;// 可用空间
			uint16_t head = cdc_list[dx]->receive_buf_head;
			uint16_t tail = cdc_list[dx]->receive_buf_tail;
			
			if (head <= tail)
			{
				free_space = MAX_RECEIVE_BUF_SIZE - (tail - head) - 1; // 保留一个字节作为边界
			}
			else
			{
				free_space = head - tail - 1; // 保留一个字节作为边界
			}
			
			if (len <= free_space)
			{
				// 计算可以连续写入的空间
				uint16_t space_until_end = MAX_RECEIVE_BUF_SIZE - tail;
				
				if (len <= space_until_end)
				{
					// 可以连续写入
					memcpy(&cdc_list[dx]->receive_buf[tail], buf, len);
					cdc_list[dx]->receive_buf_tail = (tail + len) % MAX_RECEIVE_BUF_SIZE;
				}
				else
				{
					// 需要环绕写入
					uint16_t first_part = space_until_end;// 第一部分长度
					uint16_t second_part = len - first_part;// 第二部分长度
					
					memcpy(&cdc_list[dx]->receive_buf[tail], buf, first_part);
					memcpy(cdc_list[dx]->receive_buf, buf + first_part, second_part);
					
					cdc_list[dx]->receive_buf_tail = second_part % MAX_RECEIVE_BUF_SIZE;
				}
				
			}

		}
		
		
	}
	
	
	
	
	
	void CDC::CDC_All_Task_Receive_Process(void *argument)
	{
		CDCType cdc_type_  = *(CDCType*)argument;
		uint8_t dx = (cdc_type_ == USB_CDC_HS) ? 0 : 1;
		
		if (cdc_list[dx] == nullptr) Error_Handler();
		
		for (;;)
		{
			cdc_list[dx]->CDC_Task_Receive_Process();
			osDelay(1);
		}
	}
	
	
	
	
	void CDC::CDC_Task_Receive_Process()
	{
		while (receive_buf_head != receive_buf_tail)
		{
			uint8_t data = receive_buf[receive_buf_head];
			
			receive_buf_head = (receive_buf_head + 1) % MAX_RECEIVE_BUF_SIZE;
			
			/*-----------------------------------------处理数据--------------------*/
			
			switch (receive_flag)
			{
			case WAIT_HEAD_1:// 0xaa
				if (data == 0xaa) receive_flag = WAIT_HEAD_2;
				break;
				
			case WAIT_HEAD_2:// 0x55
				if (data == 0x55) receive_flag = WAIT_ID;
				else receive_flag = WAIT_HEAD_1;
				break;
				
			case WAIT_ID:// 1~MAX
				if (data > MAX_RECEIVE_ID || data == 0) receive_flag = WAIT_HEAD_1;
				else 
				{
					receive_id = data;
					receive_flag = WAIT_LEN;
				}
				break;
				
			case WAIT_LEN:
				if (data > MAX_RECEIVE_DATA_LEN) receive_flag = WAIT_HEAD_1;
				else
				{
					receive_len = data;
					receive_flag = WAIT_DATA;
					receive_data_dx = 0;
				}
				break;
				
			case WAIT_DATA:
				receive_data[receive_data_dx] = data;
				receive_data_dx++;
				if (receive_data_dx >= receive_len) 
				{
					receive_flag = WAIT_CHECK;
					receive_data_dx = 0;
				}
				break;
			
			case WAIT_CHECK:
				if (data == xor_check(receive_data, receive_len)) receive_flag = WAIT_TAIL;
				else receive_flag = WAIT_HEAD_1;
				break;
				
			case WAIT_TAIL:// 0xee
				if (data == 0xee)
				{
					/*-----------------------分发数据-------------------------*/
					if (hd_list[receive_id - 1] != nullptr)
					{
						hd_list[receive_id - 1]->CDC_Receive_Process(receive_data, receive_len);
					}
					/*-----------------------分发数据-------------------------*/
					receive_flag = WAIT_HEAD_1;
				}
				else receive_flag = WAIT_HEAD_1;
				break;
			
			default:
				receive_flag = WAIT_HEAD_1;
				break;
			}
				

			/*-------------------------------------处理数据--------------------*/
		}
	}
	
	
	void CDC::CDC_Register_Handler(CDCHandler *hd)
	{
		if (hd != nullptr)
		{
			if (hd->hd_list_dx <= MAX_RECEIVE_ID && hd->hd_list_dx >= 1)
			{
				hd_list[hd->hd_list_dx] = hd;
			}
		}
	}
	
	
	/*---------------------------------------------CDCHandler----------------------------------------------------------*/
	
	
	
	CDCHandler::CDCHandler(CDC &cdc_, uint8_t rx_id_)
	{
		if (rx_id_ <= MAX_RECEIVE_ID && rx_id_ >= 1)
		{
			rx_id = rx_id_;
			cdc = &cdc_;
			hd_list_dx = rx_id_ - 1;
			cdc->CDC_Register_Handler(this);
		}
	}
	
	
}

// XOR校验
uint8_t xor_check(const uint8_t *data, uint32_t length)
{
	uint8_t xor_val = 0;
	for (uint16_t i = 0; i < length; i++)
	{
		xor_val ^= data[i]; // 仅异或，运算成本极低
	}
	return xor_val;
}
