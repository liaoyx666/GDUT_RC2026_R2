#include "RC_JY901S.h"

namespace imu
{
	JY901S::JY901S(UART_HandleTypeDef &huart_) : serial::UartRx(huart_, rx_buf, JY901S_RX_BUFFER_SIZE, true, true)
	{
		
	}
	
	
	void JY901S::Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_)
	{
		for (uint16_t i = 0; i < len_; i++)
		{
			switch(rx_state)
			{
			case WAIT_HEAD:
				sum_check = 0;
				data_len = 0;
				memset(data, 0, sizeof(uint16_t) * 4);
				if (buf_[i] == JY901S_HEAD)
				{
					rx_state = WAIT_FLAG;
					sum_check += buf_[i];
				}
				break;
			
			case WAIT_FLAG:
				if (buf_[i] >= 0x50)
				{
					flag = buf_[i];
					rx_state = WAIT_DATA;
					sum_check += buf_[i];
				}
				else
				{
					rx_state = WAIT_HEAD;
				}
				break;
			
			case WAIT_DATA:
				sum_check += buf_[i];
				data_len++;
				if (data_len % 2 == 1)
				{
					data[(uint8_t)((data_len - 0.5f) / 2.0f)] |= buf_[i];
				}
				else
				{
					data[(uint8_t)((data_len - 0.5f) / 2.0f)] |= (buf_[i] << 8);
				}
				
				if (data_len >= 8)
				{
					rx_state = WAIT_CHECK;
				}
				break;
			
			case WAIT_CHECK:
				if (sum_check == buf_[i])
				{
					switch(flag)
					{
					case JY901S_FLAG_GYRO:
						gyro[0] = (float)data[0] / 32768.f * 2000.f;
						gyro[1] = (float)data[1] / 32768.f * 2000.f;
						gyro[2] = (float)data[2] / 32768.f * 2000.f;
						break;
					
					case JY901S_FLAG_ACCEL:
						accel[0] = (float)data[0] / 32768.f * 16.f;
						accel[1] = (float)data[1] / 32768.f * 16.f;
						accel[2] = (float)data[2] / 32768.f * 16.f;
						break;
					
					case JY901S_FLAG_EULER:
						euler[0] = (float)data[0] / 32768.f * 180.f;
						euler[1] = (float)data[1] / 32768.f * 180.f;
						euler[2] = (float)data[2] / 32768.f * 180.f;
						break;
					
					case JY901S_FLAG_MAG:
						mag[0] = (float)data[0];
						mag[1] = (float)data[1];
						mag[2] = (float)data[2];
						temp = (float)data[2] / 100.f;
						break;
					
					default:
						break;
					}
				}
				rx_state = WAIT_HEAD;
				break;
			
			default:
				break;
			}
		}
	}
}