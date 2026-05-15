#include "RC_LiDAR.h"

namespace lidar
{
    LiDAR::LiDAR(UART_HandleTypeDef &huart_, uint8_t* buf_) : serial::UartRx(huart_, buf_, LiDAR_RX_BUFFER_SIZE, true, true)
    {
		
    }

    void LiDAR::Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_)
    {
		if (len_ != 9) return;

		// 直接检查帧头
		if (buf_[0] == 0x59 && buf_[1] == 0x59)
		{
			uint8_t sum = 0;
			for (uint8_t j = 0; j < 8; j++) {
				sum += buf_[j];
			}
			if (buf_[8] == sum)
			{
				strength = buf_[4] | (buf_[5] << 8);
				if (strength > 100) {
					distance = buf_[2] | (buf_[3] << 8);
				}
				temperature = (buf_[6] | (buf_[7] << 8)) / 8 - 256;
			}
		}
		
//		if (len_ != 9) return;
//		
//        for(uint16_t i = 0; i < len_ - 8; i++)
//        {
//            if(buf_[i] == 0x59 && buf_[i + 1] == 0x59)
//            {   
//                uint8_t sum = 0;
//				
//                for(uint8_t j = 0 ; j < 8; j++)
//                {
//                    sum += buf_[i + j];
//                }

//                if(buf_[i + 8] == sum)
//                {	
//					strength = buf_[i + 4] | buf_[i + 5] << 8;
//					
//					if(strength > 100)
//					{
//						distance = buf_[i + 2] | buf_[i + 3] << 8;
//					}

//                    temperature = (buf_[i + 6] | buf_[i + 7] << 8) / 8 - 256;
//                    
//                    i += 8;
//                }
//            }
//        }
    }
}