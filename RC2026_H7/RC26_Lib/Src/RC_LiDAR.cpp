#include "RC_LiDAR.h"

namespace liDAR
{
    LiDAR::LiDAR(UART_HandleTypeDef &huart_) : serial::UartRx(huart_, rx_buf, LiDAR_RX_BUFFER_SIZE, true, true)
    {
    }

    void LiDAR::Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_)
    {
        for(uint8_t i = last_address; i < last_address + len_; i++)
        {   
            if(buf_[i % 50] == 0x59 && buf_[(i + 1) % 50 == 0x59])
            {   
                uint16_t sum = 0;
                for(uint8_t j = 0 ; j < 8; j++)
                {
                    sum += buf_[(i + j) % 50];
                }

                if(buf_[i + 8 % 50] == (uint8_t)sum)
                {
                    distance = buf_[(i + 2) % 50] | buf_[(i + 3) % 50] << 8;
                    strength = buf_[(i + 4) % 50] | buf_[(i + 5) % 50] << 8;
                    temperature = (buf_[(i + 6) % 50] | buf_[(i + 7) % 50] << 8) / 8 - 256;
                }
            }
        }
        last_address = (last_address + len_) % 50;
    }

}
