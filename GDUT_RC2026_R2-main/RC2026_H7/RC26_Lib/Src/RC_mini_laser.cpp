#include "RC_mini_laser.h"

namespace mini_laser
{
    MiniLaser::MiniLaser(UART_HandleTypeDef &huart_, uint8_t* buf_)
     : serial::UartRx(huart_, buf_, MINI_LASER_RX_BUFFER_SIZE, true, true)
    {   
		
    }

    void MiniLaser::Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_)
    {
		if (len_ != 4) return;

		if (buf_[0] == 0x5C && buf_[3] == (uint8_t)(~(buf_[1] + buf_[2])))
		{
			distance = (uint16_t)((buf_[2] << 8) | buf_[1]);
		}
    }
}
