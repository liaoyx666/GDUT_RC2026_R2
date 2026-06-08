#pragma once

#include "RC_serial.h"
#include "RC_filter.h"
#ifdef __cplusplus

#define MINI_LASER_RX_BUFFER_SIZE 32

namespace mini_laser
{
    class MiniLaser : public serial::UartRx
    {
    public:
        MiniLaser(UART_HandleTypeDef &huart_, uint8_t* buf_);
        virtual ~MiniLaser() {}
		float distance = 0.0f;
			
		
			
    private:
        void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_) override;
		
    };
}
#endif