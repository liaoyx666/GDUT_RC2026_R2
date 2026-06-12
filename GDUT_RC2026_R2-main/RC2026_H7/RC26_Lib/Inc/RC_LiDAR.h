#pragma once

#include "RC_serial.h"
#include "RC_filter.h"

#ifdef __cplusplus

#define LiDAR_RX_BUFFER_SIZE 32

namespace lidar
{
    class LiDAR : public serial::UartRx
    {
	public:
		LiDAR(UART_HandleTypeDef &huart_, uint8_t* buf_);
		~LiDAR() = default;
		uint16_t distance = 0;
		float dis_filter = 0;
	protected:
	
		uint16_t strength = 0;
		uint16_t temperature = 0;
		uint16_t checksum = 0;

	private:
		void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_) override;
		uint8_t rx_buf[LiDAR_RX_BUFFER_SIZE] = {0};
		filter::SecondOrderLPF filter;
    };
}




#endif
