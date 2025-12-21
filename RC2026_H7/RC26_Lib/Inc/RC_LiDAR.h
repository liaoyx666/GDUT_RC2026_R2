#pragma once

#include "RC_serial.h"

#ifdef __cplusplus

#define LiDAR_RX_BUFFER_SIZE 50

namespace liDAR
{
    class LiDAR : public serial::UartRx
    {
        public:
            LiDAR(UART_HandleTypeDef &huart_);
            virtual ~LiDAR() {}
            
            uint16_t distance = 0;
            uint16_t strength = 0;
            uint16_t temperature = 0;
            uint16_t checksum = 0;

        protected:
            

        private:
            void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_);
            uint8_t rx_buf[LiDAR_RX_BUFFER_SIZE] = {0};
            uint8_t raw_data[9] = {0};
            uint8_t last_address = 0;
    };
}




#endif
