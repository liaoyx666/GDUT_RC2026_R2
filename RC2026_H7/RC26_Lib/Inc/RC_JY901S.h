#pragma once
#include "RC_serial.h"


#define JY901S_RX_BUFFER_SIZE 50

#define JY901S_FLAG_GYRO 0x52// 82
#define JY901S_FLAG_ACCEL 0x51// 81
#define JY901S_FLAG_EULER 0x53// 83
#define JY901S_FLAG_MAG 0x54// 84

#define JY901S_HEAD 0x55// 80

#ifdef __cplusplus
namespace imu
{
	typedef enum JY901SRxState
	{
		WAIT_HEAD,
		WAIT_FLAG,
		WAIT_DATA,
		WAIT_CHECK
	} JY901SRxState;
	
	
	class JY901S : public serial::UartRx
    {
    public:
		JY901S(UART_HandleTypeDef &huart_);
		virtual ~JY901S() {}
		
		float Get_Roll() const {return euler[0];}
		float Get_Pitch() const {return euler[1];}
		float Get_Yaw() const {return euler[2];}
		float* Get_Gyro() {return gyro;}
		float* Get_Accel() {return accel;}
		float* Get_Mag() {return mag;}
		
    protected:
		float euler[3] = {0};
		float gyro[3] = {0};
		float accel[3] = {0};
		float mag[3] = {0};
		float temp = 0;
		
    private:
		uint8_t rx_buf[JY901S_RX_BUFFER_SIZE];
		void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_) override;
	
		JY901SRxState rx_state = WAIT_HEAD;
		uint8_t flag = 0;
		uint8_t sum_check = 0;
		uint8_t data_len = 0;
	
		int16_t data[4] = {0};

    };
}
#endif
