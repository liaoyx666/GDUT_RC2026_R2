#pragma once
#include "RC_timer.h"

#include <string.h>

#include "semphr.h"

#ifdef __cplusplus
namespace modbus
{
	constexpr uint16_t MODBUS_RTU_MAX_DATA_LEN = 256;
	constexpr uint16_t MODBUS_RTU_MAX_WAIT_TIME = 1000; /*us*/
	
	class RtuHandler;
	
	/*TIM的count需每加1是1us*/
	class ModBusRtu
    {
    public:
		ModBusRtu(uint32_t baud_);
		~ModBusRtu() = default;
		
		SemaphoreHandle_t xMutex;// 互斥量
		
			
		bool Transmit(uint8_t addr_, uint8_t code_, uint8_t* data_, uint16_t len_, uint16_t max_wait_time, RtuHandler* hd_);
		
    protected:
		
		
		virtual void send(uint8_t* buf_, uint16_t len_) = 0;
		
		void Receive(uint8_t* buf_, uint16_t len_);
		
		
    private:
		
		uint16_t delay_3_5_byte_us;
	
		uint8_t buf[MODBUS_RTU_MAX_DATA_LEN];
		uint16_t len;
		
	
		RtuHandler* wait_hd; /*等待回传的从机, nullptr说明不等待回传*/
		uint8_t code;
		uint32_t last_time;
		
		friend class RtuHandler;
    };
	
	
	class RtuHandler
    {
    public:
		RtuHandler(uint8_t addr_, ModBusRtu& rtu_);
		~RtuHandler() = default;
		
		bool Rtu_Transmit(uint8_t code_, uint8_t* data_, uint16_t len_, uint16_t max_wait_time);
		
    protected:
		virtual void Rtu_Receive(uint8_t* buf_, uint16_t len_) = 0;
		
    private:
		ModBusRtu& rtu;
		uint8_t addr;
	
		friend class ModBusRtu;
    };
}

uint16_t crc16_modbus(uint8_t* pbuf, uint16_t num);

#endif
