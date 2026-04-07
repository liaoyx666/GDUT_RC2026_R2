#include "RC_modbus_rtu.h"

namespace modbus
{
	ModBusRtu::ModBusRtu(uint32_t baud_)
	{
		delay_3_5_byte_us = 35000000 + baud_ - 1 / baud_; /*向上取整*/
		
		wait_hd = nullptr;
		
		last_time = 0;
		
		xMutex = xSemaphoreCreateMutex();
		if (xMutex == nullptr)
		{
            Error_Handler();
        }
	}
	
	void ModBusRtu::Receive(uint8_t* buf_, uint16_t len_)
	{
		if (len_ > MODBUS_RTU_MAX_DATA_LEN) return;
		
		if (wait_hd == nullptr) return;
		
		if (buf_[0] != wait_hd->addr) return;
		
		if (buf_[1] != code) return;
		
		uint16_t crc16 = crc16_modbus(buf_, len_ - 2);
		
		if (crc16 != ((buf_[len_ - 1] << 8)| buf_[len])) return;
		
		wait_hd->Rtu_Receive(&buf_[2], len_ - 4);
		
		wait_hd = nullptr;
	}
	
	
	
	bool ModBusRtu::Transmit(uint8_t addr_, uint8_t code_, uint8_t* data_, uint16_t len_, uint16_t max_wait_time, RtuHandler* hd_)
	{
		if (len_ > MODBUS_RTU_MAX_DATA_LEN - 4) return false;
		
		if (xSemaphoreTake(xMutex, max_wait_time) == pdTRUE)// 获取互斥锁
		{
			if ((timer::Timer::Get_DeltaTime(last_time) > MODBUS_RTU_MAX_WAIT_TIME || wait_hd == nullptr) && addr_ != 0 && hd_ != nullptr)
			{
				last_time = timer::Timer::Get_TimeStamp();
				
				wait_hd = hd_;
				code = code_;
				
				buf[0] = addr_;
				buf[1] = code_;
				memcpy(&buf[2], data_, len_);
				uint16_t crc16 = crc16_modbus(buf, len_ + 2);
				buf[len_ + 2] = crc16 >> 8;
				buf[len_ + 3] = crc16;
				
				send(buf, len_ + 4);
				
				xSemaphoreGive(xMutex); 
				return true;
			}
			else
			{
				xSemaphoreGive(xMutex); 
				return false;
			}
		}
		
		return false;
	}
	
	
	
	bool RtuHandler::Rtu_Transmit(uint8_t code_, uint8_t* data_, uint16_t len_, uint16_t max_wait_time)
	{
		return rtu.Transmit(addr, code_, data_, len_, max_wait_time, this);
	}
	
	RtuHandler::RtuHandler(uint8_t addr_, ModBusRtu& rtu_) : rtu(rtu_)
	{
		addr = addr_;
	}
}


/*Modbus 协议的 CRC-16 校验值*/
uint16_t crc16_modbus(uint8_t* pbuf, uint16_t num)
{
	uint32_t i, j;
	uint32_t wcrc = 0xffff;
	
	for(i = 0; i < num; i++)
	{
		wcrc ^= (uint32_t)(pbuf[i]);
		
		for (j = 0; j < 8; j++)
		{
			if(wcrc & 0x0001)
			{
				wcrc >>= 1; wcrc ^= 0xa001;
			}
			else
			{
				wcrc >>= 1;
			}
		}
	}
	
	return wcrc;
}