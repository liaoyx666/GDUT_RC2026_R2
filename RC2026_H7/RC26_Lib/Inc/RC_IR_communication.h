#pragma once

#ifdef __cplusplus
#include "RC_serial.h"
#include <string.h>

constexpr uint8_t IR_COM_RX_BUFFER_SIZE = 32;

namespace IR
{
	constexpr uint8_t IR_MAX_CMD = 10;
	constexpr uint8_t IR_INVALID_CMD = 0;
	
	
	// 红外命令
	class IRCmd
    {
    public:
		IRCmd(uint8_t id_);
		~IRCmd() = default;

		bool Get_Cmd()
		{
			if (new_cmd)
			{
				new_cmd = false;
				return true;
			}
			else
			{
				return false;
			}
		}
		
		
    private:
		static IRCmd* cmd_list[IR_MAX_CMD];
		volatile bool new_cmd;
	
		friend class IRCom;
    };
	
	
	
	
	
	
	
	
	
	
	constexpr uint8_t IR_COM_FRAME_HEAD0 = 0xFC;
	constexpr uint8_t IR_COM_FRAME_HEAD1 = 0xFB;
	constexpr uint8_t IR_COM_FRAME_TAIL0 = 0xFD;
	constexpr uint8_t IR_COM_FRAME_TAIL1 = 0xFE;
										   
	constexpr uint8_t IR_COM_DATA_LEN    = 3;
	constexpr uint8_t IR_COM_FRAME_LEN   = 8;
					  
	constexpr uint8_t IR_COM_CHECKSUM_MASK = 0x3F;
	constexpr uint8_t IR_COM_PARITY_BIT_MASK = 0x40;
	
	
	
	// 红外通讯
	class IRCom : public serial::UartRx
    {
    public:
		IRCom(UART_HandleTypeDef &huart_, uint8_t* buf_);
		~IRCom() = default;

    private:
		void Uart_Rx_It_Process(uint8_t* buf_, uint16_t len_) override;
		uint8_t Check_Sum(uint8_t* data);
		
		volatile bool last_parity;
		volatile bool is_init;
		volatile uint8_t last_cmd;
		
		UART_HandleTypeDef &huart;
		
    };
}















#endif
