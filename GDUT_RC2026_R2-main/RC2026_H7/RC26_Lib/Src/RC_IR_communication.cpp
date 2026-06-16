#include "RC_IR_communication.h"

namespace IR
{
	IRCom::IRCom(UART_HandleTypeDef &huart_, uint8_t* buf_) : serial::UartRx(huart_, buf_, IR_COM_RX_BUFFER_SIZE, true, true), huart(huart_)
	{
		last_parity = false;
		last_cmd = 0;

		is_init = false;
	}
	
	void IRCom::Uart_Rx_It_Process(uint8_t* buf_, uint16_t len_)
	{
		if (len_ != IR_COM_FRAME_LEN) return;
		
		if (
			buf_[0] == IR_COM_FRAME_HEAD0  							&&
			buf_[1] == IR_COM_FRAME_HEAD1  							&&
			Check_Sum(&buf_[2]) == (buf_[5] & IR_COM_CHECKSUM_MASK) && 
			buf_[6] == IR_COM_FRAME_TAIL0  							&&
			buf_[7] == IR_COM_FRAME_TAIL1
		)
		{
			bool parity = (bool)(buf_[5] & IR_COM_PARITY_BIT_MASK);
			uint8_t cmd = buf_[2];
			bool is_new = false;
			
			if (!is_init)
			{
				last_parity = parity;
				last_cmd = cmd;
				is_new = true;
				
				is_init = true;
			}
			else
			{
				if (last_parity ^ parity || cmd != last_cmd)
				{
					last_parity = parity;
					last_cmd = cmd;
					is_new = true;
				}
			}
			
			
			
			if (is_new)
			{
				if (cmd != 0 && cmd <= IR_MAX_CMD)
				{
					if (IRCmd::cmd_list[cmd - 1] != nullptr)
					{
						IRCmd::cmd_list[cmd - 1]->new_cmd = true;
					}
				}
			}
		}
	}
	
	uint8_t IRCom::Check_Sum(uint8_t* data)
	{
		uint16_t sum = data[0] + data[1] + data[2];
		uint8_t crc = ~((sum * sum) & 0xFF);
		return crc & IR_COM_CHECKSUM_MASK;
	}
	
	
	
	IRCmd* IRCmd::cmd_list[IR_MAX_CMD] = { nullptr };
	
	
	IRCmd::IRCmd(uint8_t id_)
	{
		if (id_ <= IR_MAX_CMD && id_ != 0)
		{
			cmd_list[id_ - 1] = this;
		}
		
		new_cmd = false;
	}
	
	
	
}