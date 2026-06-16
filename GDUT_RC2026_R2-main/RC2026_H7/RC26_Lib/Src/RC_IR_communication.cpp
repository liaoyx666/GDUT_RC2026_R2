#include "RC_IR_communication.h"

namespace IR
{
	IRCom::IRCom(UART_HandleTypeDef &huart_, uint8_t* buf_) : serial::UartRx(huart_, buf_, IR_COM_RX_BUFFER_SIZE, true, true), huart(huart_)
	{
		last_parity = false;
		is_last_parity_init = false;
		new_cmd = false;
		cmd = 0;
	}
	
	void IRCom::Uart_Rx_It_Process(uint8_t* buf_, uint16_t len_)
	{
		if (len_ != IR_COM_FRAME_LEN) return;
		
		if (!new_cmd) // 数据被读取后才接收新数据
		{
			if (
				buf_[0] == IR_COM_FRAME_HEAD0  							&&
				buf_[1] == IR_COM_FRAME_HEAD1  							&&
				Check_Sum(&buf_[2]) == (buf_[5] & IR_COM_CHECKSUM_MASK) && 
				buf_[6] == IR_COM_FRAME_TAIL0  							&&
				buf_[7] == IR_COM_FRAME_TAIL1
			)
			{
				bool parity = (bool)(buf_[5] & IR_COM_PARITY_BIT_MASK);
				
				if (!is_last_parity_init)
				{
					last_parity = parity;
					cmd = buf_[2];
					new_cmd = true;
					is_last_parity_init = true;
				}
				else
				{
					if (last_parity ^ parity)
					{
						last_parity = parity;
						cmd = buf_[2];
						new_cmd = true;
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
}