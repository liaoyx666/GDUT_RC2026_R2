#include "RC_nine_square.h"

namespace ros
{
	Nine_Square::Nine_Square(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		//memset(nine_square, -1, 9);// 初始化为未知
	}	
	void Nine_Square::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 9 && is_init == false)
		{
			memcpy(nine_square, buf, 9);
			is_init = true;
		}
		// 应答
		uint8_t ack = 1;
		cdc->CDC_Send_Pkg(4, &ack, 1, 1000);
	}
	int8_t Nine_Square::Get_Square(uint8_t n)
	{
		if (n <= 9 && n >= 1)
		{
			return nine_square[n - 1];
		}
		else
		{
			return 0;
		}
	}
	int* Nine_Square::Get_nine_square(){
		return nine_square;
	}
	
	void Nine_Square::Set_Square(uint8_t n, int8_t kfs)
	{
		if (n <= 9 && n >= 1)
		{
			nine_square[n - 1] = kfs;
		}
	}
	
		
}