#include "RC_serial.h"


extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart_)
{
	serial::UartRx::All_Uart_Rx_It_Process(huart_, 0);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart_, uint16_t size_)
{
	serial::UartRx::All_Uart_Rx_It_Process(huart_, size_);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	// 清除所有可能的错误标志
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);// 清除奇偶校验错误标志
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);// 清除帧错误标志
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);// 清除噪声错误标志
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);// 清除溢出错误标志
    }
	
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_LBDF))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_LBDF);// LIN断点检测标志处理
    }
	
	serial::UartRx::All_Uart_Error_It_Process(huart);// 重启中断
}




namespace serial
{
	uint8_t UartRx::uart_rx_num = 0;
	UartRx* UartRx::uart_rx_list[MAX_UART_NUM] = {nullptr};
	
	void UartRx::All_Uart_Rx_It_Process(UART_HandleTypeDef *huart_, uint16_t size_)
	{
		for (uint8_t i = 0; i < uart_rx_num; i++)
		{
			if (uart_rx_list[i] != nullptr && uart_rx_list[i]->huart == huart_)
			{
				if (uart_rx_list[i]->use_idle == true && size_ != 0)
				{
					// DMA + 空闲中断模式
                    // size_ 是实际接收的数据长度
					uart_rx_list[i]->Uart_Rx_It_Process(uart_rx_list[i]->buf, size_);
					HAL_UARTEx_ReceiveToIdle_DMA(uart_rx_list[i]->huart, uart_rx_list[i]->buf, uart_rx_list[i]->buf_size);
				}
				else
				{
					// 普通DMA或中断模式
                    // 传递缓冲区大小，因为接收的是固定长度数据
					uart_rx_list[i]->Uart_Rx_It_Process(uart_rx_list[i]->buf, uart_rx_list[i]->buf_size);
					if (uart_rx_list[i]->use_DMA == true)
					{
						HAL_UART_Receive_DMA(uart_rx_list[i]->huart, uart_rx_list[i]->buf, uart_rx_list[i]->buf_size);
					}
					else
					{
						HAL_UART_Receive_IT(uart_rx_list[i]->huart, uart_rx_list[i]->buf, uart_rx_list[i]->buf_size);
					}
				}
				return;
			}
		}
	}
	
	
	
	UartRx::UartRx(UART_HandleTypeDef &huart_, uint8_t *buf_, uint16_t buf_size_, bool use_DMA_, bool use_idle_)
		: huart(&huart_), buf(buf_), buf_size(buf_size_), use_DMA(use_DMA_), use_idle(use_idle_)
	{
		is_init = false;
		
		if (huart != NULL && buf != NULL && buf_size != 0)
		{
			if (uart_rx_num < MAX_UART_NUM)
			{
				uart_rx_list[uart_rx_num] = this;
				uart_rx_num++;
				is_init = true;
			}
		}
	}
	
	
	void UartRx::Uart_Rx_Start()
	{
		if (is_init == true)
		{
			if (use_idle == true)
			{
				// 空闲中断模式强制使用DMA
				use_DMA = true;
				HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, buf_size);
			}
			else
			{
				if (use_DMA == true)
				{
					HAL_UART_Receive_DMA(huart, buf, buf_size);
				}
				else
				{
					HAL_UART_Receive_IT(huart, buf, buf_size);
				}
			}
		}
	}
	

	void UartRx::All_Uart_Error_It_Process(UART_HandleTypeDef *huart_)
	{
		for (uint8_t i = 0; i < uart_rx_num; i++)
		{
			if (uart_rx_list[i] != nullptr && uart_rx_list[i]->huart == huart_)
			{
				uart_rx_list[i]->Uart_Rx_Start();
				break;
			}
		}
	}
}





void uart_puts(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}



// 自定义printf函数
int uart_printf(const char *format, ...)
{
    char buffer[50];  // 根据需要调整缓冲区大小
    int ret;
    va_list args;
    
    va_start(args, format);
    ret = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (ret > 0) {
        // 确保不会溢出缓冲区
        if (ret > (int)sizeof(buffer)) {
            ret = sizeof(buffer);
        }
        uart_puts(buffer);
    }
    
    return ret;
}


