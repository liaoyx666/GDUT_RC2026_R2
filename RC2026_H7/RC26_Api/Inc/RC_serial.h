#pragma once
#include "usart.h"
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus

#define MAX_UART_NUM 20

namespace serial
{
	class UartRx
    {
    public:
		UartRx(UART_HandleTypeDef &huart_, uint8_t *buf_, uint16_t buf_size_, bool use_DMA_, bool use_idle_);
		virtual ~UartRx() {}
		
		static void All_Uart_Rx_It_Process(UART_HandleTypeDef *huart_, uint16_t size_);
		static void All_Uart_Error_It_Process(UART_HandleTypeDef *huart_);
		void Uart_Rx_Start();
			
    protected:
		virtual void Uart_Rx_It_Process(uint8_t *buf_, uint16_t len_) = 0;
		
    private:
		UART_HandleTypeDef *huart = NULL;
		uint8_t *buf = NULL;
		uint16_t buf_size = 0;
		bool use_DMA = false;// 是否使用DMA
		bool use_idle = false;// 是否使用空闲中断(使用空闲中断强制使用DMA)
		bool is_init = false;
	
		static uint8_t uart_rx_num;// uart外设数量
		static UartRx *uart_rx_list[MAX_UART_NUM];// uart外设列表
	
    };





	class UartTx
    {
    public:
		UartTx(UART_HandleTypeDef &huart_);
		virtual ~UartTx() {}
			
    protected:
    
    private:
		UART_HandleTypeDef *huart = NULL;
    };
}



#endif
#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>




int uart_printf(const char *format, ...);



#ifdef __cplusplus
}
#endif