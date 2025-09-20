#include "RC_serial.h"




extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

}

extern "C" void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)
{

}


namespace serial
{
	
	
	
	
	
	
	
	
	




}





void uart_puts(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
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


