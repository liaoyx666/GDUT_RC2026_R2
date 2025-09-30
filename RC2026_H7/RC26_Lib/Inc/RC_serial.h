#pragma once
#include "usart.h"
#include <stdio.h>
#include <string.h>


#ifdef __cplusplus


namespace serial
{
	class Serial
    {
    public:
		Serial();
		virtual ~Serial() {}
		
		
    protected:
    
    private:
    
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