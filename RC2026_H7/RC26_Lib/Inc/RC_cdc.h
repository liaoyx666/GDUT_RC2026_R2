#pragma once
#include "RC_task.h"
#include "usbd_cdc_if.h"

#include "semphr.h"

#ifdef __cplusplus


#define MAX_BUF_SIZE 128// 缓冲区大小

namespace cdc
{
	
	class CDCHandler;
	
	
	class CDC : task::ManagedTask
    {
    public:
		CDC();
		virtual ~CDC() {}
		
		
		uint8_t send_buf[2][MAX_BUF_SIZE] = {{0}};// 双缓冲区
		uint16_t send_buf_used[2] = {0};// 双缓冲区使用情况
		
		uint8_t writing_buf_dx = 0;// 当前可写入数据的缓冲区索引

		
		bool CDC_AddToBuf(uint8_t *data, uint16_t len, uint16_t max_retry);// 向缓冲区写入数据
		
		SemaphoreHandle_t xMutex;// 互斥量（防止两个任务同时向缓冲区数据）
		
    protected:
		void Task_Process() override;
		
    private:
		
    };
	
	
	
	
	
	
	
	
	
	
	class CDCHandler
    {
    public:
    
    protected:
    
    private:
    
    };
	
	
	
	
	
	
	
	
	
	
	
	
}

#endif
