#pragma once
#include "RC_task.h"
#include "usbd_cdc_if.h"

#include "semphr.h"

#ifdef __cplusplus


#define MAX_SEND_BUF_SIZE 128// 发送缓冲区大小

#define MAX_RECEIVE_BUF_SIZE 512// 接收缓冲区大小

#define MAX_RECEIVE_ID 10// 最大id

#define MAX_RECEIVE_DATA_LEN 64


namespace cdc
{
	typedef enum CDCType
	{
		USB_CDC_HS,// 高速
		USB_CDC_FS// 全速
	} CDCType;
	
	typedef enum RECEIVE_FLAG
	{
		WAIT_HEAD_1,// 0xaa
		WAIT_HEAD_2,// 0x55
		WAIT_ID,// 1~max
		WAIT_LEN,// 1~64
		WAIT_DATA,
		WAIT_CHECK,//xor
		WAIT_TAIL// 0xff
	} RECEIVE_FLAG;

	
	class CDCHandler;
	
	
	class CDC : task::ManagedTask
    {
    public:
		CDC(CDCType cdc_type_);
		virtual ~CDC() {}
		
		SemaphoreHandle_t xMutex;// 互斥量（防止两个任务同时向发送缓冲区数据）
		
		/*-----------------------------管理USB_CDC设备--------------------------------*/
		uint8_t cdc_list_dx;
		static CDC *cdc_list[2];
		/*-----------------------------管理USB_CDC设备--------------------------------*/
			
			
			
			
		/*-----------------------------发送数据---------------------------------------*/
		uint8_t send_buf[2][MAX_SEND_BUF_SIZE] = {{0}};// 发送双缓冲区
		uint16_t send_buf_used[2] = {0};// 发送双缓冲区使用情况
		uint8_t writing_buf_dx = 0;// 当前可写入数据的发送缓冲区索引
		bool CDC_AddToBuf(uint8_t *data, uint16_t len, uint16_t max_wait_time);// 向发送缓冲区写入数据（！用这个函数发送数据！）
		bool CDC_Send_Pkg(uint8_t id_, uint8_t *data, uint16_t len, uint16_t max_wait_time);
		uint8_t send_pkg_buf[64] = {0};
		/*-----------------------------发送数据---------------------------------------*/


		
		/*-----------------------------接收数据---------------------------------------*/
		void CDC_Task_Receive_Process();// 处理接收数据的任务函数
		task::TaskCreator receive_task;// 创建处理接收数据的任务
		
		uint8_t receive_buf[MAX_RECEIVE_BUF_SIZE] = {0};// 接收环形缓冲区
		volatile uint16_t receive_buf_head = 0;
		volatile uint16_t receive_buf_tail = 0;
		
		static void CDC_All_Task_Receive_Process(void *argument);// 管理所有处理接收数据的任务函数
		static void CDC_It_Receive(uint8_t* buf, uint32_t len, CDCType cdc_type_);// 中断接收函数
		
		RECEIVE_FLAG receive_flag = WAIT_HEAD_1;
		uint8_t receive_id;
		uint8_t receive_len;
		uint8_t receive_check;
		uint8_t receive_data[MAX_RECEIVE_DATA_LEN] = {0};
		uint16_t receive_data_dx = 0;
		/*-----------------------------接收数据---------------------------------------*/
		
		
		
		
		/*-----------------------------接收句柄管理---------------------------------------*/
		void CDC_Register_Handler(CDCHandler *hd);
		CDCHandler *hd_list[MAX_RECEIVE_ID] = {nullptr};
		/*-----------------------------接收句柄管理---------------------------------------*/
		
		
		
		
    protected:
		/*-----------------------------发送数据---------------------------------------*/
		void Task_Process() override;// 发送数据任务函数
		/*-----------------------------发送数据---------------------------------------*/
	
	
	
		
    private:
		CDCType cdc_type;
	
    };
	
	
	
	class CDCHandler
    {
    public:
		CDCHandler(CDC &cdc_, uint8_t rx_id_);
		virtual ~CDCHandler() {}
		
		virtual void CDC_Receive_Process(uint8_t *buf, uint16_t len) = 0;
		uint8_t hd_list_dx;
		uint8_t rx_id;
    protected:
		
    private:
		CDC *cdc = nullptr;
		
	
    };
	
}

#endif
#ifdef __cplusplus
extern "C" {
#endif

void CDC_It_Receive_HS(uint8_t* buf, uint32_t len);
void CDC_It_Receive_FS(uint8_t* buf, uint32_t len);

	
uint8_t xor_check(const uint8_t *data, uint32_t length);


#ifdef __cplusplus
}
#endif

