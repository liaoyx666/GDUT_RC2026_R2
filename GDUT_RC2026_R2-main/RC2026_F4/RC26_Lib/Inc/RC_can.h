#pragma once
#include "RC_task.h"

#ifdef __cplusplus


#define MAX_CAN_TX_FRAME_NUM 8
#define MAX_CAN_HANDLER_NUM 8
#define MAX_CAN_NUM 3


namespace can
{
	
	typedef enum CanFrameType
	{
		FRAME_STD,  // 标准帧
		FRAME_EXT   // 扩展帧
	} CanFrameType;

	
	typedef struct CanTxFrame
	{
		uint32_t id;
		uint32_t dlc;
		uint8_t data[8];
		
		uint8_t hd_num;// can帧上挂载的设备数量（最多四个）
		uint16_t hd_dx[4];// can帧上挂载的所有设备的设备索引
		
		CanFrameType frame_type;
	} CanTxFrame;


	class CanHandler;// 向前声明


	class Can : public task::ManagedTask
	{
	public:
		Can(CAN_HandleTypeDef &hcan_);
		virtual ~Can() {}
		
		void Can_Filter_Init(
			uint32_t bank, uint32_t fifo, 
			uint32_t idHigh, uint32_t idLow, 
			uint32_t maskIdLow, uint32_t maskIdHigh
		);
		
		void Can_Start();
		
		uint8_t Add_CanHandler(CanHandler *CanHandler);
		static void All_Can_Rx_It_Process(CAN_HandleTypeDef *hcan, uint32_t fifo);
		
		CanTxFrame tx_frame_list[MAX_CAN_TX_FRAME_NUM] = {0};// 发送帧列表
		uint8_t hd_num = 0;// 设备总数
		uint8_t tx_frame_num = 0;// 发送帧总数
	protected:
		
	private:
		void Task_Process() override;

		CAN_HandleTypeDef *hcan;
		uint8_t can_list_dx;
		
		static uint8_t can_num;
		static Can *can_list[MAX_CAN_NUM];
		
		CanHandler *hd_list[MAX_CAN_HANDLER_NUM] = {nullptr};// 设备指针
		
	};






	class CanHandler
	{
	public:
		CanHandler(Can &can_);
		virtual ~CanHandler() {}

		
		Can *can = nullptr;
		uint8_t tx_frame_dx;
		uint8_t hd_list_dx;
		CanFrameType can_frame_type;
		
		uint32_t tx_id;
		uint32_t rx_id;
		
		virtual void Can_Tx_Process() = 0;
		virtual void Can_Rx_It_Process(uint8_t *rx_data) = 0;
		
	protected:
		virtual void CanHandler_Register() = 0;
		
	private:
		
		
	};
}

#endif
