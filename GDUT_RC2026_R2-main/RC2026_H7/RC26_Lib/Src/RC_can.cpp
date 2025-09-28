/*适用STM32H723 FDCAN
 *
 *
 *
 */
#include "RC_can.h"

#define MAX_CAN_RETRY_COUNT 20 // 最大重试次数



extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
	if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
	{
		can::Can::All_Can_Rx_It_Process(hcan, FDCAN_RX_FIFO0);
	}
}


extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo1ITs)
{
	if (RxFifo1ITs == FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
	{
		can::Can::All_Can_Rx_It_Process(hcan, FDCAN_RX_FIFO1);
	}
}


namespace can
{

	Can *Can::can_list[MAX_CAN_NUM] = {nullptr};// 初始化can列表指针
	uint8_t Can::can_num = 0;// 初始化can数量为0


	Can::Can(FDCAN_HandleTypeDef &hcan_) : hcan(&hcan_), task::ManagedTask("Can", 40, 128, task::TASK_PERIOD, 1)
	{
		taskENTER_CRITICAL();
		
		can_num++;
		if (can_num > MAX_CAN_NUM) Error_Handler();
		
		can_list_dx = can_num - 1;
		can_list[can_list_dx] = this;
		
		taskEXIT_CRITICAL();
	}

	
	void Can::Can_Filter_Init(
		uint32_t idType,
		uint32_t bank, 
		uint32_t fifo, 
		uint32_t id, 
		uint32_t maskId
	)
	{
		FDCAN_FilterTypeDef  filter_init = {0};
		
		
		filter_init.FilterConfig = fifo;
		filter_init.FilterID1 = id;
		filter_init.FilterID2 = maskId;
		filter_init.FilterIndex = bank;
		filter_init.FilterType = FDCAN_FILTER_MASK;// 掩码模式
		filter_init.IdType = idType;// 标准帧或拓展帧
		filter_init.IsCalibrationMsg = 0;
		filter_init.RxBufferIndex = 0;

														  
		if (HAL_FDCAN_ConfigFilter(hcan, &filter_init) != HAL_OK) Error_Handler();// 配置失败处理
	}
	
	
	void Can::Can_Start()
	{
		// 使能CAN接收FIFO0消息挂起中断
		if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
		
		// 使能CAN接收FIFO1消息挂起中断
		if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
		
		// 启动CAN外设
		if (HAL_FDCAN_Start(hcan) != HAL_OK) Error_Handler();
		
	}
	
	

	void Can::All_Can_Rx_It_Process(FDCAN_HandleTypeDef *hcan, uint32_t fifo)
	{
		// 查找对应can对象
		for (uint8_t i = 0; i < can_num; i++)
		{
			if (can_list[i]->hcan == hcan)
			{
				FDCAN_RxHeaderTypeDef can_rx_hdr;
				uint8_t rx_data[8];
				
				if (HAL_FDCAN_GetRxMessage(hcan, fifo, &can_rx_hdr, rx_data) != HAL_OK) break;
				
				// 检查ID匹配
				bool id_matched = false;
				
				// 查找对应rx帧id的设备
				for (uint16_t j = 0; j < can_list[i]->hd_num; j++)
				{
					if (can_list[i]->hd_list[j] == nullptr) continue;
					
					if (can_rx_hdr.IdType == FDCAN_EXTENDED_ID)
					{
						if (can_list[i]->hd_list[j]->can_frame_type == FRAME_EXT)
						{
							if ((can_rx_hdr.Identifier & can_list[i]->hd_list[j]->rx_mask) == can_list[i]->hd_list[j]->rx_id)
							{
								id_matched = true;
							}
							else continue;
						}
						else continue;
					}
					else
					{
						if (can_list[i]->hd_list[j]->can_frame_type == FRAME_STD)
						{
							if ((can_rx_hdr.Identifier & can_list[i]->hd_list[j]->rx_mask) == can_list[i]->hd_list[j]->rx_id)
							{
								id_matched = true;
							} else continue;
						}
						else continue;
					}
					
					if (id_matched == true)
					{
						can_list[i]->hd_list[j]->Can_Rx_It_Process(can_rx_hdr.Identifier, rx_data);// 调用设备接收处理函数
						break;// 每个ID只对应一个设备
					}
				}
				break;
			}
		}
	}


	void Can::Task_Process()
	{
		FDCAN_TxHeaderTypeDef can_tx_hdr;// 帧头
		HAL_StatusTypeDef status;
		
		// 发送can上所有数据帧
		for (uint8_t i = 0; i < tx_frame_num; i++)
		{
			// 所有挂载在can帧上的设备处理帧数据
			for (uint8_t j = 0; j < tx_frame_list[i].hd_num; j++)
			{
				hd_list[tx_frame_list[i].hd_dx[j]]->Can_Tx_Process();
			}
			/*-------------------------------------------------------------------*/
			// 设置帧头
			if (tx_frame_list[i].frame_type == FRAME_STD) 
			{
				can_tx_hdr.Identifier = tx_frame_list[i].id;
				can_tx_hdr.IdType = FDCAN_STANDARD_ID;
			}
			else
			{
				can_tx_hdr.Identifier = tx_frame_list[i].id;
				can_tx_hdr.IdType = FDCAN_EXTENDED_ID;
			}
			
			if (tx_frame_list[i].dlc > 8) break;// 标准can都小于等于8
			can_tx_hdr.DataLength = tx_frame_list[i].dlc;// 数据长度
			
			can_tx_hdr.TxFrameType = FDCAN_DATA_FRAME;// 数据帧
			can_tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;// 关闭速率切换
			can_tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
			can_tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 无发送事件
			can_tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;// 传统CAN模式
			can_tx_hdr.MessageMarker = 0;
			
			bool send_success = false;
			uint16_t retry_count = 0;
			
			/*-------------------------------------------------------------------*/
			// 开始发送
			do
			{
				uint8_t mailbox_free_level = HAL_FDCAN_GetTxFifoFreeLevel(hcan);
				
				// 等待fifo有空闲
				if (mailbox_free_level != 0)
				{
					status = HAL_FDCAN_AddMessageToTxFifoQ(hcan, &can_tx_hdr, tx_frame_list[i].data);// 发送数据
					
					if (status == HAL_OK)
					{
						send_success = true;
					}
					else if (status == HAL_BUSY || status == HAL_TIMEOUT)
					{
						retry_count++;
						osDelay(1);
					}
					else
					{
						Error_Handler();
					}
				}
				else
				{
					retry_count++;
					osDelay(1);
				}
				
				if (send_success == true) break;
				
				if (retry_count > MAX_CAN_RETRY_COUNT) break;

			} while (send_success == false);
		}
	}


	
	uint8_t Can::Add_CanHandler(CanHandler *CanHandler)
	{
		hd_num++;
		if (hd_num > MAX_CAN_HANDLER_NUM) Error_Handler();
		uint8_t dx = hd_num - 1;
		
		hd_list[dx] = CanHandler;
		return dx;
	}
	
	
	
	/*-------------------------------------------------------------*/

	CanHandler::CanHandler(Can &can_) : can(&can_)
	{
		hd_list_dx = can->Add_CanHandler(this);// 保存设备列表索引
	}

}