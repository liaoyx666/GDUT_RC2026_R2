#include "RC_can.h"

#define MAX_RETRY_COUNT 20 // 最大重试次数



extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can::Can::All_Can_Rx_It_Process(hcan, CAN_RX_FIFO0);
}


extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	can::Can::All_Can_Rx_It_Process(hcan, CAN_RX_FIFO1);
}



namespace can
{

	Can *Can::can_list[MAX_CAN_NUM] = {nullptr};// 初始化can列表指针
	uint8_t Can::can_num = 0;// 初始化can数量为0


	Can::Can(CAN_HandleTypeDef &hcan_) : hcan(&hcan_), task::ManagedTask("Can", 40, 128, task::TASK_PERIOD, 1)
	{
		taskENTER_CRITICAL();
		
		can_num++;
		if (can_num > MAX_CAN_NUM) Error_Handler();
		
		can_list_dx = can_num - 1;
		can_list[can_list_dx] = this;
		
		taskEXIT_CRITICAL();
	}

	
	void Can::Can_Filter_Init(
		uint32_t bank, uint32_t fifo, 
		uint32_t idHigh, uint32_t idLow, 
		uint32_t maskIdLow, uint32_t maskIdHigh
	)
	{
		CAN_FilterTypeDef  filter_init = {0};
		
		filter_init.FilterActivation = ENABLE;// 启用过滤器
		
		filter_init.FilterMode = CAN_FILTERMODE_IDMASK;// 采用ID掩码模式
		
		filter_init.FilterScale = CAN_FILTERSCALE_32BIT;// 32位过滤器宽度
		
		filter_init.FilterBank = bank;// 过滤器组号
		
		filter_init.FilterFIFOAssignment = fifo; // 配置FIFO
		
		filter_init.FilterIdHigh = idHigh;// 32位ID配置（高16位和低16位）
		filter_init.FilterIdLow = idLow;
		
		filter_init.FilterMaskIdHigh = maskIdHigh;// 32位掩码配置（高16位和低16位）
		filter_init.FilterMaskIdLow = maskIdLow;
		
		filter_init.SlaveStartFilterBank = 14;// 配置从机过滤器起始组（仅在双CAN外设时需要），单CAN外设时可设为0，双CAN时通常设为14
														  
		if (HAL_CAN_ConfigFilter(hcan, &filter_init) != HAL_OK) Error_Handler();// 配置失败处理
	}
	
	
	void Can::Can_Start()
	{
		// 启动CAN外设
		if (HAL_CAN_Start(hcan) != HAL_OK) Error_Handler();
		
		// 使能CAN接收FIFO0消息挂起中断
		if (HAL_CAN_ActivateNotification((hcan), CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
		
		// 使能CAN接收FIFO1消息挂起中断
		if (HAL_CAN_ActivateNotification((hcan), CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) Error_Handler();
	}
	
	

	void Can::All_Can_Rx_It_Process(CAN_HandleTypeDef *hcan, uint32_t fifo)
	{
		// 查找对应can对象
		for (uint8_t i = 0; i < can_num; i++)
		{
			if (can_list[i]->hcan == hcan)
			{
				CAN_RxHeaderTypeDef can_rx_hdr;
				uint8_t rx_data[8];
				
				if (HAL_CAN_GetRxMessage(hcan, fifo, &can_rx_hdr, rx_data) != HAL_OK) break;
				
				// 查找对应rx帧id的设备
				for (uint16_t j = 0; j < can_list[i]->hd_num; j++)
				{
					if (can_list[i]->hd_list[j] == nullptr) continue;
					
					// 检查ID匹配
					bool id_matched = false;
					
					if (can_rx_hdr.IDE == CAN_ID_EXT) id_matched = (can_rx_hdr.ExtId == can_list[i]->hd_list[j]->rx_id);
					else id_matched = (can_rx_hdr.StdId == can_list[i]->hd_list[j]->rx_id);
					
					if (id_matched == true)
					{
						can_list[i]->hd_list[j]->Can_Rx_It_Process(rx_data);// 调用设备接收处理函数
						break;// 每个ID只对应一个设备
					}
				}
				break;
			}
		}
	}


	void Can::Task_Process()
	{
		CAN_TxHeaderTypeDef can_tx_hdr;// 帧头
		uint32_t tx_mailbox;
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
				can_tx_hdr.StdId = tx_frame_list[i].id;
				can_tx_hdr.IDE = CAN_ID_STD;
			}
			else
			{
				can_tx_hdr.ExtId = tx_frame_list[i].id;
				can_tx_hdr.IDE = CAN_ID_EXT;
			}
			
			can_tx_hdr.DLC = tx_frame_list[i].dlc;
			can_tx_hdr.RTR = CAN_RTR_DATA;
			can_tx_hdr.TransmitGlobalTime = DISABLE;
			
			bool send_success = false;
			uint16_t retry_count = 0;
			
			/*-------------------------------------------------------------------*/
			// 开始发送
			do
			{
				uint8_t mailbox_free_level = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
				
				// 等待邮箱有空闲
				if (mailbox_free_level != 0)
				{
					status = HAL_CAN_AddTxMessage(hcan, &can_tx_hdr, tx_frame_list[i].data, &tx_mailbox);// 发送数据
					
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
				
				if (retry_count > MAX_RETRY_COUNT) break;

			} while (send_success == false);
		}
	}


	
	uint8_t Can::Add_CanHandler(CanHandler *CanHandler)
	{
		hd_num++;
		uint8_t dx = hd_num - 1;
		
		hd_list[dx] = CanHandler;
		return dx;
	}
	
	
	
	/*-------------------------------------------------------------*/

	CanHandler::CanHandler(Can &can_) : can(&can_)
	{
		hd_list_dx = can->Add_CanHandler(this);// 
	}

}