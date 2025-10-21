#include "RC_vesc.h"

namespace motor
{
    Vesc::Vesc(uint8_t id_, can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_), Motor()
	{
        // 初始化ID（1-255有效）
        if (id_ <= 255 && id_ != 0)
		{
            id = id_;
        }
		else
		{
            Error_Handler();
        }
		
		
         if (can->hd_num > 8)
		{  	
			// 设备数量限制
            Error_Handler();
        }

        can_frame_type = can::FRAME_EXT;// 扩展帧
        UpdateTxId();// 初始化发送ID
        rx_id = (CAN_PACKET_STATUS<< 8) | id;
		
        // 注册CAN设备
        CanHandler_Register();
        
      
       
    }
    
	
    void Vesc::CanHandler_Register()
	{
       can->tx_frame_num++;// 帧加一
		tx_frame_dx = can->tx_frame_num - 1;// 帧索引后移一位
		
		can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
		can->tx_frame_list[tx_frame_dx].id = tx_id;
		can->tx_frame_list[tx_frame_dx].dlc = 8;

		can->tx_frame_list[tx_frame_dx].hd_num = 1;
		can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
    }
	
	
	
    void Vesc::Set_Rpm(float target_rpm_)
	{
        vesc_motor_mode = vesc_rpm;
        target_rpm = target_rpm_;
        UpdateTxId();// 模式切换时更新发送ID
        
    }
	
    //target_c=10A
    void Vesc::Set_Current(float target_c_)
	{
        vesc_motor_mode = vesc_current;
        target_current = target_c_;
        UpdateTxId();// 模式切换时更新发送ID
      
    }
	
	//位置式非常不建议使用
    void Vesc::Set_Pos(float target_pos_)
    {
        vesc_motor_mode = vesc_pos;
        target_pos = target_pos_;
        UpdateTxId();// 模式切换时更新发送ID
    }
	

	
    void Vesc::Tim_It_Process()
	{
		
    }

    void Vesc::Can_Tx_Process()
	{
        switch (vesc_motor_mode)
		{
            case vesc_current:
			{
                send_current = (int32_t)(target_current * 1000);  
                can->tx_frame_list[tx_frame_dx].data[0] = (send_current >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_current >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_current >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_current & 0xFF;
                break;
            }
            case vesc_duty:
			{
                send_duty = (int32_t)(target_duty * 100000);  
                can->tx_frame_list[tx_frame_dx].data[0] = (send_duty >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_duty >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_duty >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_duty & 0xFF;
                break;
            }
            case vesc_rpm:
			{
                // RPM值直接转换为32位整数
                send_rpm = (int32_t)target_rpm;
                // 填充CAN数据（大端模式）
                can->tx_frame_list[tx_frame_dx].data[0] = (send_rpm >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_rpm >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_rpm >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_rpm & 0xFF;
                break;
            }
            case vesc_pos:
			{
                // POS值直接转换为32位整数
                send_pos = (int32_t)(target_pos*1000000);
                // 填充CAN数据（大端模式）
                can->tx_frame_list[tx_frame_dx].data[0] = (send_pos >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_pos >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_pos >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_pos & 0xFF;
                break;
            }
            default:
                break;
        }
       
        can->tx_frame_list[tx_frame_dx].dlc = 4;
    }

    void Vesc::Can_Rx_It_Process(uint8_t *rx_data)
	{
        rpm = (int32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);
        current = ((int16_t)((rx_data[4] << 8) | rx_data[5])) * 0.01f;  // 修正缩放因子为0.01A/LSB
		duty = ((int16_t)((rx_data[6] << 8) | rx_data[7]))/1000.0f;
    }

	// 辅助函数：更新CAN发送ID
	void Vesc::UpdateTxId()
	{
		switch (vesc_motor_mode)
		{
			case vesc_current:
				tx_id = (CAN_PACKET_SET_CURRENT << 8) | id;
				break;
			case vesc_rpm:
				tx_id = (CAN_PACKET_SET_RPM << 8) | id;
				break;
			case vesc_pos:
				tx_id = (CAN_PACKET_SET_POS << 8) | id;
				break;
			case vesc_duty:
				tx_id = (CAN_PACKET_SET_DUTY << 8) | id;
				break;  
			default:
				break;
		}
		// 更新CAN帧的ID
		can->tx_frame_list[tx_frame_dx].id = tx_id;
	}
}