#include "RC_vesc.h"

namespace motor
{
    Vesc::Vesc(uint8_t id_, can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_), Motor()
	{
		motor_mode = CURRENT_MODE;
        // 初始化ID（1-255有效）
        if (id_ <= 255 && id_ != 0)
		{
            id = id_;
        }
		else
		{
            Error_Handler();
        }
		rx_mask = 0xFF;
		
         if (can->hd_num > 8)
		{  	
			// 设备数量限制
            Error_Handler();
        }

        can_frame_type = can::FRAME_EXT;// 扩展帧
        rx_id = id;
		
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
	
	
	
    void Vesc::Tim_It_Process()
	{
		
    }

    void Vesc::Can_Tx_Process()
	{
        switch (motor_mode)
		{
            case CURRENT_MODE:
			{
				tx_id = (CAN_PACKET_SET_CURRENT << 8) | id;
				can->tx_frame_list[tx_frame_dx].id = tx_id;
                send_current = (int32_t)(target_current * 1000);  
                can->tx_frame_list[tx_frame_dx].data[0] = (send_current >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_current >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_current >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_current & 0xFF;
                break;
            }

            case RPM_MODE:
			{
				tx_id = (CAN_PACKET_SET_RPM << 8) | id;
				can->tx_frame_list[tx_frame_dx].id = tx_id;
                // RPM值直接转换为32位整数
                send_rpm = (int32_t)target_rpm;
                // 填充CAN数据（大端模式）
                can->tx_frame_list[tx_frame_dx].data[0] = (send_rpm >> 24) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[1] = (send_rpm >> 16) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[2] = (send_rpm >> 8) & 0xFF;
                can->tx_frame_list[tx_frame_dx].data[3] = send_rpm & 0xFF;
                break;
            }
			
			case POS_MODE:
			{
				tx_id = (CAN_PACKET_SET_POS << 8) | id;
				can->tx_frame_list[tx_frame_dx].id = tx_id;
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

    void Vesc::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
	{
		if (((rx_id_ >> 8) & 0xff) == 0x09)
		{
			rpm = (int32_t)((rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);
			current = ((int16_t)((rx_data[4] << 8) | rx_data[5])) * 0.01f;  // 修正缩放因子为0.01A/LSB
		}
		else if(((rx_id_ >> 8) & 0xff) == 0x10)
		{
			temperature = ((int16_t)((rx_data[0] << 8) | rx_data[1]))/10.0f;
			pos = ((int16_t)((rx_data[6] << 8) | rx_data[7]))/50.0f;
		}
    }
}