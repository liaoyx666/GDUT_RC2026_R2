#include "RC_DM_imu.h"

namespace imu
{
    /**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
    int DM_imu::float_to_uint(float x_float, float x_min, float x_max, int bits)
    {
        /* Converts a float to an unsigned int, given range and number of bits */
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
    }
    /**
    ************************************************************************
    * @brief:      	uint_to_float: 无符号整数转换为浮点数函数
    * @param[in]:   x_int: 待转换的无符号整数
    * @param[in]:   x_min: 范围最小值
    * @param[in]:   x_max: 范围最大值
    * @param[in]:   bits:  无符号整数的位数
    * @retval:     	浮点数结果
    * @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
    ************************************************************************
    **/
    float DM_imu::uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /* converts unsigned int to float, given range and number of bits */
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }


    DM_imu::DM_imu(uint8_t id_,can::Can &can_) : can::CanHandler(can_)
    {
        id = id_;
        master_id = 0xFD;

        tx_id=id |0x00;
        rx_id=(id <<8)|0x00;
        can_frame_type=can::FRAME_STD;//标准帧

        CanHandler_Register();

        can->tx_frame_list[tx_frame_dx].new_message = true;
    }

    void DM_imu::CanHandler_Register()
	{
		can->tx_frame_num++;// 帧加一
		tx_frame_dx = can->tx_frame_num - 1;// 帧索引后移一位
		
		can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
		can->tx_frame_list[tx_frame_dx].id = tx_id;
		can->tx_frame_list[tx_frame_dx].dlc = 8;

		can->tx_frame_list[tx_frame_dx].hd_num = 1;
		can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
	}

    void DM_imu::Can_Tx_Process()
    {
        //根据不同的发送命令，组装不同的数据内容
        can->tx_frame_list[tx_frame_dx].id = tx_id;

        can->tx_frame_list[tx_frame_dx].data[0] = 0xCC;
        can->tx_frame_list[tx_frame_dx].data[1] = reg_id;
        can->tx_frame_list[tx_frame_dx].data[2] = ac;
        can->tx_frame_list[tx_frame_dx].data[3] = 0xDD;
        can->tx_frame_list[tx_frame_dx].data[4] = (data >> 24) & 0xFF;
        can->tx_frame_list[tx_frame_dx].data[5] = (data >> 16) & 0xFF;
        can->tx_frame_list[tx_frame_dx].data[6] = (data >> 8) & 0xFF;
        can->tx_frame_list[tx_frame_dx].data[7] = data & 0xFF;
    }

    void DM_imu::write_reg(uint8_t reg_id_,uint32_t data_)
    {
				reg_id = reg_id_;
				ac = CMD_WRITE;
				data = data_;
    }

    void DM_imu::read_reg(uint8_t reg_id_)
    {
				reg_id = reg_id_;
				ac = CMD_READ;
				data= 0;
    }

    inline void DM_imu::reboot()
    {
        write_reg(REBOOT_IMU,0);
    }

    inline void DM_imu::accel_calibration()
    {
        write_reg(ACCEL_CALI,0);
    }

    inline void DM_imu::gyro_calibration()
    {
        write_reg(GYRO_CALI,0);
    }


    inline void DM_imu::change_com_port(imu_com_port_e port)
    {
        write_reg(CHANGE_COM,(uint8_t)port);
    }

    inline void DM_imu::set_active_mode_delay(uint32_t delay)
    {
        write_reg(SET_DELAY,delay);
    }

    //设置成主动模式
    inline void DM_imu::change_to_active()
    {
        write_reg(CHANGE_ACTIVE,1);
    }

    inline void DM_imu::change_to_request()
    {
        write_reg(CHANGE_ACTIVE,0);
    }

    inline void DM_imu::set_baud(imu_baudrate_e baud)
    {
        write_reg(SET_BAUD,(uint8_t)baud);
    }

    inline void DM_imu::set_can_id(uint8_t can_id)
    {
        write_reg(SET_CAN_ID,can_id);
    }

    inline void DM_imu::set_mst_id(uint8_t mst_id)
    {
        write_reg(SET_MST_ID,mst_id);
    }

    inline void DM_imu::save_parameters()
    {
        write_reg(SAVE_PARAM,0);
    }

    inline void DM_imu::restore_settings()
    {
        write_reg(RESTORE_SETTING,0);
    }


    inline void DM_imu::request_accel()
    {
        read_reg(ACCEL_DATA);
    }

    inline void DM_imu::request_gyro()
    {
        read_reg(GYRO_DATA);
    }

    inline void DM_imu::request_euler()
    {
        read_reg(EULER_DATA);
    }

    inline void DM_imu::request_quat()
    {
        read_reg(QUAT_DATA);
    }

    void DM_imu::IMU_UpdateAccel(uint8_t* pData)
    {
        uint16_t accel_[3];
        
        accel_[0]=pData[3]<<8|pData[2];
        accel_[1]=pData[5]<<8|pData[4];
        accel_[2]=pData[7]<<8|pData[6];
        
        accel[0]=uint_to_float(accel_[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
        accel[1]=uint_to_float(accel_[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
        accel[2]=uint_to_float(accel_[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
        
    }

    void DM_imu::IMU_UpdateGyro(uint8_t* pData)
    {
        uint16_t gyro_[3];
        
        gyro_[0]=pData[3]<<8|pData[2];
        gyro_[1]=pData[5]<<8|pData[4];
        gyro_[2]=pData[7]<<8|pData[6];
        
        gyro[0]=uint_to_float(gyro_[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
        gyro[1]=uint_to_float(gyro_[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
        gyro[2]=uint_to_float(gyro_[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
    }


    void DM_imu::IMU_UpdateEuler(uint8_t* pData)
    {
        int euler_[3];
        
        euler_[0]=pData[3]<<8|pData[2];
        euler_[1]=pData[5]<<8|pData[4];
        euler_[2]=pData[7]<<8|pData[6];
        
        euler[0]=uint_to_float(euler_[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
        euler[1]=uint_to_float(euler_[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
        euler[2]=uint_to_float(euler_[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
    }


    void DM_imu::IMU_UpdateQuaternion(uint8_t* pData)
    {
        int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
        int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
        int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
        int z = (pData[6]&0x3F)<<8|pData[7];
        
        quaternion[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
        quaternion[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
        quaternion[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
        quaternion[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
    }

    void DM_imu::IMU_UpdateData(uint8_t* pData)
    {
        switch(pData[0])
        {
            case 1:
                IMU_UpdateAccel(pData);
                break;
            case 2:
                IMU_UpdateGyro(pData);
                break;
            case 3:
                IMU_UpdateEuler(pData);
                break;
            case 4:
                IMU_UpdateQuaternion(pData);
                break;
        }
    }

    void DM_imu::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
    {
        //根据不同的接收命令，解析不同的数据内容
        if(rx_id_ != rx_id)
        {
            return;
        }
        IMU_UpdateData(rx_data);
    }
}

