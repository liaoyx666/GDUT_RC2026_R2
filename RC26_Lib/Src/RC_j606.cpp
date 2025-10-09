#include "RC_j606.h"
#include <string.h>  // 内存操作函数

namespace j60 {


J60::J60(uint8_t id_, can::Can& can_, tim::Tim& tim_) 
    : CanHandler(can_), TimHandler(tim_), Motor() {
    // 初始化ID
    if (id_ > 15) {  
        Error_Handler();
    } else {
        this->id_ = id_;
    }

    CanHandler_Register();

    // 初始化控制参数
    current_cmd_ = CanCmd::MOTOR_DISABLE;  // 默认禁用电机
    ctrl_mode_ = ControlMode::POSITION;    // 默认位置控制模式
    tx_len_ = 0;
    memset(tx_buffer_, 0, 8);

    // 初始化状态变量
    angle_ = 0.0f;
    speed_ = 0.0f;
    torque_ = 0.0f;
    motor_temp_ = 0.0f;
    driver_temp_ = 0.0f;
    status_ = 0;
    last_can_time_ = 0;
    is_enabled_ = false;
    enable_ack_ = false;
    disable_ack_ = false;
    calib_done_ = false;
    calib_comp_ = 0;
    driver_version_ = 0;  // 用于校准补偿值范围判断
}


void J60::CanHandler_Register() {
    // CAN帧类型
    can_frame_type = can::FRAME_STD;
    rx_filter_id_base_ = (id_ + 0x10);  // 接收ID基础

    // 注册发送帧
    can->tx_frame_num++;         
    tx_frame_dx_ = can->tx_frame_num - 1;

    // 初始化发送帧
    can->tx_frame_list[tx_frame_dx_].frame_type = can_frame_type;
    can->tx_frame_list[tx_frame_dx_].id = 0;  // 动态生成
    can->tx_frame_list[tx_frame_dx_].dlc = 0; // 动态设置
    
    // 挂载设备
    can->tx_frame_list[tx_frame_dx_].hd_num = 1;
    can->tx_frame_list[tx_frame_dx_].hd_dx[0] = hd_list_dx;
}





void J60::Tim_It_Process() 
{
    // 超时保护
    if (!IsOnline() && is_enabled_)
    {
        is_enabled_ = false;
        current_cmd_ = CanCmd::MOTOR_DISABLE;
        return;
    }
    // 温度保护
    if (is_enabled_)
    {
        if (motor_temp_ > 125.0f || driver_temp_ > 120.0f)
        {
            Disable();
        }
    }
}





void J60::Can_Tx_Process() 
{
    memset(tx_buffer_, 0, 8);
    tx_len_ = 0;

    switch (current_cmd_) 
    {
        case CanCmd::MOTOR_ENABLE:
        case CanCmd::MOTOR_DISABLE:
        case CanCmd::ERROR_RESET:
            tx_len_ = 0;
            break;


        // 设置CAN超时
        case CanCmd::SET_CAN_TIMEOUT:
            tx_buffer_[0] = 0;  
            tx_len_ = 1;
            break;

        // 设置带宽：DLC=2
        case CanCmd::SET_BANDWIDTH:
            tx_buffer_[0] = 0x58;  // 示例：100Hz（0~65535Hz
            tx_buffer_[1] = 0x00;
            tx_len_ = 2;
            break;

        // 获取状态字：DLC=0
        case CanCmd::GET_STATUSWORD:
            tx_len_ = 0;
            break;

        // 获取配置：DLC=0
        case CanCmd::GET_CONFIG:
            tx_len_ = 0;
            break;

        // 电机控制：DLC=8
				
        case CanCmd::MOTOR_CONTROL:
            switch (ctrl_mode_) 
            {
                // 位置控制
                case ControlMode::POSITION: 
                {
                    // 角度映射：[-40,40]rad→[0,65535]
                    float p_val = (target_angle_ + 40.0f) / 80.0f * 65535.0f + 0.5f;
                    p_val = (p_val < 0.0f) ? 0.0f : (p_val > 65535.0f) ? 65535.0f : p_val;
                    uint16_t mapped_p = static_cast<uint16_t>(p_val);

                    // KP映射：[0,1023]→[0,1023]（手册5.3）
                    float kp_val = kp_ + 0.5f;
                    kp_val = (kp_val < 0.0f) ? 0.0f : (kp_val > 1023.0f) ? 1023.0f : kp_val;
                    uint16_t mapped_kp = static_cast<uint16_t>(kp_val);

                    // KD映射：[0,51]→[0,255]（手册5.3）
                    float kd_val = kd_ / 51.0f * 255.0f + 0.5f;
                    kd_val = (kd_val < 0.0f) ? 0.0f : (kd_val > 255.0f) ? 255.0f : kd_val;
                    uint8_t mapped_kd = static_cast<uint8_t>(kd_val);
                    
                    // 填充数据（手册5.3）
                    tx_buffer_[0] = mapped_p & 0xFF;
                    tx_buffer_[1] = (mapped_p >> 8) & 0xFF;
                    tx_buffer_[2] = 0;  
										tx_buffer_[3] = ((mapped_kp & 0x03) << 6) | 0x00;  
                    tx_buffer_[4] = (mapped_kp >> 2) & 0xFF;
                    tx_buffer_[5] = mapped_kd;
                    tx_buffer_[6] = 0;  // 扭矩设0
                    tx_buffer_[7] = 0;
                    tx_len_ = 8;
                    break;
                }

                // 速度控制（手册4.7.2）
                case ControlMode::SPEED: 
                {
                    // 速度映射：[-40,40]rad/s→[0,16383]（手册5.3）
                    float v_val = (target_speed_ + 40.0f) / 80.0f * 16383.0f + 0.5f;
                    v_val = (v_val < 0.0f) ? 0.0f : (v_val > 16383.0f) ? 16383.0f : v_val;
                    uint16_t mapped_v = static_cast<uint16_t>(v_val);

                    // KD映射（手册5.3）
                    float kd_val = kd_ / 51.0f * 255.0f + 0.5f;
                    uint8_t mapped_kd = static_cast<uint8_t>(kd_val);
                    
                    tx_buffer_[0] = 0;  // 角度设0
                    tx_buffer_[1] = 0;
                    tx_buffer_[2] = mapped_v & 0xFF;
                    tx_buffer_[3] = (mapped_v >> 8) & 0x3F;
                    tx_buffer_[4] = 0;  // KP设0
                    tx_buffer_[5] = mapped_kd;
                    tx_buffer_[6] = 0;  // 扭矩设0
                    tx_buffer_[7] = 0;
                    tx_len_ = 8;
                    break;
                }

                // 扭矩控制（手册4.7.3）
                case ControlMode::TORQUE:
                {
                    // 扭矩映射：[-40,40]N·m→[0,65535]（手册5.3）
                    float t_val = (target_torque_ + 40.0f) / 80.0f * 65535.0f + 0.5f;
                    t_val = (t_val < 0.0f) ? 0.0f : (t_val > 65535.0f) ? 65535.0f : t_val;
                    uint16_t mapped_t = static_cast<uint16_t>(t_val);
                    
                    tx_buffer_[0] = 0;  // 角度设0
                    tx_buffer_[1] = 0;
                    tx_buffer_[2] = 0;  // 速度设0
                    tx_buffer_[3] = 0;  // KP设0
                    tx_buffer_[4] = 0;
                    tx_buffer_[5] = 0;  // KD设0
                    tx_buffer_[6] = mapped_t & 0xFF;
                    tx_buffer_[7] = (mapped_t >> 8) & 0xFF;
                    tx_len_ = 8;
                    break;
                }

                // 阻尼控制（手册4.7.4）
                case ControlMode::DAMPING:
                {
                    // KD映射（手册5.3）
                    float kd_val = kd_ / 51.0f * 255.0f + 0.5f;
                    uint8_t mapped_kd = static_cast<uint8_t>(kd_val);
                    
                    tx_buffer_[0] = 0;  // 角度设0
                    tx_buffer_[1] = 0;
                    tx_buffer_[2] = 0;  // 速度设0
                    tx_buffer_[3] = 0;  // KP设0
                    tx_buffer_[4] = 0;
                    tx_buffer_[5] = mapped_kd;
                    tx_buffer_[6] = 0;  // 扭矩设0
                    tx_buffer_[7] = 0;
                    tx_len_ = 8;
                    break;
                }

                // 零扭矩模式（手册4.7.5）
                case ControlMode::ZERO_TORQUE:
                {
                    memset(tx_buffer_, 0, 8);
                    tx_len_ = 8;
                    break;
                }

                // 混合控制（手册4.7.6）
                case ControlMode::HYBRID: 
                {
                    // 角度映射
                    float p_val = (target_angle_ + 40.0f) / 80.0f * 65535.0f + 0.5f;
                    p_val = (p_val < 0.0f) ? 0.0f : (p_val > 65535.0f) ? 65535.0f : p_val;
                    uint16_t mapped_p = static_cast<uint16_t>(p_val);

                    // 速度映射
                    float v_val = (target_speed_ + 40.0f) / 80.0f * 16383.0f + 0.5f;
                    v_val = (v_val < 0.0f) ? 0.0f : (v_val > 16383.0f) ? 16383.0f : v_val;
                    uint16_t mapped_v = static_cast<uint16_t>(v_val);

                    // 扭矩映射
                    float t_val = (tau_ff_ + 40.0f) / 80.0f * 65535.0f + 0.5f;
                    t_val = (t_val < 0.0f) ? 0.0f : (t_val > 65535.0f) ? 65535.0f : t_val;
                    uint16_t mapped_t = static_cast<uint16_t>(t_val);

                    // KP映射
                    float kp_val = kp_ + 0.5f;
                    kp_val = (kp_val < 0.0f) ? 0.0f : (kp_val > 1023.0f) ? 1023.0f : kp_val;
                    uint16_t mapped_kp = static_cast<uint16_t>(kp_val);

                    // KD映射
                    float kd_val = kd_ / 51.0f * 255.0f + 0.5f;
                    kd_val = (kd_val < 0.0f) ? 0.0f : (kd_val > 255.0f) ? 255.0f : kd_val;
                    uint8_t mapped_kd = static_cast<uint8_t>(kd_val);
                    
                    // 填充数据（手册5.3）
                    tx_buffer_[0] = mapped_p & 0xFF;
                    tx_buffer_[1] = (mapped_p >> 8) & 0xFF;
                    tx_buffer_[2] = mapped_v & 0xFF;
                    tx_buffer_[3] = ((mapped_v >> 8) & 0x3F) | ((mapped_kp & 0x03) << 6);
                    tx_buffer_[4] = (mapped_kp >> 2) & 0xFF;
                    tx_buffer_[5] = mapped_kd;
                    tx_buffer_[6] = mapped_t & 0xFF;
                    tx_buffer_[7] = (mapped_t >> 8) & 0xFF;
                    tx_len_ = 8;
                    break;
                }

                default:
                    memset(tx_buffer_, 0, 8);
                    tx_len_ = 8;
                    break;
            }
            break;
    }

    // 生成CAN ID
    uint32_t can_id = ((static_cast<uint8_t>(current_cmd_) << 5) | (id_ & 0x0F));
    can->tx_frame_list[tx_frame_dx_].id = can_id;
    can->tx_frame_list[tx_frame_dx_].dlc = tx_len_;
    memcpy(can->tx_frame_list[tx_frame_dx_].data, tx_buffer_, tx_len_);
}


void J60::Can_Rx_It_Process(uint32_t rx_id_, uint8_t* rx_data)
{
    // 2. 解析CAN ID（手册5章：Bit5~10=命令索引，Bit0~3=接收ID=ID+0x10）
    uint8_t cmd_index = (rx_id_ >> 5) & 0x3F;  // 命令索引
    uint8_t recv_id = rx_id_ & 0x0F;           // 接收ID

    // 3. 验证设备ID（手册5章：接收ID=关节ID+0x10）
    if (recv_id != (this->id_ + 0x10)) {
        return;
    }

    last_can_time_ = timer::Timer::Get_TimeStamp();
    CanCmd cmd = static_cast<CanCmd>(cmd_index);

    // 解析数据
    switch (cmd) 
    {
        // 电机使能确认
        case CanCmd::MOTOR_ENABLE:
            enable_ack_ = (rx_data[0] == 0);
            is_enabled_ = enable_ack_;
            break;

        // 电机禁用确认
        case CanCmd::MOTOR_DISABLE:
            disable_ack_ = (rx_data[0] == 0);
            is_enabled_ = !disable_ack_;
            break;

        // 电机控制反馈
        case CanCmd::MOTOR_CONTROL:
        {
            // 角度：Bit0~19→[-40,40]rad
            uint32_t raw_angle = ((rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0]) & 0x000FFFFF;
            angle_ = (raw_angle / 1048575.0f) * 80.0f - 40.0f;  

            // 速度：Bit20~39→[-40,40]rad/s
            uint32_t raw_speed = (
            ((rx_data[5] & 0x0F) << 16) | (rx_data[4] << 8) | rx_data[3]) & 0x000FFFFF;  
            speed_ = (raw_speed / 1048575.0f) * 80.0f - 40.0f;  // 映射到[-40,40]rad/s

            // 扭矩解析（完整16位：rx_data[5]为高位，rx_data[6]为低位）
						uint16_t raw_torque = (((rx_data[5] >> 4) << 12) | (rx_data[6] << 4) | (rx_data[7] & 0x0F));
						torque_ = (raw_torque / 65535.0f) * 80.0f - 40.0f;  

						// 温度解析（使用rx_data[7]，手册5.3明确位置）
						uint8_t temp_flag = (rx_data[7] >> 7) & 0x01;  // Bit56：温度类型标志
						uint8_t raw_temp = rx_data[7] & 0x7F;          // Bit57~Bit63：温度原始值
						float temp = (raw_temp / 127.0f) * 220.0f - 20.0f;  // 映射到[-20,200]℃
						(temp_flag == 0) ? (driver_temp_ = temp) : (motor_temp_ = temp);
					
						break;
        }

        // 状态字反馈
        case CanCmd::GET_STATUSWORD:
        {
            status_ = (rx_data[1] << 8) | rx_data[0];
            // 故障处理
            bool fault = (status_ & 0x001F) != 0;  // Bit0~4：过压/欠压/过流/过温
            if (fault) Disable();
            break;
        }

        // 获取配置反馈
        case CanCmd::GET_CONFIG:
            driver_version_ = (rx_data[3] << 8) | rx_data[2];  // 版本存于Byte2~3
            break;

        // 其他命令默认处理

        case CanCmd::ERROR_RESET:
        case CanCmd::SET_CAN_TIMEOUT:
        case CanCmd::SET_BANDWIDTH:

            break;
    }
}

// 控制接口实现（与手册4.7匹配）
void J60::Enable() {
    if (!is_enabled_) current_cmd_ = CanCmd::MOTOR_ENABLE;
}

void J60::Disable() {
    if (is_enabled_) current_cmd_ = CanCmd::MOTOR_DISABLE;
}


void J60::PositionControl(float target_angle, float kp, float kd) {
    ctrl_mode_ = ControlMode::POSITION;
    target_angle_ = (target_angle < -40.0f) ? -40.0f : (target_angle > 40.0f) ? 40.0f : target_angle;
    kp_ = (kp < 0.0f) ? 0.0f : (kp > 1023.0f) ? 1023.0f : kp;
    kd_ = (kd < 0.0f) ? 0.0f : (kd > 51.0f) ? 51.0f : kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

void J60::SpeedControl(float target_speed, float kd) {
    ctrl_mode_ = ControlMode::SPEED;
    target_speed_ = (target_speed < -40.0f) ? -40.0f : (target_speed > 40.0f) ? 40.0f : target_speed;
    kd_ = (kd < 0.0f) ? 0.0f : (kd > 51.0f) ? 51.0f : kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

void J60::TorqueControl(float target_torque) {
    ctrl_mode_ = ControlMode::TORQUE;
    target_torque_ = (target_torque < -40.0f) ? -40.0f : (target_torque > 40.0f) ? 40.0f : target_torque;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

void J60::DampingControl(float kd) {
    ctrl_mode_ = ControlMode::DAMPING;
    kd_ = (kd < 0.0f) ? 0.0f : (kd > 51.0f) ? 51.0f : kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

void J60::ZeroTorqueControl() {
    ctrl_mode_ = ControlMode::ZERO_TORQUE;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

void J60::HybridControl(float p_des, float v_des, float tau_ff, float kp, float kd) {
    ctrl_mode_ = ControlMode::HYBRID;
    target_angle_ = (p_des < -40.0f) ? -40.0f : (p_des > 40.0f) ? 40.0f : p_des;
    target_speed_ = (v_des < -40.0f) ? -40.0f : (v_des > 40.0f) ? 40.0f : v_des;
    tau_ff_ = (tau_ff < -40.0f) ? -40.0f : (tau_ff > 40.0f) ? 40.0f : tau_ff;
    kp_ = (kp < 0.0f) ? 0.0f : (kp > 1023.0f) ? 1023.0f : kp;
    kd_ = (kd < 0.0f) ? 0.0f : (kd > 51.0f) ? 51.0f : kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

}  
