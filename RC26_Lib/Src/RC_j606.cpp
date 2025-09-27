#include "RC_j606.h"
#include <string.h>  // 内存操作函数

namespace j60 {

/**
 * @brief 构造函数：初始化J60关节控制器
 * @param id_ 关节ID(1-15)
 * @param can_ CAN通信对象引用
 * @param tim_ 定时器对象引用
 */
J60::J60(uint8_t id_, can::Can& can_, tim::Tim& tim_) 
    : CanHandler(can_), TimHandler(tim_), Motor() {
    // 初始化ID(0~15有效)
    if (id_ <= 15 && id_ != 0) {
        this->id_ = id_;  // 注意：修复原代码中的变量名冲突
    } else {
        Error_Handler();  // ID无效时调用错误处理
    }

    // 登记CAN设备
    CanHandler_Register();

    // 初始化控制参数
    current_cmd_ = CanCmd::MOTOR_DISABLE;  // 默认禁用电机
    ctrl_mode_ = ControlMode::POSITION;    // 默认位置控制模式
    tx_len_ = 0;
    memset(tx_buffer_, 0, 8);  // 清空发送缓冲区

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

    // 初始化PID参数(10ms周期)
    pid_pos_.Pid_Mode_Init(false, false, 0);  // 位置环PID初始化
    pid_pos_.Pid_Param_Init(50.0f, 0.1f, 0.5f, 0, 0.01f, 0, 
                           40.0f, 40.0f, 5.0f, 5.0f, 5.0f);  // 位置环参数

    pid_spd_.Pid_Mode_Init(true, false, 0);  // 速度环PID初始化
    pid_spd_.Pid_Param_Init(20.0f, 0.5f, 2.0f, 0, 0.01f, 0, 
                           40.0f, 40.0f, 3.0f, 3.0f, 3.0f);  // 速度环参数
}

/**
 * @brief 注册CAN设备，配置发送/接收参数
 */
void J60::CanHandler_Register() {
    if (can->hd_num > 16) Error_Handler();  // J60最多支持16个关节

    can_frame_type = can::FRAME_STD;  // 使用标准帧

    // 设置发送/接收ID(符合J60协议)
    tx_id_ = 0x1FF;                   // 基础发送ID
    rx_id = 0x200 + id_;              // 接收ID = 0x200 + 关节ID

    // 注册到CAN发送帧列表
    if (can->tx_frame_num == 0) {
        // CAN总线上还没有发送帧，创建第一个帧
        tx_frame_dx_ = 0;
        can->tx_frame_num = 1;
        
        can->tx_frame_list[tx_frame_dx_].frame_type = can_frame_type;
        can->tx_frame_list[tx_frame_dx_].id = tx_id_;
        can->tx_frame_list[tx_frame_dx_].dlc = 8;  // CAN数据长度8字节
        
        can->tx_frame_list[tx_frame_dx_].hd_num = 1;
        can->tx_frame_list[tx_frame_dx_].hd_dx[0] = hd_list_dx;
    } else {
        bool have_same_tx_id = false;  // 是否有相同ID的发送帧
        
        // 查找相同ID的发送帧
        for (uint16_t i = 0; i < can->tx_frame_num; i++) {
            if (can->tx_frame_list[i].frame_type == can_frame_type && 
                can->tx_frame_list[i].id == tx_id_) {
                have_same_tx_id = true;
                tx_frame_dx_ = i;  // 记录相同帧的索引
                
                // 增加该帧挂载的设备数量
                can->tx_frame_list[tx_frame_dx_].hd_num++;
                if (can->tx_frame_list[tx_frame_dx_].hd_num > 4) {
                    Error_Handler();  // 一个帧最多挂载4个设备
                }
                // 记录当前设备在帧中的索引
                can->tx_frame_list[tx_frame_dx_].hd_dx[
                    can->tx_frame_list[tx_frame_dx_].hd_num - 1] = hd_list_dx;
                break;
            }
        }
        
        // 无相同ID的帧，创建新帧
        if (!have_same_tx_id) {
            can->tx_frame_num++;
            tx_frame_dx_ = can->tx_frame_num - 1;  // 新帧索引
            
            can->tx_frame_list[tx_frame_dx_].frame_type = can_frame_type;
            can->tx_frame_list[tx_frame_dx_].id = tx_id_;
            can->tx_frame_list[tx_frame_dx_].dlc = 8;
    
            can->tx_frame_list[tx_frame_dx_].hd_num = 1;
            can->tx_frame_list[tx_frame_dx_].hd_dx[0] = hd_list_dx;
        }
    }
}

void J60::Tim_It_Process() 
	{
    // 超时保护：如果超过500ms未收到数据且电机处于使能状态，则禁用电机
    if (!IsOnline() && is_enabled_)
			{
        is_enabled_ = false;
        current_cmd_ = CanCmd::MOTOR_DISABLE;
        return;
    }

    // 根据不同控制模式执行相应控制算法
    switch (ctrl_mode_)
			{
        case ControlMode::POSITION:
            // 位置环控制：通过位置PID计算目标速度
            pid_pos_.Update_Real(angle_);       // 更新当前位置反馈
            pid_pos_.Update_Target(target_angle_);  // 更新目标位置
            // 计算位置环输出(目标速度)，限幅±40rad/s
            target_speed_ = pid_pos_.Pid_Calculate(true, 40.0f);  
            break;

        case ControlMode::SPEED:
            // 速度环控制：直接使用目标速度(在Can_Tx_Process中处理)
            break;

        case ControlMode::HYBRID:
            // 混合控制：位置环输出 + 前馈扭矩
            pid_pos_.Update_Real(angle_);
            pid_pos_.Update_Target(target_angle_);
            target_torque_ = tau_ff_ + pid_pos_.Pid_Calculate(true, 40.0f);
            break;

        default:
            // 其他模式无需定时器计算
            break;
    }

    // 安全保护：温度过高时自动禁用电机
    if (is_enabled_)
			{
        if (motor_temp_ > 125.0f || driver_temp_ > 120.0f)
					{
            Disable();  // 温度超限，禁用电机
        }
    }
}


void J60::Can_Tx_Process() 
	{
    // 清空发送缓冲区（基础初始化）
    memset(tx_buffer_, 0, 8);
    tx_len_ = 0;

    // 根据当前命令填充发送数据
    switch (current_cmd_) 
			{
        case CanCmd::MOTOR_ENABLE:
        case CanCmd::MOTOR_DISABLE:
        case CanCmd::SAVE_CONFIG:
        case CanCmd::ERROR_RESET:
            // 无参数命令：显式填充所有字节为0
            tx_buffer_[0] = 0;
            tx_buffer_[1] = 0;
            tx_buffer_[2] = 0;
            tx_buffer_[3] = 0;
            tx_buffer_[4] = 0;
            tx_buffer_[5] = 0;
            tx_buffer_[6] = 0;
            tx_buffer_[7] = 0;
            tx_len_ = 8;  // 固定8字节长度
            break;

        case CanCmd::CALIBRATE:
        case CanCmd::SET_ZERO:
            // 仅设置ID，其余字节显式为0
            tx_buffer_[0] = id_;
            tx_buffer_[1] = 0;
            tx_buffer_[2] = 0;
            tx_buffer_[3] = 0;
            tx_buffer_[4] = 0;
            tx_buffer_[5] = 0;
            tx_buffer_[6] = 0;
            tx_buffer_[7] = 0;
            tx_len_ = 8;  // 固定8字节长度
            break;

        case CanCmd::SET_CAN_TIMEOUT:
            // 超时参数(500ms)，其余字节显式为0
            tx_buffer_[0] = 20;
            tx_buffer_[1] = 0;
            tx_buffer_[2] = 0;
            tx_buffer_[3] = 0;
            tx_buffer_[4] = 0;
            tx_buffer_[5] = 0;
            tx_buffer_[6] = 0;
            tx_buffer_[7] = 0;
            tx_len_ = 8;  // 固定8字节长度
            break;

        case CanCmd::MOTOR_CONTROL:
            // 根据控制模式填充不同的控制参数，未使用字节显式为0
            switch (ctrl_mode_) 
							{
                case ControlMode::POSITION: 
									{
                    // 位置控制：目标角度 + KP + KD
                    uint16_t mapped_p = AngleToCan(target_angle_);
                    uint16_t mapped_kp = KpToCan(kp_);
                    uint8_t mapped_kd = KdToCan(kd_);
                    
                    tx_buffer_[0] = mapped_p & 0xFF;               // 角度低8位
                    tx_buffer_[1] = (mapped_p >> 8) & 0xFF;        // 角度高8位
                    tx_buffer_[2] = 0;                             // 显式填充0
                    tx_buffer_[3] = (mapped_kp & 0x03) << 6;       // KP高2位
                    tx_buffer_[4] = (mapped_kp >> 2) & 0xFF;       // KP低8位
                    tx_buffer_[5] = mapped_kd;                     // KD
                    tx_buffer_[6] = 0;                             // 显式填充0
                    tx_buffer_[7] = 0;                             // 显式填充0
                    tx_len_ = 8;
                    break;
                }

                case ControlMode::SPEED: 
									{
                    // 速度控制：目标速度 + KD
                    uint16_t mapped_v = SpeedToCan(target_speed_);
                    uint8_t mapped_kd = KdToCan(kd_);
                    
                    tx_buffer_[0] = 0;                             // 显式填充0
                    tx_buffer_[1] = 0;                             // 显式填充0
                    tx_buffer_[2] = mapped_v & 0xFF;               // 速度低8位
                    tx_buffer_[3] = (mapped_v >> 8) & 0x3F;        // 速度高6位
                    tx_buffer_[4] = 0;                             // 显式填充0
                    tx_buffer_[5] = mapped_kd;                     // KD
                    tx_buffer_[6] = 0;                             // 显式填充0
                    tx_buffer_[7] = 0;                             // 显式填充0
                    tx_len_ = 8;
                    break;
                }

                case ControlMode::TORQUE:
									{
                    // 扭矩控制：目标扭矩
                    uint16_t mapped_t = TorqueToCan(target_torque_);
                    
                    tx_buffer_[0] = 0;                             // 显式填充0
                    tx_buffer_[1] = 0;                             // 显式填充0
                    tx_buffer_[2] = 0;                             // 显式填充0
                    tx_buffer_[3] = 0;                             // 显式填充0
                    tx_buffer_[4] = 0;                             // 显式填充0
                    tx_buffer_[5] = 0;                             // 显式填充0
                    tx_buffer_[6] = mapped_t & 0xFF;               // 扭矩低8位
                    tx_buffer_[7] = (mapped_t >> 8) & 0xFF;        // 扭矩高8位
                    tx_len_ = 8;
                    break;
                }

                case ControlMode::DAMPING:
									{
                    // 阻尼控制：KD参数
                    tx_buffer_[0] = 0;                             // 显式填充0
                    tx_buffer_[1] = 0;                             // 显式填充0
                    tx_buffer_[2] = 0;                             // 显式填充0
                    tx_buffer_[3] = 0;                             // 显式填充0
                    tx_buffer_[4] = 0;                             // 显式填充0
                    tx_buffer_[5] = KdToCan(kd_);                  // KD参数
                    tx_buffer_[6] = 0;                             // 显式填充0
                    tx_buffer_[7] = 0;                             // 显式填充0
                    tx_len_ = 8;
                    break;
                }

                case ControlMode::HYBRID: 
									{
                    // 混合控制：位置 + 速度 + 扭矩前馈 + KP + KD
                    uint16_t mapped_p = AngleToCan(target_angle_);
                    uint16_t mapped_v = SpeedToCan(target_speed_);
                    uint16_t mapped_t = TorqueToCan(target_torque_);
                    uint16_t mapped_kp = KpToCan(kp_);
                    uint8_t mapped_kd = KdToCan(kd_);
                    
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

                // 补充零扭矩控制模式
                case ControlMode::ZERO_TORQUE:
									{
                    // 零扭矩模式：所有字节显式为0
                    tx_buffer_[0] = 0;
                    tx_buffer_[1] = 0;
                    tx_buffer_[2] = 0;
                    tx_buffer_[3] = 0;
                    tx_buffer_[4] = 0;
                    tx_buffer_[5] = 0;
                    tx_buffer_[6] = 0;
                    tx_buffer_[7] = 0;
                    tx_len_ = 8;
                    break;
                }

                default:
                    // 未知模式：所有字节显式为0
                    tx_buffer_[0] = 0;
                    tx_buffer_[1] = 0;
                    tx_buffer_[2] = 0;
                    tx_buffer_[3] = 0;
                    tx_buffer_[4] = 0;
                    tx_buffer_[5] = 0;
                    tx_buffer_[6] = 0;
                    tx_buffer_[7] = 0;
                    tx_len_ = 8;
                    break;
            }
            break;

        default:
            // 未知命令：所有字节显式为0
            tx_buffer_[0] = 0;
            tx_buffer_[1] = 0;
            tx_buffer_[2] = 0;
            tx_buffer_[3] = 0;
            tx_buffer_[4] = 0;
            tx_buffer_[5] = 0;
            tx_buffer_[6] = 0;
            tx_buffer_[7] = 0;
            tx_len_ = 8;
            break;
    }

    // 计算CAN ID并发送（确保发送完整的8字节数据）
    uint32_t can_id = ((static_cast<uint8_t>(current_cmd_) << 5) | (id_ & 0x0F));
    can->tx_frame_list[tx_frame_dx_].id = can_id;
    memcpy(can->tx_frame_list[tx_frame_dx_].data, tx_buffer_, tx_len_);
}


void J60::Can_Rx_It_Process(uint8_t* rx_data)
	{
    last_can_time_ = timer::Timer::Get_TimeStamp();  // 更新最后接收时间
    CanCmd cmd = static_cast<CanCmd>(rx_data[0]);    // 解析命令类型
    
    if (rx_data[1] != id_) return;  // 验证关节ID，只处理本设备的数据

    // 所有case使用代码块包裹，避免跳转错误
    switch (cmd) 
			{
        case CanCmd::MOTOR_ENABLE:
					{
            // 处理使能确认
            enable_ack_ = (rx_data[2] == 0);  // 0表示成功
            if (enable_ack_) is_enabled_ = true;
            break;
        }

        case CanCmd::MOTOR_DISABLE:
					{
            // 处理禁用确认
            disable_ack_ = (rx_data[2] == 0);  // 0表示成功
            if (disable_ack_) is_enabled_ = false;
            break;
        }

        case CanCmd::MOTOR_CONTROL:
					{
            // 解析角度(24位)
            uint32_t raw_angle = (rx_data[4] << 16) | (rx_data[3] << 8) | rx_data[2];
            angle_ = CanToAngle(raw_angle);
            
            // 解析速度(24位)
            uint32_t raw_speed = (rx_data[7] << 16) | (rx_data[6] << 8) | rx_data[5];
            speed_ = CanToSpeed(raw_speed);
            
            // 解析扭矩(16位)
            uint16_t raw_torque = (rx_data[9] << 8) | rx_data[8];
            torque_ = CanToTorque(raw_torque);
            
            // 解析温度
            uint8_t temp_flag = (rx_data[10] >> 7) & 0x01;  // 0=驱动器温度,1=电机温度
            uint8_t raw_temp = rx_data[10] & 0x7F;          // 温度数据
            float temp = CanToTemperature(raw_temp);
            if (temp_flag == 0) driver_temp_ = temp;
            else motor_temp_ = temp;
            break;
        }

        case CanCmd::CALIBRATE: 
					{
            // 解析校准结果
            calib_comp_ = (int16_t)((rx_data[3] << 8) | rx_data[2]);  // 校准补偿值
            calib_done_ = true;  // 标记校准完成
            break;
        }

        case CanCmd::GET_STATUSWORD:
					{
            // 解析状态字
            status_ = (rx_data[3] << 8) | rx_data[2];
            break;
        }

        default: {
            // 未定义命令的默认处理
            break;
        }
    }
}

// 控制接口实现
/**
 * @brief 使能电机
 */
void J60::Enable() 
	{
    current_cmd_ = CanCmd::MOTOR_ENABLE;
}

/**
 * @brief 禁用电机
 */
void J60::Disable() 
	{
    current_cmd_ = CanCmd::MOTOR_DISABLE;
}

/**
 * @brief 校准电机
 */
void J60::Calibrate() 
	{
    current_cmd_ = CanCmd::CALIBRATE;
}

/**
 * @brief 设置零点位置
 */
void J60::SetZero() 
{
    current_cmd_ = CanCmd::SET_ZERO;
}

/**
 * @brief 保存配置参数到电机
 */
void J60::SaveConfig() 
	{
    current_cmd_ = CanCmd::SAVE_CONFIG;
}

/**
 * @brief 位置控制模式设置
 * @param target_angle 目标角度(rad)
 * @param kp 位置环比例增益
 * @param kd 速度环阻尼增益
 */
void J60::PositionControl(float target_angle, float kp, float kd) 
	{
    ctrl_mode_ = ControlMode::POSITION;
    target_angle_ = target_angle;
    kp_ = kp;
    kd_ = kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

/**
 * @brief 速度控制模式设置
 * @param target_speed 目标速度(rad/s)
 * @param kd 速度环阻尼增益
 */
void J60::SpeedControl(float target_speed, float kd)
	{
    ctrl_mode_ = ControlMode::SPEED;
    target_speed_ = target_speed;
    kd_ = kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

/**
 * @brief 扭矩控制模式设置
 * @param target_torque 目标扭矩(N·m)
 */
void J60::TorqueControl(float target_torque) 
	{
    ctrl_mode_ = ControlMode::TORQUE;
    target_torque_ = target_torque;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

/**
 * @brief 阻尼控制模式设置
 * @param kd 阻尼系数
 */
void J60::DampingControl(float kd)
	{
    ctrl_mode_ = ControlMode::DAMPING;
    kd_ = kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

/**
 * @brief 零扭矩控制模式设置
 */
void J60::ZeroTorqueControl()
	{
    ctrl_mode_ = ControlMode::ZERO_TORQUE;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

/**
 * @brief 混合控制模式设置(位置+速度+扭矩前馈)
 * @param p_des 目标位置(rad)
 * @param v_des 目标速度(rad/s)
 * @param tau_ff 前馈扭矩(N·m)
 * @param kp 位置环比例增益
 * @param kd 速度环阻尼增益
 */
void J60::HybridControl(float p_des, float v_des, float tau_ff, float kp, float kd)
	{
    ctrl_mode_ = ControlMode::HYBRID;
    target_angle_ = p_des;
    target_speed_ = v_des;
    tau_ff_ = tau_ff;
    kp_ = kp;
    kd_ = kd;
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

// 参数映射函数实现(物理量与CAN数据的转换)
/**
 * @brief 角度转CAN数据
 * @param angle 角度值(rad)，范围[-40, 40]
 * @return CAN数据(16位)，范围[0, 65535]
 */
uint16_t J60::AngleToCan(float angle) 
	{
    float value = (angle + 40.0f) / 80.0f * 65535.0f + 0.5f;  // 线性映射
    // 手动限幅(替代std::clamp)
    if (value < 0.0f) value = 0.0f;
    else if (value > 65535.0f) value = 65535.0f;
    return static_cast<uint16_t>(value);
}

/**
 * @brief CAN数据转角度
 * @param raw CAN数据(24位)
 * @return 角度值(rad)，范围[-40, 40]
 */
float J60::CanToAngle(uint32_t raw) 
	{
    return (raw / 1048575.0f) * 80.0f - 40.0f;  // 24位数据最大值为1048575
}

/**
 * @brief 速度转CAN数据
 * @param speed 速度值(rad/s)，范围[-40, 40]
 * @return CAN数据(14位)，范围[0, 16383]
 */
uint16_t J60::SpeedToCan(float speed) 
	{
    float value = (speed + 40.0f) / 80.0f * 16383.0f + 0.5f;  // 线性映射
    if (value < 0.0f) value = 0.0f;
    else if (value > 16383.0f) value = 16383.0f;
    return static_cast<uint16_t>(value);
}

/**
 * @brief CAN数据转速度
 * @param raw CAN数据(24位)
 * @return 速度值(rad/s)，范围[-40, 40]
 */
float J60::CanToSpeed(uint32_t raw)
	{
    return (raw / 1048575.0f) * 80.0f - 40.0f;
}

/**
 * @brief 扭矩转CAN数据
 * @param torque 扭矩值(N·m)，范围[-40, 40]
 * @return CAN数据(16位)，范围[0, 65535]
 */
uint16_t J60::TorqueToCan(float torque)
{
    float value = (torque + 40.0f) / 80.0f * 65535.0f + 0.5f;
    if (value < 0.0f) value = 0.0f;
    else if (value > 65535.0f) value = 65535.0f;
    return static_cast<uint16_t>(value);
}

/**
 * @brief CAN数据转扭矩
 * @param raw CAN数据(16位)
 * @return 扭矩值(N·m)，范围[-40, 40]
 */
float J60::CanToTorque(uint16_t raw) 
	{
    return (raw / 65535.0f) * 80.0f - 40.0f;
}

/**
 * @brief 阻尼系数转CAN数据
 * @param kd 阻尼系数，范围[0, 51]
 * @return CAN数据(8位)，范围[0, 255]
 */
uint8_t J60::KdToCan(float kd) 
	{
    float value = kd / 51.0f * 255.0f + 0.5f;
    if (value < 0.0f) value = 0.0f;
    else if (value > 255.0f) value = 255.0f;
    return static_cast<uint8_t>(value);
}

/**
 * @brief 比例系数转CAN数据
 * @param kp 比例系数，范围[0, 1023]
 * @return CAN数据(10位)，范围[0, 1023]
 */
uint16_t J60::KpToCan(float kp)
	{
    float value = kp + 0.5f;  // 四舍五入
    if (value < 0.0f) value = 0.0f;
    else if (value > 1023.0f) value = 1023.0f;
    return static_cast<uint16_t>(value);
}

/**
 * @brief CAN数据转温度
 * @param raw CAN数据(7位)
 * @return 温度值(℃)，范围[-20, 200]
 */
float J60::CanToTemperature(uint8_t raw)
	{
    return (raw / 127.0f) * 220.0f - 20.0f;  // 7位数据范围[0,127]对应温度[-20,200]
}

} 
