#include "RC_j606.h"
#include <string.h> // 内存操作函数
#include <cmath>    // 用于fabsf等数学函数
#include <cstdint>  // 用于int32_t, uint32_t, uint64_t
#include <algorithm> // 用于 std::clamp, std::min/max
#include <iostream>  // 用于std::round

namespace j60 {

// 常量定义
const int32_t SIGNED_MAX_20_BIT = 524287;    // 2^19 - 1 (20位有符号最大值)
const int32_t SIGNED_MAX_16_BIT = 32767;     // 2^15 - 1 (16位有符号最大值)

// 14位速度指令相关的常量
const uint16_t V_MAX_RAW_14_BIT = 16383;     // 14位无符号最大值: 2^14 - 1
const uint16_t V_MOD_14_BIT = 16384;         // 14位模数: 2^14

// 20位数据的零点偏移 (2^19)，用于 Rx 无符号映射
const uint32_t OFFSET_20_BIT = 524288;       // 0x80000 
// 16位数据的零点偏移 (2^15)，用于 Rx 无符号映射
const uint32_t OFFSET_16_BIT = 32768;        // 0x8000 

// 最大物理速度范围 (rad/s)
const float MAX_PHYSICAL_SPEED = 40.0f;      
// 最大协议扭矩范围 (N·m)
const float MAX_TORQUE_PROTOCOL_NM = 40.0f;

// 20位有符号数到物理值的缩放分母 (524287.0f)
const float SPEED_SCALE_MAX_RAW = (float)SIGNED_MAX_20_BIT; 
// 16位有符号数到物理值的缩放分母 (32767.0f)
const float TORQUE_SCALE_MAX_RAW = (float)SIGNED_MAX_16_BIT;
// ***********************

// 构造函数：初始化关节ID、通讯参数、状态变量
J60::J60(uint8_t id_, can::Can& can_, tim::Tim& tim_)
    : CanHandler(can_), TimHandler(tim_), Motor() {
    // 关节ID校验（手册5章：ID范围0~15）
    if (id_ > 15) {
        Error_Handler();
    } else {
        this->id_ = id_;
    }

    CanHandler_Register();

    // 初始化控制参数（默认禁用电机，位置控制模式）
    current_cmd_ = CanCmd::MOTOR_DISABLE;
    ctrl_mode_ = ControlMode::POSITION;
    tx_len_ = 0;
    memset(tx_buffer_, 0, 8);

    // 初始化状态变量（默认值均为0或false）
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
    driver_version_ = 0;
    last_angle_ = 0.0f;
    round_count_ = 0;
    total_angle_ = 0.0f;
    angle_offset_ = 0.0f;
    offset_initialized_ = false;
    angle_rad_ = 0;
    raw_angle_ = 0;
    prev_raw_angle_ = 0;
    rotation_count_ = 0;
}

// CAN Handler注册：初始化发送帧、接收ID过滤
void J60::CanHandler_Register() {
    // CAN帧类型：标准帧（手册5章：CAN使用标准帧格式）
    can_frame_type = can::FRAME_STD;
    // 接收ID基础：关节ID + 0x10（手册5章：接收ID=自身ID+0x10）
    rx_filter_id_base_ = (id_ + 0x10);

    // 注册发送帧到CAN设备
    can->tx_frame_num++;
    tx_frame_dx_ = can->tx_frame_num - 1;

    // 初始化发送帧参数（ID和DLC动态生成）
    can->tx_frame_list[tx_frame_dx_].frame_type = can_frame_type;
    can->tx_frame_list[tx_frame_dx_].id = 0;
    can->tx_frame_list[tx_frame_dx_].dlc = 0;
    
    // 挂载当前设备到发送帧
    can->tx_frame_list[tx_frame_dx_].hd_num = 1;
    can->tx_frame_list[tx_frame_dx_].hd_dx[0] = hd_list_dx;
}

// 定时器中断处理：超时保护、温度保护、速度超限保护
void J60::Tim_It_Process() {
    // 1. 速度超限保护
    if (is_enabled_ && fabsf(speed_) > (MAX_PHYSICAL_SPEED * 1.05f)) {
        Disable();
        return;
    }

    // 2. 温度保护
    if (is_enabled_) {
        if (motor_temp_ > 125.0f || driver_temp_ > 120.0f) {
            Disable();
        }
    }
}


void J60::Can_Tx_Process() {
    // ---------------------- 物理量映射 (参考手册5.3章) ----------------------
    
    // 角度 p 映射：[-40,40]rad (80 rad 范围) -> [0,65535] (16位) (无符号映射)
						// (1) 位置 p : [-40,40]rad → [0,65535]
				float p_val = (target_angle_ + 40.0f) / 80.0f * 65535.0f;
				p_val = std::clamp(p_val, 0.0f, 65535.0f);
				uint16_t p = static_cast<uint16_t>(p_val);
				
				// (2) 速度 v : [-40,40]rad/s → [0,16383] (unsigned midpoint)
				float v_val = (target_speed_ )/ (MAX_PHYSICAL_SPEED) * (V_MAX_RAW_14_BIT / 2.0f)
										+ (V_MAX_RAW_14_BIT / 2.0f);
				v_val = std::clamp(v_val, 0.0f, (float)V_MAX_RAW_14_BIT);
				uint16_t v = static_cast<uint16_t>(std::round(v_val));
				
				// (3) KP : [0,1023]
				float kp_val = std::clamp(kp_, 0.0f, 1023.0f);
				uint16_t kp = static_cast<uint16_t>(kp_val);
				
				// (4) KD : [0,51] → [0,255]
				float kd_val = (kd_ / 51.0f) * 255.0f;
				kd_val = std::clamp(kd_val, 0.0f, 255.0f);
				uint8_t kd = static_cast<uint8_t>(kd_val);
				
				// (5) 扭矩 t : [-40,40]N·m → [0,65535]
				float t_val = (tau_ff_ + 40.0f) / 80.0f * 65535.0f;
				t_val = std::clamp(t_val, 0.0f, 65535.0f);
				uint16_t t = static_cast<uint16_t>(t_val);
    // ---------------------- 按模式打包 ----------------------
    switch (current_cmd_) {
        // 无数据命令（使能、失能、查询等）
        case CanCmd::MOTOR_ENABLE:
        case CanCmd::MOTOR_DISABLE:
        case CanCmd::ERROR_RESET:
        case CanCmd::GET_STATUSWORD:
        case CanCmd::GET_CONFIG:
            tx_len_ = 0;
            break;

        // 核心控制命令
        case CanCmd::MOTOR_CONTROL:
        {
            // 只有使能确认后才发送控制指令
            if (!enable_ack_) {
                return;
            }
            
            // 根据控制模式清零不需要的参数
            switch (ctrl_mode_) {
            
                case ControlMode::POSITION:
                    // 仅使用 p, kp, kd
                    v = 0; t = 0;
                    break;
                case ControlMode::SPEED:
                    // 仅使用 v, kd
                    p = 0; t = 0; kp = 0;
                    break;
                case ControlMode::TORQUE:
                    // 仅使用 t
                    p = 0; v = 0; kp = 0; kd = 0;
                    break;
                case ControlMode::DAMPING:
                    // 仅使用 kd (p, v, t, kp 均置零)
                    p = 0; v = 0; kp = 0; t = 0;
                    break;
                case ControlMode::ZERO_TORQUE:
                    p = v = kp = kd = t = 0;
                    break;
                case ControlMode::HYBRID:
                default:
                    // 所有参数都可能被使用
                    break;
            }

            // ---------------------- 按手册拼接位域 (8 字节) ----------------------
            // 保持 Little Endian 拼接方式
            tx_buffer_[0] = p & 0xFF;                                       // 角度 p 低8位 (Bit 0-7)
            tx_buffer_[1] = (p >> 8) & 0xFF;                                // 角度 p 高8位 (Bit 8-15)
            tx_buffer_[2] = v & 0xFF;                                       // 速度 v 低8位 (Bit 0-7)
            
            // 速度 v 高6位 (Bit 8-13) | KP 低2位 (Bit 0-1)
            tx_buffer_[3] = ((v >> 8) & 0x3F) | ((kp & 0x03) << 6);          // v[8-13] | kp[0-1]
            
            tx_buffer_[4] = (kp >> 2) & 0xFF;                               // KP 高8位 (Bit 2-9)
            tx_buffer_[5] = kd;                                             // KD (8位)
            tx_buffer_[6] = t & 0xFF;                                       // 扭矩 t 低8位 (Bit 0-7)
            tx_buffer_[7] = (t >> 8) & 0xFF;                                // 扭矩 t 高8位 (Bit 8-15)
            tx_len_ = 8;
            break;
        } // end of MOTOR_CONTROL case
    }

    // ---------------------- 生成CAN ID ----------------------
    uint32_t can_id = (static_cast<uint32_t>(current_cmd_) << 5) | (id_ & 0x0F);
    can->tx_frame_list[tx_frame_dx_].id = can_id;
    can->tx_frame_list[tx_frame_dx_].dlc = tx_len_;
    
    // ---------------------- 数据拷贝 ----------------------
    if (tx_len_ > 0 && tx_len_ <= 8) {
        memcpy(can->tx_frame_list[tx_frame_dx_].data, tx_buffer_, tx_len_);
    }
}


// CAN接收中断处理
void J60::Can_Rx_It_Process(uint32_t rx_id_, uint8_t* rx_data) {
    // 1. 解析CAN ID并验证
    uint8_t received_joint_id_bits = rx_id_ & 0x0F;
    uint8_t return_flag = (rx_id_ >> 4) & 0x01;
    uint8_t cmd_index = (rx_id_ >> 5) & 0x3F;
    CanCmd cmd = static_cast<CanCmd>(cmd_index);

    if (received_joint_id_bits != this->id_ || return_flag != 1) {
        return;
    }

    // 2. 更新时间
    last_can_time_ = timer::Timer::Get_TimeStamp();

    // 3. 解析数据
    switch (cmd) {
        
        case CanCmd::MOTOR_ENABLE:
            enable_ack_ = (rx_data[0] == 0);
            is_enabled_ = enable_ack_;
            break;

        case CanCmd::MOTOR_DISABLE:
            disable_ack_ = (rx_data[0] == 0);
            is_enabled_ = !disable_ack_;
            break;

        case CanCmd::GET_STATUSWORD:
            status_ = (rx_data[1] << 8) | rx_data[0];
            break;

        case CanCmd::MOTOR_CONTROL: {
            
            // 组装 8 字节数据为 uint64_t，Little Endian (Bit 0 starts at rx_data[0])
            uint64_t raw_data_64 = 0;
            for (int i = 0; i < 8; i++) {
                raw_data_64 |= ((uint64_t)rx_data[i] << (i * 8)); 
            }
            
            // --- ① 角度反馈 (p)：Bit0~19 ---
            uint32_t raw_angle = (uint32_t)((raw_data_64 >> 0) & 0xFFFFF);
            const float P_RANGE = 80.0f;
            float current_angle_rad = (static_cast<float>(raw_angle) / 1048575.0f) * P_RANGE - 40.0f;
            
            if (!offset_initialized_) {
                angle_offset_ = current_angle_rad;
                offset_initialized_ = true;
            }
            
            if (offset_initialized_) {
                float angle_diff = current_angle_rad - last_angle_;
                if (angle_diff > (P_RANGE * 0.9f)) {
                    rotation_count_--;
                }
                else if (angle_diff < -(P_RANGE * 0.9f)) {
                    rotation_count_++;
                }
            }
            
            last_angle_ = current_angle_rad;
            total_angle_ = (current_angle_rad - angle_offset_) + rotation_count_ * P_RANGE;
            angle_ = total_angle_;
            
            
												// --- 速度反馈 (Bit20~39) ---
						uint32_t raw_speed = (uint32_t)((raw_data_64 >> 20) & 0xFFFFF);
						// 实际为反向映射：大raw → 负速度
						speed_ = 40.0f - ((float)raw_speed / 1048575.0f) * 80.0f;
						if (fabsf(speed_) < 0.05f) speed_ = 0.0f;
						
						// --- 扭矩反馈 (Bit40~55) ---
						uint16_t raw_torque = (uint16_t)((raw_data_64 >> 40) & 0xFFFF);
						// 扭矩方向与速度一致（同样是反向线性）
						torque_ = 40.0f - ((float)raw_torque / 65535.0f) * 80.0f;
						if (fabsf(torque_) < 0.1f) torque_ = 0.0f;


            // --- ④ 温度反馈：Bit56~63 ---
            uint8_t raw_temp_byte = (uint8_t)((raw_data_64 >> 56) & 0xFF);
            uint8_t temp_flag = raw_temp_byte & 0x01;
            uint8_t raw_temp = (raw_temp_byte >> 1) & 0x7F;
            
            float temp = (static_cast<float>(raw_temp) / 127.0f) * 220.0f - 20.0f;
            
            if (temp >= -20.0f && temp <= 200.0f) {
                (temp_flag == 0) ? (driver_temp_ = temp) : (motor_temp_ = temp);
            }
            break;
        }

        default:
            break;
    }
}


// 控制接口：使能电机（带重发防护）
void J60::Enable() {
    // 只有在未使能且尚未收到使能确认时，才发送使能命令
    if (!is_enabled_ && !enable_ack_) {
        current_cmd_ = CanCmd::MOTOR_ENABLE;
    }
}

// 控制接口：禁用电机
void J60::Disable() {
    if (is_enabled_) {
        current_cmd_ = CanCmd::MOTOR_DISABLE;
        enable_ack_ = false;  // 重置使能确认标志，下次Enable需要重新确认
    }
}

// 控制接口：位置控制（输入角度单位：deg）
void J60::PositionControl(float target_angle_deg, float kp, float kd) {
    if (!is_enabled_) return;

    ctrl_mode_ = ControlMode::POSITION;
    // 1. 单位转换：deg -> rad，并限幅（[-40,40]rad）
    target_angle_ = target_angle_deg * DEG_TO_RAD;
    target_angle_ = std::clamp(target_angle_, -40.0f, 40.0f);

    // 2. KP/KD限幅（KP[0,1023]，KD[0,51]）
    kp_ = std::clamp(kp, 0.0f, 1023.0f);
    kd_ = std::clamp(kd, 0.0f, 51.0f);
    
    // 3. 清零速度和扭矩前馈
    target_speed_ = 0.0f;
    tau_ff_ = 0.0f;

    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

// 控制接口：速度控制（输入速度单位：deg/s）
void J60::SpeedControl(float target_speed_deg, float kd) {
    if (!is_enabled_) return;

    ctrl_mode_ = ControlMode::SPEED;
    // 1. 单位转换：deg/s -> rad/s，并限幅（[-40,40]rad/s）
    target_speed_ = (target_speed_deg + 230) * DEG_TO_RAD;
    target_speed_ = std::clamp(target_speed_, -40.0f, 40.0f);

    // 2. KD限幅（KD[0,51]）
    kd_ = std::clamp(kd, 0.0f, 51.0f);

    // 3. 清零位置和扭矩前馈
    target_angle_ = 0.0f;
    kp_ = 0.0f;
    tau_ff_ = 0.0f;

    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

// 控制接口：扭矩控制
void J60::TorqueControl(float target_torque) {
    if (!is_enabled_) return;

    ctrl_mode_ = ControlMode::TORQUE;
    // 扭矩限幅（[-40,40]N·m）
    tau_ff_ = std::clamp(target_torque, -MAX_TORQUE_PROTOCOL_NM, MAX_TORQUE_PROTOCOL_NM);

    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

// 控制接口：阻尼控制
void J60::DampingControl(float kd) {
    if (!is_enabled_) return;

    ctrl_mode_ = ControlMode::DAMPING;
    // KD限幅（KD[0,51]）
    kd_ = std::clamp(kd, 0.0f, 51.0f);
    
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

// 控制接口：零扭矩控制
void J60::ZeroTorqueControl() {
    if (!is_enabled_) return;

    ctrl_mode_ = ControlMode::ZERO_TORQUE;
    
    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

// 控制接口：混合控制（输入角度/速度单位：deg、deg/s）
void J60::HybridControl(float p_des_deg, float v_des_deg, float tau_ff, float kp, float kd) {
    if (!is_enabled_) return;

    ctrl_mode_ = ControlMode::HYBRID;
    // 1. 角度转换+限幅
    target_angle_ = p_des_deg * DEG_TO_RAD;
    target_angle_ = std::clamp(target_angle_, -40.0f, 40.0f);

    // 2. 速度转换+限幅
    target_speed_ = (v_des_deg + 230)* DEG_TO_RAD;
    target_speed_ = std::clamp(target_speed_, -40.0f, 40.0f);

    // 3. 前馈扭矩+KP/KD限幅
    tau_ff_ = std::clamp(tau_ff, -MAX_TORQUE_PROTOCOL_NM, MAX_TORQUE_PROTOCOL_NM);
    kp_ = std::clamp(kp, 0.0f, 1023.0f);
    kd_ = std::clamp(kd, 0.0f, 51.0f);

    current_cmd_ = CanCmd::MOTOR_CONTROL;
}

} // namespace j60
