#pragma once
#include "RC_can.h"   
#include "RC_timer.h"      
#include "RC_motor.h"     
#include <stdint.h>         
#include <math.h>           
#include "RC_pid.h"

#ifdef __cplusplus
namespace j60 {
/**
 * @brief CAN命令枚举
 */
enum class CanCmd : uint8_t {
    MOTOR_DISABLE    = 1,    // 禁用电机
    MOTOR_ENABLE     = 2,    // 使能电机
    MOTOR_CONTROL    = 4,    // 电机控制指令
    SET_CAN_TIMEOUT  = 9,    // 设置CAN通信超时时间
    SET_BANDWIDTH    = 10,   // 设置控制带宽
    ERROR_RESET      = 17,   // 重置错误状态
    GET_STATUSWORD   = 23,   // 获取状态字
    GET_CONFIG       = 24,   // 获取配置参数

};

/**
 * @brief 控制模式枚举
 */
enum class ControlMode : uint8_t {
    POSITION    = 0,    // 位置控制模式
    SPEED       = 1,    // 速度控制模式
    TORQUE      = 2,    // 扭矩控制模式
    DAMPING     = 3,    // 阻尼控制模式
    ZERO_TORQUE = 4,    // 零扭矩模式
    HYBRID      = 5     // 混合控制模式
};

/**
 * @brief J60关节控制器类
 */
class J60 : public can::CanHandler, public tim::TimHandler, public motor::Motor {
public:
    J60(uint8_t id_, can::Can& can_, tim::Tim& tim_);
    
    void CanHandler_Register() override;
    void Tim_It_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t* rx_data) override;
    void Can_Tx_Process() override;

    // 控制接口
    void Enable();                          // 使能电机
    void Disable();                         // 禁用电机
    void Calibrate();                       // 校准电机
    void SetZero();                         // 设置零点
    void SaveConfig();                      // 保存配置
    void PositionControl(float target_angle, float kp, float kd);  // 位置控制
    void SpeedControl(float target_speed, float kd);                // 速度控制
    void TorqueControl(float target_torque);                        // 扭矩控制
    void DampingControl(float kd);                                  // 阻尼控制
    void ZeroTorqueControl();                                       // 零扭矩控制
    void HybridControl(float p_des, float v_des, float tau_ff, float kp, float kd);  // 混合控制

    float GetAngle() const { return angle_; }          // 获取当前角度(rad)
    float GetSpeed() const { return speed_; }          // 获取当前速度(rad/s)
    float GetTorque() const { return torque_; }        // 获取当前扭矩(N·m)
    float GetMotorTemp() const { return motor_temp_; } // 获取电机温度(℃)
    float GetDriverTemp() const { return driver_temp_; } // 获取驱动器温度(℃)
    bool IsEnabled() const { return is_enabled_; }     // 获取使能状态
  
    bool IsOnline() const { return (timer::Timer::Get_DeltaTime(last_can_time_) < 10000); }  

private:
    uint8_t id_;                  // 关节ID
    CanCmd current_cmd_;          // 当前CAN命令
    ControlMode ctrl_mode_;       // 控制模式

    uint8_t tx_buffer_[8];        // 发送数据缓冲区(8字节)
    uint8_t tx_len_;              // 发送数据长度（按命令动态设置）
    uint16_t tx_frame_dx_;        // 发送帧索引
    uint8_t rx_filter_id_base_;   // 接收ID基础

    float angle_;                 // 角度(rad)
    float speed_;                 // 速度(rad/s)
    float torque_;                // 扭矩(N·m)
    float motor_temp_;            // 电机温度(℃)
    float driver_temp_;           // 驱动器温度(℃)
    uint16_t status_;             // 状态字（手册5.8）
    uint32_t last_can_time_;      // 最后接收时间(us)
    bool is_enabled_;             // 使能状态
    uint16_t driver_version_;     // 驱动器版本

    float target_angle_;          // 目标角度(rad)
    float target_speed_;          // 目标速度(rad/s)
    float target_torque_;         // 目标扭矩(N·m)
    float kp_;                    // 位置环比例增益0~1023
    float kd_;                    // 速度环阻尼增益0~51
    float tau_ff_;                // 前馈扭矩(N·m)

    // 反馈标志
    bool enable_ack_;             // 使能确认标志
    bool disable_ack_;            // 禁用确认标志
    bool calib_done_;             // 校准完成标志
    int16_t calib_comp_;          // 校准补偿

};

}  // namespace j60
#endif
