#pragma once
#include "RC_can.h"         // CAN通信接口
#include "RC_timer.h"       // 定时器接口
#include "RC_motor.h"       // 电机基类
#include <stdint.h>         // 标准整数类型定义(替换C++的<cstdint>)
#include <math.h>           // 数学函数(替换C++的<cmath>)

// 引入PID控制器头文件(根据实际框架修改)
#include "RC_pid.h"

#ifdef __cplusplus
namespace j60 {
/**
 * @brief CAN命令枚举，定义J60关节支持的所有CAN指令
 */
enum class CanCmd : uint8_t
	{
    MOTOR_DISABLE    = 1,    // 禁用电机
    MOTOR_ENABLE     = 2,    // 使能电机
    MOTOR_CONTROL    = 4,    // 电机控制指令
    SET_CAN_TIMEOUT  = 9,    // 设置CAN通信超时时间
    SET_BANDWIDTH    = 10,   // 设置控制带宽
    SAVE_CONFIG      = 16,   // 保存配置参数
    ERROR_RESET      = 17,   // 重置错误状态
    GET_STATUSWORD   = 23,   // 获取状态字
    GET_CONFIG       = 24,   // 获取配置参数
    CALIBRATE        = 25,   // 校准电机
    SET_ZERO         = 26    // 设置零点位置
};

/**
 * @brief 控制模式枚举，定义J60支持的控制方式
 */
enum class ControlMode : uint8_t {
    POSITION    = 0,    // 位置控制模式
    SPEED       = 1,    // 速度控制模式
    TORQUE      = 2,    // 扭矩控制模式
    DAMPING     = 3,    // 阻尼控制模式
    ZERO_TORQUE = 4,    // 零扭矩模式
    HYBRID      = 5     // 混合控制模式(位置+速度+扭矩前馈)
};

/**
 * @brief J60关节控制器类，继承自CAN处理类、定时器处理类和电机基类
 */
class J60 : public can::CanHandler, public tim::TimHandler, public motor::Motor 
	{
public:
    /**
     * @brief 构造函数
     * @param id_ 关节ID(1-15)
     * @param can_ CAN通信对象引用
     * @param tim_ 定时器对象引用
     */
    J60(uint8_t id_, can::Can& can_, tim::Tim& tim_);
    
    /**
     * @brief 注册CAN处理函数(重写基类方法)
     */
    void CanHandler_Register() override;
    
    /**
     * @brief 定时器中断处理函数(重写基类方法)
     * 用于周期性执行控制算法
     */
    void Tim_It_Process() override;
    
    /**
     * @brief CAN发送处理函数(重写基类方法)
     * 用于打包并发送控制指令
     */
    void Can_Tx_Process() override;
    
    /**
     * @brief CAN接收中断处理函数(重写基类方法)
     * 用于解析接收到的关节状态数据
     * @param rx_data 接收数据缓冲区
     */
    void Can_Rx_It_Process(uint8_t* rx_data) override;

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

    // 状态获取接口
    float GetAngle() const { return angle_; }          // 获取当前角度(rad)
    float GetSpeed() const { return speed_; }          // 获取当前速度(rad/s)
    float GetTorque() const { return torque_; }        // 获取当前扭矩(N·m)
    float GetMotorTemp() const { return motor_temp_; } // 获取电机温度(℃)
    bool IsEnabled() const { return is_enabled_; }     // 获取使能状态
    bool IsOnline() const { return (timer::Timer::Get_DeltaTime(last_can_time_) < 500000); }  // 判断是否在线(500ms超时)

private:
    // 参数映射函数：物理量与CAN通信数据的转换
    uint16_t AngleToCan(float angle);       // 角度转CAN数据
    float CanToAngle(uint32_t raw);         // CAN数据转角度
    uint16_t SpeedToCan(float speed);       // 速度转CAN数据
    float CanToSpeed(uint32_t raw);         // CAN数据转速度
    uint16_t TorqueToCan(float torque);     // 扭矩转CAN数据
    float CanToTorque(uint16_t raw);        // CAN数据转扭矩
    uint8_t KdToCan(float kd);              // 阻尼系数转CAN数据
    uint16_t KpToCan(float kp);             // 比例系数转CAN数据
    float CanToTemperature(uint8_t raw);    // CAN数据转温度

    uint8_t id_;                  // 关节ID(1-15)
    CanCmd current_cmd_;          // 当前CAN命令
    ControlMode ctrl_mode_;       // 控制模式

    // 发送缓冲区
    uint8_t tx_buffer_[8];        // 发送数据缓冲区(8字节)
    uint8_t tx_len_;              // 发送数据长度
    uint32_t tx_id_;              // 发送CAN ID
    uint16_t tx_frame_dx_;        // 发送帧索引

    // 状态变量
    float angle_;                 // 角度(rad)
    float speed_;                 // 速度(rad/s)
    float torque_;                // 扭矩(N·m)
    float motor_temp_;            // 电机温度(℃)
    float driver_temp_;           // 驱动器温度(℃)
    uint16_t status_;             // 状态字
    uint32_t last_can_time_;      // 最后接收时间(us)
    bool is_enabled_;             // 使能状态

    // 控制参数
    float target_angle_;          // 目标角度(rad)
    float target_speed_;          // 目标速度(rad/s)
    float target_torque_;         // 目标扭矩(N·m)
    float kp_;                    // 位置环比例增益
    float kd_;                    // 速度环阻尼增益
    float tau_ff_;                // 前馈扭矩(N·m)

    // 反馈标志
    bool enable_ack_;             // 使能确认标志
    bool disable_ack_;            // 禁用确认标志
    bool calib_done_;             // 校准完成标志
    int16_t calib_comp_;          // 校准补偿值

    pid::Pid pid_pos_;  // 位置环PID控制器
    pid::Pid pid_spd_;  // 速度环PID控制器
};

} 
#endif
