#ifndef SPEED_PLAN_H
#define SPEED_PLAN_H
#include "stdint.h"
#include "math.h"

// 工具函数：绝对值和符号函数
inline float _tool_Abs(float value) { return fabsf(value); }  // 计算浮点数绝对值
inline float _tool_Sign(float value) { return value >= 0 ? 1.0f : -1.0f; }  // 取符号（正为1，负为-1）

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif 

/**
 * 速度规划器基类
 * 定义所有规划器的通用接口和基础属性
 */
class PlannerBase
{
public:
    // 构造函数：初始化运动约束参数
    PlannerBase(float max_accel, float max_decel, float max_vel, float deadzone)
        : max_accel_(max_accel),  // 最大加速度
          max_decel_(max_decel),  // 最大减速度（绝对值）
          max_vel_(max_vel),      // 最大速度限制
          deadzone_(deadzone)     // 位置死区（到达目标的判定阈值）
    {
        Reset();  // 初始化状态
    }
    
    virtual ~PlannerBase() = default;  // 虚析构函数，确保子类正确析构
    
    // 设置运动参数：起始/目标位置和速度
    virtual void SetMotionParams(float start_pos, float end_pos, 
                               float start_vel = 0.0f, float end_vel = 0.0f)
    {
        start_pos_ = start_pos;  // 起始位置
        end_pos_ = end_pos;      // 目标位置
        start_vel_ = start_vel;  // 起始速度
        end_vel_ = end_vel;      // 目标速度（到达时的速度）
        Reset();  // 重置规划器状态，准备新的规划
    }
    
    // 执行规划：根据当前位置和速度，计算下一时刻的目标速度（纯虚函数，子类实现）
    virtual float Plan(float current_pos, float current_vel) = 0;
    
    // 获取是否到达目标位置
    virtual bool GetArrivedFlag() const { return arrived_flag_; }
    
    // 重置规划器状态（清空中间变量，准备新规划）
    virtual void Reset()
    {
        arrived_flag_ = false;    // 未到达目标
        is_initialized_ = false;  // 未初始化
        pos_last_ = 0.0f;         // 上一时刻位置（预留）
        phase_time_ = 0.0f;       // 当前阶段的累计时间
    }

protected:
    // 检查是否到达目标位置（在死区内即判定为到达）
    virtual bool CheckArrived(float current_pos) const
    {
        return _tool_Abs(current_pos - end_pos_) < deadzone_;
    }

protected:
    // 运动参数
    float start_pos_;    // 起始位置
    float end_pos_;      // 目标位置
    float start_vel_;    // 起始速度
    float end_vel_;      // 目标速度（到达时的速度）
    
    // 约束参数
    float max_accel_;    // 最大加速度（正值）
    float max_decel_;    // 最大减速度（正值，实际减速时取负）
    float max_vel_;      // 最大速度限制
    float deadzone_;     // 位置死区（判定到达的阈值）
    
    // 状态变量
    bool arrived_flag_;  // 到达目标标志
    bool is_initialized_;// 初始化标志（规划参数是否计算完成）
    float pos_last_;     // 上一时刻位置（用于计算位移等）
    float phase_time_;   // 当前运动阶段的累计时间
    float direction_;    // 运动方向（1.0为正方向，-1.0为负方向）
    float distance_total_; // 总运动距离（绝对值）
};

/**
 * 梯形速度规划器
 * 速度曲线呈梯形：加速→匀速→减速
 */
class TrapePlanner : public PlannerBase
{
public:
    // 构造函数：继承基类的约束参数
    TrapePlanner(float max_accel, float max_decel, float max_vel, float deadzone)
        : PlannerBase(max_accel, max_decel, max_vel, deadzone) {}
    
    // 执行规划（实现基类纯虚函数）
    virtual float Plan(float current_pos, float current_vel) override;
    // 重置状态（重写基类方法）
    virtual void Reset() override;

private:
    // 初始化规划参数（计算各阶段距离、巡航速度等）
    void Initialize();
    
    // 运动阶段枚举
    enum Phase { 
        PHASE_ACCEL,   // 加速阶段
        PHASE_CRUISE,  // 匀速阶段
        PHASE_DECEL    // 减速阶段
    } current_phase_;  // 当前所处阶段
    
    // 阶段参数
    float distance_accel_;  // 加速阶段需要的距离
    float distance_cruise_; // 匀速阶段需要的距离
    float distance_decel_;  // 减速阶段需要的距离
    float vel_cruise_;      // 巡航速度（加速后的稳定速度）
};

/**
 * S曲线速度规划器
 * 速度曲线更平滑：通过控制加加速度（jerk）实现加速度的连续变化
 * 阶段：加加速→匀加速→减加速→匀速→加减速→匀减速→减减速
 */
class SPlanner : public PlannerBase
{
public:
    // 构造函数：比基类多一个最大加加速度参数
    SPlanner(float max_accel, float max_decel, float max_vel, float max_jerk, float deadzone)
        : PlannerBase(max_accel, max_decel, max_vel, deadzone), 
          max_jerk_(max_jerk)  // 最大加加速度（加速度的变化率）
    {}
    
    // 执行规划（实现基类纯虚函数）
    virtual float Plan(float current_pos, float current_vel) override;
    // 重置状态（重写基类方法）
    virtual void Reset() override;

private:
    // 初始化规划参数（计算各阶段时间）
    void Initialize();
    // 计算各阶段的时间参数（加加速、匀加速等阶段的持续时间）
    void CalculatePhaseTimes();
    // 切换到下一运动阶段
    void NextPhase();
    
    // 运动阶段枚举（S曲线比梯形多4个阶段，实现加速度连续）
    enum Phase {
        PHASE_JERK_UP,       // 加加速阶段（加速度增大）
        PHASE_ACCEL_CONST,   // 匀加速阶段（加速度不变）
        PHASE_JERK_DOWN,     // 减加速阶段（加速度减小到0）
        PHASE_CRUISE,        // 匀速阶段（加速度为0）
        PHASE_JERK_NEG,      // 加减速阶段（负向加速度增大）
        PHASE_DECEL_CONST,   // 匀减速阶段（负向加速度不变）
        PHASE_JERK_END,      // 减减速阶段（负向加速度减小到0）
        PHASE_FINISHED       // 完成阶段
    } current_phase_;  // 当前所处阶段
    
    // 阶段时间参数（各阶段的持续时间）
    float t_jerk_up_;      // 加加速阶段时间
    float t_accel_const_;  // 匀加速阶段时间
    float t_jerk_down_;    // 减加速阶段时间
    float t_cruise_;       // 匀速阶段时间
    float t_jerk_neg_;     // 加减速阶段时间
    float t_decel_const_;  // 匀减速阶段时间
    float t_jerk_end_;     // 减减速阶段时间
    
    // 运动状态变量
    float current_accel_;  // 当前加速度
    float current_jerk_;   // 当前加加速度（控制加速度变化）
    float max_jerk_;       // 最大加加速度限制
};

/**
 * 多项式速度规划器
 * 通过5次多项式曲线描述位置随时间的变化，确保速度、加速度连续
 */
class PolynomialPlanner : public PlannerBase
{
public:
    // 构造函数：继承基类的约束参数
    PolynomialPlanner(float max_accel, float max_decel, float max_vel, float deadzone)
        : PlannerBase(max_accel, max_decel, max_vel, deadzone) {}
    
    // 执行规划（实现基类纯虚函数）
    virtual float Plan(float current_pos, float current_vel) override;
    // 重置状态（重写基类方法）
    virtual void Reset() override;

private:
    // 初始化规划参数（计算多项式系数、总时间）
    void Initialize();
    // 计算总运动时间（基于距离和约束）
    float CalculateTotalTime();
    
    // 5次多项式系数：位置函数为 s(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    float a0_, a1_, a2_;  // 低次项系数
    float a3_, a4_, a5_;  // 高次项系数
    float total_time_;    // 总运动时间（从开始到到达目标的时间）
    float time_elapsed_;  // 已流逝的时间（从规划开始计算）
};

#endif // !SPEED_PLAN_H