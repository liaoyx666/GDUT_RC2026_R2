#include "speed_plan.h"
#include "stdlib.h"

/**
 * 梯形速度规划器实现
 */
// 重置规划器状态
void TrapePlanner::Reset()
{
    PlannerBase::Reset();  // 调用基类重置
    current_phase_ = PHASE_ACCEL;  // 初始阶段为加速
    distance_accel_ = 0.0f;        // 加速距离清零
    distance_cruise_ = 0.0f;       // 匀速距离清零
    distance_decel_ = 0.0f;        // 减速距离清零
    vel_cruise_ = 0.0f;            // 巡航速度清零
}

// 初始化规划参数（计算各阶段距离和巡航速度）
void TrapePlanner::Initialize()
{
    // 计算总距离（绝对值）和运动方向
    distance_total_ = _tool_Abs(end_pos_ - start_pos_);  // 总运动距离
    direction_ = _tool_Sign(end_pos_ - start_pos_);      // 方向（1或-1）
    
    // 如果目标位置与起始位置非常接近（在死区内），直接判定为到达
    if (distance_total_ < deadzone_)
    {
        arrived_flag_ = true;
        is_initialized_ = true;
        return;
    }
    
    // 计算加速阶段所需距离（从起始速度加速到最大速度）
    // 公式：v² - v0² = 2*a*s → s = (v² - v0²)/(2*a)
    distance_accel_ = (max_vel_ * max_vel_ - start_vel_ * start_vel_) / (2 * max_accel_);
    distance_accel_ = (distance_accel_ < 0) ? 0 : distance_accel_;  // 确保非负
    
    // 计算减速阶段所需距离（从最大速度减速到目标速度）
    distance_decel_ = (max_vel_ * max_vel_ - end_vel_ * end_vel_) / (2 * max_decel_);
    distance_decel_ = (distance_decel_ < 0) ? 0 : distance_decel_;  // 确保非负
    
    // 判断是否能达到最大速度：加速+减速距离是否超过总距离
    if (distance_accel_ + distance_decel_ > distance_total_)
    {
        // 无法达到最大速度，需要计算实际能达到的巡航速度（通过二次方程）
        // 推导：加速距离s1=(v² - v0²)/(2a)，减速距离s2=(v² - ve²)/(2d)
        // s1 + s2 = 总距离 → 整理为 a*v² + b*v + c = 0
        float a = (1.0f / (2 * max_accel_)) + (1.0f / (2 * max_decel_));
        float b = start_vel_ / max_accel_ - end_vel_ / max_decel_;
        float c = (start_vel_ * start_vel_) / (2 * max_accel_) - 
                 (end_vel_ * end_vel_) / (2 * max_decel_) - distance_total_;
        
        // 解二次方程：判别式 b²-4ac
        float discriminant = b*b - 4*a*c;
        if (discriminant < 0) discriminant = 0;  // 避免开方出错
        
        // 取合理的解（负号保证速度为正）
        vel_cruise_ = (-b - sqrtf(discriminant)) / (2*a);
        
        // 限制巡航速度范围（不超过最大速度，不小于目标速度）
        if (vel_cruise_ > max_vel_) vel_cruise_ = max_vel_;
        if (vel_cruise_ < _tool_Abs(end_vel_)) vel_cruise_ = _tool_Abs(end_vel_);
        
        // 根据实际巡航速度重新计算加速和减速距离
        distance_accel_ = (vel_cruise_ * vel_cruise_ - start_vel_ * start_vel_) / (2 * max_accel_);
        distance_decel_ = (vel_cruise_ * vel_cruise_ - end_vel_ * end_vel_) / (2 * max_decel_);
    }
    else
    {
        // 可以达到最大速度，巡航速度即为最大速度
        vel_cruise_ = max_vel_;
    }
    
    // 计算匀速阶段距离（总距离 - 加速 - 减速）
    distance_cruise_ = distance_total_ - distance_accel_ - distance_decel_;
    if (distance_cruise_ < 0) distance_cruise_ = 0;  // 确保非负
    
    is_initialized_ = true;  // 初始化完成
}

// 执行规划：根据当前位置计算目标速度
float TrapePlanner::Plan(float current_pos, float current_vel)
{
    // 检查是否到达目标，到达则返回目标速度
    if (CheckArrived(current_pos))
    {
        arrived_flag_ = true;
        return end_vel_ * direction_;  // 应用方向
    }
    arrived_flag_ = false;  // 未到达
    
    // 未初始化则先初始化
    if (!is_initialized_)
    {
        Initialize();
        if (arrived_flag_)  // 初始化时发现已到达
            return end_vel_ * direction_;
    }
    
    // 计算已行驶的距离（从起始位置到当前位置）
    float distance_traveled = _tool_Abs(current_pos - start_pos_);
    
    // 梯形速度规划逻辑：根据已行驶距离判断当前阶段
    float vel_output = 0.0f;
    
    if (distance_traveled < distance_accel_)
    {
        // 加速阶段：v² = v0² + 2*a*s（s为已行驶距离）
        current_phase_ = PHASE_ACCEL;
        vel_output = sqrtf(start_vel_ * start_vel_ + 2 * max_accel_ * distance_traveled);
        if (vel_output > vel_cruise_) vel_output = vel_cruise_;  // 不超过巡航速度
    }
    else if (distance_traveled < distance_accel_ + distance_cruise_)
    {
        // 匀速阶段：保持巡航速度
        current_phase_ = PHASE_CRUISE;
        vel_output = vel_cruise_;
    }
    else
    {
        // 减速阶段：v² = v_cruise² - 2*d*s（s为减速阶段已行驶距离）
        current_phase_ = PHASE_DECEL;
        float distance_decel_traveled = distance_traveled - distance_accel_ - distance_cruise_;
        vel_output = sqrtf(vel_cruise_ * vel_cruise_ - 2 * max_decel_ * distance_decel_traveled);
        if (vel_output < _tool_Abs(end_vel_)) vel_output = _tool_Abs(end_vel_);  // 不低于目标速度
    }
    
    // 应用运动方向（正负号）
    vel_output *= direction_;
    
    return vel_output;
}

/**
 * S曲线速度规划器实现
 */
// 重置规划器状态
void SPlanner::Reset()
{
    PlannerBase::Reset();  // 调用基类重置
    current_phase_ = PHASE_JERK_UP;  // 初始阶段为加加速
    current_accel_ = 0.0f;            // 当前加速度清零
    current_jerk_ = 0.0f;             // 当前加加速度清零
    
    // 各阶段时间参数清零
    t_jerk_up_ = 0.0f;
    t_accel_const_ = 0.0f;
    t_jerk_down_ = 0.0f;
    t_cruise_ = 0.0f;
    t_jerk_neg_ = 0.0f;
    t_decel_const_ = 0.0f;
    t_jerk_end_ = 0.0f;
}

// 切换到下一运动阶段
void SPlanner::NextPhase()
{
    phase_time_ = 0.0f;  // 重置当前阶段的累计时间
    switch (current_phase_)
    {
        case PHASE_JERK_UP:
            // 加加速→匀加速（加速度达到最大值，加加速度归0）
            current_phase_ = PHASE_ACCEL_CONST;
            current_jerk_ = 0.0f;
            break;
        case PHASE_ACCEL_CONST:
            // 匀加速→减加速（开始减小加速度，加加速度为负）
            current_phase_ = PHASE_JERK_DOWN;
            current_jerk_ = -max_jerk_;  // 负的加加速度（减小加速度）
            break;
        case PHASE_JERK_DOWN:
            // 减加速→匀速或直接减速（加速度归0）
            current_phase_ = (t_cruise_ > 0) ? PHASE_CRUISE : PHASE_JERK_NEG;
            current_jerk_ = 0.0f;
            current_accel_ = 0.0f;
            break;
        case PHASE_CRUISE:
            // 匀速→加减速（开始减速，加加速度为负）
            current_phase_ = PHASE_JERK_NEG;
            current_jerk_ = -max_jerk_;  // 负的加加速度（产生负向加速度）
            break;
        case PHASE_JERK_NEG:
            // 加减速→匀减速（负向加速度达到最大值，加加速度归0）
            current_phase_ = PHASE_DECEL_CONST;
            current_jerk_ = 0.0f;
            break;
        case PHASE_DECEL_CONST:
            // 匀减速→减减速（开始减小负向加速度，加加速度为正）
            current_phase_ = PHASE_JERK_END;
            current_jerk_ = max_jerk_;  // 正的加加速度（减小负向加速度）
            break;
        case PHASE_JERK_END:
            // 减减速→完成（加速度归0）
            current_phase_ = PHASE_FINISHED;
            current_jerk_ = 0.0f;
            current_accel_ = 0.0f;
            break;
        default:
            current_phase_ = PHASE_FINISHED;
            break;
    }
}

// 计算各阶段的时间参数
void SPlanner::CalculatePhaseTimes()
{
    // 加加速/减加速阶段时间（根据最大加速度和加加速度计算）
    // 公式：a = j*t → t = a/j
    t_jerk_up_ = max_accel_ / max_jerk_;       // 加加速时间（加速度从0到max_accel）
    t_jerk_down_ = max_accel_ / max_jerk_;     // 减加速时间（加速度从max_accel到0）
    t_jerk_neg_ = max_decel_ / max_jerk_;      // 加减速时间（加速度从0到-max_decel）
    t_jerk_end_ = max_decel_ / max_jerk_;      // 减减速时间（加速度从-max_decel到0）

    // 初始化解：匀加速、匀减速、匀速时间先设为0
    t_accel_const_ = 0.0f;
    t_decel_const_ = 0.0f;
    t_cruise_ = 0.0f;

    // 计算加速阶段的速度增量（从start_vel到巡航速度）
    // 加加速阶段速度增量：0.5*j*t²（三角形面积）
    float delta_vel_jerk_up = 0.5f * max_jerk_ * t_jerk_up_ * t_jerk_up_;
    // 匀加速阶段速度增量：a*t
    float delta_vel_accel_const = max_accel_ * t_accel_const_;
    // 减加速阶段速度增量：0.5*a*t（三角形面积，与加加速对称）
    float delta_vel_jerk_down = 0.5f * max_accel_ * t_jerk_down_;
    // 加速阶段总速度增量
    float delta_vel_accel = delta_vel_jerk_up + delta_vel_accel_const + delta_vel_jerk_down;

    // 计算减速阶段的速度增量（从巡航速度到end_vel）
    float delta_vel_jerk_neg = 0.5f * max_decel_ * t_jerk_neg_;      // 加减速阶段
    float delta_vel_decel_const = max_decel_ * t_decel_const_;        // 匀减速阶段
    float delta_vel_jerk_end = 0.5f * max_decel_ * t_jerk_end_;      // 减减速阶段
    float delta_vel_decel = delta_vel_jerk_neg + delta_vel_decel_const + delta_vel_jerk_end;

    // 速度关系：start_vel + 加速增量 - 减速增量 = end_vel → 调整加速增量
    float required_total_delta = end_vel_ - start_vel_ + delta_vel_decel - delta_vel_accel;
    delta_vel_accel += required_total_delta;  // 确保最终速度满足目标

    // 计算需要的匀加速时间（如果加速增量不足）
    if (delta_vel_accel > delta_vel_jerk_up + delta_vel_jerk_down)
    {
        // 加加速和减加速阶段的增量不够，需要增加匀加速时间
        t_accel_const_ = (delta_vel_accel - delta_vel_jerk_up - delta_vel_jerk_down) / max_accel_;
        delta_vel_accel_const = max_accel_ * t_accel_const_;  // 更新匀加速增量
        delta_vel_accel = delta_vel_jerk_up + delta_vel_accel_const + delta_vel_jerk_down;
    }
    else
    {
        // 加速增量不足，缩放加加速和减加速的时间（保持对称）
        float scale_factor = delta_vel_accel / (delta_vel_jerk_up + delta_vel_jerk_down);
        t_jerk_up_ *= scale_factor;
        t_jerk_down_ *= scale_factor;
        
        // 重新计算速度增量
        delta_vel_jerk_up = 0.5f * max_jerk_ * t_jerk_up_ * t_jerk_up_;
        delta_vel_jerk_down = 0.5f * max_accel_ * t_jerk_down_;
        delta_vel_accel = delta_vel_jerk_up + delta_vel_jerk_down;
    }

    // 确保最大速度不超过限制（start_vel + 加速增量 不能超过max_vel）
    float max_vel_achieved = start_vel_ + delta_vel_accel;
    if (max_vel_achieved > max_vel_)
    {
        // 超过最大速度，缩放加速阶段时间
        float scale_factor = (max_vel_ - start_vel_) / delta_vel_accel;
        t_jerk_up_ *= scale_factor;
        t_accel_const_ *= scale_factor;
        t_jerk_down_ *= scale_factor;
        
        // 重新计算速度增量
        delta_vel_jerk_up = 0.5f * max_jerk_ * t_jerk_up_ * t_jerk_up_;
        delta_vel_accel_const = max_accel_ * t_accel_const_;
        delta_vel_jerk_down = 0.5f * max_accel_ * t_jerk_down_;
        delta_vel_accel = delta_vel_jerk_up + delta_vel_accel_const + delta_vel_jerk_down;
        max_vel_achieved = start_vel_ + delta_vel_accel;  // 确保不超过max_vel
    }

    // 调整减速阶段以匹配末端速度（减速增量应等于 巡航速度 - end_vel）
    float required_decel_delta = max_vel_achieved - end_vel_;
    float decel_scale = required_decel_delta / delta_vel_decel;  // 缩放减速阶段时间
    
    t_jerk_neg_ *= decel_scale;
    t_decel_const_ *= decel_scale;
    t_jerk_end_ *= decel_scale;

    // 计算加速阶段各部分的距离（用于后续计算匀速阶段距离）
    // 加加速阶段距离：1/6*j*t³（积分公式）
    float dist_jerk_up = (1.0f/6.0f) * max_jerk_ * t_jerk_up_ * t_jerk_up_ * t_jerk_up_;
    
    // 匀加速阶段距离：v0*t + 0.5*a*t²（v0为加加速结束时的速度）
    float vel_after_jerk_up = start_vel_ + delta_vel_jerk_up;
    float dist_accel_const = vel_after_jerk_up * t_accel_const_ + 
                           0.5f * max_accel_ * t_accel_const_ * t_accel_const_;
    
    // 减加速阶段距离：v0*t + 0.5*a*t² - 1/6*j*t³（加速度减小）
    float vel_before_jerk_down = vel_after_jerk_up + max_accel_ * t_accel_const_;
    float dist_jerk_down = vel_before_jerk_down * t_jerk_down_ + 
                         0.5f * max_accel_ * t_jerk_down_ * t_jerk_down_ -
                         (1.0f/6.0f) * max_jerk_ * t_jerk_down_ * t_jerk_down_ * t_jerk_down_;
    
    // 加速阶段总距离
    float distance_accel_phase = dist_jerk_up + dist_accel_const + dist_jerk_down;

    // 计算减速阶段各部分的距离
    float vel_before_jerk_neg = max_vel_achieved;  // 减速开始时的速度（巡航速度）
    
    // 加减速阶段距离：v0*t - 0.5*d*t² + 1/6*j*t³（负向加速度增大）
    float dist_jerk_neg = vel_before_jerk_neg * t_jerk_neg_ - 
                        0.5f * max_decel_ * t_jerk_neg_ * t_jerk_neg_ +
                        (1.0f/6.0f) * max_jerk_ * t_jerk_neg_ * t_jerk_neg_ * t_jerk_neg_;
    
    // 匀减速阶段距离：v0*t - 0.5*d*t²（v0为加减速结束时的速度）
    float vel_after_jerk_neg = vel_before_jerk_neg - 0.5f * max_decel_ * t_jerk_neg_;
    float dist_decel_const = vel_after_jerk_neg * t_decel_const_ - 
                           0.5f * max_decel_ * t_decel_const_ * t_decel_const_;
    
    // 减减速阶段距离：v0*t - 0.5*d*t² + 1/6*j*t³（负向加速度减小）
    float vel_before_jerk_end = vel_after_jerk_neg - max_decel_ * t_decel_const_;
    float dist_jerk_end = vel_before_jerk_end * t_jerk_end_ - 
                        0.5f * max_decel_ * t_jerk_end_ * t_jerk_end_ +
                        (1.0f/6.0f) * max_jerk_ * t_jerk_end_ * t_jerk_end_ * t_jerk_end_;
    
    // 减速阶段总距离
    float distance_decel_phase = dist_jerk_neg + dist_decel_const + dist_jerk_end;

    // 计算巡航距离和时间（总距离 - 加速 - 减速）
    float distance_cruise = distance_total_ - distance_accel_phase - distance_decel_phase;
    
    // 确保巡航距离非负，否则调整减速阶段时间
    if (distance_cruise < 0)
    {
        distance_cruise = 0;
        // 距离超出，通过增加匀减速时间抵消
        float excess_distance = -(distance_cruise);  // 超出的距离
        t_decel_const_ += excess_distance / max_vel_achieved;  // 延长匀减速时间
    }
    
    // 巡航时间 = 巡航距离 / 巡航速度（确保速度为正）
    t_cruise_ = (distance_cruise > 0 && max_vel_achieved > 0) ? 
               distance_cruise / max_vel_achieved : 0.0f;
}

// 初始化规划参数
void SPlanner::Initialize()
{
    // 计算总距离和方向
    distance_total_ = _tool_Abs(end_pos_ - start_pos_);
    direction_ = _tool_Sign(end_pos_ - start_pos_);
    
    // 目标位置在死区内，直接判定为到达
    if (distance_total_ < deadzone_)
    {
        arrived_flag_ = true;
        is_initialized_ = true;
        return;
    }
    
    // 计算各阶段时间参数
    CalculatePhaseTimes();
    
    // 初始化状态：从加加速阶段开始
    current_phase_ = PHASE_JERK_UP;
    current_jerk_ = max_jerk_;  // 初始加加速度为最大值
    current_accel_ = 0.0f;      // 初始加速度为0
    phase_time_ = 0.0f;         // 阶段时间清零
    
    is_initialized_ = true;  // 初始化完成
}

// 执行规划：根据当前位置和速度计算目标速度
float SPlanner::Plan(float current_pos, float current_vel)
{
    // 检查是否到达目标
    if (CheckArrived(current_pos))
    {
        arrived_flag_ = true;
        return end_vel_ * direction_;
    }
    arrived_flag_ = false;
    
    // 未初始化或已完成则重新初始化
    if (!is_initialized_ || current_phase_ == PHASE_FINISHED)
    {
        Initialize();
        if (arrived_flag_)
            return end_vel_ * direction_;
    }
    
    // 时间增量（实际应用中应使用系统时间差，此处简化为1ms）
    float dt = 0.001f;  // 1ms控制周期，后期记得更改！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    phase_time_ += dt;  // 累加当前阶段的时间

    // 根据当前阶段更新加速度（通过加加速度积分）
    switch (current_phase_)
    {
        case PHASE_JERK_UP:  // 加加速阶段：加速度随时间线性增大
            current_accel_ += current_jerk_ * dt;  // a = a0 + j*dt
            // 若加速度达到最大值或阶段时间结束，切换到下一阶段
            if (current_accel_ >= max_accel_ || phase_time_ >= t_jerk_up_)
            {
                current_accel_ = max_accel_;  // 钳位到最大加速度
                NextPhase();
            }
            break;

        case PHASE_ACCEL_CONST:  // 匀加速阶段：加速度保持最大
            // 阶段时间结束则切换
            if (phase_time_ >= t_accel_const_)
            {
                NextPhase();
            }
            break;

        case PHASE_JERK_DOWN:  // 减加速阶段：加速度减小到0
            current_accel_ += current_jerk_ * dt;  // j为负，加速度减小
            if (current_accel_ <= 0 || phase_time_ >= t_jerk_down_)
            {
                current_accel_ = 0;  // 加速度归0
                NextPhase();
            }
            break;

        case PHASE_CRUISE:  // 匀速阶段：加速度为0
            current_accel_ = 0;
            if (phase_time_ >= t_cruise_)
            {
                NextPhase();
            }
            break;

        case PHASE_JERK_NEG:  // 加减速阶段：负向加速度增大（减速开始）
            current_accel_ += current_jerk_ * dt;  // j为负，加速度变负
            if (current_accel_ <= -max_decel_ || phase_time_ >= t_jerk_neg_)
            {
                current_accel_ = -max_decel_;  // 钳位到最大减速度
                NextPhase();
            }
            break;

        case PHASE_DECEL_CONST:  // 匀减速阶段：负向加速度保持最大
            if (phase_time_ >= t_decel_const_)
            {
                NextPhase();
            }
            break;

        case PHASE_JERK_END:  // 减减速阶段：负向加速度减小到0
            current_accel_ += current_jerk_ * dt;  // j为正，负向加速度减小
            if (current_accel_ >= 0 || phase_time_ >= t_jerk_end_)
            {
                current_accel_ = 0;  // 加速度归0
                NextPhase();
            }
            break;

        default:  // 完成阶段：返回目标速度
            return end_vel_ * direction_;
    }

    // 计算速度（对加速度积分：v = v0 + a*dt）
    float vel_output = current_vel + current_accel_ * dt;
    
    // 限制速度在最大范围内
    if (vel_output > max_vel_) vel_output = max_vel_;
    if (vel_output < -max_vel_) vel_output = -max_vel_;

    // 应用运动方向
    vel_output *= direction_;

    return vel_output;
}

/**
 * 多项式速度规划器实现
 */
// 重置规划器状态
void PolynomialPlanner::Reset()
{
    PlannerBase::Reset();  // 调用基类重置
    // 多项式系数清零
    a0_ = a1_ = a2_ = 0.0f;
    a3_ = a4_ = a5_ = 0.0f;
    total_time_ = 0.0f;    // 总时间清零
    time_elapsed_ = 0.0f;  // 已流逝时间清零
}

// 计算总运动时间（基于距离和约束）
float PolynomialPlanner::CalculateTotalTime()
{
    // 基于最大速度的最小时间（距离/最大速度）
    float max_possible_vel = max_vel_;
    float min_time_based_on_vel = distance_total_ / max_possible_vel;
    
    // 基于最大加速度的最小时间（假设匀加速到中点再匀减速）
    float min_time_based_on_accel = sqrtf(distance_total_ * 2 / max_accel_);
    
    // 取较大值作为基础，加20%安全系数（避免理论值过紧）
    return fmaxf(min_time_based_on_vel, min_time_based_on_accel) * 1.2f;
}

// 初始化规划参数（计算多项式系数）
void PolynomialPlanner::Initialize()
{
    // 计算总距离和方向
    distance_total_ = _tool_Abs(end_pos_ - start_pos_);
    direction_ = _tool_Sign(end_pos_ - start_pos_);
    
    // 目标位置在死区内，直接判定为到达
    if (distance_total_ < deadzone_)
    {
        arrived_flag_ = true;
        is_initialized_ = true;
        return;
    }
    
    // 计算总运动时间
    total_time_ = CalculateTotalTime();
    if (total_time_ <= 0) total_time_ = 0.1f;  // 避免时间为0


    float T = total_time_;  // 总时间
    float T2 = T * T;       // T²
    float T3 = T2 * T;      // T³
    float T4 = T3 * T;      // T⁴
    float T5 = T4 * T;      // T⁵

    // 求解5次多项式系数（满足边界条件）
    // 位置函数：s(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    // 边界条件：
    // t=0时：s=0, v=start_vel, a=0（简化）
    // t=T时：s=distance_total, v=end_vel, a=0（简化）
    a0_ = 0.0f;  // t=0时位置为0（相对起始位置）
    a1_ = start_vel_;  // t=0时速度为start_vel
    a2_ = 0.0f;  // t=0时加速度为0（简化）
    
    // 高次项系数（通过解方程组推导）
    a3_ = (10 * distance_total_ - (6 * start_vel_ - 4 * end_vel_) * T) / T3;
    a4_ = (-(15 * distance_total_) + (8 * start_vel_ - 7 * end_vel_) * T) / T4;
    a5_ = (6 * distance_total_ - (3 * start_vel_ - 3 * end_vel_) * T) / T5;

    time_elapsed_ = 0.0f;  // 已流逝时间清零
    is_initialized_ = true;  // 初始化完成
}

// 执行规划：根据多项式计算目标速度
float PolynomialPlanner::Plan(float current_pos, float current_vel)
{
    // 检查是否到达目标
    if (CheckArrived(current_pos))
    {
        arrived_flag_ = true;
        return end_vel_ * direction_;
    }
    arrived_flag_ = false;
    
    // 未初始化则先初始化
    if (!is_initialized_)
    {
        Initialize();
        if (arrived_flag_)
            return end_vel_ * direction_;
    }
    
    // 更新已流逝时间（实际应用中应使用系统时间差，此处简化为1ms）
    time_elapsed_ += 0.001f;  // 1ms控制周期，后期要更改！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    // 限制时间不超过总规划时间
    if (time_elapsed_ >= total_time_)
    {
        time_elapsed_ = total_time_;
    }

    // 计算多项式的一阶导数（速度）
    float t = time_elapsed_;  // 当前时间
    float t2 = t * t;         // t²
    float t3 = t2 * t;        // t³
    float t4 = t3 * t;        // t⁴
    
    // 速度函数：v(t) = s’(t) = a1 + 2*a2*t + 3*a3*t² + 4*a4*t³ + 5*a5*t⁴
    float vel_output = a1_ + 2*a2_*t + 3*a3_*t2 + 4*a4_*t3 + 5*a5_*t4;

    // 应用运动方向
    vel_output *= direction_;

    // 限制速度在安全范围内
    if (vel_output > max_vel_) vel_output = max_vel_;
    if (vel_output < -max_vel_) vel_output = -max_vel_;

    return vel_output;
}