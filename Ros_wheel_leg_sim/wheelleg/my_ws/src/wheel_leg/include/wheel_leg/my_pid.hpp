#pragma once
#include <algorithm>

class MyPid
{
private:
    // PID参数
    double kp_ = 1.0;          // 比例增益
    double ki_ = 0.0;          // 积分增益  
    double kd_ = 0.0;          // 微分增益
    
    // 控制器状态
    double integral_ = 0.0;    // 积分项累积
    double prev_error_ = 0.0;  // 上一次误差
    
    // 限制参数
    double output_limit_max_ = 1000.0;   // 输出上限
    double output_limit_min_ = -1000.0;  // 输出下限
    double integral_limit_max_ = 1000.0; // 积分项上限
    double integral_limit_min_ = -1000.0;// 积分项下限
    
    // 使能状态
    bool enabled_ = true;      // 控制器使能状态

public:
    MyPid() = default;
    
    MyPid(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}
    
    virtual ~MyPid() = default;

    // 参数设置
    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    
    void setOutputLimits(double min, double max) {
        output_limit_min_ = min;
        output_limit_max_ = max;
    }
    
    void setIntegralLimits(double min, double max) {
        integral_limit_min_ = min;
        integral_limit_max_ = max;
    }
    
    // 状态控制
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }
    
    bool isEnabled() const { return enabled_; }
    
    // 获取参数
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }
    double getIntegral() const { return integral_; }
    
    // 核心计算函数 - 使用误差输入
    double compute(double error, double dt) {
        if (!enabled_ || dt <= 0.0) {
            return 0.0;
        }

        // 比例项
        double proportional = kp_ * error;
        
        // 积分项 - 先计算增量，再应用限幅
        double integral_increment = ki_ * error * dt;
        double new_integral = integral_ + integral_increment;
        
        // 应用积分限幅
        if (new_integral > integral_limit_max_) {
            integral_ = integral_limit_max_;
        } else if (new_integral < integral_limit_min_) {
            integral_ = integral_limit_min_;
        } else {
            integral_ = new_integral;
        }
        
        // 微分项
        double derivative = 0.0;
        if (dt > 0.0) {
            derivative = kd_ * (error - prev_error_) / dt;
        }
        
        // 计算输出
        double output = proportional + integral_ + derivative;
        
        // 应用输出限制
        output = std::max(output_limit_min_, std::min(output, output_limit_max_));
        
        // 更新状态
        prev_error_ = error;
        
        return output;
    }

    // 核心计算函数 - 使用设定值和实际值
    double compute(double setpoint, double actual, double dt) {
        double error = setpoint - actual;
        return compute(error, dt);
    }
};