// NMPC_calc.cpp
#include "wheel_leg/NMPC_control.hpp"
#include <vector> 

void NMPC_control::setParam(double M_, double m_, double I_, double i_, double h_) {
    M = M_;
    m = m_;
    I = I_;
    i = i_;
    h = h_;
}

double NMPC_control::get_output() {
    try {
        // 添加状态监控
        std::cout << "当前状态 - pitch: " << this->pitch 
                  << ", ang_vel_y: " << this->ang_vel_y
                  << ", total_pos: " << this->total_pos
                  << ", robot_vel: " << this->robot_vel << std::endl;
        // 添加计时
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 1. 实时更新状态
        this->State_update();
        
        // 2. 设置参考值
        set_reference(0.0, 0.0, 0.0, 0.0);
        
        // 3. 准备参数
        std::vector<double> params = {
            this->pitch, this->ang_vel_y, this->total_pos, -this->robot_vel,
            theta_ref_val, omega_ref_val, x_ref_val, v_ref_val
        };
        arg["p"] = params;
        
        // 4. Warm start
        if (!last_solution.empty()) {
            std::vector<double> warm_start = last_solution;
            // ... 现有的warm start代码
        }
        
        // 5. 求解
        casadi::DMDict res = solver(arg);
        
        // 6. 计算求解时间
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        std::cout << "NMPC求解时间: " << duration.count() << "ms" << std::endl;
        
        // 如果求解时间过长，发出警告
        if (duration.count() > 10) { // 超过10ms
            std::cout << "警告: NMPC求解时间过长,可能无法实时控制!" << std::endl;
        }
        
        last_solution = res.at("x").get_elements();
        double first_control = last_solution[4*(N+1)];
        
        // 7. 返回控制量
        return first_control; // 先去掉滤波看原始响应
        
    } catch (std::exception& e) {
        std::cout << "NMPC求解失败: " << e.what() << std::endl;
        return 0.0;
    }
}