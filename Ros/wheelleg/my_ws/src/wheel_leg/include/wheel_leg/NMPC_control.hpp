// NMPC_control.hpp
#pragma once
#include "base_control.hpp"
#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>

class NMPC_control : NMPC_Balance_Base
{
public:
    NMPC_control(
        ros::NodeHandle &nh, double yaw_kp, double yaw_ki, double yaw_kd,
        double Pos_kp, double Pos_ki, double Pos_kd,
        double M_, double m_, double I_, double i_, double h_,
        int N, double u_max, double u_min, 
        double Q_theta, double Q_omega, double Q_x,double Q_v,
        double R_u,double dt)
        : NMPC_Balance_Base(nh, yaw_kp, yaw_ki, yaw_kd, Pos_kp, Pos_ki, Pos_kd)
    {
        setParam(M_, m_, I_, i_, h_);
        this->N = N;
        this->u_max = u_max;
        this->u_min = u_min;
        this->Q_theta = Q_theta;
        this->Q_omega = Q_omega;
        this->Q_x = Q_x;
        this->Q_v = Q_v;
        this->R_u = R_u;
        this->dt_ = dt;
        this->T = this->N*this->dt_;
        this->x_ = casadi::MX::sym("X", 4, N+1); // 状态: [q,qd,x,xd]
        this->u_ = casadi::MX::sym("U", 1, N);   // 控制输入: 力 F
        // 在构造函数中预构建求解器
        Build_nmpc_problem();
        
        // 初始化参考值
        x_ref_val = 0.0;
        theta_ref_val = 0.0;
        v_ref_val = 0.0;
        omega_ref_val = 0.0;
    }

    void Build_nmpc_problem() {
        cost = 0; // 重置cost
        
        // 使用符号变量表示参考值，在运行时更新
        casadi::MX theta_ref_sym = casadi::MX::sym("theta_ref");
        casadi::MX omega_ref_sym = casadi::MX::sym("omega_ref");
        casadi::MX x_ref_sym = casadi::MX::sym("x_ref");
        casadi::MX v_ref_sym = casadi::MX::sym("v_ref");
        
        // 阶段代价
        for (int k = 0; k < N; k++) {
            cost += Q_theta * (x_(0,k) - theta_ref_sym) * (x_(0, k) - theta_ref_sym);
            cost += Q_omega * (x_(1,k) - omega_ref_sym) * (x_(1, k) - omega_ref_sym);
            cost += Q_x * (x_(2, k) - x_ref_sym) * (x_(2, k) - x_ref_sym);
            cost += Q_v * (x_(3, k) - v_ref_sym) * (x_(3, k) - v_ref_sym);
            cost += R_u * u_(0, k) * u_(0, k);
        }
        
        // 终端代价
        cost += 10 * Q_theta * (x_(0, N) - theta_ref_sym) * (x_(0, N) - theta_ref_sym);
        cost += 10 * Q_omega * (x_(1, N) - omega_ref_sym) * (x_(1, N) - omega_ref_sym);
        cost += 10 * Q_x * (x_(2, N) - x_ref_sym) * (x_(2, N) - x_ref_sym);
        cost += 10 * Q_v * (x_(3, N) - v_ref_sym) * (x_(3, N) - v_ref_sym);
        
        // 约束
        std::vector<casadi::MX> constraints;
        
        // 初始条件约束 - 使用符号变量，在运行时更新
        casadi::MX x0 = casadi::MX::sym("x0", 4);
        constraints.push_back(x_(0, 0) - x0(0));    // 初始角度
        constraints.push_back(x_(1, 0) - x0(1));    // 初始角速度
        constraints.push_back(x_(2, 0) - x0(2));    // 初始位置
        constraints.push_back(x_(3, 0) - x0(3));    // 初始速度
        
        // 动力学约束 (倒立摆模型)
        for (int k = 0; k < N; k++) {
            casadi::MX x1 = x_(0, k);//q
            casadi::MX x2 = x_(1, k);//qd
            casadi::MX x3 = x_(2, k);//x
            casadi::MX x4 = x_(3, k);//xd
            casadi::MX uk = u_(0, k);

            casadi::MX denominator = ((M+m)*(I + m*h*h) - m*m*h*h*cos(x1)*cos(x1));
            casadi::MX safe_denominator = casadi::MX::fmax(denominator, 1e-6);
            // 离散化状态更新
            casadi::MX x1_next = x1 + dt_*x2;
            casadi::MX x2_next = x2 + dt_*((M+m)*m*h*g*sin(x1) - m*h*cos(x1)*(uk + m*h*x2*x2*sin(x1)))/ denominator;
            casadi::MX x3_next = x3 + dt_ * x4;
            casadi::MX x4_next = x4 + dt_ * ((uk + m*h*x2*x2*sin(x1))*(I+m*h*h)-(m*h*cos(x1))*(m*h*g*sin(x1)))/ denominator;

            constraints.push_back(x_(0, k+1) - x1_next);
            constraints.push_back(x_(1, k+1) - x2_next);
            constraints.push_back(x_(2, k+1) - x3_next);
            constraints.push_back(x_(3, k+1) - x4_next);
        }
        
        // 构建NLP问题
        casadi::MX opt_vars = casadi::MX::vertcat({casadi::MX::reshape(x_, 4*(N+1), 1), 
                                                casadi::MX::reshape(u_, N, 1)});
        
        // 参数向量：初始状态 + 参考值
        casadi::MX params = casadi::MX::vertcat({x0, theta_ref_sym, omega_ref_sym, x_ref_sym, v_ref_sym});
        
        casadi::MXDict nlp = {
            {"x", opt_vars},
            {"f", cost},
            {"g", casadi::MX::vertcat(constraints)},
            {"p", params}  // 添加参数
        };
        
        // 求解器选项
        casadi::Dict opts;
        opts["ipopt.print_level"] = 0;
        opts["ipopt.max_iter"] = 50;
        opts["ipopt.tol"] = 1e-4;
        opts["ipopt.constr_viol_tol"] = 1e-6;
        opts["ipopt.acceptable_tol"] = 1e-4;
        opts["print_time"] = false;
        opts["ipopt.linear_solver"] = "mumps";
        
        // 创建求解器
        solver = casadi::nlpsol("solver", "ipopt", nlp, opts);

        // 变量边界
        std::vector<double> lbx(4*(N+1) + N, -casadi::inf);
        std::vector<double> ubx(4*(N+1) + N, casadi::inf);
        
        // 控制输入约束
        for (int i = 4*(N+1); i < 4*(N+1) + N; i++) {
            lbx[i] = u_min;
            ubx[i] = u_max;
        }
        
        // 角度约束 (防止摆杆旋转过多)
        for (int i = 0; i < 4*(N+1); i += 4) {
            lbx[i] = -M_PI/2;
            ubx[i] = M_PI/2;
        }
        
        // 约束边界 (等式约束为0)
        std::vector<double> lbg(4 + 4*N, 0.0);
        std::vector<double> ubg(4 + 4*N, 0.0);
        
        // 初始猜测
        std::vector<double> x0_guess(4*(N+1) + N, 0.0);
        for (int i = 0; i < 4*(N+1); i += 4) {
            x0_guess[i] = 0.0;  // 角度初始猜测为0（平衡位置）
        }
        // 设置参数
        arg["lbx"] = lbx;
        arg["ubx"] = ubx;
        arg["lbg"] = lbg;
        arg["ubg"] = ubg;
        arg["x0"] = x0_guess;
        
        // 初始化参数
        std::vector<double> param_vals(8, 0.0); // 4个状态 + 4个参考值
        arg["p"] = param_vals;
    }
    
    double get_output() override;
    void setParam(double M_, double m_, double I_, double i_, double h_);
    
    // 更新缓存状态的方法
    void update_current_state() {
        std::lock_guard<std::mutex> lock(state_mutex);
        current_pitch = this->pitch;
        current_ang_vel_y = this->ang_vel_y;
        current_total_pos = this->total_pos; 
        current_robot_vel = this->robot_vel;
    }
    // 设置参考值的方法
    void set_reference(double theta_ref, double omega_ref, double x_ref, double v_ref) {
        theta_ref_val = theta_ref;
        omega_ref_val = omega_ref;
        x_ref_val = x_ref;
        v_ref_val = v_ref;
    }

private:
    double M = 0;
    double m = 0;
    double I = 0;
    double i = 0;
    double h = 0;
    double u_max = 0;
    double u_min = 0;
    int N = 10;
    const double g = 9.8;
    double dt_;
    //权重
    double Q_x;
    double Q_v;
    double Q_theta;
    double Q_omega;
    double R_u;

    double T;
    casadi::MX x_;//状态[q,qd,x,xd]
    casadi::MX u_;//输入
    casadi::MX cost = 0;

    // 参考值的实际数值（在运行时更新）
    double x_ref_val = 0;
    double theta_ref_val = 0;
    double v_ref_val = 0;
    double omega_ref_val = 0;

    // 添加状态缓存，避免在求解过程中状态变化
    double current_pitch = 0.0;
    double current_ang_vel_y = 0.0; 
    double current_total_pos = 0.0;
    double current_robot_vel = 0.0;
    std::mutex state_mutex;

    casadi::Function solver;
    casadi::DMDict arg;
    std::vector<double> last_solution; // 改为成员变量，更好的warm start

};