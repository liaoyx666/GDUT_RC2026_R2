#pragma
#include "base_control.hpp"
#include "mpc_solver/mpc_solver.h" 
#include <casadi/casadi.hpp>

const int _mpc_window = 30;//预测时域

class State_eqCalc
{
private:
    double M = 0; // 车身质量
    double m = 0; // 轮子质量
    double I = 0; // 车身转动惯量
    double i = 0; // 轮子转动惯量
    double h = 0; // 车身重心高度
    double r = 0; // 轮子半径
    const double g = 9.8; // 重力加速度
    
public:
    State_eqCalc(
        double M_, double m_, double I_, double i_, double h_, double r_,
        const Eigen::Matrix4d& Q_, Eigen::Matrix<double, 1, 1> R_,const Eigen::Matrix4d &Q_terminal);
    virtual ~State_eqCalc();
    void A_B_calc();
    void setParam(
        double M_, double m_, double I_, double i_, double h_, double r_,
        const Eigen::Matrix4d& Q_, Eigen::Matrix<double, 1, 1>  R_,const Eigen::Matrix4d &Q_terminal);
    Eigen::Matrix4d A;
    Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix4d Q; // 权重对角线元素
    Eigen::Matrix4d Q_terminal; // 终端权重对角线元素
    Eigen::Matrix<double, 1, 1> R;// 控制权重矩阵
};


class MPC_control : public Balance_Base
{
public:
    MPC_control(ros::NodeHandle &nh,double yaw_kp,double yaw_ki,double yaw_kd,
                                    double Pos_kp,double Pos_ki,double Pos_kd,
                                  const Eigen::Matrix4d &A,
                                  const Eigen::Matrix<double, 4, 1> &B,
                                  const Eigen::Matrix4d &Q,
                                  const Eigen::Matrix<double, 1, 1> &R,
                                  const Eigen::Matrix4d  &Q_terminal,
                                  double dt
                                )
        : Balance_Base(nh,yaw_kp,yaw_ki,yaw_kd,Pos_kp,Pos_ki,Pos_kd), A_(A), B_(B),R_(R),dt_(dt)
    {
        Q_ = Q;
        _Q_terminal_ = Q_terminal;
        Q_2 = Q;
        _Q_terminal_2 = Q_terminal;

        Q_2(2) = 0;
        Q_2(3) = 0;
        _Q_terminal_2(2) = 0;
        _Q_terminal_2(3) = 0;
        mpc_solver = std::make_shared<MpcSolver<4, 1, _mpc_window>>(A_, B_, Q_, R_,_Q_terminal_,dt);
        mpc_solver2 = std::make_shared<MpcSolver<4, 1, _mpc_window>>(A_, B_, Q_2, R_,_Q_terminal_2,dt);
        //初始化约束（默认无）
        setupDefaultConstraints();
        mpc_solver->updateReference(desired_state);
        mpc_solver2->updateReference(desired_state);
    }
    // 添加实时更新A、B矩阵的方法
    void updateSystemMatrices(const Eigen::Matrix4d &new_A, 
                             const Eigen::Matrix<double, 4, 1> &new_B) {
        A_ = new_A;
        B_ = new_B;
        mpc_solver->updateAB(A_,B_);//平衡时求解器
        mpc_solver2->updateAB(A_,B_);//平衡时求解器
        setupDefaultConstraints(); // 重新设置约束
    }
    void main_control(double Leg_L,double target_yaw,Eigen::Vector4d desire) override{
        Balance_Base::main_control(Leg_L,target_yaw,desire);  // 调用基类状态更新
        this->balance();
    }
    // 设置输入约束
    void setInputConstraints(double u_min, double u_max) {
        int total_vars = 1 * _mpc_window;  // 控制变量总数
        
        // 约束矩阵：单位矩阵（对每个控制输入单独约束）
        constraintMatrix = Eigen::MatrixXd::Identity(total_vars, total_vars);
        lowerBound = Eigen::VectorXd::Constant(total_vars, u_min);
        upperBound = Eigen::VectorXd::Constant(total_vars, u_max);
        
        mpc_solver->setConstraints(constraintMatrix, lowerBound, upperBound);
        mpc_solver2->setConstraints(constraintMatrix, lowerBound, upperBound);
    }
    // 清除所有约束
    void clearConstraints() {
        setupDefaultConstraints();
    }
    void STATE_INFO(){
        ROS_INFO(
            "欧拉角:pitch:%.4f,roll:%.4f,yaw:%.4f\n"
            "状态:q:%.4f,q_d: %.4f,x: %.4f,x_d:%.4f\n"
            "轮子输出：%.4f"
            "--------------------------------------------------------",
            this->pitch,this->roll,this->yaw,
            this->pitch,this->ang_vel_y,this->total_pos,this->robot_vel,
            this->wheel_out
        );
    }
    double get_output() override;

private:
    Eigen::Matrix4d A_;
    Eigen::Matrix<double, 4, 1> B_;
    Eigen::Matrix4d Q_;
    Eigen::Matrix4d _Q_terminal_;
    Eigen::Matrix4d Q_2;
    Eigen::Matrix4d _Q_terminal_2;
    Eigen::Matrix<double, 1, 1> R_;
    Eigen::Matrix<double, 1, 1> u;
    double dt_;
    double last_out = 0;
    double out =0;
    std::shared_ptr<MpcSolver<4, 1, _mpc_window>> mpc_solver;
    std::shared_ptr<MpcSolver<4, 1, _mpc_window>> mpc_solver2;
    Eigen::MatrixXd constraintMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    void setupDefaultConstraints() {
        // 默认无约束（空矩阵）
        constraintMatrix.resize(0, 1 * _mpc_window);
        lowerBound.resize(0);
        upperBound.resize(0);
        mpc_solver->setConstraints(constraintMatrix, lowerBound, upperBound);
    }
};