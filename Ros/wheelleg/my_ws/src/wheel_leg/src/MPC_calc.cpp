#include "wheel_leg/MPC_control.hpp"


State_eqCalc::State_eqCalc(
    double M_, double m_, double I_, double i_, double h_, double r_,
    const Eigen::Matrix4d& Q_, Eigen::Matrix<double, 1, 1>  R_,const Eigen::Matrix4d &Q_terminal)
{
    M = M_;
    m = m_;
    I = I_;
    i = i_;
    h = h_;
    r = r_;
    Q = Q_;
    R = R_;
    this->Q_terminal = Q_terminal;
}


State_eqCalc::~State_eqCalc()
{
}

void State_eqCalc::setParam(
    double M_, double m_, double I_, double i_, double h_, double r_,
    const Eigen::Matrix4d& Q_, Eigen::Matrix<double, 1, 1>  R_,const Eigen::Matrix4d &Q_terminal)
{
    M = M_;
    m = m_;
    I = I_;
    i = i_;
    h = h_;
    r = r_;
    Q = Q_;
    R = R_;
    this->Q_terminal = Q_terminal;
}

void State_eqCalc::A_B_calc()
{
    double Z = M + m + i / (r * r);
    double D = Z * (I + M * h * h) - M * M * h * h;

    double a = (M * g * h * Z) / D;
    double b = -(M * M * h * h * g) / D;
    double c = -(M * h) / (D * r);
    double d = 1 / (Z * r) - (M * M * h * h) / (D * Z * r);


    A << 0,       1,      0,      0,
         a,       0,      0,      0,
         0,       0,      0,      1,
         b,       0,      0,      0;

    B << 0,
         c,
         0,
         d; 
}


double MPC_control::get_output()
{
    if(-0.1<current_state(0)<0.1){
        // 更新MPC状态反馈
        mpc_solver->updateFeedBack(current_state);
        
        // 更新MPC参考状态
        mpc_solver->updateReference(desired_state);
        Eigen::Matrix<double, 1, 1> u_mpc;
        if (mpc_solver->solveMPC(u_mpc))
        {
            
            out = u_mpc(0)*0.7 + last_out*0.3;
            ROS_INFO_THROTTLE(1.0, "MPC求解成功,控制量: %.4f",out);
            last_out = u_mpc(0);
            return out;  // 正确访问矩阵元素
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "MPC求解失败,返回零控制量");
            return 0.0;
        }
    }else{
                // 更新MPC状态反馈
        mpc_solver2->updateFeedBack(current_state);
        
        // 更新MPC参考状态
        mpc_solver2->updateReference(desired_state);
        Eigen::Matrix<double, 1, 1> u_mpc;
        if (mpc_solver2->solveMPC(u_mpc))
        {
            out = u_mpc(0)*0.7 + last_out*0.3;
            ROS_INFO_THROTTLE(1.0, "MPC求解成功,控制量: %.4f",out);
            last_out = u_mpc(0);
            return out;  // 正确访问矩阵元素
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "MPC求解失败,返回零控制量");
            return 0.0;
        }
    }

    return 0.0;
}





