//
// created by linxif2008 on 04/23/2024
//

#ifndef MPC_SOLVER_H
#define MPC_SOLVER_H

#include <OsqpEigen/OsqpEigen.h>

template <int _states, int _commands, int _mpc_window>
class MpcSolver
{
    // check template
    static_assert(_states > 0 && _commands > 0 && _mpc_window > 0);

private:
    Eigen::Matrix<double, _states, _states> _A, _A_bar;
    Eigen::Matrix<double, _states, _commands> _B, _B_bar;

    Eigen::Matrix<double, _states * _mpc_window, _states> _phi;
    Eigen::Matrix<double, _states * _mpc_window, _commands * _mpc_window> _theta;

    Eigen::Matrix<double, _states * _mpc_window, 1> _x_R, _E;

    Eigen::Matrix<double, _states * _mpc_window, _states * _mpc_window> _Q;
    Eigen::Matrix<double, _commands * _mpc_window, _commands * _mpc_window> _R;
    //终端权重矩阵
    Eigen::Matrix<double, _states, _states> _Q_terminal;

    Eigen::Matrix<double, _states, 1> _x_fdb, _x_ref; // state feedback and state reference
    Eigen::Matrix<double, _commands, 1> _u;           // mpc output

    double _predict_interval; // the time inverval between two mpc predictions

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _constraintMatrix;//约束矩阵
    Eigen::Matrix<double,Eigen::Dynamic,1> _lowerBound;//约束下限
    Eigen::Matrix<double,Eigen::Dynamic,1> _upperBound;//约束上限
public:
    explicit MpcSolver(const Eigen::Matrix<double, _states, _states> &A,
                       const Eigen::Matrix<double, _states, _commands> &B,
                       const Eigen::Matrix<double, _states, _states> &Q,   
                        const Eigen::Matrix<double, _commands, _commands> &R,
                        const Eigen::Matrix<double, _states, _states> &Q_terminal,
                       const double &predict_interval);
    void updateAB(const Eigen::Matrix<double, _states, _states> &A,
                  const Eigen::Matrix<double, _states, _commands> &B);//更新A，B
    void setConstraints(const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> &constraintMatrix,
                        const Eigen::Matrix<double,Eigen::Dynamic,1> &lowerBound,
                        const Eigen::Matrix<double,Eigen::Dynamic,1> &upperBound);//设置约束    
  
    void updateQR(const Eigen::Matrix<double, _states, _states> &Q, const Eigen::Matrix<double, _commands, _commands> &R,
                const Eigen::Matrix<double, _states, _states> &Q_terminal = Eigen::Matrix<double, _states, _states>::Zero());

    void updateReference(const Eigen::Matrix<double, _states, 1> &ref);

    void updateFeedBack(const Eigen::Matrix<double, _states, 1> &fdb);

    bool solveMPC(Eigen::Matrix<double, _commands, 1> &u);
};

#include "mpc_solver/mpc_solver.tpp"

#endif // MPC_SOLVER_H