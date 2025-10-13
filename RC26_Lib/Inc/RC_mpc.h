#pragma once
#include "arm_matrix.h"
#include "arm_math.h"
#include <stdint.h>

namespace mpc {

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
class DynamicMatrixMPC {
public:
    DynamicMatrixMPC();
    void init(const float a0[P][OUTPUT_DIM][INPUT_DIM], float q_val, float r_val);
    void setFilterCoefficient(float alpha_val);
    void setErrorCompensation(float h_val);
    void setConstraints(const ArmMatrix<1, INPUT_DIM>& delta_u_min, 
                       const ArmMatrix<1, INPUT_DIM>& delta_u_max);
    ArmMatrix<1, INPUT_DIM> compute(const ArmMatrix<1, OUTPUT_DIM>& Y_meas,
                                   const ArmMatrix<1, OUTPUT_DIM>& Y_ref,
                                   const ArmMatrix<1, INPUT_DIM>& u_prev);
    const ArmMatrix<P, OUTPUT_DIM>& getPredictedTrajectory() const;

private:
    ArmMatrix<M*INPUT_DIM, 1> solveOptimization();
    bool solveLinearSystem(const ArmMatrix<M*INPUT_DIM, M*INPUT_DIM>& H,
                          const ArmMatrix<M*INPUT_DIM, 1>& b,
                          ArmMatrix<M*INPUT_DIM, 1>& x);
    void buildDynamicMatrix(const float a0[P][OUTPUT_DIM][INPUT_DIM]);
    void buildShiftMatrix();
    void initWeightMatrices();
    void generateReferenceTrajectory(const ArmMatrix<1, OUTPUT_DIM>& Y_ref);
    void compensateError(const ArmMatrix<1, OUTPUT_DIM>& Y_meas);
    void updatePredictionWindow();

private:
    ArmMatrix<P, INPUT_DIM> A;                // 动态矩阵
    ArmMatrix<P, P> S;                        // 移位矩阵
    ArmMatrix<P*OUTPUT_DIM, M*INPUT_DIM> Phi; // 预测矩阵
    ArmMatrix<P*OUTPUT_DIM, P*OUTPUT_DIM> Q;  // 输出权重矩阵
    ArmMatrix<M*INPUT_DIM, M*INPUT_DIM> R;    // 控制权重矩阵

    ArmMatrix<P, OUTPUT_DIM> Y_pred;          // 预测输出
    ArmMatrix<P, OUTPUT_DIM> Y_cor;           // 补偿后预测
    ArmMatrix<P, OUTPUT_DIM> Y_ref_traj;      // 平滑参考轨迹
    ArmMatrix<P, OUTPUT_DIM> Y0;              // 初始预测窗口
    ArmMatrix<1, OUTPUT_DIM> Y_meas_prev;     // 上一次测量值

    ArmMatrix<M, INPUT_DIM> delta_u;          // 控制量变化序列
    ArmMatrix<1, INPUT_DIM> u_prev;           // 上一时刻控制量
    ArmMatrix<1, INPUT_DIM> delta_u_min;      // 最小控制变化
    ArmMatrix<1, INPUT_DIM> delta_u_max;      // 最大控制变化

    float q;          // 输出误差权重
    float r;          // 控制量权重
    float alpha;      // 滤波系数
    float h;          // 补偿系数
    bool  is_initialized; // 初始化标志
};




} // namespace mpc