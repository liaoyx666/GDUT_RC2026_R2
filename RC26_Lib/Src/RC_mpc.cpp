#include "RC_mpc.h"
#include "arm_math.h"
#include <stdint.h>

namespace mpc {


static float saturate_f32(float value, float min_val, float max_val) 
	{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// ------------------------------ DynamicMatrixMPC 实现 ------------------------------
template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::DynamicMatrixMPC() 
    : q(1.0f), r(0.1f), alpha(0.5f), h(0.8f), is_initialized(false) {
				A.setZero();
				S.setZero();
				Phi.setZero();
				Q.setZero();
				R.setZero();
				Y_pred.setZero();
				Y_cor.setZero();
				Y_ref_traj.setZero();
				Y0.setZero();
				Y_meas_prev.setZero();
				delta_u.setZero();
				u_prev.setZero();
				delta_u_min.setZero();
				delta_u_max.setZero();
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::init(
    const float a0[P][OUTPUT_DIM][INPUT_DIM], float q_val, float r_val) {
    this->q = q_val;
    this->r = r_val;
    buildDynamicMatrix(a0);
    buildShiftMatrix();
    initWeightMatrices();
    is_initialized = true;
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::setFilterCoefficient(float alpha_val) {
    if (alpha_val > 0 && alpha_val <= 1.0f) {
        this->alpha = alpha_val;
    }
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::setErrorCompensation(float h_val) {
    if (h_val >= 0 && h_val <= 1.0f) {
        this->h = h_val;
    }
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::setConstraints(
    const ArmMatrix<1, INPUT_DIM>& delta_u_min, 
    const ArmMatrix<1, INPUT_DIM>& delta_u_max) {
    this->delta_u_min = delta_u_min;
    this->delta_u_max = delta_u_max;
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
ArmMatrix<1, INPUT_DIM> DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::compute(
    const ArmMatrix<1, OUTPUT_DIM>& Y_meas,
    const ArmMatrix<1, OUTPUT_DIM>& Y_ref,
    const ArmMatrix<1, INPUT_DIM>& u_prev) {
    if (!is_initialized) {
        ArmMatrix<1, INPUT_DIM> zero_mat;
        for (int i = 0; i < 1 * INPUT_DIM; ++i) zero_mat.getData()[i] = 0.0f;
        return zero_mat;
    }

    this->u_prev = u_prev;
    generateReferenceTrajectory(Y_ref);
    compensateError(Y_meas);
    ArmMatrix<M*INPUT_DIM, 1> delta_u_vec = solveOptimization();

    // 提取第1步控制变化，应用约束
    ArmMatrix<1, INPUT_DIM> delta_u0;
    for (int ipt = 0; ipt < INPUT_DIM; ++ipt) {
        delta_u0(0, ipt) = delta_u_vec(ipt);
        delta_u0(0, ipt) = saturate_f32(
            delta_u0(0, ipt),
            delta_u_min(0, ipt),
            delta_u_max(0, ipt)
        );
    }

    updatePredictionWindow();
    return this->u_prev + delta_u0;
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
const ArmMatrix<P, OUTPUT_DIM>& DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::getPredictedTrajectory() const {
    return Y_pred;
}

// ------------------------------ 私有函数实现 ------------------------------
template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::buildDynamicMatrix(
    const float a0[P][OUTPUT_DIM][INPUT_DIM]) {
    // 构建动态矩阵A
    for (int i = 0; i < P; ++i) {
        for (int o = 0; o < OUTPUT_DIM; ++o) {
            for (int ipt = 0; ipt < INPUT_DIM; ++ipt) {
                A(i, ipt) = a0[i][o][ipt];
            }
        }
    }

    // 构建预测矩阵Phi
    for (int i = 0; i < P; ++i) {
        for (int m = 0; m < M && m <= i; ++m) {
            for (int o = 0; o < OUTPUT_DIM; ++o) {
                for (int ipt = 0; ipt < INPUT_DIM; ++ipt) {
                    int row = i * OUTPUT_DIM + o;
                    int col = m * INPUT_DIM + ipt;
                    Phi(row, col) = a0[i - m][o][ipt];
                }
            }
        }
    }
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::buildShiftMatrix() {
    // 移位矩阵置零
    for (int i = 0; i < P * P; ++i) S.getData()[i] = 0.0f;
    // 主对角线右移
    for (int i = 0; i < P - 1; ++i) {
        S(i, i + 1) = 1.0f;
    }
    // 最后一行复制倒数第二行
    if (P > 1) {
        for (int j = 0; j < P; ++j) {
            S(P - 1, j) = S(P - 2, j);
        }
    }
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::initWeightMatrices() {
    // Q矩阵置零并设置对角线
    for (int i = 0; i < P*OUTPUT_DIM * P*OUTPUT_DIM; ++i) Q.getData()[i] = 0.0f;
    for (int i = 0; i < P * OUTPUT_DIM; ++i) {
        Q(i, i) = this->q;
    }

    // R矩阵置零并设置对角线
    for (int i = 0; i < M*INPUT_DIM * M*INPUT_DIM; ++i) R.getData()[i] = 0.0f;
    for (int i = 0; i < M * INPUT_DIM; ++i)
		{
        R(i, i) = this->r;
    }
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::generateReferenceTrajectory(
    const ArmMatrix<1, OUTPUT_DIM>& Y_ref)
			{
    for (int i = 0; i < P; ++i) 
			{
        if (i == 0) 
					{
            for (int o = 0; o < OUTPUT_DIM; ++o) 
						{
                Y_ref_traj(i, o) = alpha * Y_meas_prev(0, o) + (1 - alpha) * Y_ref(0, o);
            }
        } else {
            for (int o = 0; o < OUTPUT_DIM; ++o) 
					{
                Y_ref_traj(i, o) = alpha * Y_ref_traj(i - 1, o) + (1 - alpha) * Y_ref(0, o);
            }
        }
    }
    Y_meas_prev = Y_ref;
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::compensateError(
    const ArmMatrix<1, OUTPUT_DIM>& Y_meas)
			{
    ArmMatrix<1, OUTPUT_DIM> error;
    for (int o = 0; o < OUTPUT_DIM; ++o) 
				{
        error(0, o) = Y_meas(0, o) - Y0(0, o);
    }

    for (int i = 0; i < P; ++i)
		{
        for (int o = 0; o < OUTPUT_DIM; ++o) 
			{
            Y_cor(i, o) = Y0(i, o) + h * error(0, o);
        }
    }

    Y_pred = Y_cor;
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
void DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::updatePredictionWindow()
	{
    Y0 = S * Y_cor;
	}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
ArmMatrix<M*INPUT_DIM, 1> DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::solveOptimization() 
{
    ArmMatrix<P*OUTPUT_DIM, 1> Y_tilde;
    for (int i = 0; i < P; ++i) 
	{
        for (int o = 0; o < OUTPUT_DIM; ++o)
		{
            int idx = i * OUTPUT_DIM + o;
            Y_tilde(idx) = Y_ref_traj(i, o) - Y_cor(i, o);
        }
    }

    ArmMatrix<M*INPUT_DIM, P*OUTPUT_DIM> Phi_T = Phi.transpose();
    ArmMatrix<M*INPUT_DIM, P*OUTPUT_DIM> temp1 = Phi_T * Q;
    ArmMatrix<M*INPUT_DIM, M*INPUT_DIM> H = temp1 * Phi + R;
    ArmMatrix<M*INPUT_DIM, 1> g = temp1 * Y_tilde;

    ArmMatrix<M*INPUT_DIM, 1> delta_u_vec;
    for (int i = 0; i < M*INPUT_DIM; ++i) delta_u_vec.getData()[i] = 0.0f;
    if (!solveLinearSystem(H, g, delta_u_vec))
			{
        return delta_u_vec;
    }

    return delta_u_vec;
}

template<int OUTPUT_DIM, int INPUT_DIM, int P, int M>
	
bool DynamicMatrixMPC<OUTPUT_DIM, INPUT_DIM, P, M>::solveLinearSystem(
    const ArmMatrix<M*INPUT_DIM, M*INPUT_DIM>& H,
    const ArmMatrix<M*INPUT_DIM, 1>& b,
    ArmMatrix<M*INPUT_DIM, 1>& x) 
		{
    ArmMatrix<M*INPUT_DIM, M*INPUT_DIM> H_inv = H;
    if (!H_inv.invert())
			{
        return false;
    }

    x = H_inv * b;
    return true;
}


template class DynamicMatrixMPC<1, 1, 5, 2>; 

} // namespace mpc