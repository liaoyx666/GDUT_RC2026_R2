#include "wheel_leg/lqr_calc.hpp"

LqrCalc::LqrCalc(
    double M_, double m_, double I_, double i_, double h_, double r_,
    const Eigen::Vector4d& Q_, double R_)
{
    M = M_;
    m = m_;
    I = I_;
    i = i_;
    h = h_;
    r = r_;
    Q = Q_;
    R = R_;
}


LqrCalc::~LqrCalc()
{
}

void LqrCalc::setParam(
    double M_, double m_, double I_, double i_, double h_, double r_,
    const Eigen::Vector4d& Q_, double R_)
{
    M = M_;
    m = m_;
    I = I_;
    i = i_;
    h = h_;
    r = r_;
    Q = Q_;
    R = R_;
}

Eigen::Vector4d LqrCalc::getLqrK()
{
    double Z = M + m + i / (r * r);
    double D = Z * (I + M * h * h) - M * M * h * h;

    double a = (M * g * h * Z) / D;
    double b = -(M * M * h * h * g) / D;
    double c = -(M * h) / (D * r);
    double d = 1 / (Z * r) - (M * M * h * h) / (D * Z * r);

    Eigen::Matrix4d A;

    A << 0,       1,      0,      0,
         a,       0,      0,      0,
         0,       0,      0,      1,
         b,       0,      0,      0;

    Eigen::Matrix<double, 4, 1> B;

    B << 0,
         c,
         0,
         d;

    Eigen::Vector4d K;

    K = calcLqr(A, B, Q, R);

    return K;
}




Eigen::Vector4d calcLqr(const Eigen::Matrix4d A, const Eigen::Matrix<double, 4, 1> B, const Eigen::Vector4d Q_, const double R_)
{
    const int n = 4;
    const double tol = 1e-9;

    if (R_ <= 0.0) {
        throw std::runtime_error("R must be positive");
    }
    double R_inv = 1.0 / R_;

    // 将 Q_ 转换为对角矩阵 Q
    Eigen::Matrix4d Q = Q_.asDiagonal();

    // 构建 8x8 Hamilton 矩阵
    Eigen::Matrix<double, 8, 8> H;
    H.setZero();
    H.topLeftCorner(n, n)     = A;
    H.topRightCorner(n, n)    = -B * R_inv * B.transpose(); // 4x4
    H.bottomLeftCorner(n, n)  = -Q;
    H.bottomRightCorner(n, n) = -A.transpose();

    // 求特征值/特征向量
    Eigen::EigenSolver<Eigen::Matrix<double, 8, 8>> es(H);
    if (es.info() != Eigen::Success) {
        throw std::runtime_error("Eigen decomposition of Hamiltonian failed");
    }

    Eigen::Matrix<std::complex<double>, 8, 1> eigvals = es.eigenvalues();
    Eigen::Matrix<std::complex<double>, 8, 8> eigvecs = es.eigenvectors();

    // 选择具有严格负实部的 n 个特征值对应的特征向量（稳定子空间）
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    for (int i = 0; i < 2 * n; ++i) {
        if (eigvals(i).real() < -tol) {
            stable_indices.push_back(i);
        }
    }

    // 如果精确条件不满足，放宽到实部 < 0
    if (stable_indices.size() != (size_t)n) {
        stable_indices.clear();
        for (int i = 0; i < 2 * n; ++i) {
            if (eigvals(i).real() < 0.0) {
                stable_indices.push_back(i);
            }
        }
    }

    if (stable_indices.size() != (size_t)n) {
        throw std::runtime_error("Cannot find exactly n stable eigenvalues for Hamiltonian (needed for Riccati solution)");
    }

    // 构建 U = [U11; U21]，U11 和 U21 为复矩阵
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> U11(n, n), U21(n, n);
    for (int i = 0; i < n; ++i) {
        U11.col(i) = eigvecs.block(0, stable_indices[i], n, 1);
        U21.col(i) = eigvecs.block(n, stable_indices[i], n, 1);
    }

    // 检查 U11 是否可逆，使用 fullPivLu 判秩
    Eigen::FullPivLU<Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>> lu(U11);
    if (lu.rank() < n) {
        throw std::runtime_error("U11 is rank-deficient; numerical instability or repeated eigenvalues encountered");
    }

    // P = U21 * U11^{-1}
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> P_c = U21 * U11.inverse();

    // 若存在很小的虚部，则忽略（应当为对称实矩阵）
    Eigen::Matrix4d P = P_c.real();

    // 对称化（数值稳定）
    Eigen::Matrix4d P_sym = (P + P.transpose()) * 0.5;

    // 计算反馈增益 k (列向量)，注意 LQR 的 K_row = R^{-1} B^T P，
    // 这里返回 k_col = (K_row)^T，使得 u = -K_row * x = -k_col.transpose() * x
    Eigen::Vector4d k = (R_inv * B.transpose() * P_sym).transpose();

    return k;
}
