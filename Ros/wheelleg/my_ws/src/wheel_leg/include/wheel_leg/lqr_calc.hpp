#pragma once
#include <Eigen/Dense>
#include <vector>
#include <numeric>

class LqrCalc
{
private:
    double M = 0; // 车身质量
    double m = 0; // 轮子质量
    double I = 0; // 车身转动惯量
    double i = 0; // 轮子转动惯量
    double h = 0; // 车身重心高度
    double r = 0; // 轮子半径
    const double g = 9.8; // 重力加速度

    Eigen::Vector4d Q = Eigen::Vector4d(1, 1, 1, 1); // 状态权重矩阵
    double R = 5;// 控制权重矩阵
public:
    LqrCalc(
        double M_, double m_, double I_, double i_, double h_, double r_,
        const Eigen::Vector4d& Q_, double R_);


    virtual ~LqrCalc();

    void setParam(
        double M_, double m_, double I_, double i_, double h_, double r_,
        const Eigen::Vector4d& Q_, double R_);
    
    Eigen::Vector4d getLqrK();

    
};


Eigen::Vector4d calcLqr(const Eigen::Matrix4d A, const Eigen::Matrix<double, 4, 1> B, const Eigen::Vector4d Q_, const double R_);