import numpy
import numpy as np
from math import cos, sin, sqrt, acos
import os
from datetime import datetime

#拟合功能
def polyfit(x, y, degree):
    results = {}
    coeffs = numpy.polyfit(x, y, degree)
    results['polynomial'] = coeffs.tolist()
    p = numpy.poly1d(coeffs)
    yhat = p(x)
    ybar = numpy.sum(y)/len(y)
    ssreg = numpy.sum((yhat-ybar)**2)
    sstot = numpy.sum((y - ybar)**2)
    results['determination'] = ssreg / sstot #准确率
    return results

def Print_Poly_eq(Poly_eq):
    coeffs = Poly_eq['polynomial']
    equation_parts = []

    for i, coeff in enumerate(coeffs):
        power = len(coeffs) - i - 1
        
        if abs(coeff) < 1e-10:  # 忽略接近零的系数
            continue
        
        # 格式化系数
        if power == 0:
            term = f"{coeff:.6f}"
        elif power == 1:
            term = f"{coeff:.6f}*x"
        else:
            term = f"{coeff:.6f}*x^{power}"
        # 处理符号
        if coeff >= 0 and equation_parts:
            term = "+ " + term
        elif coeff < 0:
            term = "- " + term.lstrip("-")
        
        equation_parts.append(term)

    equation = "y = " + " ".join(equation_parts)
    print(f"拟合方程: {equation}")
    print(f"决定系数 R² = {Poly_eq['determination']:.6f}")

def Poly_eq_test(Poly_eq,x_value):
    poly_func = numpy.poly1d(Poly_eq['polynomial'])
    y_predicted = poly_func(x_value)
    print(f"当 x = {x_value} 时,y = {y_predicted}")

def calc_lqr_eigen(A, B, Q_vec, R):
    n = A.shape[0]  # 应该是4
    
    if R <= 0.0:
        raise ValueError("R must be positive")
    
    R_inv = 1.0 / R
    
    # 将Q_vec转换为对角矩阵
    Q = np.diag(Q_vec)
    
    # 构建8x8 Hamilton矩阵
    H = np.zeros((2*n, 2*n), dtype=np.complex128)
    H[:n, :n] = A
    H[:n, n:] = -B @ B.T * R_inv
    H[n:, :n] = -Q
    H[n:, n:] = -A.T
    
    # 计算特征值和特征向量
    eigvals, eigvecs = np.linalg.eig(H)
    
    # 选择具有严格负实部的特征值对应的特征向量（稳定子空间）
    stable_indices = []
    for i in range(2*n):
        if eigvals[i].real < -1e-9:
            stable_indices.append(i)
    
    # 如果精确条件不满足，放宽到实部 < 0
    if len(stable_indices) != n:
        stable_indices = []
        for i in range(2*n):
            if eigvals[i].real < 0.0:
                stable_indices.append(i)
    
    if len(stable_indices) != n:
        raise RuntimeError(f"Cannot find exactly {n} stable eigenvalues for Hamiltonian")
    
    # 构建U11和U21
    U11 = np.zeros((n, n), dtype=np.complex128)
    U21 = np.zeros((n, n), dtype=np.complex128)
    
    for i, idx in enumerate(stable_indices):
        U11[:, i] = eigvecs[:n, idx]
        U21[:, i] = eigvecs[n:, idx]
    
    # 检查U11是否可逆
    if np.linalg.matrix_rank(U11) < n:
        raise RuntimeError("U11 is rank-deficient")
    
    # 计算P = U21 * U11^{-1}
    P_complex = U21 @ np.linalg.inv(U11)
    
    # 取实部并对称化
    P_real = P_complex.real
    P_sym = 0.5 * (P_real + P_real.T)
    
    # 计算反馈增益 k = R^{-1} * B^T * P 的转置（列向量）
    K_row = R_inv * B.T @ P_sym  # 1x4 行向量
    k = K_row.T  # 4x1 列向量
    
    return k.flatten()  # 返回一维数组

def LQR_calc(M,m,I,i,h,r,Q,R,g):   
    Z = M + m + i / (r * r)
    D = Z * (I + M * h * h) - M * M * h * h

    a = (M * g * h * Z) / D
    b = -(M * M * h * h * g) / D
    c = -(M * h) / (D * r)
    d = 1 / (Z * r) - (M * M * h * h) / (D * Z * r)

    A = np.array([
        [0,1,0,0],
        [a,0,0,0],
        [0,0,0,1],
        [b,0,0,0]
    ])
    B = np.array([
        [0],
        [c],
        [0],
        [d]
    ])

    return calc_lqr_eigen(A,B,Q,R)

def LQR_Poly_eq(Poly_eq,param_name):
    coeffs = Poly_eq['polynomial']
    equation_parts = []

    for i, coeff in enumerate(coeffs):
        power = len(coeffs) - i - 1
        
        if abs(coeff) < 1e-10:  # 忽略接近零的系数
            continue
        
        # 格式化系数
        if power == 0:
            term = f"{coeff:.6f}"
        elif power == 1:
            term = f"{coeff:.6f}*{param_name}"
        else:
            term = f"{coeff:.6f}*{param_name}"+f"*{param_name}" *(power -1)
            #term = f"{coeff:.6f}*x*{power}"
        
        # 处理符号
        if coeff >= 0 and equation_parts:
            term = "+ " + term
        elif coeff < 0:
            term = "- " + term.lstrip("-")
        
        equation_parts.append(term)

    equation = " ".join(equation_parts)
    return equation


def forwardKinematics(L1, L2, L3, L4, L23, alpha):
    # 计算角度BAD
    angle_BAD = np.pi/4 - alpha
    # 计算BD的长度
    L_BD = sqrt(L2**2 + L4**2 - 2*L2*L4*cos(angle_BAD))
    
    # 计算中间值
    middle_val1 = (L_BD**2 + L2**2 - L4**2) / (2*L_BD*L2)
    middle_val2 = (L_BD**2 + L23**2 - L3**2) / (2*L_BD*L23)
    
    # 确保中间值在[-1, 1]范围内，避免acos数值错误
    middle_val1 = np.clip(middle_val1, -1.0, 1.0)
    middle_val2 = np.clip(middle_val2, -1.0, 1.0)
    
    # 计算belta角度
    belta = np.pi + alpha - acos(middle_val1) - acos(middle_val2)
    
    # 直接计算x和y坐标（不使用sympy，更高效）
    x = L2 * cos(alpha) - L1 * cos(belta)
    y = L2 * sin(alpha) - L1 * sin(belta)
    
    result = {
        'x': float(x),
        'y': float(y),
        'belta': float(belta)  # 可选：返回belta角度
    }
    
    return result


def Write_cpp_content(Header_File_name,namespace,class_name,name,poly_func):
    header_content  = f"""#ifndef __{class_name.upper()}_H_
#define __{class_name.upper()}_H_

typedef struct{{
	float K1;
	float K2;
	float K3;
	float K4;
}}LQR_K;
typedef LQR_K (*FitFunction)(float Leg_H);

#ifdef __cplusplus
namespace {namespace}{{
{';\n '.join([f'LQR_K {func}(float Leg_H)' for i,func in name])};
    class {class_name}{{
    public:
    {class_name}(){{
{';\n '.join([f'fitFunctions[{i}]={func}' for i,func in name])};
    }}
    
    FitFunction fitFunctions[10]; 
    private:

    }};
}}
#endif
#endif
"""
    source_content = f"""#include "{Header_File_name}"

namespace {namespace}{{
{'\n\n '.join([f'LQR_K {func}(float Leg_H){{LQR_K K;K.K1 = {K1};K.K2 = {K2};K.K3 = {K3};K.K4 = {K4};return K;}}' 
                    for func, K1,K2,K3,K4 in poly_func
                ])};
    }}
    """ 
    return header_content,source_content,Header_File_name

def Generate_content(header_content,source_content,header_filename,
                     source_filename,header_path,source_path):
    # 创建输出目录
    os.makedirs(header_path, exist_ok=True)

    # 保存头文件
    H_path = os.path.join(header_path, header_filename)

    with open(H_path, 'w', encoding='utf-8') as f:
        f.write(header_content)
    print(f"✓ 头文件已保存: {H_path}")
    
    os.makedirs(source_path,exist_ok=True)
    # 保存源文件
    s_path = os.path.join(source_path, source_filename)
    with open(s_path, 'w', encoding='utf-8') as f:
        f.write(source_content)
    print(f"✓ 源文件已保存: {s_path}")
    
    return H_path, s_path



