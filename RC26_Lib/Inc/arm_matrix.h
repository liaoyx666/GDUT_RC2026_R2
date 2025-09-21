#ifndef ARM_MATRIX_H
#define ARM_MATRIX_H

#include <arm_math.h>
#include <cstring>
/*
		brief     将arm_math库再次封装使得调用更加方便
		param			实现了大部分的功能如:
		
		
							矩阵的创建和初始化
							单位矩阵的创建
							矩阵的加减法和乘法以及矩阵的数乘
							矩阵的转置，求逆
							矩阵的复制
							获取行列数
							矩阵元素的获取与修改
							
							
							
							how to use
							1.矩阵的创建
							ArmMatrix<3, 3> mat_3x3;  // 3x3 零矩阵
							ArmVector<5> vec_5;       // 5 元素零向量
							
							2.矩阵的初始化
							(数组需按 行优先 存储，长度不小于 ROWS * COLS)
							float arr_2x2[4] = {1.0f, 2.0f, 3.0f, 4.0f};
							ArmMatrix<2, 2> mat_from_arr(arr_2x2);  // 从数组创建 2x2 矩阵
							
							
							3.矩阵的拷贝
							ArmMatrix<2, 2> mat_a;
							ArmMatrix<2, 2> mat_b(mat_a);  // 拷贝 mat_a 到 mat_b
							
							
							4.单位矩阵的初始化
							ArmMatrix<4, 4> identity_mat;
							identity_mat.setIdentity();  // 4x4 单位矩阵
							
							
							5.元素访问
							ArmMatrix<2, 3> mat;
							mat(0, 1) = 2.5f;  // 设置第 0 行第 1 列元素为 2.5
							float val = mat(1, 2);  // 获取第 1 行第 2 列元素
							
							
							6.元素修改
							ArmMatrix<2, 3> mat;
							mat(0,1) = 2
							
							
							7.矩阵加法
							ArmMatrix<2, 2> A, B, C;
							A(0,0) = 1; A(0,1) = 2; A(1,0) = 3; A(1,1) = 4;
							B(0,0) = 5; B(0,1) = 6; B(1,0) = 7; B(1,1) = 8;
							C = A + B;  
							// C = [[6,8],[10,12]]
							
							
							8.矩阵剑法
							ArmMatrix<2, 2> D = A - B;  // D = [[-4,-4],[-4,-4]]
							
							
							9.矩阵数乘
							ArmMatrix<2, 2> E = A * 2.0f;  
							// E = [[2,4],[6,8]]
							
							
							10.矩阵乘法
							ArmMatrix<3, 2> mat_left;  // 3x2 矩阵
							ArmMatrix<2, 4> mat_right; // 2x4 矩阵
							ArmMatrix<3, 4> mat_product = mat_left * mat_right;  // 结果为 3x4 矩阵
							
							
							11.矩阵转置
							ArmMatrix<2, 3> original;
							original(0,0) = 1; original(0,1) = 2; original(0,2) = 3;
							original(1,0) = 4; original(1,1) = 5; original(1,2) = 6;
							ArmMatrix<3, 2> transposed = original.transpose();  
							// 转置为 3x2 矩阵
							
							
							12.矩阵求逆	
							ArmMatrix<3, 3> mat, mat_inv;
							if (mat.invert()) 
							{
										mat_inv = mat;  // 保存逆矩阵到 mat_inv
							}
							//需要注意的是，这里矩阵求逆的返回值是0，1
							//当返回值为1时，说明原矩阵已经被逆矩阵所覆盖

							
							13.从外部复制到矩阵
							ArmMatrix<2, 3> mat;
							float src_arr[6] = {10, 20, 30, 40, 50, 60};
							mat.copyFrom(src_arr); 
							// 矩阵元素变为 [[10,20,30],[40,50,60]]
							
							
							14.从矩阵复制到外部
							float dest_arr[4];
							ArmMatrix<2, 2> mat;
							mat(0,0) = 1; mat(0,1) = 2; mat(1,0) = 3; mat(1,1) = 4;
							mat.copyTo(dest_arr);  // dest_arr = [1,2,3,4]
							
							
							15.获取元素指针
							ArmMatrix<3, 3> mat;
							float* data_ptr = mat.getData();  // 获取数据指针
							data_ptr[0] = 1.0f;  // 直接修改第 0 行第 0 列元素
							
							
							16.获取行列数
							ArmMatrix<4, 2> mat;
							int row_cnt = mat.rows();  // row_cnt = 4
							int col_cnt = mat.cols();  // col_cnt = 2
							
							
							
							
							
*/	
// 矩阵类模板，封装ARM CMSIS-DSP的矩阵操作
template<int ROWS, int COLS>
class ArmMatrix {
public:
    // 构造函数
    ArmMatrix() 
    {
        handle.numRows = ROWS;
        handle.numCols = COLS; 
        handle.pData = data;
        memset(data, 0, sizeof(data));
    }

    // 从数组初始化矩阵
    ArmMatrix(const float* initData) {
        handle.numRows = ROWS;
        handle.numCols = COLS;  
        handle.pData = data;
        memcpy(data, initData, sizeof(data));
    }

    // 拷贝构造函数
    ArmMatrix(const ArmMatrix<ROWS, COLS>& other) {
        handle.numRows = ROWS;
        handle.numCols = COLS;  
        handle.pData = data;
        memcpy(data, other.data, sizeof(data));
    }

    // 赋值运算符
    ArmMatrix<ROWS, COLS>& operator=(const ArmMatrix<ROWS, COLS>& other) {
        if (this != &other) {
            memcpy(data, other.data, sizeof(data));
        }
        return *this;
    }

    // 索引访问（行, 列）
    float& operator()(int row, int col) {
        // 调试模式下检查索引范围
        #ifdef DEBUG
        if (row < 0 || row >= ROWS || col < 0 || col >= COLS) {
            // 索引越界，可添加错误处理
            while(1);
        }
        #endif
        return data[row * COLS + col];
    }

    // 常量索引访问
    const float& operator()(int row, int col) const {
        return data[row * COLS + col];
    }

    // 向量专用索引访问（仅适用于列向量）
    template<int C = COLS>
    typename std::enable_if<C == 1, float&>::type operator()(int index) {
        #ifdef DEBUG
        if (index < 0 || index >= ROWS) {
            while(1);  // 索引越界
        }
        #endif
        return data[index];
    }

    // 向量专用常量索引访问
    template<int C = COLS>
    typename std::enable_if<C == 1, const float&>::type operator()(int index) const {
        return data[index];
    }

    // 矩阵乘法
    template<int COLS2>
    ArmMatrix<ROWS, COLS2> operator*(const ArmMatrix<COLS, COLS2>& other) const {
        ArmMatrix<ROWS, COLS2> result;
        // 使用公共接口getHandle()访问句柄，而非直接访问私有成员
        arm_mat_mult_f32(getHandle(), other.getHandle(), result.getHandle());
        return result;
    }

    // 矩阵加法
    ArmMatrix<ROWS, COLS> operator+(const ArmMatrix<ROWS, COLS>& other) const {
        ArmMatrix<ROWS, COLS> result;
        arm_mat_add_f32(getHandle(), other.getHandle(), result.getHandle());
        return result;
    }

    // 矩阵减法
    ArmMatrix<ROWS, COLS> operator-(const ArmMatrix<ROWS, COLS>& other) const {
        ArmMatrix<ROWS, COLS> result;
        arm_mat_sub_f32(getHandle(), other.getHandle(), result.getHandle());
        return result;
    }

    // 矩阵数乘
    ArmMatrix<ROWS, COLS> operator*(float scalar) const {
        ArmMatrix<ROWS, COLS> result;
        arm_mat_scale_f32(getHandle(), scalar, result.getHandle());
        return result;
    }

    // 矩阵转置
    ArmMatrix<COLS, ROWS> transpose() const {
        ArmMatrix<COLS, ROWS> result;
        arm_mat_trans_f32(getHandle(), result.getHandle());
        return result;
    }

    // 设置为单位矩阵
    void setIdentity() {
        memset(data, 0, sizeof(data));
        for (int i = 0; i < ROWS && i < COLS; ++i) {
            data[i * COLS + i] = 1.0f;
        }
    }

    // 矩阵求逆（仅适用于方阵）
    bool invert() {
        #if ROWS != COLS
            // 非方阵不能求逆
            return false;
        #else
            ArmMatrix<ROWS, COLS> temp(*this);
            // 使用getHandle()访问句柄
            arm_status status = arm_mat_inverse_f32(temp.getHandle(), getHandle());
            return status == ARM_MATH_SUCCESS;
        #endif
    }

    // 从数组复制数据
    void copyFrom(const float* src) {
        memcpy(data, src, sizeof(data));
    }

    // 复制数据到数组
    void copyTo(float* dest) const {
        memcpy(dest, data, sizeof(data));
    }

    // 获取ARM矩阵句柄（公共接口，替代直接访问私有成员）
    const arm_matrix_instance_f32* getHandle() const {
        return &handle;
    }

    // 获取非const句柄（用于需要修改矩阵的CMSIS-DSP函数）
    arm_matrix_instance_f32* getHandle() {
        return &handle;
    }

    // 获取数据指针
    float* getData() {
        return data;
    }

    // 获取常量数据指针
    const float* getData() const {
        return data;
    }

    // 获取行数
    int rows() const { return ROWS; }

    // 获取列数
    int cols() const { return COLS; }

private:
    arm_matrix_instance_f32 handle;  // ARM矩阵句柄（私有成员）
    float data[ROWS * COLS];         // 矩阵数据（行优先存储）
};

// 向量类（列向量）
template<int SIZE>
using ArmVector = ArmMatrix<SIZE, 1>;

#endif // ARM_MATRIX_H
    