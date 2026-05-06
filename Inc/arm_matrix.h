#ifndef ARM_MATRIX_H  // 检查宏是否未定义

#define ARM_MATRIX_H
#ifdef __cplusplus
#include "arm_math.h"
#include <string.h>  // 替换 <cstring>：Keil 完美支持 C 标准库的 <string.h>
#include <stdint.h>  // 补充 Keil 常用头文件，确保 memset/memcpy 等函数声明
#include <type_traits>
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
							mat(0,1) = 2.0f;
							
							
							7.矩阵加法
							ArmMatrix<2, 2> A, B, C;
							A(0,0) = 1; A(0,1) = 2; A(1,0) = 3; A(1,1) = 4;
							B(0,0) = 5; B(0,1) = 6; B(1,0) = 7; B(1,1) = 8;
							C = A + B;  
							// C = [[6,8],[10,12]]
							
							
							8.矩阵减法
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
							//需要注意的是，这里矩阵求逆的返回值是0（失败）或1（成功）
							//当返回值为1时，原矩阵已被逆矩阵覆盖

							
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
							
							17.矩阵置零
*/	


/**
 * @brief 矩阵类模板，封装ARM CMSIS-DSP的矩阵操作
 * @tparam ROWS 矩阵行数（编译期常量）
 * @tparam COLS 矩阵列数（编译期常量）
 * 特性：静态内存分配，无动态内存操作，适合嵌入式系统
 */
template<int ROWS, int COLS>
class ArmMatrix
	{
public:
    /**
     * @brief 构造函数，初始化矩阵句柄并置零所有元素
     */
    ArmMatrix() 
    {
        handle.numRows = ROWS;
        handle.numCols = COLS; 
        handle.pData = data;
        setZero(); // 构造时自动置零
    }

    /**
     * @brief 从外部数组初始化矩阵
     * @param initData 初始化数据（行优先存储）
     */
    explicit ArmMatrix(const float* initData) 
			{
        handle.numRows = ROWS;
        handle.numCols = COLS;  
        handle.pData = data;
        memcpy(data, initData, sizeof(data));
    }

    /**
     * @brief 拷贝构造函数
     * @param other 待拷贝的矩阵
     */
    ArmMatrix(const ArmMatrix<ROWS, COLS>& other) 
			{
        handle.numRows = ROWS;
        handle.numCols = COLS;  
        handle.pData = data;
        memcpy(data, other.data, sizeof(data));
    }

    /**
     * @brief 赋值运算符
     * @param other 待赋值的矩阵
     * @return 自身引用
     */
    ArmMatrix<ROWS, COLS>& operator=(const ArmMatrix<ROWS, COLS>& other)
			{
        if (this != &other) {
            memcpy(data, other.data, sizeof(data));
        }
        return *this;
    }

    /**
     * @brief 将矩阵所有元素置为0.0f
     * 实现：通过循环遍历所有元素，确保兼容性
     */
    void setZero()
			{
        for (int i = 0; i < ROWS * COLS; ++i) 
				{
            data[i] = 0.0f;
        }
    }

    /**
     * @brief 矩阵元素访问（行, 列）- 非const版本
     * @param row 行索引（0-based）
     * @param col 列索引（0-based）
     * @return 元素引用
     */
    float& operator()(int row, int col)
			{
        #ifdef DEBUG
        if ((row < 0) || (row >= ROWS) || (col < 0) || (col >= COLS)) 
				{
            while(1); // 索引越界时死循环，便于调试
        }
        #endif
        return data[row * COLS + col]; // 行优先存储
    }

    /**
     * @brief 矩阵元素访问（行, 列）- const版本
     * @param row 行索引（0-based）
     * @param col 列索引（0-based）
     * @return 常量元素引用
     */
    const float& operator()(int row, int col) const 
			{
        #ifdef DEBUG
        if ((row < 0) || (row >= ROWS) || (col < 0) || (col >= COLS)) 
				{
            while(1);
        }
        #endif
        return data[row * COLS + col];
    }

    /**
     * @brief 向量专用访问（仅适用于列向量）
     * @param index 元素索引（0-based）
     * @return 元素引用
     */
    template<int C = COLS>
    typename std::enable_if<C == 1, float&>::type operator()(int index) 
			{
        #ifdef DEBUG
        if ((index < 0) || (index >= ROWS))
				{
            while(1);
        }
        #endif
        return data[index];
    }

    /**
     * @brief 向量专用访问（const版本，仅适用于列向量）
     * @param index 元素索引（0-based）
     * @return 常量元素引用
     */
    template<int C = COLS>
    typename std::enable_if<C == 1, const float&>::type operator()(int index) const 
			{
        #ifdef DEBUG
        if ((index < 0) || (index >= ROWS))
				{
            while(1);
        }
        #endif
        return data[index];
    }

    /**
     * @brief 矩阵乘法
     * @tparam COLS2 右矩阵列数
     * @param other 右矩阵（COLS行 × COLS2列）
     * @return 乘积矩阵（ROWS行 × COLS2列）
     */
    template<int COLS2>
    ArmMatrix<ROWS, COLS2> operator*(const ArmMatrix<COLS, COLS2>& other) const 
			{
        ArmMatrix<ROWS, COLS2> result;
        arm_mat_mult_f32(this->getHandle(), other.getHandle(), result.getHandle());
        return result;
    }

    /**
     * @brief 矩阵加法
     * @param other 同维度矩阵
     * @return 和矩阵
     */
    ArmMatrix<ROWS, COLS> operator+(const ArmMatrix<ROWS, COLS>& other) const
			{
        ArmMatrix<ROWS, COLS> result;
        arm_mat_add_f32(this->getHandle(), other.getHandle(), result.getHandle());
        return result;
    }

    /**
     * @brief 矩阵减法
     * @param other 同维度矩阵
     * @return 差矩阵
     */
    ArmMatrix<ROWS, COLS> operator-(const ArmMatrix<ROWS, COLS>& other) const 
			{
        ArmMatrix<ROWS, COLS> result;
        arm_mat_sub_f32(this->getHandle(), other.getHandle(), result.getHandle());
        return result;
    }

    /**
     * @brief 矩阵数乘
     * @param scalar 标量因子
     * @return 缩放后的矩阵
     */
    ArmMatrix<ROWS, COLS> operator*(float scalar) const 
			{
        ArmMatrix<ROWS, COLS> result;
        arm_mat_scale_f32(this->getHandle(), scalar, result.getHandle());
        return result;
    }

    /**
     * @brief 矩阵转置
     * @return 转置矩阵（COLS行 × ROWS列）
     */
    ArmMatrix<COLS, ROWS> transpose() const
			{
        ArmMatrix<COLS, ROWS> result;
        arm_mat_trans_f32(this->getHandle(), result.getHandle());
        return result;
    }

    /**
     * @brief 设置为单位矩阵
     */
    void setIdentity()
			{
        setZero(); // 先置零（使用新增的setZero函数）
        const int min_dim = (ROWS < COLS) ? ROWS : COLS;
        for (int i = 0; i < min_dim; ++i)
				{
            data[i * COLS + i] = 1.0f;
        }
    }

    /**
     * @brief 矩阵求逆
     * @return 求逆成功返回true，否则返回false
     */
    bool invert()
			{
        #if ROWS != COLS
            return false; // 非方阵不能求逆
        #else
            ArmMatrix<ROWS, COLS> temp(*this);
            arm_status status = arm_mat_inverse_f32(temp.getHandle(), this->getHandle());
            return (status == ARM_MATH_SUCCESS);
        #endif
    }

    /**
     * @brief 从外部数组复制数据
     * @param src 源数据指针
     */
    void copyFrom(const float* src)
			{
        memcpy(data, src, sizeof(data));
    }

    /**
     * @brief 复制数据到外部数组
     * @param dest 目标数组指针
     */
    void copyTo(float* dest) const 
			{
        memcpy(dest, data, sizeof(data));
    }

    /**
     * @brief 获取ARM矩阵句柄
     * @return 句柄指针
     */
    const arm_matrix_instance_f32* getHandle() const
			{
        return &handle;
    }

    /**
     * @brief 获取ARM矩阵句柄
     * @return 句柄指针
     */
    arm_matrix_instance_f32* getHandle()
		{
        return &handle;
    }

    /**
     * @brief 获取数据指针
     * @return 数据数组指针
     */
    float* getData() 
			{
        return data;
    }

    /**
     * @brief 获取数据指针
     * @return 常量数据数组指针
     */
    const float* getData() const
			{
        return data;
    }

    /**
     * @brief 获取矩阵行数
     * @return 行数
     */
    int rows() const { return ROWS; }

    /**
     * @brief 获取矩阵列数
     * @return 列数
     */
    int cols() const { return COLS; }

private:
    arm_matrix_instance_f32 handle; 
    float data[ROWS * COLS];       
};

/**
 * @brief 列向量类型定义（简化常用场景）
 * @tparam SIZE 向量长度
 */
template<int SIZE>
using ArmVector = ArmMatrix<SIZE, 1>;

#endif // __cplusplus
#endif // ARM_MATRIX_H
