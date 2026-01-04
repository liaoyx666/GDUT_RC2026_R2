#ifndef RC_MATRIX_H
#define RC_MATRIX_H


#ifdef __cplusplus  
#include <assert.h> 
#include "arm_math.h"

namespace MathLib {
    
    template <size_t row,size_t col>
    class matrix
    {
    private:
        size_t assign_index = 0;
        float32_t data[row*col];
        arm_matrix_instance_f32 dsp_mat;
    public:
        matrix(){
            arm_mat_init_f32(&dsp_mat,row, col, data);
            arm_fill_f32(0.0f,data,row*col);
            assign_index = 0;
        }
        matrix(const matrix& other) {
            
            for (size_t i = 0; i < row * col; ++i) {
                data[i] = other.data[i];
            }
            arm_mat_init_f32(&dsp_mat, row, col, data);
            assign_index = other.assign_index;
        }

        
        matrix& operator=(const matrix& other) {
            if (this != &other) {  
                
                for (size_t i = 0; i < row * col; ++i) {
                    data[i] = other.data[i];
                }
                
                arm_mat_init_f32(&dsp_mat, row, col, data);
                assign_index = other.assign_index;
            }
            return *this;
        }
        
        ~matrix() {}
        size_t rows() const { return row; }
        size_t cols() const { return col; } 
        matrix<row,col> &operator <<(const float32_t &val) {
            assign_index = 0;
            if(assign_index <row*col){
                data[assign_index++] = val;
            }
            return *this;
        }
        matrix<row,col> &operator ,(const float32_t &val) {
            if(assign_index < row*col){
                data[assign_index++] = val;
            }else{
                assert(false && "Too many elements assigned to matrix");
            }
            return *this;
        }

        const arm_matrix_instance_f32* get_dsp_mat() const {
            return &dsp_mat;
        }
        arm_matrix_instance_f32* get_dsp_mat_noconst(){
            return &dsp_mat;
        }
        
        float32_t& operator()(size_t r, size_t c) {
            assert(r < row && c < col && "Matrix index out of bounds");
            return (r < row && c < col) ? data[r * col + c] : data[0];
        }
        
        matrix<row, col> operator +(const matrix &val) const{
            matrix<row, col> result;
            arm_status status = arm_mat_add_f32(&dsp_mat,val.get_dsp_mat(),result.get_dsp_mat_noconst());
            assert(status == ARM_MATH_SUCCESS && "Matrix add fail");
            return result;
        }
        matrix<row, col> operator -(const matrix &val) const{
            matrix<row, col> result;
            arm_status status = arm_mat_sub_f32(&dsp_mat,val.get_dsp_mat(),result.get_dsp_mat_noconst());
            assert(status == ARM_MATH_SUCCESS && "Matrix sub fail");
            return result;
        }
        matrix<row,col> operator*(float32_t val)const{
            matrix<row,  col> result;
            arm_status status = arm_mat_scale_f32(&dsp_mat,val,result.get_dsp_mat_noconst());
            assert(status == ARM_MATH_SUCCESS && "Matrix scale fail");
            return result;
        }

        
        template <size_t row1, size_t col1, size_t col2>
        friend matrix<row1, col2> operator*(const matrix<row1, col1>& matA, const matrix<col1, col2>& matB);
        
        template <size_t r,size_t c>
        friend matrix<r, c> operator*(float32_t val, const matrix<r, c>& mat);
        

        //DSP????
        template <size_t r, size_t c>
        friend matrix<c, r> transpose(const matrix<r, c>& mat);
        template <size_t n>
        friend matrix<n, n> inverse(const matrix<n, n>& mat);
    };
    
    
    template <size_t row1, size_t col1, size_t col2>
    matrix<row1, col2> operator*(const matrix<row1, col1>& matA, const matrix<col1, col2>& matB) {
        matrix<row1, col2> result;
        arm_status status = arm_mat_mult_f32(matA.get_dsp_mat(),matB.get_dsp_mat(),result.get_dsp_mat_noconst() );
        assert(status == ARM_MATH_SUCCESS && "Matrix multi failed!");
        return result;
    }

    template <size_t r,size_t c>
    matrix<r, c> operator*(float32_t val, const matrix<r, c>& mat){
        matrix<r,  c> result;
        arm_status status = arm_mat_scale_f32(mat.get_dsp_mat(),val,result.get_dsp_mat_noconst() );
        assert(status == ARM_MATH_SUCCESS && "Matrix scale fail");
        return result;
    }

    
    template <size_t r, size_t c>
    matrix<c, r> transpose(const matrix<r, c>& mat) {
        matrix<c, r> result;
        arm_status status = arm_mat_trans_f32(mat.get_dsp_mat(),result.get_dsp_mat_noconst());
        assert(status == ARM_MATH_SUCCESS && "Matrix transpose failed!");
        return result;
    }
    
    template <size_t n>
    matrix<n, n> inverse(const matrix<n, n>& mat) {
        matrix<n, n> result;
        matrix<n, n> mat_copy = mat;
        arm_status status = arm_mat_inverse_f32(mat_copy.get_dsp_mat(),result.get_dsp_mat_noconst());
        assert(status == ARM_MATH_SUCCESS && "Matrix inverse failed!");
        return result;
    }
}

#endif 
#endif 