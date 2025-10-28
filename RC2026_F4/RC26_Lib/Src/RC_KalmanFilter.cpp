#include "RC_KalmanFilter.h"

namespace KalmanFilter{
    void KalmanFilter_Base::X_calculate(const float32_t* control_input){
        //求解X
            // X = A * X + B * u (如果有控制输入)
            arm_matrix_instance_f32 AX;
            arm_matrix_instance_f32 BU;
            
            float32_t AX_data[state_dim];
            float32_t BU_data[state_dim];
            arm_mat_init_f32(&AX, state_dim, 1, AX_data);
            arm_mat_init_f32(&BU, state_dim, 1, BU_data);
            arm_mat_mult_f32(&my_element.A, &my_element.X, &AX);
            float32_t u_data[control_dim];
            for (uint16_t i = 0; i < control_dim; ++i) {
                u_data[i] = control_input[i];
            }
            arm_matrix_instance_f32 U;
            arm_mat_init_f32(&U, control_dim, 1, u_data);
            arm_mat_mult_f32(&my_element.B, &U, &BU);
            
            arm_status status =arm_mat_add_f32(&AX, &BU, &my_element.X);

            if(status == ARM_MATH_SIZE_MISMATCH){
                // 处理矩阵维度不匹配错误
                return;
            }
    }

    void KalmanFilter_Base::K_calculate(void){
        //求解K
            //K = P * H' /(H * P * H' + R)
            arm_matrix_instance_f32 HT;
            float32_t HT_data[state_dim * meas_dim];
            arm_mat_init_f32(&HT, state_dim, meas_dim, HT_data);
            // 转置H矩阵
            for (uint16_t i = 0; i < meas_dim; ++i){
                for (uint16_t j = 0; j < state_dim; ++j){
                    HT_data[j * meas_dim + i] = my_element.H.pData[i * state_dim + j];
                }
            }
            arm_matrix_instance_f32 PHt;
            float32_t PHt_data[state_dim * meas_dim];
            arm_mat_init_f32(&PHt, state_dim, meas_dim, PHt_data);
            arm_mat_mult_f32(&my_element.P, &HT, &PHt);
            arm_matrix_instance_f32 HP;
            float32_t HP_data[meas_dim * state_dim];
            arm_mat_init_f32(&HP, meas_dim, state_dim, HP_data);
            arm_mat_mult_f32(&my_element.H, &my_element.P, &HP);
            arm_matrix_instance_f32 HPHt;
            float32_t HPHt_data[meas_dim * meas_dim];
            arm_mat_init_f32(&HPHt, meas_dim, meas_dim, HPHt_data);
            arm_mat_mult_f32(&HP, &HT, &HPHt);
            arm_matrix_instance_f32 S;
            float32_t S_data[meas_dim * meas_dim];
            arm_mat_init_f32(&S, meas_dim, meas_dim, S_data);
            arm_mat_add_f32(&HPHt, &my_element.R, &S);
            // 计算S的逆矩阵
            arm_matrix_instance_f32 S_inv;
            float32_t S_inv_data[meas_dim * meas_dim];
            arm_mat_init_f32(&S_inv, meas_dim, meas_dim, S_inv_data);
            arm_status status_inv = arm_mat_inverse_f32(&S, &S_inv);
            if (status_inv != ARM_MATH_SUCCESS) {
                // 处理矩阵不可逆错误
                return;
            }
            arm_mat_mult_f32(&PHt, &S_inv, &my_element.K);
    }

    void KalmanFilter_Base::P_calculate(void){
        //求解P
            //P = A * P * A' + Q
            arm_matrix_instance_f32 AP;
            float32_t AP_data[state_dim * state_dim];
            arm_mat_init_f32(&AP, state_dim, state_dim, AP_data);
            arm_mat_mult_f32(&my_element.A, &my_element.P, &AP);
            arm_matrix_instance_f32 AT;
            float32_t AT_data[state_dim * state_dim];
            arm_mat_init_f32(&AT, state_dim, state_dim, AT_data);
            // 转置A矩阵
            for (uint16_t i = 0; i < state_dim; ++i) {
                for (uint16_t j = 0; j < state_dim; ++j) {
                    AT_data[j * state_dim + i] = my_element.A.pData[i * state_dim + j];
                }
            }
            arm_matrix_instance_f32 APA_T;
            float32_t APA_T_data[state_dim * state_dim];
            arm_mat_init_f32(&APA_T, state_dim, state_dim, APA_T_data);
            arm_mat_mult_f32(&AP, &AT, &APA_T);
            arm_mat_add_f32(&APA_T, &my_element.Q, &my_element.P);
    }
    void KalmanFilter_Base::X_updata(void){

         //更新X
            //X = X + K * (Z - H * X)
            arm_matrix_instance_f32 HX;
            float32_t HX_data[meas_dim];
            arm_mat_init_f32(&HX, meas_dim, 1, HX_data);
            arm_mat_mult_f32(&my_element.H, &my_element.X, &HX);
            arm_matrix_instance_f32 Z_HX;
            float32_t Z_HX_data[meas_dim];
            arm_mat_init_f32(&Z_HX, meas_dim, 1, Z_HX_data);
            arm_mat_sub_f32(&my_element.Z, &HX, &Z_HX);
            arm_matrix_instance_f32 K_ZHX;
            float32_t K_ZHX_data[state_dim];
            arm_mat_init_f32(&K_ZHX, state_dim, 1, K_ZHX_data);
            arm_mat_mult_f32(&my_element.K, &Z_HX, &K_ZHX);
            arm_mat_add_f32(&my_element.X, &K_ZHX, &my_element.X);
    }

    void KalmanFilter_Base::P_updata(void){
        //更新P
            //P = (I - K * H) * P
            arm_matrix_instance_f32 KH;
            float32_t KH_data[state_dim * state_dim];
            arm_mat_init_f32(&KH, state_dim, state_dim, KH_data);
            arm_mat_mult_f32(&my_element.K, &my_element.H, &KH);
            arm_matrix_instance_f32 I;
            float32_t I_data[state_dim * state_dim];
            arm_mat_init_f32(&I, state_dim, state_dim, I_data);
            // 初始化单位矩阵I
            for (uint16_t i = 0; i < state_dim; ++i) {
                for (uint16_t j = 0; j < state_dim; ++j) {
                    I_data[i * state_dim + j] = (i == j) ? 1.0f : 0.0f;
                }
            }
            arm_matrix_instance_f32 I_KH;
            float32_t I_KH_data[state_dim * state_dim];
            arm_mat_init_f32(&I_KH, state_dim, state_dim, I_KH_data);
            arm_mat_sub_f32(&I, &KH, &I_KH);
            arm_mat_mult_f32(&I_KH, &my_element.P, &my_element.P);
    }
}
