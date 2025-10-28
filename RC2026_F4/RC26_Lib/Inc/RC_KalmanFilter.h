#pragma once
#include "arm_math.h"

#ifdef __cplusplus


namespace KalmanFilter {
    typedef struct KalmanFilter_element{
        arm_matrix_instance_f32 A; // 状态转移矩阵
        arm_matrix_instance_f32 B; // 控制输入矩阵
        arm_matrix_instance_f32 H; // 观测矩阵
        arm_matrix_instance_f32 Q; // 过程噪声协方差
        arm_matrix_instance_f32 R; // 观测噪声协方差
        arm_matrix_instance_f32 P; // 估计误差协方差

        arm_matrix_instance_f32 X; // 状态估计
        arm_matrix_instance_f32 K; // 卡尔曼增益
        arm_matrix_instance_f32 Z; // 观测值
    };
    //A,B矩阵外部输入，Q,R矩阵外部输入,观测值Z外部输入，H矩阵为外部输入
    class KalmanFilter_Base{
    protected:
        KalmanFilter_element my_element;
        arm_matrix_instance_f32 Best_X; // 最优状态估计
        uint16_t state_dim; // 状态维度
        uint16_t meas_dim;  // 观测维度
        uint16_t control_dim; // 控制输入维度

        bool is_initialized = false;

    public:
		KalmanFilter_Base(uint16_t state_dimension, uint16_t measurement_dimension, uint16_t control_dimension)
            : state_dim(state_dimension), meas_dim(measurement_dimension), control_dim(control_dimension) {
            // 初始化矩阵
            float32_t A_data[state_dim * state_dim];
            float32_t B_data[state_dim * control_dim];
            float32_t H_data[meas_dim * state_dim];
            float32_t Q_data[state_dim * state_dim];
            float32_t R_data[meas_dim * meas_dim];
            float32_t P_data[state_dim * state_dim];
            float32_t X_data[state_dim];
            float32_t K_data[state_dim * meas_dim];
            float32_t Z_data[meas_dim];

            arm_mat_init_f32(&my_element.A, state_dim, state_dim, A_data);
            arm_mat_init_f32(&my_element.B, state_dim, control_dim, B_data);
            arm_mat_init_f32(&my_element.H, meas_dim, state_dim, H_data);
            arm_mat_init_f32(&my_element.Q, state_dim, state_dim, Q_data);
            arm_mat_init_f32(&my_element.R, meas_dim, meas_dim, R_data);
            arm_mat_init_f32(&my_element.P, state_dim, state_dim, P_data);
            arm_mat_init_f32(&my_element.X, state_dim, 1, X_data);
            arm_mat_init_f32(&my_element.K, state_dim, meas_dim, K_data);
            arm_mat_init_f32(&my_element.Z, meas_dim, 1, Z_data);

            // 初始化矩阵数据为零
            memset(A_data, 0, sizeof(A_data));
            memset(B_data, 0, sizeof(B_data));
            memset(H_data, 0, sizeof(H_data));
            memset(Q_data, 0, sizeof(Q_data));
            memset(R_data, 0, sizeof(R_data));
            memset(P_data, 0, sizeof(P_data));
            memset(X_data, 0, sizeof(X_data));
            memset(K_data, 0, sizeof(K_data));
            memset(Z_data, 0, sizeof(Z_data));

            is_initialized = true;
        }
        void Set_element(KalmanFilter_element my_Kalman){
            my_element = my_Kalman;
        }
        void Z_set(const float32_t* measurement) {
            if (!is_initialized) return;
            for (uint16_t i = 0; i < meas_dim; ++i) {
                my_element.Z.pData[i] = measurement[i];
            }
        }
        void predict(const float32_t* control_input = nullptr) {
            if (!is_initialized) return;
            X_calculate(control_input);//预测状态
            P_calculate();//预测误差协方差
            K_calculate();//计算卡尔曼增益
            X_updata();//更新状态
            //最优状态X
            Best_X = my_element.X;
            P_updata();//更新误差协方差
        }
        arm_matrix_instance_f32 Get_Best_X(void){
            return Best_X;
        }
        void K_calculate(void);
        void P_calculate(void);
        void X_calculate(const float32_t* control_input);
        void X_updata(void);
        void P_updata(void);
    };

}


#endif 