#pragma once

#include "math.h"
#include "arm_matrix.h" // 假设这个矩阵库头文件存在且路径正确
#include "RC_mpc.h"
// -------------------------- 机械臂结构参数 (宏定义保持不变) --------------------------
#define L1_LENGTH       0.45657f    // 大臂长度（米）
#define L2_LENGTH       0.41476f    // 小臂长度（米）
#define L3_LENGTH       0.13350f    // 末端执行器长度（米）
#define BASE_HEIGHT     0.1f        // 底座高度（米）

// PI 常量
#ifndef PI
#define PI 3.1415926535f
#endif

// -------------------------- 关节角度限制 --------------------------
#define THETA0_MIN      -PI         // 底座旋转最小角度（弧度，-180°）
#define THETA0_MAX       PI         // 底座旋转最大角度（弧度，180°）
#define THETA1_MIN       -PI       // 大臂最小角度（弧度，0°）
#define THETA1_MAX       PI      // 大臂最大角度（弧度，90°）
#define THETA2_MIN      -PI       // 小臂最小角度（弧度，-90°）
#define THETA2_MAX       PI       // 小臂最大角度（弧度，90°）
#define THETA3_MIN      -PI       // 末端最小角度（弧度，-90°）
#define THETA3_MAX       PI       // 末端最大角度（弧度，90°）

// -------------------------- 关节角度偏移 --------------------------
#define THETA0_OFFSET    0.0f
#define THETA1_OFFSET    0.0f
#define THETA2_OFFSET    0.0f//27.32f * PI / 180.0f
#define THETA3_OFFSET    0.0f//6.20f * PI / 180.0f


#ifdef __cplusplus
// -------------------------- 数据结构体 --------------------------
// 关节角度结构体（弧度）
typedef struct {
    float theta0;
    float theta1;
    float theta2;
    float theta3;
} JointAngles;


// 末端执行器位姿结构体
typedef struct {
    float x;      // X轴坐标（米）
    float y;      // Y轴坐标（米）
    float z;      // Z轴坐标（米）
    float angle;  // 末端姿态角（弧度）
} EndEffectorPos;

// 执行器类型枚举
typedef enum {
    ACTUATOR_1,
    ACTUATOR_2
} ActuatorType;

#define P_HORIZON 10 
#define M_HORIZON 3  
#define OUT_DIM 4    
#define IN_DIM 4     


// -------------------------- 机械臂运动学类 --------------------------
class ArmKinematics {
public:
    /**
     * @brief 正运动学：输入关节角度（弧度），输出末端位姿（米）
     * @param angles 关节角度（弧度）
     * @param actuator 当前执行器类型
     * @param end_pos 计算出的末端位姿（米）
     */
    static void forward(const JointAngles& angles, ActuatorType actuator, EndEffectorPos& end_pos);
    
   
    /**
     * @brief 逆运动学：输入末端位姿，输出最优关节角度
     * @param target_pos 目标末端位姿（米）
     * @param current_angles 当前机械臂的关节角度，用于选择最优解
     * @param actuator 当前执行器类型
     * @param result_angles 计算出的最优关节角度（弧度）
     * @return 0=成功, 1=超工作空间, 2=无有效解
     */
     int inverse(const EndEffectorPos& target_pos, const JointAngles& current_angles, ActuatorType actuator, JointAngles& result_angles);
    
		 static ArmMatrix<4, 4> calculateJacobian(const JointAngles& q_current, ActuatorType actuator);

		 int mpc_init(const JointAngles& initial_angles, float q_weight, float r_weight);
		
		 JointAngles mpc_control_step(const EndEffectorPos& target_pos, const JointAngles& current_angles);
		
		

private:
		JointAngles valid_solutions_[2];
    int num_valid_solutions_ = 0;

		mpc::DynamicMatrixMPC<OUT_DIM, IN_DIM, P_HORIZON, M_HORIZON> mpc_controller_;

		bool mpc_initialized_ = false;
    // 将角度归一化到 [-PI, PI] 范围
    static float normalizeAngle(float angle);

    // 数值限幅
    static float constrainValue(float value, float min, float max);
    
    // 构建DH变换矩阵
    static ArmMatrix<4, 4> buildDHTable(float theta, float alpha, float a, float d, float offset);

    // 奖励函数，用于在多个可行解中选择最优解
    static float calc_reward(const JointAngles& q, const EndEffectorPos& end_pos, const EndEffectorPos& target, const JointAngles& current_angles);
};











#endif // __cplusplus
