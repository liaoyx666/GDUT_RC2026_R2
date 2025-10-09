#include "RC_arm.h"

// 初始化静态成员变量
ActuatorType ArmKinematics::current_actuator = ACTUATOR_1;

// -------------------------- 私有工具函数 --------------------------
// 数值限幅
float ArmKinematics::constrainValue(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 检查目标点是否在工作空间内（r=XY平面投影距离，z_prime=相对底座高度）
bool ArmKinematics::isPointInWorkspace(float r, float z_prime) {
    float D = sqrtf(r * r + z_prime * z_prime);  // 底座到目标点的直线距离
    float max_reach = L1_LENGTH + L2_LENGTH + L3_LENGTH;  // 最大工作半径
    float min_reach = fabsf(L1_LENGTH - L2_LENGTH - L3_LENGTH);  // 最小工作半径
    return (D >= min_reach && D <= max_reach);
}

// 构建DH变换矩阵（DH参数：theta=关节角，alpha=连杆扭转角，a=连杆长度，d=连杆偏移）
ArmMatrix<4, 4> ArmKinematics::buildDHTable(float theta, float alpha, float a, float d) {
    ArmMatrix<4, 4> dh_mat;
    dh_mat.setZero();  // 先置零矩阵
    
    // DH矩阵公式：齐次变换矩阵（旋转+平移）
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(alpha);
    float sa = sinf(alpha);
    
    dh_mat(0, 0) = ct;
    dh_mat(0, 1) = -st * ca;
    dh_mat(0, 2) = st * sa;
    dh_mat(0, 3) = a * ct;
    
    dh_mat(1, 0) = st;
    dh_mat(1, 1) = ct * ca;
    dh_mat(1, 2) = -ct * sa;
    dh_mat(1, 3) = a * st;
    
    dh_mat(2, 1) = sa;
    dh_mat(2, 2) = ca;
    dh_mat(2, 3) = d;
    
    dh_mat(3, 3) = 1.0f;  // 齐次矩阵最后一行固定为[0,0,0,1]
    
    return dh_mat;
}

// -------------------------- 正运动学实现 --------------------------
void ArmKinematics::forward(const JointAngles& angles, EndEffectorPos& end_pos) {
    // 1. 关节角度限幅（避免超量程）
    float theta0 = constrainValue(angles.theta0, THETA0_MIN, THETA0_MAX);
    float theta1 = constrainValue(angles.theta1, THETA1_MIN, THETA1_MAX);
    float theta2 = constrainValue(angles.theta2, THETA2_MIN, THETA2_MAX);
    float theta3 = constrainValue(angles.theta3, THETA3_MIN, THETA3_MAX);

    // 2. 定义四自由度机械臂的DH参数表
    ArmMatrix<4, 4> T01 = buildDHTable(theta0,  PI/2, 0.0f, BASE_HEIGHT);  // 关节0→1（底座→大臂）
    ArmMatrix<4, 4> T12 = buildDHTable(theta1,   0.0f, L1_LENGTH, 0.0f);   // 关节1→2（大臂→小臂）
    ArmMatrix<4, 4> T23 = buildDHTable(theta2,   0.0f, L2_LENGTH, 0.0f);   // 关节2→3（小臂→末端）
    ArmMatrix<4, 4> T34;
    
    // 根据当前执行器类型构建末端DH矩阵
    if (current_actuator == ACTUATOR_1) {
        T34 = buildDHTable(theta3,   0.0f, L3_LENGTH, 0.0f);
    } else {
        T34 = buildDHTable(theta3,   PI/2, L3_LENGTH, 0.0f);
    }

    // 3. 计算总变换矩阵
    ArmMatrix<4, 4> T02 = T01 * T12;
    ArmMatrix<4, 4> T03 = T02 * T23;
    ArmMatrix<4, 4> T04 = T03 * T34;

    // 4. 从总变换矩阵提取末端位姿
    end_pos.x = T04(0, 3);
    end_pos.y = T04(1, 3);
    end_pos.z = T04(2, 3);
    
    // 5. 根据执行器类型计算末端姿态角
    if (current_actuator == ACTUATOR_1) {
        end_pos.angle = theta1 + theta2 + theta3;
    } else {
        end_pos.angle = theta1 + theta2 + theta3 + PI/2;  // 叠加90°
    }
}

// -------------------------- 逆运动学实现 --------------------------
int ArmKinematics::inverse(const EndEffectorPos& target_pos, JointAngles& result_angles) {
    // 1. 计算目标点在基坐标系下的关键参数
    float x = target_pos.x;
    float y = target_pos.y;
    float z = target_pos.z;
    float target_angle = target_pos.angle;
    
    // 根据执行器类型调整目标角度
    if (current_actuator == ACTUATOR_2) {
        target_angle -= PI/2;  // 补偿90°偏移
    }
    
    float r = sqrtf(x * x + y * y);  // XY平面投影距离
    float z_prime = z - BASE_HEIGHT;  // 相对底座高度

    // 2. 检查工作空间
    if (!isPointInWorkspace(r, z_prime)) {
        return 1;  // 超工作空间
    }

    // 3. 解算关节0（底座旋转角）
    float theta0 = atan2f(y, x);
    theta0 = constrainValue(theta0, THETA0_MIN, THETA0_MAX);

    // 4. 解算关节1（大臂角度）
    float D = sqrtf(r * r + z_prime * z_prime);
    float a = L1_LENGTH;
    float b = L2_LENGTH + L3_LENGTH;
    
    float cos_alpha = (a*a + b*b - D*D) / (2*a*b);
    cos_alpha = constrainValue(cos_alpha, -1.0f, 1.0f);
    float alpha = acosf(cos_alpha);
    
    float beta = atan2f(z_prime, r);
    float theta1 = beta + alpha;
    theta1 = constrainValue(theta1, THETA1_MIN, THETA1_MAX);

    // 5. 解算关节2（小臂角度）
    float cos_gamma = (a*a + D*D - b*b) / (2*a*D);
    cos_gamma = constrainValue(cos_gamma, -1.0f, 1.0f);
    float gamma = acosf(cos_gamma);
    float theta2 = PI - gamma;
    theta2 = constrainValue(theta2, THETA2_MIN, THETA2_MAX);

    // 6. 解算关节3（末端角度）
    float theta3 = target_angle - (theta1 + theta2);
    theta3 = constrainValue(theta3, THETA3_MIN, THETA3_MAX);

    // 7. 最终检查所有关节角度
    if (theta0 < THETA0_MIN || theta0 > THETA0_MAX ||
        theta1 < THETA1_MIN || theta1 > THETA1_MAX ||
        theta2 < THETA2_MIN || theta2 > THETA2_MAX ||
        theta3 < THETA3_MIN || theta3 > THETA3_MAX) {
        return 2;  // 角度超限
    }

    // 8. 赋值结果
    result_angles.theta0 = theta0;
    result_angles.theta1 = theta1;
    result_angles.theta2 = theta2;
    result_angles.theta3 = theta3;

    return 0;  // 解算成功
}
