#pragma once
#include "math.h"
#include "arm_matrix.h"

// -------------------------- 机械臂结构参数 --------------------------
#define L1_LENGTH       0.3f     // 大臂长度（米）
#define L2_LENGTH       0.25f    // 小臂长度（米）
#define L3_LENGTH       0.15f    // 末端执行器长度（米）
#define BASE_HEIGHT     0.1f     // 底座高度（米）

// -------------------------- 关节角度限制 --------------------------
#define THETA0_MIN      -PI      // 底座旋转最小角度（弧度，-180°）
#define THETA0_MAX       PI      // 底座旋转最大角度（弧度，180°）
#define THETA1_MIN       0.0f    // 大臂最小角度（弧度，0°）
#define THETA1_MAX       PI/2    // 大臂最大角度（弧度，90°）
#define THETA2_MIN      -PI/2    // 小臂最小角度（弧度，-90°）
#define THETA2_MAX       PI/2    // 小臂最大角度（弧度，90°）
#define THETA3_MIN      -PI/2    // 末端最小角度（弧度，-90°）
#define THETA3_MAX       PI/2    // 末端最大角度（弧度，90°）
#define gravity          9.788   // 重力加速度（m/s²）


#ifdef __cplusplus
// -------------------------- 数据结构体 --------------------------
// 关节角度结构体（输入：四个关节的角度）
typedef struct {
    float theta0;  // 底座旋转角度（绕Z轴）
    float theta1;  // 大臂旋转角度（绕X轴）
    float theta2;  // 小臂旋转角度（绕X轴）
    float theta3;  // 末端旋转角度（绕X轴）
} JointAngles;

// 末端执行器位姿结构体（输入/输出：XYZ坐标+姿态角）
typedef struct {
    float x;       // X轴坐标（米）
    float y;       // Y轴坐标（米）
    float z;       // Z轴坐标（米）
    float angle;   // 末端姿态角（弧度，与X轴夹角）
} EndEffectorPos;

// 执行器类型枚举
typedef enum {
    ACTUATOR_1,  // 第一个执行器
    ACTUATOR_2   // 第二个执行器（与第一个呈90°）
} ActuatorType;


// -------------------------- 机械臂运动学类 --------------------------
class ArmKinematics {
public:
    // 正运动学：输入关节角度，输出末端位姿
    static void forward(const JointAngles& angles, EndEffectorPos& end_pos);
    
    // 逆运动学：输入末端位姿，输出关节角度
    // 返回值：0=成功，1=超工作空间，2=角度超限
    static int inverse(const EndEffectorPos& target_pos, JointAngles& result_angles);

    // 执行器切换函数
    static void setActuator(ActuatorType type) { current_actuator = type; }

private:
    // 数值限幅：确保在[min, max]范围内
    static float constrainValue(float value, float min, float max);
    
    // 检查目标点是否在工作空间内
    static bool isPointInWorkspace(float r, float z_prime);
    
    // 构建单个关节的DH变换矩阵（4x4齐次矩阵）
    static ArmMatrix<4, 4> buildDHTable(float theta, float alpha, float a, float d);

    static ActuatorType current_actuator;  // 当前激活的执行器
};

#endif // __cplusplus
