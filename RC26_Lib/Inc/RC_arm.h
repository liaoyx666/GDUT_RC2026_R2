#ifndef RC_ARM_H
#define RC_ARM_H
#define PI 3.14159265358979f
// ------------------ 常量定义 ------------------
#include "math.h"
#include "arm_matrix.h"
// 机械臂参数（根据你原始定义调整）
#define L1_LENGTH       0.45657f    // 大臂长度（米）
#define L2_LENGTH       0.41476f    // 小臂长度（米）
#define L3_LENGTH       0.13350f    // 末端执行器长度（米）
#define BASE_HEIGHT     0.0f        // 底座高度（米）


// -------------------------- 关节角度限制 --------------------------
#define THETA0_MIN      -PI         // 底座旋转最小角度（弧度，-180°）
#define THETA0_MAX       PI         // 底座旋转最大角度（弧度，180°）
#define THETA1_MIN       0       // 大臂最小角度（弧度，0°）
#define THETA1_MAX       PI      // 大臂最大角度（弧度，90°）
#define THETA2_MIN       0      // 小臂最小角度（弧度，-90°）
#define THETA2_MAX      1.5 * PI       // 小臂最大角度（弧度，90°）
#define THETA3_MIN       -PI/2       // 末端最小角度（弧度，-90°）
#define THETA3_MAX       PI       // 末端最大角度（弧度，90°）

// -------------------------- 关节角度偏移 --------------------------
#define THETA0_OFFSET    0.0f
#define THETA1_OFFSET    0.0f
#define THETA2_OFFSET    0.0f//27.32f * PI / 180.0f
#define THETA3_OFFSET    0.0f//6.20f * PI / 180.0f

// ------------------ 结构体定义 ------------------
struct JointAngles {
    float theta0;
    float theta1;
    float theta2;
    float theta3;
};

struct EndEffectorPos {
    float x;
    float y;
    float z;
    float angle;
};

enum ActuatorType {
    ACTUATOR_1,
    ACTUATOR_2
};
#ifdef __cplusplus
// ------------------ ArmKinematics 类定义 ------------------
class ArmKinematics {
public:
    // 核心函数
    int inverse(const EndEffectorPos& target_pos,
                const JointAngles& current_angles,
                ActuatorType actuator,
                JointAngles& result_angles);

    void forward(const JointAngles& angles,
                 ActuatorType actuator,
                 EndEffectorPos& end_pos);
		
	static	ArmMatrix<4, 4> T01;
   static ArmMatrix<4, 4> T12;
   static ArmMatrix<4, 4> T23;
  static  ArmMatrix<4, 4> T34;
  static  ArmMatrix<4, 4> T04;
		
private:
    // 工具函数


    float normalizeAngle(float angle);
    float constrainValue(float value, float min, float max);
    float calc_reward(const JointAngles& q,
                      const EndEffectorPos& end_pos,
                      const EndEffectorPos& target,
                      const JointAngles& current_angles);
		static ArmMatrix<4, 4> buildDHTable(float theta, float alpha, float a, float d, float offset);
    // 成员变量（静态分配）
    JointAngles valid_solutions_[2];
    int num_valid_solutions_ = 0;
};

#endif // RC_ARM_H
#endif
