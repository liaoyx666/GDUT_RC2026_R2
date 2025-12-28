#include "RC_arm.h"

namespace arm
{
	ArmDynamics::ArmDynamics() {}

	void ArmDynamics::gravity_compensation()
	{
		joint_angle_now.theta1 = -motor_angle.theta1;
		joint_angle_now.theta2 = -motor_angle.theta1 + motor_angle.theta2 + PI - 0.146084f;
		joint_angle_now.theta3 = -motor_angle.theta3 + motor_angle.theta2 - motor_angle.theta1 - 0.146084f;


		joint_gravity_compensation.joint1 = L1_gravity * L1_Particle_LENGTH * cosf(joint_angle_now.theta1) 
										  + L2_gravity * (L2_Particle_LENGTH * cosf(joint_angle_now.theta2) + L1_LENGTH * cosf(joint_angle_now.theta1))
										  + L3_gravity * (L3_Particle_LENGTH * cosf(joint_angle_now.theta3) + L2_LENGTH * cosf(joint_angle_now.theta2) + L1_LENGTH * cos(joint_angle_now.theta1));

		joint_gravity_compensation.joint2 = L2_gravity * L2_Particle_LENGTH * cosf(joint_angle_now.theta2)
										  + L3_gravity * (L3_Particle_LENGTH * cosf(joint_angle_now.theta3) + L2_LENGTH * cosf(joint_angle_now.theta2));

		joint_gravity_compensation.joint3 = L3_gravity * L3_Particle_LENGTH * cosf(joint_angle_now.theta3);
	}
	

	
	// ------------------ 静态成员变量 ------------------
	ArmMatrix<4, 4> ArmKinematics::T01;
	ArmMatrix<4, 4> ArmKinematics::T12;
	ArmMatrix<4, 4> ArmKinematics::T23;
	ArmMatrix<4, 4> ArmKinematics::T34;
	ArmMatrix<4, 4> ArmKinematics::T45;
	ArmMatrix<4, 4> ArmKinematics::T05;
	ArmMatrix<4, 4> temp1;
	ArmMatrix<4, 4> temp2;

	// ------------------ 工具函数 ------------------
	float ArmKinematics::normalizeAngle(float angle) {
		while (angle <= -PI) angle += 2 * PI;
		while (angle > PI) angle -= 2 * PI;
		return angle;
	}

	float ArmKinematics::constrainValue(float value, float min, float max) {
		if (value < min) return min;
		if (value > max) return max;
		return value;
	}

	ArmMatrix<4, 4> ArmKinematics::buildDHTable(float theta, float alpha, float a, float d, float offset) {
		ArmMatrix<4, 4> dh;
		dh.setZero();
		float ct = cosf(theta + offset);
		float st = sinf(theta + offset);
		float ca = cosf(alpha);
		float sa = sinf(alpha);

		dh(0,0) = ct; dh(0,1) = -st*ca; dh(0,2) = st*sa; dh(0,3) = a*ct;
		dh(1,0) = st; dh(1,1) = ct*ca;  dh(1,2) = -ct*sa; dh(1,3) = a*st;
		dh(2,1) = sa; dh(2,2) = ca;     dh(2,3) = d;
		dh(3,3) = 1.0f;
		return dh;
	}

	// ------------------ 正运动学 ------------------
	void ArmKinematics::forward(const JointAngles& angles, EndEffectorPos& end_pos) {
		float theta1 = angles.theta1;
		float theta2 = angles.theta2;
		float theta3 = angles.theta3;
		float theta4 = angles.theta4;

		T01 = buildDHTable(theta1, PI/2, 	0.0f,      BASE_HEIGHT,  THETA1_OFFSET);
		T12 = buildDHTable(theta2, 0.0f,    L1_LENGTH, 0.0f,  		 THETA2_OFFSET);    
		T23 = buildDHTable(theta3, 0.0f,    L2_LENGTH, 0.0f,  		 THETA3_OFFSET);     
		T34 = buildDHTable(0.0f,   0.0f,    L3_LENGTH, 0.0f,  		 THETA4_OFFSET);
		T45 = buildDHTable(theta4, 0.0f,    L4_LENGTH, 0.0f,  		 THETA5_OFFSET);

		T05 = T01 * T12 * T23 * T34 * T45;

		end_pos.x = T05(0,3);
		end_pos.y = T05(1,3);
		end_pos.z = T05(2,3);

		// angle = q1 + q2 + q3 + π (小臂 offset)
		end_pos.angle = normalizeAngle(theta2  + theta3  + THETA4_OFFSET + theta4 + THETA1_OFFSET + THETA2_OFFSET+ THETA3_OFFSET +THETA5_OFFSET) ;
			
	}

	bool ArmKinematics::inverse(const EndEffectorPos& target_pos, JointAngles& result_angles)
{
float px = target_pos.x;
float py = target_pos.y;
float pz = target_pos.z;
float pitch_global = target_pos.angle;

// 假设 DH 参数常量已在 RC_arm.h 中定义:
// BASE_HEIGHT (d1=0.1), L1_LENGTH (a2=0.34858), L2_LENGTH (a3=0.06553), L3_LENGTH (a4=0.28166), L4_LENGTH (a5=0.13350)
// THETA2_OFFSET (off2), THETA3_OFFSET (off3), THETA4_OFFSET (off4), THETA5_OFFSET (off5)
// THETA2_MIN/MAX, THETA3_MIN/MAX, THETA4_MIN/MAX

// Step 1: q1
float q1 = atan2f(py, px);
q1 = normalizeAngle(q1);
// Step 2: 腕心坐标 (Wrist Center, Pw)
float R_end = sqrtf(px*px + py*py);
float Z_end = pz - BASE_HEIGHT;

// 使用 L4_LENGTH (a5) 进行回退
float Wx = R_end - L4_LENGTH * cosf(pitch_global);
float Wz = Z_end - L4_LENGTH * sinf(pitch_global);

// Step 3: 构建 L3 + L4 的等效连杆 (Virtual Link)
float Vx = L2_LENGTH + L3_LENGTH * cosf(THETA4_OFFSET);
float Vy = L3_LENGTH * sinf(THETA4_OFFSET);
float L_virtual = sqrtf(Vx*Vx + Vy*Vy);
float beta = atan2f(Vy, Vx);

// Step 4: 工作空间检查
float dist_wrist = sqrtf(Wx*Wx + Wz*Wz);
// L1_LENGTH 对应 a2
if (dist_wrist > (L1_LENGTH + L_virtual) || dist_wrist < fabsf(L1_LENGTH - L_virtual))
return false;

// Step 5: 求解 q2（大臂）
float cos_gamma = (L1_LENGTH*L1_LENGTH + L_virtual*L_virtual - dist_wrist*dist_wrist) / (2.0f * L1_LENGTH * L_virtual);
cos_gamma = fmaxf(fminf(cos_gamma, 1.0f), -1.0f);
float gamma = acosf(cos_gamma); // 暂时未使用，但为完整性保留

float cos_alpha = (L1_LENGTH*L1_LENGTH + dist_wrist*dist_wrist - L_virtual*L_virtual) / (2.0f * L1_LENGTH * dist_wrist);
cos_alpha = fmaxf(fminf(cos_alpha, 1.0f), -1.0f);
float alpha = acosf(cos_alpha);

float theta_wrist = atan2f(Wz, Wx);
float theta_L2_global = theta_wrist + alpha;

// DH 关系: q2 = Global - Offset。使用减号。
float q2_raw = theta_L2_global - THETA2_OFFSET;

// *** 修正：规范化 q2 到 (-PI, PI] 范围，解决周期性问题 ***
result_angles.theta2 = atan2f(sinf(q2_raw), cosf(q2_raw));
float q2 = result_angles.theta2; // 使用规范化后的值进行后续计算

// Step 6: 求解 q3（小臂）
// 计算 Elbow 关节的空间位置
float Ex = L1_LENGTH * cosf(theta_L2_global);
float Ez = L1_LENGTH * sinf(theta_L2_global);

// Virtual Link 的全局角度
float theta_virtual_global = atan2f(Wz - Ez, Wx - Ex);

// DH 关系: q3 = Global(Virtual) - Global(L2) - THETA3_OFFSET - beta
result_angles.theta3 = theta_virtual_global - theta_L2_global - THETA3_OFFSET - beta;
result_angles.theta3 = atan2f(sinf(result_angles.theta3), cosf(result_angles.theta3));
float q3 = result_angles.theta3;

// Step 7: 求解 q4（末端 pitch 关节）
// Pitch_global = (q2 + off2) + (q3 + off3) + (0 + off4) + (q4 + off5)
float current_sum = (q2 + THETA2_OFFSET) + (q3 + THETA3_OFFSET) + THETA4_OFFSET + THETA5_OFFSET;

// 姿态求解: q4 = Pitch_global - Sum of (Other Rotations)
result_angles.theta4 = pitch_global - current_sum;
result_angles.theta4 = atan2f(sinf(result_angles.theta4), cosf(result_angles.theta4));

// 限制关节限位
result_angles.theta2 = constrainValue(result_angles.theta2, THETA2_MIN, THETA2_MAX);
result_angles.theta3 = constrainValue(result_angles.theta3, THETA3_MIN, THETA3_MAX);
result_angles.theta4 = constrainValue(result_angles.theta4, THETA4_MIN, THETA4_MAX);

result_angles.theta1 = q1;

return true;
}


}