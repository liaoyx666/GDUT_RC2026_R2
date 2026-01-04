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
	

	float ArmKinematics::unwrapAngle(float now, float last)
{
    while (now - last >  PI) now -= 2.0f * PI;
    while (now - last < -PI) now += 2.0f * PI;
    return now;
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

	bool ArmKinematics::inverse(const EndEffectorPos& target, JointAngles& result)
{
    float px = target.x;
    float py = target.y;
    float pz = target.z;
    float pitch = target.angle;

    /* ---------- q1 ---------- */
    float q1 = atan2f(py, px);

    /* ---------- 腕心 ---------- */
    float R = sqrtf(px * px + py * py);
    float Z = pz - BASE_HEIGHT;

    float Wx = R - L4_LENGTH * cosf(pitch);
    float Wz = Z - L4_LENGTH * sinf(pitch);

    /* ---------- 虚拟连杆 ---------- */
    float Vx = L2_LENGTH + L3_LENGTH * cosf(THETA4_OFFSET);
    float Vy = L3_LENGTH * sinf(THETA4_OFFSET);
    float L_virtual = sqrtf(Vx * Vx + Vy * Vy);
    float beta = atan2f(Vy, Vx);

    float dist = sqrtf(Wx * Wx + Wz * Wz);
    if (dist > (L1_LENGTH + L_virtual)) return false;
    if (dist < fabsf(L1_LENGTH - L_virtual)) return false;

    /* ---------- q2 ---------- */
    float cos_alpha =
        (L1_LENGTH * L1_LENGTH + dist * dist - L_virtual * L_virtual) /
        (2.0f * L1_LENGTH * dist);
    cos_alpha = constrainValue(cos_alpha, -1.0f, 1.0f);
    float alpha = acosf(cos_alpha);

    float theta_wrist = atan2f(Wz, Wx);
    float theta_L2 = theta_wrist + alpha;

    float q2 = theta_L2 - THETA2_OFFSET;
    q2 = unwrapAngle(q2, last_joint.theta2);

    /* ---------- q3 ---------- */
    float Ex = L1_LENGTH * cosf(theta_L2);
    float Ez = L1_LENGTH * sinf(theta_L2);

    float theta_virtual = atan2f(Wz - Ez, Wx - Ex);
    float q3 = theta_virtual - theta_L2 - THETA3_OFFSET - beta;
    q3 = unwrapAngle(q3, last_joint.theta3);

    /* ---------- q4（重点补丁） ---------- */
    float sum =
        (q2 + THETA2_OFFSET) +
        (q3 + THETA3_OFFSET) +
        THETA4_OFFSET +
        THETA5_OFFSET;

    float q4_raw = pitch - sum;
    q4_raw = unwrapAngle(q4_raw, last_joint.theta4);

    /* ===== 工程级连续性保护（只针对 q4） ===== */
    const float MAX_Q4_STEP = 0.3f;   // 单周期最大变化（rad）
    float dq4 = q4_raw - last_joint.theta4;

    if (dq4 >  MAX_Q4_STEP) dq4 =  MAX_Q4_STEP;
    if (dq4 < -MAX_Q4_STEP) dq4 = -MAX_Q4_STEP;

    float q4 = last_joint.theta4 + dq4;

    /* ---------- 限位 ---------- */
    result.theta1 = constrainValue(q1, THETA1_MIN, THETA1_MAX);
    result.theta2 = constrainValue(q2, THETA2_MIN, THETA2_MAX);
    result.theta3 = constrainValue(q3, THETA3_MIN, THETA3_MAX);
    result.theta4 = constrainValue(q4, THETA4_MIN, THETA4_MAX);

    /* ---------- 记录上一帧 ---------- */
    last_joint = result;

    return true;
}


}
