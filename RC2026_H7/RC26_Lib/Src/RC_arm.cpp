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
	ArmMatrix<4, 4> ArmKinematics::T04;
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
	void ArmKinematics::forward(const JointAngles& angles, ActuatorType actuator, EndEffectorPos& end_pos) {
		float theta0 = angles.theta0;
		float theta1 = angles.theta1;
		float theta2 = angles.theta2;
		float theta3 = angles.theta3;

		T01 = buildDHTable(theta0, PI/2, 0.0f, BASE_HEIGHT, 0.0f);
		T12 = buildDHTable(theta1, 0, L1_LENGTH, 0.0f, THETA1_OFFSET);    
		T23 = buildDHTable(theta2, 0, L2_LENGTH, 0.0f, THETA2_OFFSET);     
		T34 = (actuator == ACTUATOR_1)
			  ? buildDHTable(theta3, 0.0f, L3_LENGTH, 0.0f, 0.0f)
			  : buildDHTable(theta3, PI/2, L3_LENGTH, 0.0f, 0.0f);

		T04 = T01 * T12 * T23 * T34;

		end_pos.x = T04(0,3);
		end_pos.y = T04(1,3);
		end_pos.z = T04(2,3);

		// angle = q1 + q2 + q3 + π (小臂 offset)
		end_pos.angle = (actuator == ACTUATOR_1)
			? (theta1 + theta2 + theta3 )
			: (theta1 + theta2 + theta3 + PI/2);

		end_pos.angle = normalizeAngle(end_pos.angle);
	}

	bool ArmKinematics::inverse(
		const EndEffectorPos& target_pos, 
		JointAngles& result_angles
	)
	{

		float L1 = BASE_HEIGHT;
		float L2 = L1_LENGTH;
		float L3 = L2_LENGTH;
		float L4 = L3_LENGTH;
		float x = target_pos.x;
		float y = target_pos.y;
		float pitch = target_pos.angle;
		float z = target_pos.z;

		// Step 1: q1_dh
		float q1_dh = atan2f(y, x);
		if ((x*cosf(q1_dh) + y*sinf(q1_dh)) < 0)
			q1_dh = normalizeAngle(q1_dh + PI);
		float q1 = q1_dh ;

		// Step 2: 腕点位置
		float x1 = cosf(q1_dh)*x + sinf(q1_dh)*y;
		float z1 = z;
		float xw = x1 - L4*cosf(pitch);
		float zw = z1 - L4*sinf(pitch);

		float dx = xw;
		float dz = zw - L1;
		float d = sqrtf(dx*dx + dz*dz);

		if (d > (L2 + L3) || d < fabsf(L2 - L3)) return false;

		// Step 3: q3
		float cos_q3 = (dx*dx + dz*dz - L2*L2 - L3*L3) / (2*L2*L3);
		cos_q3 = constrainValue(cos_q3, -1.0f, 1.0f);
		float q3 = -acosf(cos_q3);

		// Step 4: q2
		float q2 = atan2f(dz, dx) - atan2f(L3*sinf(q3), L2 + L3*cosf(q3));

		// Step 5: q4
		float q4 = pitch - (q2 + q3);

		// Step 6: 加 offset 并限幅
		q2 += THETA1_OFFSET;
		q3 += THETA2_OFFSET;

		q2 = constrainValue(q2, THETA1_MIN, THETA1_MAX);
		q3 = constrainValue(q3, THETA2_MIN, THETA2_MAX);
		q4 = constrainValue(q4, THETA3_MIN, THETA3_MAX);

		// 输出
		result_angles.theta0 = q1;
		result_angles.theta1 = q2;
		result_angles.theta2 = q3;
		result_angles.theta3 = q4;
		return true;
	}

}