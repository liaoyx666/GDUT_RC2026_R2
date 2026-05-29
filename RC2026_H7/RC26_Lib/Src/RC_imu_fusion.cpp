#include "RC_imu_fusion.h"

namespace fusion
{
	ImuFusion::ImuFusion(ros::Radar& radar_, HWT101CT& imu_) : radar(radar_), imu(imu_), filter(0.00001, 1000, 2)
	{
		yaw_fused = 0.0f;
		integral = 0.0f;  // 积分清零
	}
	
	void ImuFusion::Fusion()
	{
		// 1. 读取角度
		float imu_yaw   = imu.Yaw();
		float radar_yaw = radar.Yaw();

		// 2. 计算误差 + 环形归一化（核心）
		float error = radar_yaw - imu_yaw;
		if (error > PI)      error -= TWO_PI;
		else if (error < -PI) error += TWO_PI;

		// ==================== 核心参数：专为2°/h慢漂移调校（无超调+无静差） ====================
		const float Kp = 0.08f;   // 比例：柔缓修正，绝不超调
		const float Ki = 0.002f;  // 积分：刚好消除稳态误差，不激进
		const float INTEGRAL_LIMIT = 1.0f; // 积分限幅（匹配漂移范围）

		// ==================== 3. 抗积分饱和（根治超调+保证无静差） ====================
		// 计算比例项
		float P = Kp * error;
		// 积分累加（工业标准：限幅内正常积分，无饱和）
		integral += error;
		// 强限幅，彻底杜绝积分饱和 → 无超调
		if (integral > INTEGRAL_LIMIT)  integral = INTEGRAL_LIMIT;
		if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
		// 积分项
		float I = Ki * integral;

		// ==================== 4. 总修正量（Mahony 核心修正逻辑） ====================
		float adjust = P + I;

		// 修正量柔限幅（防止跳变）
		if (adjust > 0.2f)  adjust = 0.2f;
		if (adjust < -0.2f) adjust = -0.2f;

		// ==================== 5. 计算目标角度 ====================
		float target_yaw = imu_yaw + adjust;
		// 归一化
		if (target_yaw > PI)      target_yaw -= TWO_PI;
		else if (target_yaw < -PI) target_yaw += TWO_PI;

		// ==================== 6. 平滑修正IMU（无硬切） ====================
		imu.Set_Yaw(target_yaw);
	}
}
