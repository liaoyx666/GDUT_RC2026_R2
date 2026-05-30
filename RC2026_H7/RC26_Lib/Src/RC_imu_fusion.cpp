#include "RC_imu_fusion.h"

namespace fusion
{
	ImuFusion::ImuFusion(ros::Radar& radar_, HWT101CT& imu_) : radar(radar_), imu(imu_), filter(3, 1000)
	{
		// 初始化协方差矩阵
		P[0][0] = 1.0f; P[0][1] = 0.0f;
		P[1][0] = 0.0f; P[1][1] = 1.0f;
		last_time = 0;
	}
	
void ImuFusion::Fusion()
{
    // ================================================================
    //  1. 读取传感器数据
    // ================================================================
    float imu_yaw   = imu.Yaw();
    float imu_omega = imu.W();
    float radar_yaw = radar.Yaw();

    // ================================================================
    //  2. 获取时间步长 (秒)
    // ================================================================
    uint64_t now = timer::Timer::Get_TimeStamp();
    float dt = (float)(now - last_time) * (1.0f / 1000000.0f);
    last_time = now;

    // 防止异常 dt
    if (dt <= 0.0f)   dt = 0.001f;
    if (dt > 0.1f)    dt = 0.001f;

    // ================================================================
    //  3. 卡尔曼滤波参数
    //     【修正】增大了 Q 值，使滤波器不会过度信任 IMU 预测
    //     根据你的实际传感器噪声特性进一步调整
    // ================================================================
    const float Q_angle = 1e-3f;   // 角度过程噪声  (原值 1e-6 太小)
    const float Q_bias  = 1e-5f;   // 偏置过程噪声  (原值 1e-9 太小)
    const float R_radar = 0.05f;   // 雷达观测噪声  (根据雷达精度调整, 典型值 0.01~0.5)

    // P 矩阵元素下限，防止协方差坍缩导致滤波器"休眠"（不再响应观测）
    const float P_MIN_DIAG = 1e-6f;

    // ================================================================
    //  4. 首次运行：用雷达角度初始化
    // ================================================================
    if (first_run)
    {
        angle = radar_yaw;
        bias  = 0.0f;

        // 初始协方差设大一点，让滤波器快速收敛
        P[0][0] = 1.0f;   P[0][1] = 0.0f;
        P[1][0] = 0.0f;   P[1][1] = 0.1f;

        last_time = timer::Timer::Get_TimeStamp();  // 【修正】初始化 last_time
        first_run = false;
        imu.Set_Yaw(angle);
        return;
    }

    // ================================================================
    //  5. 预测步骤 (Predict)
    // ================================================================

    // --- 状态预测 ---
    float omega_corrected = imu_omega - bias;
    angle += omega_corrected * dt;

    // --- 角度归一化到 [-π, π] ---
    while (angle >  PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;

    // --- 协方差预测: P = F * P * F^T + Q ---
    //  F = [1, -dt]   F^T = [1,   0]
    //      [0,   1]         [-dt, 1]
    //
    //  展开后:
    //   P00' = P00 - dt*(P01+P10) + dt²*P11 + Q_angle
    //   P01' = P01 - dt*P11
    //   P10' = P10 - dt*P11
    //   P11' = P11 + Q_bias

    float P00 = P[0][0], P01 = P[0][1];
    float P10 = P[1][0], P11 = P[1][1];

    float new_P00 = P00 - dt * (P01 + P10) + dt * dt * P11 + Q_angle;
    float new_P01 = P01 - dt * P11;
    float new_P10 = P10 - dt * P11;
    float new_P11 = P11 + Q_bias;

    // 【修正】强制对称 (P01 == P10)，消除浮点累积误差
    float P01_sym = 0.5f * (new_P01 + new_P10);
    new_P01 = P01_sym;
    new_P10 = P01_sym;

    P[0][0] = new_P00;  P[0][1] = new_P01;
    P[1][0] = new_P10;  P[1][1] = new_P11;

    // ================================================================
    //  6. 观测更新 (Update) —— 仅在雷达数据有效时执行
    // ================================================================

	// --- 新息 (Innovation) ---
	float y = radar_yaw - angle;
	// 角度差归一化到 [-π, π]
	while (y >  PI) y -= TWO_PI;
	while (y < -PI) y += TWO_PI;

	// --- 新息协方差: S = H*P*H^T + R = P[0][0] + R ---
	float S = P[0][0] + R_radar;

	// --- 卡尔曼增益: K = P * H^T / S ---
	//  H = [1, 0]  =>  K = [P00/S, P10/S]^T
	float K_angle = P[0][0] / S;
	float K_bias  = P[1][0] / S;

	// --- 状态更新 ---
	angle += K_angle * y;
	bias  += K_bias  * y;

	// --- 角度归一化 ---
	while (angle >  PI) angle -= TWO_PI;
	while (angle < -PI) angle += TWO_PI;

	// --- 协方差更新: Joseph 形式 ---
	//  P_new = (I - K*H) * P * (I - K*H)^T + K * R * K^T
	//
	//  令 ia = 1 - K_angle
	//  (I-KH) = [ia,   0]
	//           [-Kb,  1]
	//
	//  展开后:
	//   P00' = ia² * P00 + R * Ka²
	//   P01' = ia * (P01 - Ka * P00)  +  R * Ka * Kb    ← 不含此项时为 ia*P01
	//   P10' = ia * (P10 - Kb * P00)  +  R * Ka * Kb    ← 但化简后等价
	//   P11' = P11 - Kb*(P01+P10) + Kb²*P00 + R*Kb²
	//
	//  化简 (利用 Ka = P00/S, Kb = P10/S, S = P00+R):

	float ia  = 1.0f - K_angle;
	float ia2 = ia * ia;
	float R_Ka2 = R_radar * K_angle * K_angle;
	float R_KaKb = R_radar * K_angle * K_bias;
	float R_Kb2 = R_radar * K_bias * K_bias;

	float P00_upd = ia2 * P[0][0] + R_Ka2;
	float P01_upd = ia * (P[0][1] - K_angle * P[0][1])  // = ia*(ia*P01) ... 
					- K_bias * ia * P[0][0]               // 交叉项
					+ R_KaKb;
	// 更清晰的展开:
	//  (I-KH)*P*(I-KH)^T 的 [0][1]:
	//   = ia * P01 * ia  +  ia * P00 * (-Kb)   ... 不对，让我重新推导
	// 直接用矩阵乘法:
	//  (I-KH)*P = [ia*P00,           ia*P01         ]
	//             [-Kb*P00 + P10,    -Kb*P01 + P11  ]
	//  ((I-KH)*P) * (I-KH)^T:
	//  (I-KH)^T = [ia,  -Kb]
	//             [0,    1 ]
	//  [0][1] = ia*P01*(-Kb) + ia*P01*1 ... 不对

	// 让我重新仔细推导。
	// M = (I-KH)*P
	// M = [ia,   0] * [P00, P01] = [ia*P00,         ia*P01        ]
	//     [-Kb,  1]   [P10, P11]   [-Kb*P00+P10,    -Kb*P01+P11   ]
	//
	// (I-KH)^T = [ia,  -Kb]
	//            [0,    1 ]
	//
	// P_new = M * (I-KH)^T + K*R*K^T
	//
	// P_new[0][0] = ia*P00*ia + ia*P01*0 + KRK^T[0][0]
	//             = ia²*P00 + R*Ka²
	//
	// P_new[0][1] = ia*P00*(-Kb) + ia*P01*1 + KRK^T[0][1]
	//             = -ia*Kb*P00 + ia*P01 + R*Ka*Kb
	//
	// P_new[1][0] = (-Kb*P00+P10)*ia + (-Kb*P01+P11)*0 + KRK^T[1][0]
	//             = ia*(-Kb*P00+P10) + R*Kb*Ka
	//
	// P_new[1][1] = (-Kb*P00+P10)*(-Kb) + (-Kb*P01+P11)*1 + KRK^T[1][1]
	//             = Kb²*P00 - Kb*P10 - Kb*P01 + P11 + R*Kb²

	// 重写（用上面正确的推导）:
	P00_upd = ia * ia * P[0][0]
			  + R_radar * K_angle * K_angle;

	P01_upd = -ia * K_bias * P[0][0]
			  + ia * P[0][1]
			  + R_radar * K_angle * K_bias;

	float P10_upd = ia * (-K_bias * P[0][0] + P[1][0])
					+ R_radar * K_bias * K_angle;

	float P11_upd = K_bias * K_bias * P[0][0]
					- K_bias * (P[1][0] + P[0][1])
					+ P[1][1]
					+ R_radar * K_bias * K_bias;

	// 【修正】强制对称
	float off_diag = 0.5f * (P01_upd + P10_upd);

	P[0][0] = P00_upd;
	P[0][1] = off_diag;
	P[1][0] = off_diag;
	P[1][1] = P11_upd;

	// 【新增】对角元素下限保护，防止协方差坍缩
	if (P[0][0] < P_MIN_DIAG) P[0][0] = P_MIN_DIAG;
	if (P[1][1] < P_MIN_DIAG) P[1][1] = P_MIN_DIAG;


    // ================================================================
    //  7. 输出融合结果
    // ================================================================
    imu.Set_Yaw(angle);
}
}
