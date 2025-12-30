#include "RC_arm.h"

// -------------------------- 私有工具函数 --------------------------
// 将角度归一化到 [-PI, PI] 范围
float ArmKinematics::normalizeAngle(float angle) {
    while (angle <= -PI) angle += 2.0f * PI;
    while (angle > PI) angle -= 2.0f * PI;
    return angle;
}

// 数值限幅
float ArmKinematics::constrainValue(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 构建DH变换矩阵
ArmMatrix<4, 4> ArmKinematics::buildDHTable(float theta, float alpha, float a, float d, float offset) {
    ArmMatrix<4, 4> dh_mat;
    dh_mat.setZero();

    float ct = cosf(theta + offset);
    float st = sinf(theta + offset);
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

    dh_mat(3, 3) = 1.0f;
    return dh_mat;
}

// -------------------------- 正运动学实现 --------------------------
void ArmKinematics::forward(const JointAngles& angles, ActuatorType actuator, EndEffectorPos& end_pos) {
    // 关节角度使用弧度输入，带偏移
    float theta0 = constrainValue(angles.theta0 + THETA0_OFFSET, THETA0_MIN, THETA0_MAX);
    float theta1 = constrainValue(angles.theta1 + THETA1_OFFSET, THETA1_MIN, THETA1_MAX);
    float theta2 = constrainValue(angles.theta2 + THETA2_OFFSET, THETA2_MIN, THETA2_MAX);
    float theta3 = constrainValue(angles.theta3 + THETA3_OFFSET, THETA3_MIN, THETA3_MAX);

    // 构建DH矩阵
    ArmMatrix<4, 4> T01 = buildDHTable(theta0, PI / 2, 0.0f, BASE_HEIGHT, 0.0f);
    ArmMatrix<4, 4> T12 = buildDHTable(theta1, 0.0f, L1_LENGTH, 0.0f, 0.0f);
    ArmMatrix<4, 4> T23 = buildDHTable(theta2, 0.0f, L2_LENGTH, 0.0f, 0.0f);
    ArmMatrix<4, 4> T34;
    if (actuator == ACTUATOR_1)
        T34 = buildDHTable(theta3, 0.0f, L3_LENGTH, 0.0f, 0.0f);
    else
        T34 = buildDHTable(theta3, PI / 2, L3_LENGTH, 0.0f, 0.0f);

    ArmMatrix<4, 4> T04 = T01 * T12 * T23 * T34;

    end_pos.x = T04(0, 3);
    end_pos.y = T04(1, 3);
    end_pos.z = T04(2, 3);

<<<<<<< Updated upstream
    // 末端角度用弧度表示
    end_pos.angle = (actuator == ACTUATOR_1)
        ? (theta1 + theta2 + theta3)
        : (theta1 + theta2 + theta3 + PI / 2);
=======
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
>>>>>>> Stashed changes
}



// -------------------------- 逆运动学实现 (已重构, 不使用vector) --------------------------
int ArmKinematics::inverse(const EndEffectorPos& target_pos,
                           const JointAngles& current_angles,
                           ActuatorType actuator,
                           JointAngles& result_angles)
{
    float x = target_pos.x;
    float y = target_pos.y;
    float z = target_pos.z;
    float target_angle = target_pos.angle;

    if (actuator == ACTUATOR_2)
        target_angle -= PI / 2.0f;

    // 底座两个候选角度
    float theta0_cand[2] = {
        normalizeAngle(atan2f(y, x)),
        normalizeAngle(atan2f(y, x) + PI)
    };

    // 使用成员变量代替局部数组
    num_valid_solutions_ = 0;

    for (int i = 0; i < 2; ++i) {
        float t0 = theta0_cand[i];
        float ct0 = cosf(t0);
        float st0 = sinf(t0);

        // 转到平面坐标
        float x_prime = x * ct0 + y * st0;
        float y_prime = -x * st0 + y * ct0;
        float r = sqrtf(x_prime * x_prime + y_prime * y_prime);
        float z_prime = z - BASE_HEIGHT;

        // 手腕中心位置
        float r_wrist = r - L3_LENGTH * cosf(target_angle);
        float z_wrist = z_prime - L3_LENGTH * sinf(target_angle);

        float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
        if (D_sq <= 0.0f) continue; // 防止 sqrtf(负数)

        float D = sqrtf(D_sq);

        // 可达性检查
        if (D > L1_LENGTH + L2_LENGTH || D < fabsf(L1_LENGTH - L2_LENGTH))
            continue;

        // 肘上解
        float cos_t2 = (D_sq - L1_LENGTH * L1_LENGTH - L2_LENGTH * L2_LENGTH)
                     / (2 * L1_LENGTH * L2_LENGTH);
        cos_t2 = constrainValue(cos_t2, -1.0f, 1.0f);
        float t2 = acosf(cos_t2);

        // 求 t1
        float beta = atan2f(L2_LENGTH * sinf(t2), L1_LENGTH + L2_LENGTH * cosf(t2));
        float t1 = atan2f(z_wrist, r_wrist) - beta;
        float t3 = target_angle - t1 - t2;

        // 归一化
        t0 = normalizeAngle(t0);
        t1 = normalizeAngle(t1);
        t2 = normalizeAngle(t2);
        t3 = normalizeAngle(t3);

        // 限位检查
        if (t0 >= THETA0_MIN && t0 <= THETA0_MAX &&
            t1 >= THETA1_MIN && t1 <= THETA1_MAX &&
            t2 >= THETA2_MIN && t2 <= THETA2_MAX &&
            t3 >= THETA3_MIN && t3 <= THETA3_MAX)
        {
            if (num_valid_solutions_ < 2) {
                valid_solutions_[num_valid_solutions_++] = {t0, t1, t2, t3};
            }
        }
    }

    if (num_valid_solutions_ == 0)
        return 2; // 无解

    // -------------------- 选择最优解 --------------------
    int best_idx = 0;
    float best_score = -1e9f;

    for (int i = 0; i < num_valid_solutions_; ++i) {
        EndEffectorPos check;
        forward(valid_solutions_[i], actuator, check);

        float reward = calc_reward(valid_solutions_[i], check, target_pos, current_angles);

        float base_diff = fabsf(normalizeAngle(valid_solutions_[i].theta0 - current_angles.theta0));

        // 惩罚底座旋转幅度过大
        float score = reward - 5.0f * base_diff;

        if (score > best_score) {
            best_score = score;
            best_idx = i;
        }
    }

    result_angles = valid_solutions_[best_idx];
    return 0; // 成功
}


// 奖励函数实现
float ArmKinematics::calc_reward(const JointAngles& q,
                                 const EndEffectorPos& end_pos,
                                 const EndEffectorPos& target,
                                 const JointAngles& current_angles)
{
    const float W_DIST_ERROR = 1.0f;
    const float W_ROT_PENALTY = 0.5f;
    const float W_BASE_PENALTY = 2.0f;
    const float BASE_PENALTY_THRESHOLD = 5.0f * PI / 6.0f;

    float dist_err = sqrtf(powf(end_pos.x - target.x, 2) +
                           powf(end_pos.y - target.y, 2) +
                           powf(end_pos.z - target.z, 2));

    float rot_penalty = fabsf(q.theta0 - current_angles.theta0);
    float base_rot = fabsf(q.theta0);
    float base_penalty = (base_rot > BASE_PENALTY_THRESHOLD)
                             ? (base_rot - BASE_PENALTY_THRESHOLD)
                             : 0.0f;

    float reward = -(W_DIST_ERROR * dist_err + W_ROT_PENALTY * rot_penalty + W_BASE_PENALTY * base_penalty);
    return reward;
}


ArmMatrix<4, 4> ArmKinematics::calculateJacobian(const JointAngles& q_current, ActuatorType actuator) {
    const float EPSILON = 1e-4f; // 微小变化量
    ArmMatrix<4, 4> J;
    JointAngles q = q_current;
    EndEffectorPos y_base;
    // 1. 计算当前位姿
    forward(q_current, actuator, y_base); 
    
    // 遍历 4 个关节
    for (int j = 0; j < 4; ++j) { 
        // 确保使用副本 q，而不是 q_current
        float original_q_j = (j == 0) ? q.theta0 : (j == 1) ? q.theta1 : (j == 2) ? q.theta2 : q.theta3;

        // 引入微小变化到 q.theta_j
        float* theta_j_ptr = (j == 0) ? &q.theta0 : (j == 1) ? &q.theta1 : (j == 2) ? &q.theta2 : &q.theta3;
        *theta_j_ptr += EPSILON; 

        EndEffectorPos y_new;
        forward(q, actuator, y_new); // 2. 计算微小变化后的新位姿
        
        // 3. 计算雅可比列 (数值微分近似 J = dY / dQ)
        // J(row, col) = (Y_new.row - Y_base.row) / EPSILON
        J(0, j) = (y_new.x - y_base.x) / EPSILON;      // X 对 dTheta_j 的影响
        J(1, j) = (y_new.y - y_base.y) / EPSILON;      // Y 对 dTheta_j 的影响
        J(2, j) = (y_new.z - y_base.z) / EPSILON;      // Z 对 dTheta_j 的影响
        J(3, j) = (y_new.angle - y_base.angle) / EPSILON; // Angle 对 dTheta_j 的影响
        
        // 恢复原值
        *theta_j_ptr = original_q_j;
    }
    return J;
}

int ArmKinematics::mpc_init(const JointAngles& initial_angles, float q_weight, float r_weight) {
    if (mpc_initialized_) {
        return 0; 
    }

    // 1. 计算初始雅可比矩阵 J 作为动态矩阵 A
    ArmMatrix<OUT_DIM, IN_DIM> J = calculateJacobian(initial_angles, ACTUATOR_1); 
    
    // 2. 构造动态矩阵 a0 (纯增益模型近似：所有预测步都使用 J)
    float a0_data[P_HORIZON][OUT_DIM][IN_DIM];
    const float* j_data = J.getData(); 
    size_t j_size = sizeof(float) * OUT_DIM * IN_DIM;
    
    for (int i = 0; i < P_HORIZON; ++i) {
        // 使用 memcpy 复制雅可比数据到 a0 数组
        memcpy(a0_data[i], j_data, j_size);
    }

    // 3. 初始化 MPC
    mpc_controller_.init(a0_data, q_weight, r_weight); 

    // 4. 设置约束 (限制关节变化量，重要！)
    // 使用 float 数组初始化 ArmMatrix，符合 ArmMatrix(const float*) 构造函数
    float du_min_data[IN_DIM] = {-0.05f, -0.05f, -0.05f, -0.05f}; 
    float du_max_data[IN_DIM] = {0.05f, 0.05f, 0.05f, 0.05f}; 
    
    ArmMatrix<1, IN_DIM> du_min(du_min_data); 
    ArmMatrix<1, IN_DIM> du_max(du_max_data); 
    mpc_controller_.setConstraints(du_min, du_max); 
    
    // 5. 设置平滑/补偿系数
    mpc_controller_.setFilterCoefficient(0.8f); 
    mpc_controller_.setErrorCompensation(0.9f); 
    
    mpc_initialized_ = true;
    return 0;
}

// -------------------------- MPC 控制步长实现 --------------------------

JointAngles ArmKinematics::mpc_control_step(const EndEffectorPos& target_pos, const JointAngles& current_angles) {
    if (!mpc_initialized_) {
        // 未初始化则返回当前角度
        return current_angles; 
    }

    // 1. 测量当前末端位姿 (Y_meas)
    EndEffectorPos current_pos;
    forward(current_angles, ACTUATOR_1, current_pos); 

    // 2. 构造 MPC 输入/输出矩阵 (ArmMatrix<1, 4>)
    
    // Y_meas: 当前位姿 [x, y, z, angle]
    float Y_meas_data[OUT_DIM] = {
        current_pos.x, 
        current_pos.y, 
        current_pos.z, 
        current_pos.angle
    };
    ArmMatrix<1, OUT_DIM> Y_meas(Y_meas_data);

    // Y_ref: 目标位姿 [x, y, z, angle]
    float Y_ref_data[OUT_DIM] = {
        target_pos.x, 
        target_pos.y, 
        target_pos.z, 
        target_pos.angle
    };
    ArmMatrix<1, OUT_DIM> Y_ref(Y_ref_data);

    // u_prev: 上一时刻关节角度的绝对值 [theta0, theta1, theta2, theta3]
    float u_prev_data[IN_DIM] = {
        current_angles.theta0, 
        current_angles.theta1, 
        current_angles.theta2, 
        current_angles.theta3
    };
    ArmMatrix<1, IN_DIM> u_prev(u_prev_data);

    // 3. 运行 MPC 计算下一时刻的关节角度 (u_next)
    // mpc_controller_.compute() 返回下一时刻的绝对控制量 u(k+1)
    ArmMatrix<1, IN_DIM> u_next_mat = mpc_controller_.compute(Y_meas, Y_ref, u_prev); 

    // 4. 提取新的关节角度
    JointAngles next_angles;
    next_angles.theta0 = u_next_mat(0, 0);
    next_angles.theta1 = u_next_mat(0, 1);
    next_angles.theta2 = u_next_mat(0, 2);
    next_angles.theta3 = u_next_mat(0, 3);
    
    return next_angles;
}

void ArmDynamics::gravity_compensation(JointAngles& joint_angle_now)
{
    joint_angle_now.theta1 = j60::pos;
    joint_angle_now.theta2 = dm4310::pos + j60::pos + pi;
    joint_angle_now.theta3 = m2006::pos + dm4310::pos + j60::pos;


    joint_gravity_compensation.joint1 = L1_gravity * L1_Particle_LENGTH * cos(joint_angle_now.theta1) 
                                      + L2_gravity * (L2_Particle_LENGTH * cos(joint_angle_now.theta2) + L1_LENGTH * cos(joint_angle_now.theta1));
                                      + L3_gravity * (L3_Particle_LENGTH * cos(joint_angle_now.theta3) + L2_LENGTH * cos(joint_angle_now.theta2) + L1_LENGTH * cos(joint_angle_now.theta1));

    joint_gravity_compensation.joint2 = L2_gravity * L2_Particle_LENGTH * cos(joint_angle_now.theta2)
                                      + L3_gravity * (L3_Particle_LENGTH * cos(joint_angle_now.theta3) + L2_LENGTH * cos(joint_angle_now.theta2));

    joint_gravity_compensation.joint3 = L3_gravity * L3_Particle_LENGTH * cos(joint_angle_now.theta3);
}
