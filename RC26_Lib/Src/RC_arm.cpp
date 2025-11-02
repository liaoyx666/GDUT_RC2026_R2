//#include "RC_arm.h"

// //-------------------------- 工具函数 --------------------------
//float ArmKinematics::normalizeAngle(float angle) {
//    while (angle <= -PI) angle +=   PI;
//    while (angle > ( PI - 0.05)) angle -= PI;
//    return angle;
//}

//float ArmKinematics::constrainValue(float value, float min, float max) {
//    if (value < min) return min;
//    if (value > max) return max;
//    return value;
//}

//ArmMatrix<4, 4> ArmKinematics::buildDHTable(float theta, float alpha, float a, float d, float offset) {
//    ArmMatrix<4, 4> dh_mat;
//    dh_mat.setZero();

//    float ct = cosf(theta + offset);
//    float st = sinf(theta + offset);
//    float ca = cosf(alpha);
//    float sa = sinf(alpha);

//    dh_mat(0, 0) = ct;
//    dh_mat(0, 1) = -st * ca;
//    dh_mat(0, 2) = st * sa;
//    dh_mat(0, 3) = a * ct;

//    dh_mat(1, 0) = st;
//    dh_mat(1, 1) = ct * ca;
//    dh_mat(1, 2) = -ct * sa;
//    dh_mat(1, 3) = a * st;

//    dh_mat(2, 1) = sa;
//    dh_mat(2, 2) = ca;
//    dh_mat(2, 3) = d;

//    dh_mat(3, 3) = 1.0f;
//    return dh_mat;
//}

//// -------------------------- 正运动学实现 --------------------------
//void ArmKinematics::forward(const JointAngles& angles, ActuatorType actuator, EndEffectorPos& end_pos) {
//    // 关节角度使用弧度输入，带偏移
//    float theta0 = constrainValue(angles.theta0 + THETA0_OFFSET, THETA0_MIN, THETA0_MAX);
//    float theta1 = constrainValue(angles.theta1 + THETA1_OFFSET, THETA1_MIN, THETA1_MAX);
//    float theta2 = constrainValue(angles.theta2 + THETA2_OFFSET, THETA2_MIN, THETA2_MAX);
//    float theta3 = constrainValue(angles.theta3 + THETA3_OFFSET, THETA3_MIN, THETA3_MAX);

//    // 构建DH矩阵
//      T01 = buildDHTable(theta0, 0, 0, BASE_HEIGHT, 0);
//      T12 = buildDHTable(theta1, -PI, L1_LENGTH, 0, 0);  
//      T23 = buildDHTable(theta2 + PI, -PI, L2_LENGTH, 0, 0);  
//    if (actuator == ACTUATOR_1)
//        T34 = buildDHTable(theta3, 0, L3_LENGTH, 0, 0);
//    else
//        T34 = buildDHTable(theta3, PI / 2, L3_LENGTH, 0.0f, 0.0f);

//    ArmMatrix<4, 4> T04 = T01 * T12 * T23 * T34;

//    end_pos.x = -T04(0, 3);
//    end_pos.y = T04(1, 3);
//    end_pos.z = T04(2, 3);

//    // 末端角度用弧度表示
//    end_pos.angle = (actuator == ACTUATOR_1)
//        ? (theta1 + theta2 + theta3)
//        : (theta1 + theta2 + theta3 + PI / 2);
//}

//// -------------------------- 逆运动学 --------------------------
//int ArmKinematics::inverse(const EndEffectorPos& target_pos,
//                           const JointAngles& current_angles,
//                           ActuatorType actuator,
//                           JointAngles& result_angles)
//{
//num_valid_solutions_ = 0;

//    float x_raw = -target_pos.x;  // 还原正解中T04(0,3)的符号
//    float y = target_pos.y;
//    float z = target_pos.z;
//    float target_angle = target_pos.angle;

//    if (actuator == ACTUATOR_2)
//        target_angle -= PI / 2.0f;

//    // 求解theta0（适配x_raw的符号）
//    float theta0_cand[2] = {
//        atan2f(y, x_raw),
//        atan2f(y, x_raw) - PI
//    };

//    for (int i = 0; i < 2; ++i) {
//        float t0 = theta0_cand[i];
//        float ct0 = cosf(t0);
//        float st0 = sinf(t0);

//        // 转换到关节1坐标系（匹配正解的旋转逻辑）
//        float x_prime = x_raw * ct0 + y * st0;
//        float y_prime = -x_raw * st0 + y * ct0;
//        float r = sqrtf(x_prime * x_prime + y_prime * y_prime);
//        float z_prime = z - BASE_HEIGHT;  // 减去T01的d偏移

//        // 计算腕部中心（关节3）位置（匹配T34的平移）
//        float r_wrist = r - L3_LENGTH * cosf(target_angle);
//        float z_wrist = z_prime - L3_LENGTH * sinf(target_angle);

//        float D_sq = r_wrist * r_wrist + z_wrist * z_wrist;
//        if (D_sq < 0.0f) continue;

//        float D = sqrtf(D_sq);
//        if (D > L1_LENGTH + L2_LENGTH || D < fabsf(L1_LENGTH - L2_LENGTH))
//            continue;

//        // 求解theta2（抵消正解中T23的+π偏移）
//        float cos_t2 = (D_sq - L1_LENGTH*L1_LENGTH - L2_LENGTH*L2_LENGTH) / (2*L1_LENGTH*L2_LENGTH);
//        cos_t2 = constrainValue(cos_t2, -1.0f, 1.0f);
//        float t2_calc = acosf(cos_t2);  // 原始计算角度
//        float t2 = normalizeAngle(t2_calc - PI);  // 减去正解中的+π
//				if (t2<0) t2 +=PI;

//        float beta = atan2f(L2_LENGTH * sinf(t2_calc), L1_LENGTH + L2_LENGTH * cosf(t2_calc));
//        float t1 = atan2f(z_wrist, r_wrist) - beta;
//        //t1 = normalizeAngle(t1);
//				if(t1 < 0) t1 += PI;


//        float t3 = target_angle - t1 - t2;
//				if (t3 <0) t3 +=PI;

//        if (t0 >= THETA0_MIN && t0 <= THETA0_MAX &&
//            t1 >= 0.0f && t1 <= PI &&  // 大臂0~π
//            t2 >= THETA2_MIN && t2 <= THETA2_MAX &&
//            t3 >= THETA3_MIN && t3 <= THETA3_MAX)
//        {
//            if (num_valid_solutions_ < 2)
//                valid_solutions_[num_valid_solutions_++] = {t0, t1, t2, t3};
//        }
//    }

//    if (num_valid_solutions_ == 0)
//        return 2;

//    int best_idx = 0;
//    float best_score = -1e9f;

//    for (int i = 0; i < num_valid_solutions_; ++i) {
//        EndEffectorPos check;
//        forward(valid_solutions_[i], actuator, check);

//        float reward = calc_reward(valid_solutions_[i], check, target_pos, current_angles);
//        float base_diff = fabsf(valid_solutions_[i].theta0 - current_angles.theta0);
//        float score = reward - 5.0f * base_diff;

//        if (score > best_score) {
//            best_score = score;
//            best_idx = i;
//        }
//    }

//    result_angles = valid_solutions_[best_idx];
//    return 0;
//}

//// -------------------------- 奖励函数 --------------------------
//float ArmKinematics::calc_reward(const JointAngles& q,
//                                 const EndEffectorPos& end_pos,
//                                 const EndEffectorPos& target,
//                                 const JointAngles& current_angles)
//{
//    const float W_DIST_ERROR = 1.0f;
//    const float W_ROT_PENALTY = 0.5f;
//    const float W_BASE_PENALTY = 2.0f;
//    const float BASE_PENALTY_THRESHOLD = 5.0f * PI / 6.0f;

//    float dist_err = sqrtf(powf(end_pos.x - target.x, 2) +
//                           powf(end_pos.y - target.y, 2) +
//                           powf(end_pos.z - target.z, 2));

//    float rot_penalty = fabsf(q.theta0 - current_angles.theta0);
//    float base_rot = fabsf(q.theta0);
//    float base_penalty = (base_rot > BASE_PENALTY_THRESHOLD)
//                             ? (base_rot - BASE_PENALTY_THRESHOLD)
//                             : 0.0f;

//    return -(W_DIST_ERROR * dist_err +
//             W_ROT_PENALTY * rot_penalty +
//             W_BASE_PENALTY * base_penalty);
//}
#include "RC_arm.h"
#include "math.h"

// ------------------ 静态成员变量 ------------------
ArmMatrix<4, 4> ArmKinematics::T01;
ArmMatrix<4, 4> ArmKinematics::T12;
ArmMatrix<4, 4> ArmKinematics::T23;
ArmMatrix<4, 4> ArmKinematics::T34;
ArmMatrix<4, 4> ArmKinematics::T04;

// ------------------ 工具函数 ------------------
float ArmKinematics::normalizeAngle(float angle) {
    while (angle <= -PI) angle += 2.0f * PI;
    while (angle > PI) angle -= 2.0f * PI;
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
    float theta0 = constrainValue(angles.theta0 + THETA0_OFFSET, THETA0_MIN, THETA0_MAX);
    float theta1 = constrainValue(angles.theta1 + THETA1_OFFSET, THETA1_MIN, THETA1_MAX);
    float theta2 = constrainValue(angles.theta2 + THETA2_OFFSET, THETA2_MIN, THETA2_MAX);
    float theta3 = constrainValue(angles.theta3 + THETA3_OFFSET, THETA3_MIN, THETA3_MAX);

    // 零位 X=0.09
    T01 = buildDHTable(theta0, PI/2, 0.0f, BASE_HEIGHT, 0.0f); // 底座旋转
    T12 = buildDHTable(theta1, 0.0f, -L1_LENGTH, 0.0f, 0.0f); // 大臂沿 -X
    T23 = buildDHTable(theta2, 0.0f, L2_LENGTH, 0.0f, 0.0f);  // 小臂沿 +X
    if (actuator == ACTUATOR_1)
        T34 = buildDHTable(theta3, 0.0f, L3_LENGTH, 0.0f, 0.0f);
    else
        T34 = buildDHTable(theta3, PI/2, L3_LENGTH, 0.0f, 0.0f);

    T04 = T01 * T12 * T23 * T34;

    end_pos.x = T04(0,3);
    end_pos.y = T04(1,3);
    end_pos.z = T04(2,3);
    end_pos.angle = (actuator==ACTUATOR_1)?(theta1+theta2+theta3):(theta1+theta2+theta3+PI/2);
}

// ------------------ 逆运动学 ------------------
int ArmKinematics::inverse(const EndEffectorPos& target_pos,
                           const JointAngles& current_angles,
                           ActuatorType actuator,
                           JointAngles& result_angles)
{
    num_valid_solutions_ = 0;

    float x = target_pos.x;
    float y = target_pos.y;
    float z = target_pos.z;
    float target_angle = target_pos.angle;

    if (actuator == ACTUATOR_2)
        target_angle -= PI/2;

    float r_target = sqrtf(x*x + y*y);
    float z_target = z - BASE_HEIGHT;

    // 两种基座解
    float t0_sols[2] = { normalizeAngle(atan2f(y,x)), normalizeAngle(atan2f(y,x)+PI) };
    float r_plane_sols[2] = { r_target, -r_target };

    for(int i=0; i<2; i++){
        float t0 = t0_sols[i];
        float r_plane = r_plane_sols[i];

        // 手腕中心位置
        float r_wrist = r_plane - L3_LENGTH*cosf(target_angle);
        float z_wrist = z_target - L3_LENGTH*sinf(target_angle);

        float D_sq = r_wrist*r_wrist + z_wrist*z_wrist;
        if(D_sq <= 1e-6f) continue;

        float D = sqrtf(D_sq);
        if(D > L2_LENGTH+L1_LENGTH + 1e-6f || D < fabsf(L2_LENGTH-L1_LENGTH) - 1e-6f) continue;

        float cos_t2 = (L1_LENGTH*L1_LENGTH + L2_LENGTH*L2_LENGTH - D_sq)/(2*L1_LENGTH*L2_LENGTH);
        // Snapping for stability
        if (fabsf(cos_t2 + 1.0f) < 1e-8f) cos_t2 = -1.0f;
        else if (fabsf(cos_t2 - 1.0f) < 1e-8f) cos_t2 = 1.0f;
        cos_t2 = constrainValue(cos_t2, -1.0f, 1.0f);
        float t2_sols[2] = { acosf(cos_t2), -acosf(cos_t2) };

        for(int j=0; j<2; j++){
            float t2 = t2_sols[j];
            float beta = atan2f(L2_LENGTH * sinf(t2), -L1_LENGTH + L2_LENGTH * cosf(t2));
            float t1 = atan2f(z_wrist, r_wrist) - beta;
            t1 = normalizeAngle(t1);
            if (t1 < 0.0f) t1 +=  PI;

            //t2 = normalizeAngle(t2);
            if (t2 < 0.0f) t2 += 2.0f * PI;

            float t3 = normalizeAngle(target_angle - t1 - t2);
            if (t3 < 0.0f) t3 +=  PI;

            t3 = constrainValue(t3, THETA3_MIN, THETA3_MAX);
            // Preference for t3 = 0 if near π (avoid flip)
            if (fabsf(t3 - PI) < 1e-6f) t3 = 0.0f;

             if(t0 >= THETA0_MIN && t0 <= THETA0_MAX &&
               t1 >= THETA1_MIN && t1 <= THETA1_MAX &&
               t2 >= THETA2_MIN && t2 <= THETA2_MAX &&
               t3 >= THETA3_MIN && t3 <= THETA3_MAX)
            {
                if(num_valid_solutions_ < 4){
                    valid_solutions_[num_valid_solutions_++] = {
                        t0 - THETA0_OFFSET,
                        t1 - THETA1_OFFSET,
                        t2 - THETA2_OFFSET,
                        t3 - THETA3_OFFSET
                    };
                }
            }
        }
    }

    if(num_valid_solutions_ == 0) return 2;

    // Solution selection with preferences
    int best_idx = 0;
    float best_score = -1e9f;
    for(int i = 0; i < num_valid_solutions_; i++){
        EndEffectorPos check;
        forward(valid_solutions_[i], actuator, check);
        
        float reward = calc_reward(valid_solutions_[i], check, target_pos, current_angles);
        float angle_penalty = 1.0f * fabsf(normalizeAngle(check.angle - target_angle));
        float base_diff = fabsf(normalizeAngle(valid_solutions_[i].theta0 - current_angles.theta0));
        float stretch_penalty = 0.2f * fabsf(valid_solutions_[i].theta1 + valid_solutions_[i].theta2 - PI);  // Favor extension
        float elbow_up_bonus = (valid_solutions_[i].theta2 > 0.0f) ? 1.0f : 0.0f;  // Strong elbow-up preference
        
        float score = reward - angle_penalty - 5.0f * base_diff - stretch_penalty + elbow_up_bonus;
        if(score > best_score){
            best_score = score;
            best_idx = i;
        }
    }

    result_angles = valid_solutions_[best_idx];
    return 0;
}

// ------------------ 奖励函数 ------------------
float ArmKinematics::calc_reward(const JointAngles& q,
                                 const EndEffectorPos& end_pos,
                                 const EndEffectorPos& target,
                                 const JointAngles& current_angles)
{
    const float W_DIST_ERROR = 1.0f;
    const float W_ROT_PENALTY = 0.5f;
    const float W_BASE_PENALTY = 2.0f;
    const float BASE_PENALTY_THRESHOLD = 5.0f*PI/6.0f;

    float dist_err = sqrtf(powf(end_pos.x - target.x,2) +
                           powf(end_pos.y - target.y,2) +
                           powf(end_pos.z - target.z,2));
    float rot_penalty = fabsf(normalizeAngle(q.theta0 - current_angles.theta0));
    float base_rot = fabsf(q.theta0);
    float base_penalty = (base_rot>BASE_PENALTY_THRESHOLD)? (base_rot-BASE_PENALTY_THRESHOLD) : 0.0f;

    // Add limit proximity penalty
    float limit_penalty = 10.0f * fmaxf(0.0f, (q.theta1 - (THETA1_MAX - 0.1f)));

    return -(W_DIST_ERROR*dist_err + W_ROT_PENALTY*rot_penalty + W_BASE_PENALTY*base_penalty + limit_penalty);
}
