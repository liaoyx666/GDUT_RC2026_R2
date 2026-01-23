#include "wheel_leg/physical_param_calc.hpp"


#define IMU_LINK "imu_link"


RobotLink::RobotLink()
{

}

RobotLink::~RobotLink()
{
}


PhysicalParam::PhysicalParam() : tf_buffer(), tf_listener(tf_buffer)
{
    body_links.clear();
    wheel_links.clear();
    M = 0;
    m = 0;
    I = 0;
    i = 0;
    h = 0;
    body_center = KDL::Vector::Zero();
    center_pitch_offset = 0.0;
    imu_pitch_offset = 0.0;
    pitch_offset = 0.0;
}

PhysicalParam::~PhysicalParam()
{
}

void PhysicalParam::getLinkParamFromURDF(const urdf::Model& urdf_model)
{
    body_links.clear();
    wheel_links.clear();

    for (const auto& link_pair : urdf_model.links_) {
        const std::string& link_name = link_pair.first;
        urdf::LinkSharedPtr link = link_pair.second;

        RobotLink current_link;
        current_link.setName(link_name);

        if (link->inertial) {
            // 质量
            current_link.setMass(link->inertial->mass);

            // 质心位置
            urdf::Vector3 urdf_com = link->inertial->origin.position;
            current_link.setCenterOfMass(KDL::Vector(urdf_com.x, urdf_com.y, urdf_com.z));

            // 旋转姿态 - 更安全的方式
            urdf::Rotation urdf_rot = link->inertial->origin.rotation;
            double roll, pitch, yaw;
            urdf_rot.getRPY(roll, pitch, yaw);  // 使用URDF自带的RPY提取方法
            KDL::Rotation kdl_rot = KDL::Rotation::RPY(roll, pitch, yaw);
            current_link.setRotation(kdl_rot);

            // 转动惯量矩阵
            urdf::InertialSharedPtr urdf_inert = link->inertial;
            KDL::RotationalInertia kdl_inert(
                urdf_inert->ixx, urdf_inert->iyy, urdf_inert->izz,  // 对角线元素
                urdf_inert->ixy, urdf_inert->ixz, urdf_inert->iyz   // 非对角线元素
            );
            current_link.setInertia(kdl_inert);
        } else {
            // 建议添加警告日志
            ROS_WARN_STREAM("Link '" << link_name << "' has no inertial parameters, using defaults");
            current_link.setMass(0.0);
            current_link.setCenterOfMass(KDL::Vector::Zero());
            current_link.setRotation(KDL::Rotation::Identity());
            current_link.setInertia(KDL::RotationalInertia::Zero());
        }

        // 分类逻辑
        std::string lower_name = link_name;
        
        // 可以扩展更多轮子标识符
        if (lower_name == "Rlleg_Rwheel" || 
            lower_name == "Lleg_Lwheel") {
            wheel_links.push_back(current_link);
        } else {
            body_links.push_back(current_link);
        }
    }
}


geometry_msgs::TransformStamped PhysicalParam::getTransform(const std::string& source_frame)
{
    if (wheel_links.empty()) {
        ROS_ERROR("轮子链接列表为空，无法获取变换");
        throw std::runtime_error("轮子链接列表为空，无法获取变换");
    }

    try {
        // 查询从source_frame到target_frame的最新变换
        return tf_buffer.lookupTransform(
            wheel_links.at(0).getName(),    // 目标坐标系
            source_frame,    // 源坐标系
            ros::Time(0),    // 最新时刻的变换
            ros::Duration(1.0)  // 超时时间1秒
        );
    } catch (tf2::TransformException& ex) {
        ROS_WARN("获取变换失败(%s → %s):%s", 
                 wheel_links.at(0).getName().c_str(), 
                 source_frame.c_str(), 
                 ex.what());
        // 抛出异常或返回空变换（根据需求处理）
        throw ex;  // 建议抛出，让调用者处理
    }
}



// 从tf变换获取旋转矩阵和平移向量（KDL类型）
void PhysicalParam::getTFTransform(const geometry_msgs::TransformStamped& tf, 
                   KDL::Rotation& R, KDL::Vector& t) {
    // 从四元数转换为KDL旋转矩阵
    const auto& q = tf.transform.rotation;
    R = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
    // 平移向量转换
    const auto& trans = tf.transform.translation;
    t = KDL::Vector(trans.x, trans.y, trans.z);
}







KDL::RotationalInertia PhysicalParam::transformInertia(
    const KDL::RotationalInertia& I_inertial,     // 惯性坐标系下的惯性矩阵
    const KDL::Vector& c_inertial,                // 惯性坐标系下的质心位置
    double mass,                                  // 质量
    const KDL::Rotation& R_inertia_to_link,       // 惯性坐标系→连杆坐标系的旋转矩阵
    const KDL::Rotation& R_link_to_target,        // 连杆坐标系→目标坐标系的旋转矩阵
    const KDL::Vector& t_link_to_target           // 连杆坐标系→目标坐标系的平移向量
)
{
    if (mass <= 0) {
        //ROS_WARN("质量为0,惯性矩阵变换无效");
        return KDL::RotationalInertia::Zero();
    }

    // 将 KDL 向量转换为 Eigen 类型
    Eigen::Vector3d c_inertial_eigen(c_inertial.x(), c_inertial.y(), c_inertial.z());
    Eigen::Vector3d t_link_to_target_eigen(t_link_to_target.x(), t_link_to_target.y(), t_link_to_target.z());

    // 正确的方法将 KDL::Rotation 转换为 Eigen::Matrix3d
    Eigen::Matrix3d R_inertia_to_link_eigen;
    for (int k = 0; k < 3; k++) {
        for (int j = 0; j < 3; j++) {
            R_inertia_to_link_eigen(k, j) = R_inertia_to_link(k, j);
        }
    }

    Eigen::Matrix3d R_link_to_target_eigen;
    for (int k = 0; k < 3; k++) {
        for (int j = 0; j < 3; j++) {
            R_link_to_target_eigen(k, j) = R_link_to_target(k, j);
        }
    }

    // 正确的方法将 KDL::RotationalInertia 转换为 Eigen::Matrix3d
    Eigen::Matrix3d I_inertial_eigen;
    I_inertial_eigen << I_inertial.data[0], I_inertial.data[1], I_inertial.data[2],
                       I_inertial.data[3], I_inertial.data[4], I_inertial.data[5],
                       I_inertial.data[6], I_inertial.data[7], I_inertial.data[8];

    // --------------------------
    // 步骤0：惯性坐标系→连杆坐标系
    // --------------------------
    // 1. 质心从惯性坐标系转换到连杆坐标系
    Eigen::Vector3d c_link = R_inertia_to_link_eigen * c_inertial_eigen;

    // 2. 惯性矩阵从惯性坐标系转换到连杆坐标系
    Eigen::Matrix3d I_link = R_inertia_to_link_eigen * I_inertial_eigen * R_inertia_to_link_eigen.transpose();

    // --------------------------
    // 步骤1：连杆坐标系→连杆质心坐标系（平行轴定理修正）
    // --------------------------
    double c_link_dot = c_link.dot(c_link);
    Eigen::Matrix3d E = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d c_link_outer = c_link * c_link.transpose();  // 外积矩阵 c⊗c
    
    // 平行轴定理：从连杆原点到连杆质心
    Eigen::Matrix3d parallel_axis_link = mass * (c_link_dot * E - c_link_outer);
    Eigen::Matrix3d I_cm_link = I_link - parallel_axis_link;

    // --------------------------
    // 步骤2：连杆质心坐标系→目标坐标系（旋转）
    // --------------------------
    Eigen::Matrix3d I_cm_target = R_link_to_target_eigen * I_cm_link * R_link_to_target_eigen.transpose();

    // --------------------------
    // 步骤3：目标坐标系中的质心位置
    // --------------------------
    Eigen::Vector3d c_target = R_link_to_target_eigen * c_link + t_link_to_target_eigen;

    // --------------------------
    // 步骤4：目标坐标系质心→目标坐标系原点（平行轴定理）
    // --------------------------
    double c_target_dot = c_target.dot(c_target);
    Eigen::Matrix3d c_target_outer = c_target * c_target.transpose();
    
    // 平行轴定理：从质心到目标坐标系原点
    Eigen::Matrix3d parallel_axis_target = mass * (c_target_dot * E - c_target_outer);
    Eigen::Matrix3d I_target_mat = I_cm_target + parallel_axis_target;

    // 转换为 KDL::RotationalInertia
    return KDL::RotationalInertia(
        I_target_mat(0,0), I_target_mat(1,1), I_target_mat(2,2),
        I_target_mat(0,1), I_target_mat(0,2), I_target_mat(1,2)
    );
}







void PhysicalParam::calcPhysicalParam()
{
    // 初始化
    M = 0;
    I = 0;

    m = 0;
    i = 0;

    body_center = KDL::Vector::Zero();

    // 计算车身质量和转动惯量
    for (size_t j = 0; j < body_links.size(); ++j) {
        M += body_links[j].getMass();

        geometry_msgs::TransformStamped transform;

        try {
            // 获取车身链接相对于轮子链接的变换
            transform = getTransform(body_links.at(j).getName());
        } catch (tf2::TransformException& ex) {
            ROS_WARN("跳过车身链接'%s'的惯性变换计算: %s", 
                     body_links.at(j).getName().c_str(), 
                     ex.what());
            continue;  // 跳过当前链接，继续下一个
        }
        
        KDL::Rotation R_link_to_target = KDL::Rotation::Quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);


        KDL::Vector t_link_to_target(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);


        KDL::RotationalInertia rotated_inertia = transformInertia(
            body_links[j].getInertia(),
            body_links[j].getCenterOfMass(),
            body_links[j].getMass(),
            body_links[j].getRotation(),
            R_link_to_target,  // 车身相对于轮子坐标系的旋转矩阵（假设为单位矩阵）
            t_link_to_target       // 车身相对于轮子坐标系的平移向量（假设为零向量）
        );

        body_center = body_center + t_link_to_target * body_links[j].getMass();

        I += rotated_inertia.data[4];// 绕y轴的转动惯量
    }


    // 计算车身质心
    if (M > 0) {
        body_center = body_center / M;
    }


    // 计算车身重心距离
    h = std::sqrt(body_center.z() * body_center.z() + body_center.x() * body_center.x());

    // 计算轮子质量和转动惯量
    for (size_t j = 0; j < wheel_links.size(); ++j) {
        m += wheel_links[j].getMass();

        i += wheel_links[j].getInertia().data[4]; // 绕y轴的转动惯量
    }

    geometry_msgs::TransformStamped imu_to_wheel_transform;

    geometry_msgs::TransformStamped wheel_to_imu_transform;

    Eigen::Vector2d center_pos(
        body_center.x(),
        body_center.z()
    );

    try {
        // 获取IMU相对于轮子链接的变换
        imu_to_wheel_transform = getTransform(IMU_LINK);

        Eigen::Vector2d imu_pos(
            imu_to_wheel_transform.transform.translation.x,
            imu_to_wheel_transform.transform.translation.z
        );

        center_pitch_offset = std::atan2(
            imu_pos.x(),
            imu_pos.y()
        ) - std::atan2(
            center_pos.x(),
            center_pos.y()
        );
    } catch (tf2::TransformException& ex) {
        ROS_WARN("获取IMU到轮子链接的变换失败: %s", ex.what());
        center_pitch_offset = 0.0;
        throw ex;  // 建议抛出，让调用者处理
    }

    try {
        wheel_to_imu_transform = tf_buffer.lookupTransform(
            IMU_LINK,
            wheel_links.at(0).getName(),
            ros::Time(0),
            ros::Duration(1.0)
        );

        Eigen::Vector2d wheel_pos(
            wheel_to_imu_transform.transform.translation.x,
            wheel_to_imu_transform.transform.translation.z
        );

        imu_pitch_offset = std::atan2(
            wheel_pos.x(),
            -wheel_pos.y()
        );

    } catch (tf2::TransformException& ex) {
        ROS_WARN("获取轮子链接到IMU的变换失败: %s", ex.what());
        imu_pitch_offset = 0.0;
        throw ex;  // 建议抛出，让调用者处理
    }


    pitch_offset = center_pitch_offset + imu_pitch_offset;
}