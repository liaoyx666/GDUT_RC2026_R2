#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>  // 用于监听TF变换
#include <tf2_ros/buffer.h>              // 用于缓存TF变换
#include <geometry_msgs/TransformStamped.h>  // 存储单组变换的消息类型
#include <tf2/exceptions.h>              // 用于处理TF异常
#include <cmath>

class RobotLink
{
private:
    KDL::RotationalInertia inertia;// 转动惯量
    KDL::Vector center_of_mass;// 质心位置
    KDL::Rotation rotation;// 旋转矩阵
    double mass;// 质量
    std::string name;// 链接名称

public:
    RobotLink();
    virtual ~RobotLink();

    void setName(const std::string& link_name) {
        name = link_name;
    }
    void setMass(double link_mass) {
        mass = link_mass;
    }
    void setCenterOfMass(const KDL::Vector& com) {
        center_of_mass = com;
    }
    void setRotation(const KDL::Rotation& rot) {
        rotation = rot;
    }
    void setInertia(const KDL::RotationalInertia& inert) {
        inertia = inert;
    }


    std::string getName() const {
        return name;
    }
    double getMass() const {
        return mass;
    }
    KDL::Vector getCenterOfMass() const {
        return center_of_mass;
    }
    KDL::Rotation getRotation() const {
        return rotation;
    }
    KDL::RotationalInertia getInertia() const {
        return inertia;
    }
};





class PhysicalParam
{
private:
    double M = 0; // 车身质量
    double m = 0; // 轮子质量
    double I = 0; // 车身转动惯量
    double i = 0; // 轮子转动惯量
    double h = 0; // 车身重心高度
    KDL::Vector body_center; // 车身整体质心位置
    double pitch_offset = 0; // 俯仰角偏移量

    double center_pitch_offset = 0.0;
    double imu_pitch_offset = 0.0;


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    geometry_msgs::TransformStamped getTransform(const std::string& source_frame);
    void getTFTransform(const geometry_msgs::TransformStamped& tf, 
                   KDL::Rotation& R, KDL::Vector& t);

    KDL::RotationalInertia transformInertia(
        const KDL::RotationalInertia& I_inertial,     // 惯性坐标系下的惯性矩阵（URDF读取值）
        const KDL::Vector& c_inertial,                // 惯性坐标系下的质心位置（URDF的xyz）
        double mass,                                  // 质量
        const KDL::Rotation& R_inertia_to_link,       // 惯性坐标系→连杆坐标系的旋转矩阵（你已转换的rotation）
        const KDL::Rotation& R_link_to_target,        // 连杆坐标系→目标坐标系的旋转矩阵（来自TF）
        const KDL::Vector& t_link_to_target           // 连杆坐标系→目标坐标系的平移向量（来自TF）
    );

    

public:
    PhysicalParam();
    virtual ~PhysicalParam();

    std::vector<RobotLink> body_links;
    std::vector<RobotLink> wheel_links;

    void getLinkParamFromURDF(const urdf::Model& urdf_model);
    void calcPhysicalParam();

    double getBodyMass() const {
        return M;
    }
    double getWheelMass() const {
        return m;
    }
    double getBodyInertia() const {
        return I;
    }
    double getWheelInertia() const {
        return i;
    }
    double getBodyCenterHeight() const {
        return h;
    }
    double getPitchOffset() const {
        return pitch_offset;
    }
    double getCenterPitchOffset() const {
        return center_pitch_offset;
    }
    double getImuPitchOffset() const {
        return imu_pitch_offset;
    }

};