#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "wheel_leg/lqr_calc.hpp"
#include "wheel_leg/physical_param_calc.hpp"
#include "wheel_leg/my_pid.hpp"
#include "wheel_leg/leg_pos_controlConfig.h"
#include <dynamic_reconfigure/server.h>
#include "wheel_leg/ground_off_detect.hpp"
#include "wheel_leg/wl_control.h"


// 机器人参数
constexpr double WHEEL_RADIUS = 0.06;  // 轮子半径（m）
constexpr int RATE_HZ = 1000;          // 循环频率（Hz）
constexpr double LEG_LENGTH = 0.3;    // 机械腿长度（m）

constexpr double L1 = 0.1;
constexpr double L2 = 0.2;
constexpr double L3 = 0.25;

double K[4] = {-10.9725, -1.6921, 0.4472, 0.8619};  // LQR增益矩阵
double x[4] = {0.0, 0.0, 0.0, 0.0};   // 状态变量：位置、速度、姿态角、角速度



// 全局变量：缓存数据 + 路程积分相关
double left_wheel_vel = 0.0;   // 左轮速度（rad/s）
double right_wheel_vel = 0.0;  // 右轮速度（rad/s）
double robot_vel = 0.0;        // 机器人线速度（m/s）
double roll = 0.0, pitch = 0.0, yaw = 0.0;  // IMU欧拉角（rad）
double ang_vel_x = 0.0, ang_vel_y = 0.0, ang_vel_z = 0.0;  // IMU角速度（rad/s）
double acc_z = 0.0;              // IMU加速度（m/s²）


double total_pos = 0.0;   // 总路程（m，积分结果）
ros::Time last_time;           // 上一时刻时间戳（计算dt用）

double l_leg_pos = 0.0;  // 左腿位置
double r_leg_pos = 0.0;  // 右腿位置
double l_leg_effort = 0.0; // 左腿力矩
double r_leg_effort = 0.0; // 右腿力矩


double leg_pos = 0.0;  // 添加leg_pos的声明s
double t_yaw = 0.0;  // 添加t_yaw的声明
dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig> *dr_server = nullptr;  // 添加dr_server的声明








double target_lin_vel = 0.0;   // 目标线速度（m/s）
double target_ang_vel = 0.0;   // 目标角速度（rad/s）
double target_leg_pos = 0.0;   // 目标腿位置（rad）






double rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}


const std::string LEFT_WHEEL_JOINT = "Lwheel";  // 左轮关节名
const std::string RIGHT_WHEEL_JOINT = "Rwheel"; // 右轮关节名


const std::string L_LEG_JOINT = "Lhleg";  // 左腿关节名
const std::string R_LEG_JOINT = "Rhleg";  // 右腿关节名





// 回调函数定义
void controlCallback(const wheel_leg::wl_control::ConstPtr& msg) {
    target_lin_vel = msg->linear_vel;
    target_ang_vel = msg->angular_vel;
    target_leg_pos = msg->leg_pos;
}





// 轮子速度回调函数
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name.size() != msg->velocity.size()) {
        ROS_WARN("JointState消息中name和velocity数组长度不匹配");
        return; // 数据无效，直接退出，不执行后续逻辑
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == LEFT_WHEEL_JOINT) {
            left_wheel_vel = msg->velocity[i];
        } else if (msg->name[i] == RIGHT_WHEEL_JOINT) {
            right_wheel_vel = msg->velocity[i];
        }

        if (msg->name[i] == L_LEG_JOINT) {
            l_leg_pos = msg->position[i];
            l_leg_effort = msg->effort[i];
        } else if (msg->name[i] == R_LEG_JOINT) {
            r_leg_pos = msg->position[i];
            r_leg_effort = msg->effort[i];
        }
    }

    // 计算机器人线速度（m/s）
    robot_vel = WHEEL_RADIUS * (left_wheel_vel + right_wheel_vel) / 2.0;
}



// IMU回调函数：获取姿态和角速度
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 获取角速度（x、y、z轴）
    ang_vel_x = msg->angular_velocity.x;
    ang_vel_y = msg->angular_velocity.y;
    ang_vel_z = msg->angular_velocity.z;

    acc_z = msg->linear_acceleration.z;

    // 四元数转欧拉角（姿态）
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;
    roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    pitch = asin(2.0 * (qw * qy - qz * qx));
    yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}




void dynamicReconfigCallback(wheel_leg::leg_pos_controlConfig &config, uint32_t level) {
    leg_pos = config.leg_pos;
    t_yaw = config.t_yaw;
    //ROS_INFO("动态重配置: leg_pos = %.3f", leg_pos);
}






int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lqr_controller_node");
    ros::NodeHandle nh;


    PhysicalParam physical_param;

    LqrCalc lqr_calc(
        10.113 - 0.173520730578001 * 2,    // M 机器人质量（kg）
        0.173520730578001 * 2,    // m 轮子质量（kg）
        0.870168809 - 0.000468745184860407 * 2,    // I 机器人转动惯量（kg·m²）
        0.000468745184860407 * 2,   // i 轮子转动惯量（kg·m²）
        0.300,    // h 机器人重心高度（m）
        WHEEL_RADIUS, // r 轮子半径（m）
        Eigen::Vector4d(1, 1, 1, 1), // Q 状态权重矩阵对角线元素
        40     // R 控制权重标量
    );



    MyPid sdp_pid(1.5, 0.4, 0.02);
    sdp_pid.setOutputLimits(-2.0, 2.0);
    sdp_pid.setIntegralLimits(-2.0, 2.0);



    MyPid pos_pid(1.5, 0.5, 0.01);
    pos_pid.setOutputLimits(-4, 4);
    pos_pid.setIntegralLimits(-4, 4);




    MyPid yaw_pid(2.0, 0.5, 0.01);
    yaw_pid.setOutputLimits(-2.0, 2.0);
    yaw_pid.setIntegralLimits(-2.0, 2.0);



    MyPid yaw_pos_pid(5.0, 0.0, 0.1);
    yaw_pos_pid.setOutputLimits(-3.5, 3.5);
    yaw_pos_pid.setIntegralLimits(-3.5, 3.5);


    GroundOffDetect ground_off_detect;


    // 初始化动态重配置服务器
    dr_server = new dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig>;
    dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(&dynamicReconfigCallback, _1, _2);
    dr_server->setCallback(dr_callback);





    // 初始化时间戳（记录程序启动时刻，用于首次计算dt）
    last_time = ros::Time::now();

    // 订阅话题
    ros::Subscriber vel_sub = nh.subscribe("/joint_states", 10, jointStateCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 10, imuCallback);

    // 发布话题
    ros::Publisher leg_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/leg_position_controller/command", 10);
    ros::Publisher wheel_effort_pub = nh.advertise<std_msgs::Float64MultiArray>("/wheel_effort_controller/command", 10);


    ros::Subscriber control_sub = nh.subscribe("/wl_control", 10, controlCallback);

    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description)) {
        ROS_ERROR("Failed to get 'robot_description' parameter from parameter server.");
        return 1;
    }

    // 解析URDF字符串为urdf::Model对象
    urdf::Model model;
    if (!model.initString(robot_description)) {
        ROS_ERROR("Failed to parse URDF string.");
        return 1;
    }

    ROS_INFO("Successfully loaded URDF from parameter server.");
    // 后续可通过model对象访问URDF中的关节、连杆等信息

    physical_param.getLinkParamFromURDF(model);

    // 主循环（RATE_HZ）
    ros::Rate loop_rate(RATE_HZ);
    while (ros::ok()) {
        // 1. 计算时间间隔dt（当前时刻 - 上一时刻，单位：秒）
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;  // 更新上一时刻时间戳

        // 2. 积分计算路程：路程增量 = 线速度
        total_pos += robot_vel * dt;

        if (total_pos > 1) total_pos = 1;
        else if (total_pos < -1) total_pos = -1;


        // 发布腿部位置命令
        std_msgs::Float64MultiArray leg_pos_msg;
        leg_pos_msg.data.push_back(target_leg_pos);
        leg_pos_msg.data.push_back(target_leg_pos);
        leg_pos_pub.publish(leg_pos_msg);

        double sdp_out;

        if (pitch > -0.5 && pitch < 0.5)
        {
            double sdp_t = pos_pid.compute(0, total_pos, dt);
            sdp_out =  sdp_pid.compute(target_lin_vel, robot_vel, dt);
        }
        else
        {
            sdp_out = 0;
            pos_pid.reset();
            sdp_pid.reset();
        }
        
        x[0] = pitch;
        x[1] = ang_vel_y;           
        x[2] = -total_pos + sdp_out;           
        x[3] = -robot_vel + target_lin_vel;    

        physical_param.calcPhysicalParam();

        lqr_calc.setParam(
            physical_param.getBodyMass(),
            physical_param.getWheelMass(),
            physical_param.getBodyInertia(),
            physical_param.getWheelInertia(),
            physical_param.getBodyCenterHeight(),
            WHEEL_RADIUS,
            Eigen::Vector4d(1.3, 6, 65, 10.5),
            40);



        Eigen::Vector4d KK = lqr_calc.getLqrK();

        memcpy(K, KK.data(), 4 * sizeof(double));

        double wheel_torque = x[0] * K[0] + x[1] * K[1] + x[2] * K[2] + x[3] * K[3];// LQR控制律计算轮子力矩

        if (wheel_torque > 2.5) wheel_torque = 2.5;
        else if (wheel_torque < -2.5) wheel_torque = -2.5;

        double f = ground_off_detect.calc_F(l_leg_pos + deg_to_rad(29.51), l_leg_effort);
        ground_off_detect.paramsUpdate(physical_param.getBodyCenterHeight(), pitch, ang_vel_y, acc_z - 9.8, dt);
        
        double fn = ground_off_detect.calc_Fn();

        if (fn < 0.5 && pitch > -0.5 && pitch < 0.5) {
            wheel_torque = 0;
        }


        double yaw_out;

        if (pitch > -0.5 && pitch < 0.5)
        {
            double yaw_t_spd = yaw_pos_pid.compute(t_yaw, yaw, dt);
            yaw_out =  yaw_pid.compute(target_ang_vel, ang_vel_z, dt);
        }
        else
        {
            yaw_out = 0;
            yaw_pid.reset();
            yaw_pos_pid.reset();
        }
        

        // 发布轮子力矩命令
        std_msgs::Float64MultiArray wheel_effort_msg;
        wheel_effort_msg.data.push_back(-wheel_torque - yaw_out);
        wheel_effort_msg.data.push_back(-wheel_torque + yaw_out);
        wheel_effort_pub.publish(wheel_effort_msg);

        // 3. 打印：原有数据 + pos
        ROS_INFO(
            "机器人线速度: %-8.3f m/s | 左轮: %-8.3f rad/s | 右轮: %-8.3f rad/s\n"
            "IMU姿态 - Roll: %-8.3f | Pitch: %-8.3f | Yaw: %-8.3f (rad)\n"
            "IMU角速度 - X: %-8.3f | Y: %-8.3f | Z: %-8.3f (rad/s)\n"
            "总路程: %-8.3f m\n"
            "力矩: %-8.3f\n"
            "K: [%-8.4f, %-8.4f, %-8.4f, %-8.4f]\n"
            "M: %-8.4f kg | m: %-8.4f kg\n"
            "I: %-8.6f kg·m² | i: %-8.6f kg·m² | h: %-8.6f m\n"
            "pitch_offset: %-8.4f rad\n"
            "imu_pitch_offset: %-8.4f rad\n"
            "center_pitch_offset: %-8.4f rad\n"
            "leg_pos: %-8.4f m\n"
            "f: %-8.4f N | fn: %-8.4f N\n"
            "sdp_out: %-8.4f m/s\n"
            "--------------------------------------------------------",
            robot_vel, left_wheel_vel, right_wheel_vel,
            roll, pitch, yaw,
            ang_vel_x, ang_vel_y, ang_vel_z,
            total_pos,
            wheel_torque,
            KK(0), KK(1), KK(2), KK(3),
            physical_param.getBodyMass(), physical_param.getWheelMass(), 
            physical_param.getBodyInertia(), physical_param.getWheelInertia(), physical_param.getBodyCenterHeight(),
            physical_param.getPitchOffset(),
            physical_param.getImuPitchOffset(),
            physical_param.getCenterPitchOffset(),
            leg_pos,
            f, fn,
            sdp_out
        );

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
