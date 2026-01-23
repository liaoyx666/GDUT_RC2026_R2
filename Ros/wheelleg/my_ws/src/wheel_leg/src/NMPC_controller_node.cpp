#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "wheel_leg/NMPC_control.hpp"
#include "wheel_leg/physical_param_calc.hpp"
#include "wheel_leg/leg_pos_controlConfig.h"
#include <dynamic_reconfigure/server.h>
#include "wheel_leg/wl_control.h"

constexpr double WHEEL_RADIUS = 0.06;  // 轮子半径（m）
constexpr int RATE_HZ = 50;          // 循环频率（Hz）
constexpr double LEG_LENGTH = 0.3;    // 机械腿长度（m）

const double M = 10.113 - 0.173520730578001 * 2;    // M 机器人质量（kg）
const double m = 0.173520730578001 * 2;    // m 轮子质量（kg）
const double I =0.870168809 - 0.000468745184860407 * 2;    // I 机器人转动惯量（kg·m²）
const double i = 0.000468745184860407 * 2;   // i 轮子转动惯量（kg·m²）
double wheel_R = 0.06; // r 轮子半径（m）
double g = 9.8;
double leg_pos = 0.3;  // 添加leg_pos的声明s

double yaw_kp = -1;
double yaw_ki = -0.05;
double yaw_kd = -2;
double target_yaw = 0;
double Pos_kp = -50.0;
double Pos_ki = -2.5;
double Pos_kd =-100.0;

double N = 15;
double u_max = 3;
double u_min = -3;

double Q_theta = 500; 
double Q_omega = -10;
double Q_x = -1;
double Q_v = 1;
double R_u = 0.1;

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "NMPC_controller_node");
    ros::NodeHandle nh;
    // 发布话题
    ros::Publisher leg_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/leg_position_controller/command", 10);
    ros::Publisher wheel_effort_pub = nh.advertise<std_msgs::Float64MultiArray>("/wheel_effort_controller/command", 10);

    NMPC_control controller(
        nh,
        yaw_kp,yaw_ki,yaw_kd,
        Pos_kp,Pos_ki,Pos_kd,
        10.113 - 0.173520730578001 * 2,    // M 机器人质量（kg）
        0.173520730578001 * 2,    // m 轮子质量（kg）
        0.870168809 - 0.000468745184860407 * 2,    // I 机器人转动惯量（kg·m²）
        0.000468745184860407 * 2,   // i 轮子转动惯量（kg·m²）
        0.300,    // h 机器人重心高度（m）
        N,
        u_max,u_min,
        Q_theta,Q_omega,Q_x,Q_v,R_u,  //权重
        1.0/RATE_HZ
    );
    double wheel_out;
    // 主循环（RATE_HZ）
    ros::Rate loop_rate(RATE_HZ);
    while (ros::ok()) {
        controller.update_current_state();
        wheel_out = controller.get_output();
        std::cout << "第一个控制输入: " << wheel_out << std::endl;
        //ROS_INFO("out:%.f",wheel_out);
        // 发布轮子力矩命令
        std_msgs::Float64MultiArray wheel_effort_msg;
        wheel_effort_msg.data.push_back(wheel_out);
        wheel_effort_msg.data.push_back(wheel_out);
        wheel_effort_pub.publish(wheel_effort_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

