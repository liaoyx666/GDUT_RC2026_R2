#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "wheel_leg/MPC_control.hpp"
#include "wheel_leg/physical_param_calc.hpp"
#include "wheel_leg/leg_pos_controlConfig.h"
#include <dynamic_reconfigure/server.h>
#include "wheel_leg/wl_control.h"

constexpr double WHEEL_RADIUS = 0.06;  // 轮子半径（m）
constexpr int RATE_HZ = 100;          // 循环频率（Hz）
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
double Pos_kp = 0.0;
double Pos_ki = 0.0;
double Pos_kd =0.0;

ros::Time last_time;
Eigen::Vector4d desire;
//常态Q权重矩阵
Eigen::Matrix4d getQMatrix() {
    Eigen::Matrix4d Q;
    Q << -50.0,     0.0 ,     0.0 ,     0.0,
         0.0,     1.0,      0.0 ,     0.0,
         0.0,     0.0 ,     0.0 ,     0.0,
         0.0,     0.0 ,     0.0 ,     41.0;
    return Q;
}

//最终预测Q矩阵
Eigen::Matrix4d getQTerminalMatrix() {
    Eigen::Matrix4d Q_terminal;
    Q_terminal << -50.0,     0.0 ,     0.0 ,     0.0,
                  0.0,     10.0 ,     0.0 ,     0.0,
                  0.0,     0.0 ,     0.0 ,     0.0,
                  0.0,     0.0 ,     0.0 ,     700.0;
    return Q_terminal;
}

Eigen::Matrix<double, 1, 1> getRTerminalMatrix(){
    Eigen::Matrix<double, 1, 1>  R;
    R<< 22;
    return R;
}

//dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig> *dr_server = nullptr;  // 添加dr_server的声明
std::unique_ptr<dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig>> dr_server;

void dynamicReconfigCallback(wheel_leg::leg_pos_controlConfig &config, uint32_t level) {
    leg_pos = config.leg_pos;
    //ROS_INFO("动态重配置: leg_pos = %.3f", leg_pos);
}

//回调函数定义
void controlCallback(const wheel_leg::wl_control::ConstPtr& msg) {
    desire(3) = desire(3)+msg->linear_vel*0.01;
    leg_pos = msg->leg_pos+leg_pos*0.01;
    if(target_yaw > 1){
        target_yaw = 1;
    }
    else if(target_yaw < -1){
        target_yaw = -1;
    }
    target_yaw = target_yaw + msg->angular_vel*0.001;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "MPC_controller_node");
    ros::NodeHandle nh;
    PhysicalParam physical_param;


    ros::Subscriber control_sub = nh.subscribe("/wl_control", 10, controlCallback);
    desire<< 0, 0, 0, 0;
    Eigen::Matrix<double, 1, 1> R = getRTerminalMatrix();
    Eigen::Matrix4d Q = getQMatrix();
    Eigen::Matrix4d Q_terminal = getQTerminalMatrix();
    State_eqCalc State_equation(
        10.113 - 0.173520730578001 * 2,    // M 机器人质量（kg）
        0.173520730578001 * 2,    // m 轮子质量（kg）
        0.870168809 - 0.000468745184860407 * 2,    // I 机器人转动惯量（kg·m²）
        0.000468745184860407 * 2,   // i 轮子转动惯量（kg·m²）
        0.300,    // h 机器人重心高度（m）
        WHEEL_RADIUS, // r 轮子半径（m）
        Q, // Q 状态权重矩阵对角线元素
        R,     // R 控制权重标量
        Q_terminal//Q最终预测权重矩阵
    );

    State_equation.A_B_calc();
    
    std::unique_ptr<Balance_Base> controller;
    controller = std::make_unique<MPC_control>(
        nh, yaw_kp,yaw_ki,yaw_kd,
        Pos_kp,Pos_ki,Pos_kd,
        State_equation.A, 
        State_equation.B, 
        State_equation.Q,  
        State_equation.R,    
        State_equation.Q_terminal,   
        1.0/RATE_HZ
    );

    //设置约束（扭矩-10～10）
    dynamic_cast<MPC_control*>(controller.get())->setInputConstraints(-10.0,10.0);
    // 设置期望状态
    Eigen::Vector4d desired_state;
    desired_state << 0.033, 0.0, 0.0, 0.0;  //期望状态 q qd x xd
    controller->set_desired_state(desired_state);

    // 初始化动态重配置服务器
    dr_server = std::make_unique<dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig>>();
    dynamic_reconfigure::Server<wheel_leg::leg_pos_controlConfig>::CallbackType dr_callback;
    dr_callback = boost::bind(&dynamicReconfigCallback, _1, _2);
    dr_server->setCallback(dr_callback);

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
    //后续可通过model对象访问URDF中的关节、连杆等信息

    physical_param.getLinkParamFromURDF(model);

    // 主循环（RATE_HZ）
    ros::Rate loop_rate(RATE_HZ);
    while (ros::ok()) {
        // ros::Time current_time = ros::Time::now();
        // double dt = (current_time - last_time).toSec();
        // last_time = current_time;  // 更新上一时刻时间戳
        // desire(3) = desire(3) + (0 - controller->get_current_state()(3))*dt;
        controller->main_control(leg_pos,target_yaw,desire);
        State_equation.setParam(
            physical_param.getBodyMass(),
            physical_param.getWheelMass(),
            physical_param.getBodyInertia(),
            physical_param.getWheelInertia(),
            physical_param.getBodyCenterHeight(),
            WHEEL_RADIUS,
            Q,
            R,
            Q_terminal
        );
        State_equation.A_B_calc();//更新AB矩阵
        //更新A，B
        dynamic_cast<MPC_control*>(controller.get())->updateSystemMatrices(State_equation.A, State_equation.B);
        //打印日志
        dynamic_cast<MPC_control*>(controller.get())->STATE_INFO();
        
        physical_param.calcPhysicalParam();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


