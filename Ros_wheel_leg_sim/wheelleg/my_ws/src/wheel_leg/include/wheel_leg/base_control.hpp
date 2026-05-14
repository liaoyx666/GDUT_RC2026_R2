#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <math.h>
#include <memory>
#include "wheel_leg/my_pid.hpp"

class Balance_Base
{
public:
    Balance_Base(ros::NodeHandle &nh,double yaw_kp,double yaw_ki,double yaw_kd,
                                    double Pos_kp,double Pos_ki,double Pos_kd)
    {
        yaw_control.setGains(yaw_kp,yaw_ki,yaw_kd);
        Pos_control.setGains(Pos_kp,Pos_ki,Pos_kd);
        yaw_control.setOutputLimits(-1,1);
        Pos_control.setOutputLimits(-2,2);
        joint_state_sub = nh.subscribe("/joint_states", 10, &Balance_Base::jointStateCallback, this);
        imu_state_sub = nh.subscribe("/imu", 10,&Balance_Base::imuCallback,this);

        leg_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/leg_position_controller/command", 10);
        wheel_effort_pub = nh.advertise<std_msgs::Float64MultiArray>("/wheel_effort_controller/command", 10);
        last_balance_time=ros::Time::now();
        current_state << 0.0, 0.0, 0.0, 0.0;
        desired_state << 0.0, 0.0, 0.0, 0.0;
    }
    virtual ~Balance_Base() {}

    // IMU回调函数：获取姿态和角速度
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 获取角速度（x、y、z轴）
        ang_vel_x = msg->angular_velocity.x;
        ang_vel_y = msg->angular_velocity.y;
        ang_vel_z = msg->angular_velocity.z;

        // 四元数转欧拉角（姿态）
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
        pitch = asin(2.0 * (qw * qy - qz * qx));
        yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
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
        }
        // 计算机器人线速度（m/s）
        robot_vel = WHEEL_RADIUS * (left_wheel_vel + right_wheel_vel) / 2.0;
    }

    virtual void main_control(double Leg_L,double target_yaw,Eigen::Vector4d desire){
        // 计算dt
        ros::Time current_time = ros::Time::now();
        this->dt = (current_time - last_balance_time).toSec();
        // 路程积分
        total_pos += robot_vel * dt;
        // 更新状态
        State_updata(target_yaw);
        set_leg(Leg_L);
        //更新目标状态
        set_desired_state(desire);
    }
    
    void State_updata(double target_yaw){

        if(pitch-this->desired_state(0) < 0.01 && pitch-this->desired_state(0) > -0.01){
            current_state(0) = desired_state(0);//q
        }
        else{
            current_state(0) = pitch;
        }
        current_state(1) = ang_vel_y; //q_dot
        current_state(2) = total_pos;//x
        //current_state(2) = total_pos;//x
        // if(-robot_vel-this->desired_state(3) < 0.200 && -robot_vel-this->desired_state(0) > -0.200){
        //     current_state(3) = desired_state(3);//xd
        // }
        // else{
        //     current_state(3) = -robot_vel;
        // }
        current_state(3) = -robot_vel;
        yaw_err = target_yaw - yaw;
        Pos_err = target_Pos - total_pos;
    }
    void set_leg(double L){
        // 发布腿部位置命令
        double leg_pos =  L;
        leg_pos_msg.data.push_back(leg_pos);
        leg_pos_msg.data.push_back(leg_pos);
        leg_pos_pub.publish(leg_pos_msg);
    }
    void balance()
    {
        wheel_out = get_output();
        wheel_effort_msg.data.clear();
        wheel_effort_msg.data.push_back(wheel_out+yaw_control.compute(yaw_err,this->dt));
        wheel_effort_msg.data.push_back(wheel_out-yaw_control.compute(yaw_err,this->dt));
        wheel_effort_pub.publish(wheel_effort_msg);
        last_balance_time = ros::Time::now();//更新时间
    }
    // 获取当前状态
    Eigen::Vector4d get_current_state() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return current_state;
    }

    void set_desired_state(Eigen::Vector4d desire){
        if(pitch < -0.5 || pitch > 0.5){
            target_Pos = desire(3);
            desire(3) = Pos_control.compute(Pos_err,this->dt);
        }
        else {
            desire(3) = desire(3);
        }
        this->desired_state = desire;
    }
    virtual double get_output() = 0;

protected:
    MyPid yaw_control;
    MyPid Pos_control;
    double target_yaw = 0.0;
    double yaw_err = 0.0;
    double target_Pos = 0.0;
    double Pos_err =0.0;
    double dt;
    double wheel_out;
    Eigen::Vector4d current_state;//当前状态
    Eigen::Vector4d desired_state;//期望状态
    ros::Time last_balance_time;//上次平衡时间
    double roll = 0.0, pitch = 0.0, yaw = 0.0;  // IMU欧拉角（rad）
    double ang_vel_x = 0.0, ang_vel_y = 0.0, ang_vel_z = 0.0;  // IMU角速度（rad/s）
    double left_wheel_vel = 0.0;   // 左轮速度（rad/s）
    double right_wheel_vel = 0.0;  // 右轮速度（rad/s）
    double robot_vel = 0.0;        // 机器人线速度（m/s)
    double total_pos = 0.0;   // 总路程（m，积分结果）
    const double WHEEL_RADIUS = 0.06;  // 轮子半径（m)
    const std::string LEFT_WHEEL_JOINT = "Lwheel";  // 左轮关节名
    const std::string RIGHT_WHEEL_JOINT = "Rwheel"; // 右轮关节名
private:
    ros::Publisher leg_pos_pub;
    ros::Publisher wheel_effort_pub ;
    ros::Subscriber joint_state_sub;
    ros::Subscriber imu_state_sub;
    std_msgs::Float64MultiArray wheel_effort_msg;
    std_msgs::Float64MultiArray leg_pos_msg;
    std::mutex data_mutex;  // 数据互斥锁
};


class NMPC_Balance_Base
{
    public:
        NMPC_Balance_Base(ros::NodeHandle &nh,double yaw_kp,double yaw_ki,double yaw_kd,
                                        double Pos_kp,double Pos_ki,double Pos_kd)
        {
            yaw_control.setGains(yaw_kp,yaw_ki,yaw_kd);
            Pos_control.setGains(Pos_kp,Pos_ki,Pos_kd);
            yaw_control.setOutputLimits(-1,1);
            Pos_control.setOutputLimits(-2,2);

            joint_state_sub = nh.subscribe("/joint_states", 10, &NMPC_Balance_Base::jointStateCallback, this);
            imu_state_sub = nh.subscribe("/imu", 10,&NMPC_Balance_Base::imuCallback,this);
            // leg_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/leg_position_controller/command", 10);
            // wheel_effort_pub = nh.advertise<std_msgs::Float64MultiArray>("/wheel_effort_controller/command", 10);
            last_time=ros::Time::now();
            // current_state << 0.0, 0.0, 0.0, 0.0;
            // desired_state << 0.0, 0.0, 0.0, 0.0;
        }
        virtual ~NMPC_Balance_Base() {}

        // IMU回调函数：获取姿态和角速度
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
            // 获取角速度（x、y、z轴）
            ang_vel_x = msg->angular_velocity.x;
            ang_vel_y = msg->angular_velocity.y;
            ang_vel_z = msg->angular_velocity.z;

            // 四元数转欧拉角（姿态）
            double qx = msg->orientation.x;
            double qy = msg->orientation.y;
            double qz = msg->orientation.z;
            double qw = msg->orientation.w;
            roll = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            pitch = asin(2.0 * (qw * qy - qz * qx));
            yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        }
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
        {
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
            }
            // 计算机器人线速度（m/s）
            robot_vel = WHEEL_RADIUS * (left_wheel_vel + right_wheel_vel) / 2.0;
        }
        
        
        void State_update(){
            // 计算dt
            ros::Time current_time = ros::Time::now();
            this->dt = (current_time - last_time).toSec();
            this->last_time = current_time;
            // 路程积分
            total_pos += robot_vel * dt;
        }
        virtual double get_output() = 0;

    protected:
        double dt;
        ros::Time last_time;//上次平衡时间
        MyPid yaw_control;
        MyPid Pos_control;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;  // IMU欧拉角（rad）
        double ang_vel_x = 0.0, ang_vel_y = 0.0, ang_vel_z = 0.0;  // IMU角速度（rad/s）
        double left_wheel_vel = 0.0;   // 左轮速度（rad/s）
        double right_wheel_vel = 0.0;  // 右轮速度（rad/s）
        double robot_vel = 0.0;        // 机器人线速度（m/s)
        double total_pos = 0.0;   // 总路程（m，积分结果）
        const double WHEEL_RADIUS = 0.06;  // 轮子半径（m)
        const std::string LEFT_WHEEL_JOINT = "Lwheel";  // 左轮关节名
        const std::string RIGHT_WHEEL_JOINT = "Rwheel"; // 右轮关节名
    private:
        ros::Subscriber joint_state_sub;
        ros::Subscriber imu_state_sub;
};

