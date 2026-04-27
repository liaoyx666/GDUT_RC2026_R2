#pragma once
#include "RC_motor.h"
#include "RC_vector2d.h"
#include "RC_chassis.h"
#include <math.h>
#include "RC_serial.h"

/*
 轮子顺序：
        m1  m4
          \/
          /\
        m2  m3
*/

// 全向轮底盘参数
#define OMNI4_CHASSIS_L            0.36355f // 中心到轮子距离
#define OMNI4_CHASSIS_WHEEL_RADIUS 0.0645f  // 轮子半径

#define COS_45 0.70710678118654752440084436210485f
#define SIN_45 COS_45

#ifdef __cplusplus

namespace chassis
{
    class Omni4Chassis : public Chassis
    {
    public:
        Omni4Chassis(
            motor::Motor& m1, motor::Motor& m2,
            motor::Motor& m3, motor::Motor& m4,
            float max_lv, float la, float ld,
            float max_av, float aa, float ad,
			data::RobotPose& pose_
        );

    void Chassis_Re_Init() override;

    private:
        void Kinematics_calc(vector2d::Vector2D v_, float vw_) override;
        void Chassis_Init() override;

        motor::Motor* drive_motor[4];
        const float vel_to_rpm = (1.0f / OMNI4_CHASSIS_WHEEL_RADIUS) * (60.0f / (2.0f * PI));
    };
}
#endif