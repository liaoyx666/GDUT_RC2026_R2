// RC_arm.h
#pragma once

#include <iostream>
#include <cmath>
#include "RC_motor.h"

#ifdef __cplusplus

namespace arm
{
    typedef enum{
        LAY,
        STAND
    } ArmMode;

    
    class Arm
    {
    public:
        Arm(motor::Motor motor[]);
        virtual ~Arm();
        
        void Arm_Init(motor::Motor motor[]);
        void Arm_Set_Position(float x, float y, float z, ArmMode mode);
        void Arm_Get_Angle();
        
    private:
        float* Arm_Calculate_angle(float x, float y, float z, ArmMode mode);
        void Set_Angle(float angle[]);
        
        float L1, L2, L3;
        float angle[4];
        float origin_angle[4];
        float target_angle[4];
        motor::Motor motor[4];
    };
}
#endif