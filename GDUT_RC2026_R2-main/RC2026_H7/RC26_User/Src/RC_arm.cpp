// RC_arm.cpp
#include "RC_arm.h"

namespace arm
{
    Arm::Arm(motor::Motor motor[]): L1(0.45857), L2(0.41476), L3(0.13350)
    {
        for(int i = 0; i < 4; i++)
        {
            this->motor[i] = motor[i];
        }
        this->Arm_Init(motor);
    }

    Arm::~Arm()
    {
        // 析构函数实现
    }

    void Arm::Arm_Init(motor::Motor motor[])
    {
        // 检查电机个数等逻辑
        // 初始化实现 完成当前角度校准 记录origin_angle (赋为堵转时的角度)
        for(int i = 0; i < 4; i++)
        {
            this->origin_angle[i] = motor[i].Get_Angle();
        }
    }

    void Arm::Arm_Set_Position(float x, float y, float z, ArmMode mode)
    {
        float* angle = Arm_Calculate_angle(x, y, z, mode);

        for (int i = 0; i < 4; i++)
        {
            this->target_angle[i] = angle[i] + this->origin_angle[i];
            printf("angle[%d]: %f\n", i, angle[i]);// 测试
            motor[i].Set_Angle(this->target_angle[i]);
        }
    }
    
    float* Arm::Arm_Calculate_angle(float x, float y, float z, ArmMode mode)
    {    
        static float angle[4];

        if(mode == LAY)
        {   
            float Lxy = sqrt(x*x + y*y);
            float t_ = float(Lxy/(Lxy - L3));

            x *= t_;
            y *= t_;
        }
        else // mode == STAND 
        {
            z += L3;
        }

        if(x == 0) angle[0] = 90;
        else
        {
            angle[0] = atan(float(y/x));
        }

        if(z + L3 >= 0)
        {
            angle[1] = atan(float(z/sqrt(x*x + y*y))) +
                        acos(float((L1*L1 + (x*x + y*y + z*z) - L2*L2) / (2*L1*sqrt(x*x + y*y + z*z))));
        }
        else
        {
            angle[1] = acos(float((L1*L1 + (x*x + y*y + z*z) - L2*L2) / (2*L1*sqrt(x*x + y*y + z*z)))) - 
                        atan(float((-z)/sqrt(x*x + y*y)));
        }

        angle[2] = acos(float((L1*L1 - (x*x + y*y + z*z) + L2*L2) / (2*L1*L2)));

        angle[3] = angle[2] + angle[1] - 90;

        return angle;
    }

    void Arm::Arm_Get_Angle()
    {
        // 获取角度实现
    }

    void Arm::Set_Angle(float angle[])
    {
        for(int i = 0; i < 4; i++)
        {
            this->motor[i].Set_Angle(angle[i]);
        }
    }
}