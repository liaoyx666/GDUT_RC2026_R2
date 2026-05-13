#include "RC_gripper.h"
Gripper::Gripper(motor::DjiMotor& m)
{
    motor_ = &m;
}

void Gripper::Open()
{
    motor_->Set_Current(open_current);
}

void Gripper::Close()
{
    motor_->Set_Current(close_current);
}


