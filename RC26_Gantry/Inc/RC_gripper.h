#pragma once
#include "RC_motor.h"
#include "RC_dji_motor.h"

#define open_current  -2000
#define close_current  2000

#ifdef __cplusplus
class Gripper
{
public:
     Gripper(motor::DjiMotor& m);

    void Open();
    void Close();

private:
    motor::DjiMotor* motor_;
   
};

#endif