#include "RC_gripper.h" // 请确保与你的实际头文件名一致

namespace gantry
{
    Gripper::Gripper(motor::DjiMotor& m_p_)
        : motor_p(m_p_)
    {
        // 默认初始化为打开状态
        Open();
    }


}