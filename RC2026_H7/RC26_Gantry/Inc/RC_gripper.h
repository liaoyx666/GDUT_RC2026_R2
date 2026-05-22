#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace gantry
{
    class Gripper
    {
    public:
        Gripper(motor::DjiMotor& m_p_);
        ~Gripper() = default;
        
        // 核心接口
        constexpr void Open(){
            motor_p.Set_Current(OPEN_CURRENT);
        }
        constexpr void Close(){
            motor_p.Set_Current(CLOSE_CURRENT);
        }
        constexpr bool IsPickSuccess(){
            float current_pos = motor_p.Get_Out_Pos();
            return (current_pos >= PICK_MIN_POS) && (current_pos <= PICK_MAX_POS);
        }

    private:
        motor::DjiMotor& motor_p;

        // ==================== 电流与位置常量 ====================
        float OPEN_CURRENT = -1000.0f;  // 替代原有的 -test_val5
        float CLOSE_CURRENT = 1000.0f;  // 替代原有的 test_val5
        
        // 根据原有注释 (开:-1.3713, 合:0.6222, 有武器头:1.5134) 预设的有效范围
        float PICK_MIN_POS = 1.4f;      // 最小有效位置 (需根据实际情况调整)
        float PICK_MAX_POS = 1.6f;      // 最大有效位置 (需根据实际情况调整)
    };
}
#endif