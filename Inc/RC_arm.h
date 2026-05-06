#pragma once

#include "RC_motor.h"
#include "RC_dji_motor.h"
#include "RC_tim.h"
#include "RC_task.h"
#include "RC_timer.h"
#include <math.h>
#include "gpio.h"
#define ARM_AXES 4
#ifdef __cplusplus
// ================= 任务定义 =================

enum class ARM_TASK : uint8_t
{
    PICK_UP_KFS,
    PICK_DOWN_KFS,
    PUT_KFS,
    PICK_WEAPON,
    CONNECT_WEAPON,
    REALSE_WEAPON,
    HOME,
    IDLE
};

// =================== 末端选择 ========================
enum EndType
{
    END_SUCTION,
    END_GRIPPER
};



// ================= 梯形规划 =================

class TrapezoidalPlanner
{
public:
    struct Profile
    {
        float L, v_max, a_max;
        float Ta, Tv, Td;
        float t_total;
    };

    struct State
    {
        float s;
    };

    static Profile compute(float L, float v_max, float a_max);
    static State get_state(const Profile& pf, float t);
};

// ================= Axis =================

struct AxisPlanner
{
    float start;
    float target;
    bool finished;
};

// ================= Motion =================

class ArmMotion
{
public:
    ArmMotion(motor::DjiMotor& mx,
              motor::DjiMotor& my,
              motor::DjiMotor& mz,
              motor::JointM&   ma);

    void MoveTo(float x,float y,float z,float a,
                float speed,float acc);
		
    void Update(float dt);
    bool IsBusy() const { return busy_; }
		void SetEndType(EndType type);
		
private:
		EndType end_type_;
    motor::DjiMotor* motor_[3];
		motor::JointM*   dm_motor[1];
    AxisPlanner axis_[4];
		float cmd_[4] = {0};   
    float cur_[4] = {0};
		float target_[4] = {0};   
		TrapezoidalPlanner::Profile pf_;
		float error_[4]  = {0};   
    float move_time_ = 0;
    float tgt_[4] = {0};
    bool busy_ = false;
		float length[3] = {0};
};

// ================= Gripper =================

class Gripper
{
public:
     Gripper(motor::DjiMotor& m);

    void Open();
    void Close();

private:
    motor::DjiMotor* motor_;
   
};

// ================= Suction =================

class Suction
{
public:
		Suction(GPIO_TypeDef* port, uint16_t pin);
    void On();  
    void Off(); 

private:
		GPIO_TypeDef* port_;
		int16_t pin_;
    bool state_ = false;
};

// ================= Action =================

enum ArmActionType
{
    ACTION_MOVE,
    ACTION_GRIPPER,
    ACTION_SUCTION,
    ACTION_WAIT,
    ACTION_END,
	  ACTION_SWITCH_TO_GRIPPER, 
    ACTION_SWITCH_TO_SUCTION  
};

struct ArmAction
{
    ArmActionType type;

    float x,y,z,a;
    float speed,acc; 

    float wait_ms;
    uint8_t cmd;
};

// ================= System =================

class ArmSystem :
    public tim::TimHandler,
    public task::ManagedTask
{
public:
    ArmSystem(tim::Tim* tim,
              ArmMotion* motion,
              Gripper* gripper,
              Suction* suction);

    void SetTask(ARM_TASK task);


protected:
    void Tim_It_Process() override;
    void Task_Process() override;


private:
    ArmMotion* motion_;
    Gripper*   gripper_;
    Suction*   suction_;

    ArmAction* seq_ = nullptr;
    uint32_t index_ = 0;
    uint32_t wait_start_ = 0;

    ARM_TASK current_task_ = ARM_TASK::IDLE;
};



#endif
