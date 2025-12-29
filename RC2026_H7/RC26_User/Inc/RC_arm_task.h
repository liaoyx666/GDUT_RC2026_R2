#pragma once
#include "RC_arm_path.h"
#include "RC_arm.h"
#include "RC_tim.h"
#include "RC_motor.h"
#include "RC_j60.h"
#include "RC_go.h"
#include "RC_dji_motor.h"
#include "RC_can.h"
#include "RC_m2006.h"
#include "RC_m3508.h"
#include "RC_task.h"
#include "RC_arm_path.h"
#include "main.h"
#include "task.h"
#ifdef __cplusplus
using namespace path;
/*================= 内部执行状态 =================*/
enum ArmState
{
    ARM_STATE_IDLE,
    ARM_STATE_MOVING,
    ARM_STATE_WAITING,
    ARM_STATE_FINISHED
};

/*================= 动作类型 =================*/
enum ArmActionType
{
    ARM_ACTION_MOVE,
    ARM_ACTION_HOLD,
    ARM_ACTION_IO,
    ARM_ACTION_COND,
    ARM_ACTION_END
};

/*================= 动作结构体 =================*/
struct ArmAction
{
    ArmActionType type;
    Pose3D start;
    Pose3D target;
    float smooth;
    float speed;
    float acc;
    uint32_t hold_ms;
    void (*io_func)();
    bool (*cond_func)();
};

/*================= 外部任务 =================*/
enum class ARM_TASK : uint8_t
{
    IDLE = 0,

    PICK_FRONT_UP_CUBE,
		PICK_FRONT_DOWN_CUBE,

    PLACE_LEFT_CUBE,
    PLACE_RIGHT_CUBE,

    PICK_FRONT_WEAPON,
    PLACE_FRONT_WEAPON,

    HOME
};



   
class Arm_task 
    : public tim::TimHandler
    , public task::ManagedTask
{
public:
    Arm_task(tim::Tim &tim_);
    virtual ~Arm_task() {}

    bool Arm_Control(ARM_TASK task);
    bool Arm_IsBusy(void);
			
private:
		ArmState Arm_GetState(void);
		static void hold(uint32_t duration_ms);

protected:
    void Tim_It_Process() override;
    void Task_Process() override;  
};



#endif
