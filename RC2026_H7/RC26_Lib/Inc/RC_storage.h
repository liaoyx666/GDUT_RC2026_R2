#pragma once
#include "RC_task.h"
#include "RC_motor.h"


#ifdef __cplusplus

/*================ 机构状态 =================*/
enum state
{
    home = 0,
    storage,
    place,
    mid,
    high,
    STATE_NUM     
};

/*================ 高层任务 =================*/
enum class STORAGE_TASK : uint8_t
{
    HOME = 0,

    LEFT_STORAGE,
    RIGHT_STORAGE,

    LIFT_TO_MID,
    LIFT_TO_HIGH,

		LEFT_PLACE,
		RIGHT_PLACE
	
};

/*================ Storage 类 =================*/
class Storage : public task::ManagedTask
{
public:
    Storage(motor::Motor& motor_1_,
            motor::Motor& motor_2_,
            motor::Motor& motor_3_);
    virtual ~Storage() {}  
    void Storage_task(STORAGE_TASK task);
    void update();
    bool isBusy() const { return busy; }

private:
    /* 电机指针
     * [0] 升降
     * [1] 左
     * [2] 右
     */
    motor::Motor* Storage_motor[3];
    /*================ 当前状态 =================*/
    state Left_Storage;
    state Right_Storage;
    state Lift;
    /*================ 任务 & 状态机 =================*/
    STORAGE_TASK task_mode;
    uint8_t step;
    bool busy;
    /*================ 到位判断 =================*/
    bool isLiftReached();
    bool isLeftReached();
    bool isRightReached();
    bool isAllReached();
		static void Left_Air_Pump_On();
    static void Left_Air_Pump_Off();
		static void Right_Air_Pump_On();
    static void Right_Air_Pump_Off();
protected:
		void Task_Process() override;  
};

//---------------------GRIPPER类-------------------------------------------------------------------//
enum class GRIPPER_TASK :uint8_t
	{
			HOME_,
			GRASP_READY,
			CONNECT,
			GRASP_OPEN,
			GRASP_CLOSE
	};
	enum state_
	{
			HOME_,
			OPEN_,
			CLOSE_,
			STATE_NUM_
	};
	
	
class Gripper  : public task::ManagedTask
{
	public:
			virtual ~Gripper(){}
			Gripper(motor::Motor& motor_1_, motor::Motor& motor_2_, motor::Motor& motor_3_);
			bool isBusy() const { return busy; }
			void Gripper_task(GRIPPER_TASK task);
	private:
		bool busy;
	  state_ rotate_state; 
    state_ flip_state;    
    state_ claw_state;    
		uint8_t step;
		GRIPPER_TASK gripper_task;
		motor::Motor* gripper_motor[3];
		bool isAllReached();
	protected:	
		void Task_Process() override;
};



#endif
