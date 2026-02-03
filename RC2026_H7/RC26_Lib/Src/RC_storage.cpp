#include "RC_storage.h"
/*================ pos table ================*/
static const float leftPosTable[STATE_NUM] =
{
    /* home    */  0.0f,
    /* storage */  -0.6f,
    /* place   */  3.14f,
    /* mid     */  0.0f,
    /* high    */  0.0f
};

static const float rightPosTable[STATE_NUM] =
{
    /* home    */  0.0f,
    /* storage */  0.6f,
    /* place   */  -3.14f,
    /* mid     */  0.0f,
    /* high    */  0.0f
};

static const float midliftPosTable[STATE_NUM] =
{
    /* home    */  0.0f,
    /* storage */  0.0f,
    /* place   */  0.0f,
    /* mid     */  0.0f,
    /* high    */ -30.5f
};
//IO函数
void Storage::Left_Air_Pump_On()			{HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);}
void Storage::Left_Air_Pump_Off()			{HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET);}
void Storage::Right_Air_Pump_On()			{HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);}
void Storage::Right_Air_Pump_Off()		{HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET);}
/*================ 构造函数 =================*/
Storage::Storage(motor::Motor& motor_1_,motor::Motor& motor_2_,motor::Motor& motor_3_):ManagedTask("storage_task", 10, 128, task::TASK_DELAY, 5)
{
    Storage_motor[0] = &motor_1_;   // 升降
    Storage_motor[1] = &motor_2_;   // 左
    Storage_motor[2] = &motor_3_;   // 右
    Left_Storage  = home;
    Right_Storage = home;
    Lift          = home;
    task_mode = STORAGE_TASK::HOME;
    step = 0;
    busy = false;
}

/*================ 下发任务（入口） =================*/
void Storage::Storage_task(STORAGE_TASK task)
{
		if (busy && task == task_mode)	return;     
    task_mode = task;
    step = 0;
    busy = true;
}

/*================ 到位判断 =================*/
bool Storage::isLiftReached()		{return (fabs(Storage_motor[0]->Get_Out_Pos() - midliftPosTable[Lift]) < 0.1);		}
bool Storage::isLeftReached()		{return (fabs(Storage_motor[1]->Get_Out_Pos() - leftPosTable[Left_Storage]) < 0.1);	}
bool Storage::isRightReached()	{return (fabs(Storage_motor[2]->Get_Out_Pos() - rightPosTable[Right_Storage]) < 0.1);	}
bool Storage::isAllReached()		{return isLiftReached() && isLeftReached() && isRightReached();}
/*================ 状态机 =================*/
void Storage::Task_Process() 
{
    if (!busy) return;
    switch (task_mode)
    {
    /*================ 回零 =================*/
    case STORAGE_TASK::HOME:
        switch (step)
        {
        case 0:
            Left_Storage  = home;
            Right_Storage = home;
            Lift          = home;
            step++;
            break;
        case 1:
            if (isAllReached())
            {
                busy = false;
            }
            break;
        }
        break;
    /*================ 左储存 =================*/
    case STORAGE_TASK::LEFT_STORAGE:
        switch (step)
        {
        case 0:
            Lift = home;
						Left_Storage = storage;
            step++;
            break;
        case 1:
            if (isLeftReached() && isLiftReached())
            {
								Left_Air_Pump_On();
                busy = false;
            }
            break;
        }
        break;
    /*================ 右储存 =================*/
    case STORAGE_TASK::RIGHT_STORAGE:
        switch (step)
        {
        case 0:
            Lift = mid;
						Right_Storage = storage;
            step++;
            break;
        case 1:
            if (isRightReached() && isLiftReached())
            {
                Right_Air_Pump_On();
                busy = false;
            }
            break;
        }
        break;
				    /*================ 升高 （HIGH）=================*/
    case STORAGE_TASK::LIFT_TO_HIGH:
        switch (step)
        {
        case 0:
            Lift = high;
            step++;
            break;

        case 1:
            if (isRightReached() && isLiftReached())
            {
                busy = false;
            }
            break;
        }
        break;
						/*================ 升高（MID） =================*/
				case STORAGE_TASK::LIFT_TO_MID:
        switch (step)
        {
        case 0:
            Lift = mid;
            step++;
            break;
        case 1:
            if (isLeftReached() && isLiftReached() )
            {
                busy = false;
            }
            break;
        }
        break;
    /*================ 左放 =================*/
    case STORAGE_TASK::LEFT_PLACE:
        switch (step)
        {
        case 0:
            Lift = high;
						Left_Storage = place;
            step++;
            break;
        case 1:
            if (isLeftReached() && isLiftReached() )
            {
                busy = false;
            }
            break;
        }
        break;
    /*================ 右放 =================*/
    case STORAGE_TASK::RIGHT_PLACE:
        switch (step)
        {
        case 0:
            Lift = high;
						Right_Storage = place;
            step++;
            break;
        case 1:
            if (isRightReached() && isLiftReached())
            {
                busy = false;
            }
            break;
        }
        break;
    default:
        busy = false;
        break;
    }

    /*================ 电机统一写入 =================*/
    Storage_motor[0]->Set_Out_Pos( midliftPosTable[Lift] );
    Storage_motor[1]->Set_Out_Pos( leftPosTable[Left_Storage] );
    Storage_motor[2]->Set_Out_Pos( rightPosTable[Right_Storage] );
}




//------------------------------------------------------------------------------------------------------------------------------------------




//gripper//

/*================ pos table =================*/
static const float rotatePosTable[STATE_NUM_] =
{
    /* HOME  */ 0.0f,
    /* OPEN  */ 1.57f,
		/* CLOSE*/						0.0f
};

static const float flipPosTable[STATE_NUM_] =
{
    /* HOME  */ 0.0f,
    /* OPEN  */ 1.58f,
		/* CLOSE*/					0.0f
};

static const float clawPosTable[STATE_NUM_] =
{
    /* HOME  */ 0.0f,
    /* OPEN  */ 0.0f
		/* CLOSE*/			-32.0f
};

/*================ 构造函数 =================*/
Gripper::Gripper(motor::Motor& motor_1_, motor::Motor& motor_2_, motor::Motor& motor_3_)
: ManagedTask("gripper_task", 23, 128, task::TASK_DELAY, 10)
{
    gripper_motor[0] = &motor_1_; // 旋转
    gripper_motor[1] = &motor_2_; // 翻转
    gripper_motor[2] = &motor_3_; // 夹取

    rotate_state = HOME_;
    flip_state   = HOME_;
    claw_state   = HOME_;

    gripper_task = GRIPPER_TASK::HOME_;
    step = 0;
    busy = false;
   
}

/*================ 到位判断 =================*/
bool Gripper::isAllReached()
{
    bool r = (fabs(gripper_motor[0]->Get_Out_Pos() - rotatePosTable[rotate_state]) < 0.1);
    bool f = (fabs(gripper_motor[1]->Get_Out_Pos() - flipPosTable[flip_state]) < 0.1);
    bool c = (fabs(gripper_motor[2]->Get_Out_Pos() - clawPosTable[claw_state]) < 0.1);

    return r && f && c;
}

/*================ 下发任务 =================*/
void Gripper::Gripper_task(GRIPPER_TASK task)
{
    if (busy && task == gripper_task) return;

    gripper_task = task;
    step = 0;
    busy = true;
}

/*================ 状态机 =================*/
void Gripper::Task_Process()
{
    if (!busy) return;

    switch (gripper_task)
    {
    /*================ HOME =================*/
    case GRIPPER_TASK::HOME_:
        switch (step)
        {
        case 0:
            rotate_state = HOME_;
            flip_state   = HOME_;
            claw_state   = HOME_;
            step++;
            break;

        case 1:
           // if (isAllReached())
          //  {
                busy = false;
           // }
            break;
        }
        break;

    /*================ GRASP =================*/
    case GRIPPER_TASK::GRASP_READY:
        switch (step)
        {
        case 0:
            rotate_state = OPEN_;   
            flip_state   = HOME_;  
            step++;
            break;

        case 1:
           // if (isAllReached())
            //{
                busy = false;
           // }
            break;
        }
        break;

    /*================ CONNECT =================*/
    case GRIPPER_TASK::CONNECT:
        switch (step)
        {
        case 0:
            rotate_state = OPEN_;   // 旋转到连接位置
            flip_state   = OPEN_;
            step++;
            break;

        case 1:
          //  if (isAllReached())
          //  {
                busy = false;
          //  }
            break;
        }
        break;

    /*================ OPEN =================*/
    case GRIPPER_TASK::GRASP_OPEN:
        switch (step)
        {
        case 0:
            claw_state = OPEN_;
            step++;
            break;

        case 1:
           // if (isAllReached())
          //  {
                busy = false;
           // }
            break;
        }
        break;

    /*================ CLOSE =================*/
    case GRIPPER_TASK::GRASP_CLOSE:
        switch (step)
        {
        case 0:
            claw_state = CLOSE_;
            step++;
            break;

        case 1:
           // if (isAllReached())
           // {
                busy = false;
           // }
            break;
        }
        break;

    default:
        busy = false;
        break;
    }

    /*================ 电机统一写入 =================*/
    gripper_motor[0]->Set_Out_Pos(rotatePosTable[rotate_state]);
    gripper_motor[1]->Set_Out_Pos(flipPosTable[flip_state]);
    gripper_motor[2]->Set_Out_Pos(clawPosTable[claw_state]);
}

