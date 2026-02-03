#pragma once

#include "RC_pid.h"
#include "arm_matrix.h"
#include "RC_timer.h"
#include "RC_tim.h"
#include "RC_motor.h"
#include "RC_task.h"
#include "main.h"
#include "task.h"
#include <math.h>
#include "RC_vector3d.h"
#include "RC_storage.h"

#define ARM_JOINTS 4

#define L1_Particle_LENGTH       0.22821f
#define L2_Particle_LENGTH       0.20264f
#define L3_Particle_LENGTH       0.07760f
#define GRAVITY_ACEEL            9.788f
#define L1_gravity               1.027f * GRAVITY_ACEEL
#define L2_gravity               0.416f * GRAVITY_ACEEL
#define L3_gravity               0.2f * GRAVITY_ACEEL

#define L1_LENGTH       0.32358f
#define L2_LENGTH       0.07111f
#define L3_LENGTH       0.24800f
#define L4_LENGTH       0.13994f
#define BASE_HEIGHT     0.1f

#define THETA1_MIN      -PI
#define THETA1_MAX       PI
#define THETA2_MIN       (-1.1*PI)
#define THETA2_MAX       (0.1*PI)
#define THETA3_MIN       0
#define THETA3_MAX       (1.5f * PI)
#define THETA4_MIN       (-1.8f * PI)
#define THETA4_MAX       0

#define THETA1_OFFSET    0.0f
#define THETA2_OFFSET    (-PI -(10.00f * PI / 180.0f))
#define THETA3_OFFSET    (PI - (-63.25f * PI / 180.0f))
#define THETA4_OFFSET    (PI + (116.75f * PI / 180.0f))
#define THETA5_OFFSET    (PI -  (65.00f * PI / 180.0f))

#ifdef __cplusplus
namespace arm
{
    typedef struct
    {
        float theta1;
        float theta2;
        float theta3;
        float theta4;
    } JointAngles;

    typedef struct
    {
        float joint1;
        float joint2;
        float joint3;
    } Joint_gravity_compensation;

    class ArmDynamics
    {
    public:
        ArmDynamics();
        virtual ~ArmDynamics() {}

        JointAngles motor_angle{0, 0, 0, 0};
        JointAngles joint_angle_now{0, 0, 0, 0};

        Joint_gravity_compensation joint_gravity_compensation{0, 0, 0};

        void gravity_compensation();
    };

    struct EndEffectorPos {
        float x;
        float y;
        float z;
        float angle;
    };

    class ArmKinematics
    {
    public:
        void forward(const JointAngles& angles, EndEffectorPos& end_pos);
        bool inverse(const EndEffectorPos& target_pos, JointAngles& result_angles);

    private:
        float normalizeAngle(float angle);
        float constrainValue(float value, float min, float max);
        float unwrapAngle(float now, float last);
        ArmMatrix<4, 4> buildDHTable(float theta, float alpha, float a, float d, float offset);

        static ArmMatrix<4,4> T01;
        static ArmMatrix<4,4> T12;
        static ArmMatrix<4,4> T23;
        static ArmMatrix<4,4> T34;
        static ArmMatrix<4,4> T45;
        static ArmMatrix<4,4> T05;

        JointAngles last_joint;
    };
}


namespace path
{
using Vector3D = vector3d::Vector3D;
struct Pose3D
{
    Vector3D pos;
    float pitch;
};

    #define POS(x,y,z,p) Pose3D{Vector3D(x,y,z),(p)}

		struct JointArray
		{
				float j[ARM_JOINTS];
		};

    class TimePlanner
    {
    public:
        struct Profile
        {
            float L;
            float v_max;
            float a_max;
            float t_acc;
            float t_cruise;
            float t_total;
        };

        static Profile compute(float L, float v_max, float a_max);
        static float timeToS(const Profile& pf, float t);
    };
}

enum ArmState
{
    ARM_STATE_IDLE,
    ARM_STATE_MOVING,
    ARM_STATE_WAITING,
    ARM_STATE_FINISHED
};

enum ArmActionType
{
    ARM_ACTION_MOVE,
    ARM_ACTION_HOLD,
    ARM_ACTION_IO,
    ARM_ACTION_COND,
    ARM_ACTION_END
};

struct ArmAction
{
    ArmActionType type;
		path::Pose3D start;  //起始点，可不填
    path::Pose3D target;  //终点 
    float smooth;  //平滑度，0是直线，1，-1是圆，极值是±1
    float speed;  //速度
    float acc;  //加速度
    bool  joint_space; // true: 使用关节空间规划；false: 笛卡尔空间规划
    uint32_t hold_ms;  //停止时间
    void (*io_func)();  //执行函数
    bool (*cond_func)();  //判断函数
};


enum class ARM_TASK : uint8_t
{
    IDLE = 0,
    PICK_FRONT_UP_CUBE,
    PICK_FRONT_DOWN_CUBE,
    PLACE_LEFT_CUBE,
    PLACE_RIGHT_CUBE,
    HOME
};


class Arm_task : public tim::TimHandler, public task::ManagedTask
{
public:
    Arm_task(tim::Tim &tim_,
             motor::Motor& motor_1_,
             motor::Motor& motor_2_,
             motor::Motor& motor_3_,
             motor::Motor& motor_4_);
    virtual ~Arm_task() {}
    bool Arm_Control(ARM_TASK task);
    bool Arm_IsBusy(void);

private:
		bool is_finished;
		ARM_TASK active_task_ = ARM_TASK::IDLE;
    bool task_started_ = false;
		 void startTask(ARM_TASK task);
		 bool isTaskFinished() const;
		static void Front_Air_Pump_On();
		static void Front_Air_Pump_Off();
    motor::Motor* Arm_motor[4];
    ArmState g_state;
    ARM_TASK g_task_mode;
    ARM_TASK g_last_task_mode;
    bool g_task_lock;
    uint32_t g_hold_end_tick;
    ArmAction* g_action_seq;
    uint32_t g_action_index;
    static path::Pose3D home_pose;
    static path::Pose3D pick_front_cube0;
    static path::Pose3D pick_front_cube2;
    static path::Pose3D pick_front_cube5;
    static path::Pose3D pick_front_cube6;
    static path::Pose3D pick_front_cube7;
    static path::Pose3D pick_front_cube_test1;
    static path::Pose3D pick_front_cube_test2;
		static path::Pose3D pick_front_cube_test3;
		static path::Pose3D pick_front_cube8;
    static path::Pose3D safe_pos;
		static path::Pose3D pick_front_cube_test0;
    static ArmAction PICK_FRONT_UP_CUBE_SEQ[];
    static ArmAction PICK_FRONT_DOWN_CUBE_SEQ[];
    static ArmAction PLACE_LEFT_CUBE_SEQ[];
    static ArmAction PLACE_RIGHT_CUBE_SEQ[];
    static ArmAction HOME_SEQ[];
    path::Pose3D g_last_pose;
    path::JointArray cmd_joints_;
    float move_s_;
    path::TimePlanner::Profile global_profile_;
    bool is_reseted;
    uint32_t last_tick_;
    path::Pose3D current_target_pose_;
    path::JointArray current_joint_angles;
    path::JointArray current_target_joints_;
    bool current_is_joint_space_;
    float current_speed_;
    float current_acc_;
    float move_start_time_;
    path::JointArray start_joints_;
    path::Pose3D start_pose_;
    bool isAllReached();
    ArmState Arm_GetState(void);
    void Arm_Hold(uint32_t ms);
    void Bind_Action_Sequence();
    void Arm_Path_Manager(void *arg);
    void executeMove();
		
		 
    
protected:
    void Tim_It_Process() override;
    void Task_Process() override;
};

#endif
