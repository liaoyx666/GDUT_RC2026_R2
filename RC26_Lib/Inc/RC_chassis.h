#pragma once


#ifdef __cplusplus

namespace RC_chassis {//name_space
	#define sin45   0.707106
	#define cos45   0.707106
	
    enum RoboChassisType {//底盘类型
        omni3_chassis,//三轮全向轮
        custom_chassis,
        omni4_chassis,//四轮全向轮
        mecanum_chassis,//麦克纳轮
        swerve4_chassis, // 四舵轮
        swerve3_chassis//三舵轮
    };

    enum RoboChassis_mode {//底盘运动模式
        stop,//停
        robotv,//自身参照系
        worldv,//世界参照系
        point_track,
        point_track_speedplan,
        curve_track,
        chassis_init,
        ppp_track,
        ppc_track,
        swerve_stable,
        Vel_compensation
    };

    enum chassis_yaw_mode {//底盘偏航角模式
        yaw_lock,//锁偏航角
        yaw_TurnTo,
        yaw_TurnTo_speedplan,
        yaw_free,
        correct_yaw
    };

    enum chassis_cmd_type {//底盘指令
        move_cmd,
        turn_cmd
    };

    typedef struct chassis_info {//底盘各个参数
        float wheel_r = 0.0f;    // 轮子半径
        float wide = 0.0f;       // 底盘宽度
        float length = 0.0f;     // 底盘长度
        float R = 0.0f;          // 轮子到底盘中心距离
        
        float L1 = 0.0f;         // 用于特殊底盘结构的参数
        float L2 = 0.0f;         // 用于特殊底盘结构的参数
        
        // 构造函数（c++ struct支持构造函数）
        chassis_info(float r = 0.0f, float w = 0.0f, float l = 0.0f, 
                    float center_r = 0.0f, float l1 = 0.0f, float l2 = 0.0f) :
            wheel_r(r), wide(w), length(l), R(center_r), L1(l1), L2(l2) {}
    };

    // 前向声明
    template <typename chassis_t>
    class chassis_base;
    // 速度结构体
    template<typename T>
    struct Velocity {//速度结构体，Vx,Vy,W
        T vx;
        T vy;
        T wz; // 角速度
    };

    template <typename chassis_t>
    class chassis_base {//底盘基类
    protected:
        Velocity<chassis_t> chassis_vel;//速度结构体类型
        chassis_info info;//底盘参数struct的实例化对象
        RoboChassisType type;//底盘类型的枚举变量
        RoboChassis_mode mode;//底盘模式的枚举变量
    public:
		//底盘构造函数，初始化底盘参数，底盘类型，底盘模式
        chassis_base(const chassis_info& config, RoboChassisType chassis_type) : 
            info(config), type(chassis_type), mode(chassis_init) {}
        virtual ~chassis_base() {}
        // 设置底盘速度
        virtual void setVelocity(chassis_t vx, chassis_t vy, chassis_t wz) {
            chassis_vel.vx = vx;
            chassis_vel.vy = vy;
            chassis_vel.wz = wz;
        }
        // 获取底盘速度
        Velocity<chassis_t> getVelocity() const {
            return chassis_vel;
        }
        // 设置控制模式
        void setMode(RoboChassis_mode new_mode) {
            mode = new_mode;
        }
        // 获取控制模式
        RoboChassis_mode getMode() const {
            return mode;
        }
        // 急停
        virtual void emergencyStop() {
            setVelocity(0, 0, 0);
            mode = stop;
        }
    };
    
    template <typename chassis_t>
    class omni3_Chassis : public chassis_base<chassis_t> {// 三轮全向底盘类型
    public:
        omni3_Chassis(const chassis_info& config) : chassis_base<chassis_t>(config, omni3_chassis) {
			
		}
        void omni3_chassis_calc(float* wheel_speeds[3], float vx, float vy, float wz);
    };

    template <typename chassis_t>
    class omni4_Chassis : public chassis_base<chassis_t>{//四全向轮底盘类型
	public:
		omni4_Chassis(const chassis_info& config): chassis_base<chassis_t>(config, omni4_chassis) {}
        //底盘计算函数
        void omni4_chassis_calc(float wheel_speeds[4], float vx, float vy, float wz);
    };


}//namespace

#endif

