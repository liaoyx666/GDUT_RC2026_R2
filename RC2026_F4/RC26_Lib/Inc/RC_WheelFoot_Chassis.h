#pragma once


#ifdef __cplusplus
	
//LQR 状态反馈矩阵，足部输出
//VMC 关节解算足端输出通过虚拟阻尼，映射给关节电机，关节电机输出
namespace RC_WheelFoot_Chassis{
	typedef struct {//状态矩阵
		float X_Pose;//位移
		float X_speed;//速度
		float Pitch;//绕轮毂轴心的俯仰角
		float Gyro;//绕轮毂轴心的角加速度
	}State_data;
	
	typedef struct {//状态反馈控制器
		float Left_accel;//左轮加速度
		float Right_accel;//右轮加速度
	}State_U;//状态输入量
	typedef struct {//反馈矩阵
		float K1;
		float K2;
		float K3;
		float K4;
	}State_K;//反馈矩阵
	typedef struct {//目标值
		float target_speed;//目标速度
		float target_Pitch;//目标俯仰角,用于设置机械平衡零点
		float target_turn;//目标转向
	}State_Target;
	typedef struct{//输出值
		float Velocity_Left;//左轮速度
		float Velocity_Right;//右轮速度
	}Output_Velocity;
	class WheelFoot_LQR{
	private:
		State_data Robot_State;
		State_U    Robot_Accel;
		State_K    Robot_K;
		State_Target Robot_target;
		Output_Velocity Robot_Output;
		float dt;//间隔时间
		float Ratio;//输出的比例系数
		float Turn_Ratio;//转向比例系数
		float wheel_base;//轮距
	public:
		WheelFoot_LQR(State_data my_State = {0,0,0,0},State_K my_K = {0,0,0,0},State_Target my_target = {0,0,0},float my_dt = 1,float my_wheel_base = 0.5f,float my_Turn_Ratio = 1){
			this->Robot_State = my_State;
			this->Robot_K     = my_K;
			this->Robot_target = my_target;
			this->dt = my_dt;
			this->wheel_base = my_wheel_base;
			this->Ratio = 1.0f;
			this->Turn_Ratio = my_Turn_Ratio;
		};
		void input_data(State_Target my_target);
		void Update_State(State_data my_State);
		void Change_dt(float new_dt){this->dt = new_dt;};//更改dt,用于实时计算执行时间的控制
		State_U accel_calc(void);
		void Set_K(State_K my_K);
		Output_Velocity Get_Robot_Out(void);
	};
	
	class Foot_control{
	private:
		float length1;
		float length2;
		float Foot_x;
		float Foot_y;
		float Joint_angle;
	public:
		void Positive_Solution();
		void Inverse_Solution();
		
	}
	
}



#endif
