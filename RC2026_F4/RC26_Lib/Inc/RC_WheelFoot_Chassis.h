#pragma once


#ifdef __cplusplus
	
//LQR ״̬���������㲿���
//VMC �ؽڽ���������ͨ���������ᣬӳ����ؽڵ�����ؽڵ�����
namespace RC_WheelFoot_Chassis{
	typedef struct {//״̬����
		float X_Pose;//λ��
		float X_speed;//�ٶ�
		float Pitch;//��������ĵĸ�����
		float Gyro;//��������ĵĽǼ��ٶ�
	}State_data;
	
	typedef struct {//״̬����������
		float Left_accel;//���ּ��ٶ�
		float Right_accel;//���ּ��ٶ�
	}State_U;//״̬������
	typedef struct {//��������
		float K1;
		float K2;
		float K3;
		float K4;
	}State_K;//��������
	typedef struct {//Ŀ��ֵ
		float target_speed;//Ŀ���ٶ�
		float target_Pitch;//Ŀ�긩����,�������û�еƽ�����
		float target_turn;//Ŀ��ת��
	}State_Target;
	typedef struct{//���ֵ
		float Velocity_Left;//�����ٶ�
		float Velocity_Right;//�����ٶ�
	}Output_Velocity;
	class WheelFoot_LQR{
	private:
		State_data Robot_State;
		State_U    Robot_Accel;
		State_K    Robot_K;
		State_Target Robot_target;
		Output_Velocity Robot_Output;
		float dt;//���ʱ��
		float Ratio;//����ı���ϵ��
		float Turn_Ratio;//ת�����ϵ��
		float wheel_base;//�־�
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
		void Change_dt(float new_dt){this->dt = new_dt;};//����dt,����ʵʱ����ִ��ʱ��Ŀ���
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
