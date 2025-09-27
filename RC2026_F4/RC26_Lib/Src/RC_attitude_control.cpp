#include "RC_attitude_control.h"


namespace RC_atitude{
	void yaw_control::err_calc(){//ѡ���·�������
		//ͬ����
		if(this->now_yaw*this->target_yaw > 0){
			this->angle_err = this->target_yaw -this->now_yaw;
		}
		//ȡ���·��
		else if(this->target_yaw <0  && this->now_yaw >0){
			float positive = 360 - this->now_yaw+ this->target_yaw; // ��·��
            float negative = this->target_yaw - this->now_yaw;//��·��
			if(fabs(positive)>fabs(negative)){
				this->angle_err = negative;
			}
			else{
				this->angle_err = positive;
			}
		}
		else if(this->target_yaw >0  && this->now_yaw <0){
			float positive = this->target_yaw -this->now_yaw;
            float negative = -360-this->now_yaw + this->target_yaw;
			if(fabs(positive)>fabs(negative)){
				this->angle_err = negative;
			}
			else{
				this->angle_err = positive;
			}
		}
		
	}
	float yaw_control::yaw_adjust(float now_yaw,float target_yaw){
		this->now_yaw = now_yaw;
		this->target_yaw = target_yaw;
		this->err_calc();//ȡ��̵���·��
		return this->angle_err;//�������ֱ��PID����
	}
}