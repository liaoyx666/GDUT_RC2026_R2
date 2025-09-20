#pragma once


#ifdef __cplusplus
#include "arm_math.h"
namespace RC_atitude{
	//Æ«º½½Ç¿ØÖÆ
	class yaw_control{
		public:
			yaw_control(){};
			float yaw_adjust(float now_yaw,float target_yaw);
		private:
			float now_yaw;
			float target_yaw;
			float angle_err;
			void err_calc();
	};


}


#endif
