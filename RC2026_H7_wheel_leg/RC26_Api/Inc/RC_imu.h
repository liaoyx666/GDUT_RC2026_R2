#pragma once


#ifdef __cplusplus

namespace imu{
		class imu{
			public:
				imu(){}
				virtual ~imu() {}
				float Get_Roll() const {return euler[0];}
				float Get_Pitch() const {return euler[1];}
				float Get_Yaw() const {return euler[2];}
				float* Get_Gyro() {return gyro;}
				float* Get_Accel() {return accel;}
				float* Get_Mag() {return mag;}
			protected:		
				float euler[3] = {0};
				float gyro[3] = {0};
				float accel[3] = {0};
				float mag[3] = {0};
				float quaternion[4] = {0};
				float temp = 0;
		};

}
#endif