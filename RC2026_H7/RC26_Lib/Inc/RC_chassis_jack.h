#pragma once

#include "RC_motor.h"
#include "RC_LiDAR.h"

#ifdef __cplusplus

namespace chassis_jack
{
    class Chassis_jack
    {
<<<<<<< Updated upstream
        public:
            Chassis_jack(motor::Motor& left_front_motor_,
                        motor::Motor& left_behind_motor_,
                        motor::Motor& right_front_motor_, 
                        motor::Motor& right_behind_motor_,
                        liDAR::LiDAR& LiDAR_jack_);
=======
	public:
		Chassis_jack(
			motor::Motor& left_front_motor_,
			motor::Motor& left_behind_motor_,
			motor::Motor& right_front_motor_, 
			motor::Motor& right_behind_motor_,
			lidar::LiDAR& LiDAR_jack_
		);
>>>>>>> Stashed changes

            virtual ~Chassis_jack() {}

<<<<<<< Updated upstream
            void chassis_up();
            void chassis_down();
			void chassis_up_test();
			void chassis_down_test();
							
			uint8_t b = 0;

        private:
            motor::Motor& left_front_motor;
            motor::Motor& left_behind_motor;
            motor::Motor& right_front_motor;
            motor::Motor& right_behind_motor;
            liDAR::LiDAR& LiDAR_jack;
=======
		void chassis_up();
		void chassis_down();
		void chassis_test(bool is_uping);
			
		uint8_t b = 0;
		float tag = 400;
		float dis = 0;

		bool up_or_down = 0;

	private:
		motor::Motor& left_front_motor;
		motor::Motor& left_behind_motor;
		motor::Motor& right_front_motor;
		motor::Motor& right_behind_motor;
		lidar::LiDAR& LiDAR_jack;
>>>>>>> Stashed changes
    };
}



#endif
