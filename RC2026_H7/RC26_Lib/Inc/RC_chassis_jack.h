#pragma once

#include "RC_motor.h"
#include "RC_LiDAR.h"

#ifdef __cplusplus

namespace chassis_jack
{
    class Chassis_jack
    {
        public:
            Chassis_jack(motor::Motor& left_front_motor_,
                        motor::Motor& left_behind_motor_,
                        motor::Motor& right_front_motor_, 
                        motor::Motor& right_behind_motor_,
                        liDAR::LiDAR& LiDAR_jack_);

            virtual ~Chassis_jack() {}

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
    };
}



#endif
