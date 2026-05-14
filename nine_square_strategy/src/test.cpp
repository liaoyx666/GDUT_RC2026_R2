#include "../include/nine_square_strategy/nine_square_strategy.h"
#include "ros/ros.h"
#include "nine_square_strategy/nine_square_state_type1.h"
#include "nine_square_strategy/nine_square_state_type2.h"
#include "nine_square_strategy/place_kfc_decision.h"
#include "nine_square_strategy/robot_state.h"


int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test");
    ros::NodeHandle nh;
    ros::Publisher robot_state_pub = nh.advertise<nine_square_strategy::robot_state>("/robot_state",10);
    ros::Publisher nine_square_state_pub = nh.advertise<nine_square_strategy::nine_square_state_type2>("/nine_square_state",10);
    nine_square_strategy::robot_state msg1;
    nine_square_strategy::nine_square_state_type2  msg2;
    ROS_INFO("九宫格\n  0:空\n  1:我方R1_KFC\n  2:我方R2_KFC\n  3:敌方R1_KFC\n  4:敌方R2_KFC\n");

    ros::Rate rate(1);
    while (ros::ok())
    {
        msg1.dir_x = 0;
        msg1.dir_y = 0;
        msg1.is_combine = true;
        msg1.now_vel = 0;
        msg1.pos_x = 0;
        msg1.pos_y = 0;
        msg1.r1_kfc_num = 3;
        msg1.r2_kfc_num = 4;
        
        msg2.nine_square_state[0] = 0;
        msg2.nine_square_state[1] = 1;
        msg2.nine_square_state[2] = 2;

        msg2.nine_square_state[3] = 3;
        msg2.nine_square_state[4] = 4;
        msg2.nine_square_state[5] = 0;
        
        msg2.nine_square_state[6] = 0;
        msg2.nine_square_state[7] = 0;
        msg2.nine_square_state[8] = 0;
        robot_state_pub.publish(msg1);
        nine_square_state_pub.publish(msg2);
        
        ros::spinOnce();
        rate.sleep();                  
    }
    return 0;

}


