#include "ros/ros.h"
#include "nine_square_strategy/nine_square_state_type1_srv.h"
#include "nine_square_strategy/nine_square_state_type2_srv.h"


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"nine_square_strategy_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<nine_square_strategy::nine_square_state_type2_srv>("nine_square_state_type2_srv");
    
    ros::service::waitForService("nine_square_state_type2_srv");

    nine_square_strategy::nine_square_state_type2_srv client_;
    client_.request.dir_x = 0;
    client_.request.dir_y = 0;
    client_.request.is_combine = 0;
    client_.request.now_vel= 0;
    client_.request.pos_x=0;
    client_.request.pos_y=0;
    client_.request.r1_kfc_num=3;
    client_.request.r2_kfc_num=4;

    client_.request.nine_square_state[0] = 0;
    client_.request.nine_square_state[1] = 3;
    client_.request.nine_square_state[2] = 5;
    client_.request.nine_square_state[3] = 5;
    client_.request.nine_square_state[4] = 1;
    client_.request.nine_square_state[5] = 0;
    client_.request.nine_square_state[6] = 4;
    client_.request.nine_square_state[7] = 0;
    client_.request.nine_square_state[8] = 5;


    bool flag = client.call(client_);
    // 7.处理响应
    if (flag)
    {
        ROS_INFO("请求正常处理,响应结果:%d",client_.response.place_position);
    }
    else
    {
        ROS_ERROR("请求处理失败....");
        return 1;
    }
    return 0;
}