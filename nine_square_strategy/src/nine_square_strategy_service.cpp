#include "../include/nine_square_strategy/nine_square_strategy.h"
#include "ros/ros.h"
#include "nine_square_strategy/nine_square_state_type1_srv.h"
#include "nine_square_strategy/nine_square_state_type2_srv.h"


//终点也就是三个放置方块的点位,二维坐标（目标位置，目标方向，目标速度）
Nine_Square_Strategy::Pos_Vel_Dir end_point[3] = {{{1,0},{0,0},0},
                                                    {{2,0},{0,0},0},
                                                    {{3,0},{0,0},0}};
const float R2_accel = 1.0f;//m/s^2
const float R2_max_vel = 5.0f;//m/s
const float R1_accel = 1.0f;
const float R1_max_vel = 5.0f;

Nine_Square_Strategy::CLASS_R2_Decision strategy(end_point,R2_accel,R2_max_vel,R1_accel,R1_max_vel);

static Nine_Square_Strategy::Pos_Vel_Dir robot_state;
static Nine_Square_Strategy::TYPE_KFC_Having_State kfc_num_state;
static bool is_combine_;
static int nine_state;
static Nine_Square_Strategy::TYPE_Decision decision;

bool do_process(nine_square_strategy::nine_square_state_type2_srv::Request& req,
          nine_square_strategy::nine_square_state_type2_srv::Response& resp){
    if (req.nine_square_state.size() != 9)
    {
        ROS_WARN("[错误] 九宫格数据长度非法！直接返回不行动");
        resp.black_type = 1;    
        resp.place_position = -1; // 不动
        return true;
    }
    //非法值过滤（防止NaN、无穷大、乱数）
    if (!isfinite(req.pos_x) || !isfinite(req.pos_y) ||
        !isfinite(req.now_vel) ||
        !isfinite(req.dir_x) || !isfinite(req.dir_y))
    {
        ROS_WARN("[错误] 机器人状态数据非法！直接返回不行动");
        resp.black_type = 1;
        resp.place_position = -1;
        return true;
    }
    //KFC数量不能是负数
    if (req.r1_kfc_num < 0 || req.r2_kfc_num < 0)
    {
        ROS_WARN("[错误] KFC数量非法,直接返回不行动");
        resp.black_type = 1;
        resp.place_position = -1;
        return true;
    }

    robot_state.pos.x = req.pos_x;
    robot_state.pos.y = req.pos_y;
    robot_state.vel = req.now_vel;
    robot_state.dir.x = req.dir_x;
    robot_state.dir.y = req.dir_y;
    kfc_num_state.R1_KFC_Num = req.r1_kfc_num;
    kfc_num_state.R2_KFC_Num = req.r2_kfc_num;
    is_combine_ = req.is_combine;
    int state[3][3]={
        {req.nine_square_state[8],req.nine_square_state[7],req.nine_square_state[6]},
        {req.nine_square_state[5],req.nine_square_state[4],req.nine_square_state[3]},
        {req.nine_square_state[2],req.nine_square_state[1],req.nine_square_state[0]}
    };
    nine_state = strategy.Function_2DArray_To_Int(state);
    strategy.Function_Print_Nine_Square_State(nine_state);
    

    if(is_combine_){
        int now_state_ = nine_state;
        decision = strategy.Function_R1_R2_Time_Decision(nine_state,kfc_num_state,robot_state);
        strategy.Function_Print_Decision(decision);
        ROS_INFO("决策后九宫格状态\n");
        strategy.Function_State_Change(&now_state_,decision.Place_position,decision.Black_Type);
        strategy.Function_Print_Nine_Square_State(now_state_);        
        resp.black_type = decision.Black_Type;
        resp.place_position = decision.Place_position;
    }
    else{
        int now_state_ = nine_state;
        decision = strategy.Function_R2_Time_Decision(nine_state,kfc_num_state,robot_state);
        strategy.Function_Print_Decision(decision);
        ROS_INFO("决策后九宫格状态\n");
        strategy.Function_State_Change(&now_state_,decision.Place_position,decision.Black_Type);
        strategy.Function_Print_Nine_Square_State(now_state_);

        resp.black_type = decision.Black_Type;
        resp.place_position = decision.Place_position;            
    }
    return true;
}

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"nine_square_strategy_service");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("nine_square_state_type2_srv",do_process);
    ROS_INFO("九宫格\n  0:空\n  1:我方R1_KFC\n  2:我方R2_KFC\n  3:敌方R1_KFC\n  4:敌方R2_KFC\n");
    ROS_INFO("决策位置0~8");
    ros::spin();
    return 0;

}


