#include "../include/nine_square_strategy/nine_square_strategy.h"
#include "ros/ros.h"
#include "nine_square_strategy/nine_square_state_type1.h"
#include "nine_square_strategy/nine_square_state_type2.h"
#include "nine_square_strategy/place_kfc_decision.h"
#include "nine_square_strategy/robot_state.h"

//终点也就是三个放置方块的点位,二维坐标（目标位置，目标方向，目标速度）
Nine_Square_Strategy::Pos_Vel_Dir end_point[3] = {{{1,0},{0,0},0},
                                                    {{2,0},{0,0},0},
                                                    {{3,0},{0,0},0}};
const float R2_accel = 1.0f;//m/s^2
const float R2_max_vel = 5.0f;//m/s
const float R1_accel = 1.0f;
const float R1_max_vel = 5.0f;

Nine_Square_Strategy::CLASS_R2_Decision strategy(end_point,R2_accel,R2_max_vel,R1_accel,R1_max_vel);

void test(Nine_Square_Strategy::CLASS_R2_Decision a,Nine_Square_Strategy::Pos_Vel_Dir start_pos){
    int nine_square_state = a.Function_Random_Nine_Square_State();//随机生成九宫格情况
    //3*3数组转九宫格int类型
    int state[3][3]={
        {a.SQUARE_RED_R1_BLACK,0,a.SQUARE_RED_R2_BLACK},
        {0,0,a.SQUARE_BLUE_R2_BLACK},
        {a.SQUARE_BLUE_R2_BLACK,0,0}
    };
    nine_square_state = a.Function_2DArray_To_Int(state);
    //打印九宫格情况，二进制和3*3网格
    Nine_Square_Strategy::Print_binary_System(nine_square_state);
    a.Function_Print_Nine_Square_State(nine_square_state);
    //放置方块位置和放置R1或R2 KFC
    Nine_Square_Strategy::TYPE_Decision decision = {0,0};
    //当前方块数量情况R1 KFC数，R2 KFC数
    Nine_Square_Strategy::TYPE_KFC_Having_State having_state = {3,4};
    printf("Initial KFC Having State: R1_KFC_Num = %d, R2_KFC_Num = %d\n", having_state.R1_KFC_Num, having_state.R2_KFC_Num);
    
    //考虑时间放置决定（当前九宫格情况，方块持有情况，起点）
    decision = a.Function_R1_R2_Time_Decision(nine_square_state,having_state,start_pos);
    //打印决策情况
    a.Function_Print_Decision(decision);
    //更新KFC决策后KFC情况
    a.Function_KFC_Num_Decrease(&having_state, decision, a.PLAYER_RED_PLAYER);
    printf("Updated KFC Having State: R1_KFC_Num = %d, R2_KFC_Num = %d\n", having_state.R1_KFC_Num, having_state.R2_KFC_Num);
    //更新九宫格情况
    a.Function_State_Change(&nine_square_state,decision.Place_position,decision.Black_Type);
    //打印3*3九宫格情况
    a.Function_Print_Nine_Square_State(nine_square_state);
}


static Nine_Square_Strategy::Pos_Vel_Dir robot_state;
static Nine_Square_Strategy::TYPE_KFC_Having_State kfc_num_state;
static bool is_combine_;
static int nine_state;
static Nine_Square_Strategy::TYPE_Decision decision;


void robot_state_process(const nine_square_strategy::robot_state::ConstPtr& msg){
    robot_state.pos.x = msg->pos_x;
    robot_state.pos.y = msg->pos_y;
    robot_state.vel = msg->now_vel;
    robot_state.dir.x = msg->dir_x;
    robot_state.dir.y = msg->dir_y;
    kfc_num_state.R1_KFC_Num = msg->r1_kfc_num;
    kfc_num_state.R2_KFC_Num = msg->r2_kfc_num;
    is_combine_ = msg->is_combine;
}
void nine_square_state_process(const nine_square_strategy::nine_square_state_type2::ConstPtr& msg){
    if(sizeof(msg->nine_square_state)/sizeof(int32_t) == 9){
        int state[3][3]={
            {msg->nine_square_state[8],msg->nine_square_state[7],msg->nine_square_state[6]},
            {msg->nine_square_state[5],msg->nine_square_state[4],msg->nine_square_state[3]},
            {msg->nine_square_state[2],msg->nine_square_state[1],msg->nine_square_state[0]}
        };
        nine_state = strategy.Function_2DArray_To_Int(state);
        strategy.Function_Print_Nine_Square_State(nine_state);
    }
}

int main(int argc, char  *argv[])
{   
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"nine_square_strategy");
    ros::NodeHandle nh;
    //实时订阅九宫格情况
    //实时订阅当前机器人位置，朝向，当前速度，持有KFC情况
    //根据订阅发布决策（放置九宫格位置，放置KFC类型）
    ros::Subscriber robot_state_sub = nh.subscribe<nine_square_strategy::robot_state>("/robot_state",10,robot_state_process); 
    ros::Subscriber nine_square_state_sub = nh.subscribe<nine_square_strategy::nine_square_state_type2>("/nine_square_state",10,nine_square_state_process); 
    ros::Publisher place_decision_pub = nh.advertise<nine_square_strategy::place_kfc_decision>("/place_decision",10);
    ROS_INFO("九宫格\n  0:空\n  1:我方R1_KFC\n  2:我方R2_KFC\n  3:敌方R1_KFC\n  4:敌方R2_KFC\n");
    ROS_INFO("决策位置0~8");
    nine_square_strategy::place_kfc_decision msg;
    ros::Rate rate(1);

    while (ros::ok())
    {
        if(is_combine_){
            decision = strategy.Function_R1_R2_Time_Decision(nine_state,kfc_num_state,robot_state);
            msg.black_type = decision.Black_Type;
            msg.place_position = decision.Place_position;
        }
        else{
            decision = strategy.Function_R2_Time_Decision(nine_state,kfc_num_state,robot_state);
            msg.black_type = decision.Black_Type;
            msg.place_position = decision.Place_position;            
        }
        place_decision_pub.publish(msg);
        strategy.Function_Print_Decision(decision);
        
        ros::spinOnce();
        rate.sleep();                  
    }
    return 0;

}


