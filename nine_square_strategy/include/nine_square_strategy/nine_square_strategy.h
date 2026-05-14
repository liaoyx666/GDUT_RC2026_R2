#pragma once

#ifdef __cplusplus

#include <vector>
#include <random>
#include <cmath>
#include <iostream>
#include <ctime>

namespace Nine_Square_Strategy{
    //日志打印
    #define LOG_INFO(msg) std::cout<<"Info: "<<msg<<std::endl
    #define LOG_ERROR(msg) std::cerr<<"Error: "<<msg<<std::endl
    #define LOG_DEBUG(msg) std::cout<<"Debug: "<<msg<<std::endl
    //位置结构体
    struct Position
    {
        float x;
        float y;
        Position operator +(const Position b)const{
            Position a;
            a.x = b.x + x;
            a.y = b.y + y;
            return a; 
        }
        Position operator -(const Position b)const{
            Position a;
            a.x = x-b.x;
            a.y = y -b.y;
            return a; 
        }
    };
    //行为使用状态结构体
    struct Pos_Vel_Dir{
        Position pos;
        Position dir;
        float vel;
    };
    //当前对象持有KFC状态结构体
    struct TYPE_KFC_Having_State
    {
        int R1_KFC_Num;
        int R2_KFC_Num;
    };
    //即将大胜判断状态结构体
    struct TYPE_Will_Win_State
    {
        //是否即将大胜
        bool will_win = false;
        //是谁将赢
        int player;
        //再下哪个位置他就会赢
        std::vector<int> pos;
    };
    //仅九宫格参照得分表得分状态结构体
    struct TYPE_Score_State
    {
        int red_score;
        int blue_score;
    };
    struct TYPE_Decision
    {
        int Place_position;
        int Black_Type;
    };
    //二进制打印数据
    template<typename Num_Type>
    void Print_binary_System(Num_Type num){
        for (int i = 31; i>=0;--i){
            printf("%d",(num >> i)&1);
        }
        printf("\n");
    }
    //生成随机浮点数,左闭右开
    float Random_Float(float min,float max);
    //向量归一化
    Position Dir_Normalization(Position dir);
    //根据起始位置终点位置计算直线距离
    float Line_Distance_Calc(Position start_pos,Position end_pos);
    //根据路径总路程，按照梯形速度规划计算时间
    float LinePath_To_Time(float distance,float accel,float max_vel);
    //已知起始位置、终点位置、加速度和最大速度，根据花费时间计算对应位置，速度大小，方向
    Pos_Vel_Dir Line_Time_To_Position(Position start_point,Position end_point,float time,float accel,float max_vel);
    //九宫格状态工具类,每三位表示一个状态
    class CLASS_Nine_Square_Tool
    {
    private:
        /* data */
    public:
        //九宫格状态空,红R1,红R2,蓝R1,蓝R2
        static const int SQUARE_EMPTY = 0;
        static const int SQUARE_RED_R1_BLACK = 1;
        static const int SQUARE_RED_R2_BLACK = 2;
        static const int SQUARE_BLUE_R1_BLACK = 3;
        static const int SQUARE_BLUE_R2_BLACK = 4;
        static const int SQUARE_INVALID = 5;

        //玩家标识
        static const int PLAYER_RED_PLAYER = 6;
        static const int PLAYER_BLUE_PLAYER = 7;
        //全局下来各自对应R1_KFC,R2_KFC最大持有量
        static const int MAX_R1_KFC_NUM = 3;
        static const int MAX_R2_KFC_NUM = 4; 
        //规则得分
        static const int SCORE_BOTTOM_R1_KFC = 40;
        static const int SCORE_BOTTOM_R2_KFC = 50;
        static const int SCORE_MIDDLE_R1_KFC = 60;
        static const int SCORE_MIDDLE_R2_KFC = 70;
        static const int SCORE_UPPER_R1_KFC = 90;
        static const int SCORE_UPPER_R2_KFC = 100;

        //随机生成九宫格状态
        static int Function_Random_Nine_Square_State();
        //直观打印九宫格状况
        static void Function_Print_Nine_Square_State(int nine_square_state);
        //3*3二维数组转int类型九宫格状态
        static int Function_2DArray_To_Int(int nine_square_array[3][3]);
        //9位数组转int类型九宫格状态，逆序
        static int Function_Reverse_Vector_To_Int(int nine_square_vector[9]);
        //9位数组转int类型九宫格状态，正序
        static int Function_Vector_To_Int(int nine_square_vector[9]);
        //KFC持有数量判断是否合理
        static bool Function_KFC_Having_Judge(TYPE_KFC_Having_State having_state);
        //九宫格红蓝双方各自总得分
        static TYPE_Score_State Function_Nine_Square_Score_State(int nine_square_state);
        //判断九宫格即将大胜,返回值包含是否即将大胜，谁将赢，再下哪个位置他就会赢(可能有多个位置)
        static TYPE_Will_Win_State Function_Will_Win(int nine_square_state);
        //九宫格判断是否大胜
        static bool Function_IS_Win(int num,int player);
        //索引对应九宫格状态,pos为0-8，num为存储状态的整数，每三位表示一个状态
        static int Function_State_Search(int* num,int pos);
        //更改对应九宫格状态
        static void Function_State_Change(int* num, int pos, int input);
        //根据格子状态判断玩家类型
        static int Function_Cell_Player_Jungle(int* state,int pos);
        CLASS_Nine_Square_Tool() = default;
    };

    class CLASS_R2_Decision: public CLASS_Nine_Square_Tool
    {
    private:
        /* data */
        Pos_Vel_Dir PARAM_end_state[3];
        float PARAM_R2_accel=0.0f;
        float PARAM_R2_max_vel=0.0f;
        float PARAM_R1_accel=0.0f;
        float PARAM_R1_max_vel=0.0f;
        float PARAM_R2_Place_Black_Using_Time = 0.0f;
    public:        

        static const int R2_POSITION_START_0 = 0;
        static const int R2_POSITION_START_1 = 1;
        static const int R2_POSITION_START_2 = 2;
        //根据决策,KFC数量减少
        static void Function_KFC_Num_Decrease(TYPE_KFC_Having_State* having_state, TYPE_Decision decision,int Player);
        //决策打印
        static void Function_Print_Decision(TYPE_Decision decision);   
        //伪随机放置决策(纯R2版),如果能直接赢会下直接赢的位置，以及堵对面赢，否则随机放一个位置，放置的方块类型根据持有状态随机放置
        TYPE_Decision Function_R2_Randon_Decision(int nine_square_state,TYPE_KFC_Having_State having_state);
        //伪随机放置决策(R1-R2合体,合体时R2决策),如果能直接赢会下直接赢的位置，以及堵对面赢，否则随机放一个位置，放置的方块类型根据持有状态随机放置
        TYPE_Decision Function_R1_R2_Combined_Random_Decision(int nine_square_state,TYPE_KFC_Having_State having_state);
        //根据得分,时间和是否能大胜,或对方是否能大胜进行决策(纯R2版)
        TYPE_Decision Function_R2_Time_Decision(int nine_square_state,TYPE_KFC_Having_State having_state,Pos_Vel_Dir start_state);
        //根据得分和是否能大胜,或对方是否能大胜进行决策,不考虑时间使用长短(R1-R2合体,合体时R2决策)
        TYPE_Decision Function_R1_R2_Time_Decision(int nine_square_state,TYPE_KFC_Having_State having_state,Pos_Vel_Dir start_state);
        
        CLASS_R2_Decision(Pos_Vel_Dir end_point[3],
                        float PARAM_R2_accel_,float PARAM_R2_max_vel_,
                        float PARAM_R1_accel_,float PARAM_R1_max_vel_){
            std::srand(static_cast<unsigned int>(std::time(nullptr)));
            for (int i = 0; i < 3; ++i)
            {
                PARAM_end_state[i] = end_point[i];
            }
            PARAM_R2_accel=PARAM_R2_accel_;
            PARAM_R2_max_vel=PARAM_R2_max_vel_;
            PARAM_R1_accel=PARAM_R1_accel_;
            PARAM_R1_max_vel=PARAM_R1_max_vel_;
        }

    };
}
#endif
