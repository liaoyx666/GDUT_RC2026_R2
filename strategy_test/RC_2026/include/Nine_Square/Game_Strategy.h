#pragma once

#ifdef __cplusplus

#include <vector>
#include <algorithm> // 用于组合生成
#include <cmath>
#include <random>
#include <iostream>
#include <ctime>

namespace Strategy{
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

    enum {
        Empty = 0,//格子为空
        OurBlack = 1,//格子为我方方块
        OppBlack = 2 //格子为敌方方块
    };
    
    enum {
        Nothing = 0,//对当前格子无任何行为
        Place_Block = 1,//对当前格子放置我方方块
        Push_Block = 2 //推掉当前格子敌方方块
    };

    enum BehaviorExtraPos {
        R1_USED = 9,
        R2_USED = 10,
        WEAPON_USED = 11
    };
    
    struct Cost_Weight
    {
        int Q1;
        int Q2;
        int Q3;
        int Q4;
    };

    enum ROBOT_BEHAVIOR{
        EMPTY,
        R1_PLACE,
        R1_PUSH,
        R1_WAITING_FIT,
        R2_PLACE_MIDDLE,
        R2_PLACE_UPPER1,
        R2_PLACE_UPPER2
    };

    struct Behavior_Using_State
    {
        float using_time;//执行该步骤所需时间
        //目标位置点位置
        float x;
        float y;
        //行为：R1：放置|推|等待合体    R2: 放置中行|放置上行1（前往合体）|放置上行2（合体后由R1主导）
        int doing_place;//分解的作用到九宫格的位置（1~9）
        ROBOT_BEHAVIOR Behavior;
        Behavior_Using_State operator +(const Behavior_Using_State b)const{
            Behavior_Using_State a;
            a.x = b.x + x;
            a.y = b.y + y;
            return a; 
        }
        Behavior_Using_State operator -(const Behavior_Using_State b)const{
            Behavior_Using_State a;
            a.x = x-b.x;
            a.y = y -b.y;
            return a; 
        }
        
    };
    
    struct Robot_State
    {
        int Person;
        int Robot;
        float Max_vel;
        float Accel;
        Position now_Position;
    };

    struct Range{
        float min;
        float max;
    };

    const int ALL_BEHAVIOR_SIZE = 204;
    
    constexpr float EPS = 1e-6f;   //浮点数判断相等的误差
    int State_Change(int num, int pos, int input);
    int State_Search(int num,int pos);
    template<typename Num_Type>
    void Print_binary_System(Num_Type num);
    float Distance_Calc(Position start_pos,Position end_pos);
    float RandFloat(float min,float max);
    struct Behavior_state_
    {
        int Behavior;
        float sim_using_time;
    };
    

    class Game_Strategy
    {
    private:

        static constexpr float R1_BLACK_SCORE_SCORE = 40.0f;
        static constexpr float R2_MIDDLE_BLACK_SCORE_SCORE = 70.0f;
        static constexpr float R2_UPPER_BLACK_SCORE_SCORE = 100.0f;

        static constexpr float DIAGON_SCORE = 70.0f;   // 对角位置得分
        static constexpr float LINE_SCORE = 40.0f;     // 边位置得分
        static constexpr float MIDDLE_SCORE = 100.0f;  // 中心位置得分

        int Now_State;
        int Now_Behavior;
        int All_Behavior[204]; //State_Search   0~8  为九宫格行为  9 为该行为R1使用数量 10 为该行为R2使用数量 11 为该行为武器使用数量 12 为接受处理该信息的机器人是R1或R2 13 执行方为红队或蓝队 
        
        Position End_pos[2][2][3];//九宫格执行操作的位置，分为红方，蓝方，以及各自的R1,R2对应的0，1，2三个格子放置物块的坐标位置 
        static const float Black_Score[9];
        static const float Black_pos_Score[9];

        bool Behavior_Jungle(int now_state,int behavior);
        void GenerateAllBehaviors();
        float Path_To_Time(float distance,float accel,float max_vel);//总轨迹长计算时间
        
        float Total_Black_Scores_Calc(int behavior);
        float Total_BlackSite_Scores_Calc(int behavior);
        float Total_LostScore_Calc(int behavior);
        float Total_UsingTime_Calc(int behavior,Robot_State R1_state,Robot_State R2_state);

        float Calc_Score(Cost_Weight weight,float Total_Black_Scores,float Total_BlackSite_Scores,float Total_LostScore,float Total_UsingTime);//代价函数 得分 = 方块得分系数*当前行为方块得分 + 方块位置系数*当前行为方块位置总得分 + 对分失分系数*当前行为对方总失分  - 时间系数*当前得分总时间
    protected:
        Position Time_To_Position(Position start_point,Position end_point,float time,float accel,float max_vel);
        std::vector<Behavior_Using_State> Whole_Behavior_To_Step(int person,int Robot,Position position,int behavior,float Accel,float Max_vel);  
    public:
        static constexpr int PERSON_RED = 1;//左边三区
        static constexpr int PERSON_BLUE = 2;//右边三区

        static constexpr int ROBOT_R1 = 1;
        static constexpr int ROBOT_R2 = 2;
        static constexpr int ROBOT_POS_ = 12;
        static constexpr int PERSON_POS_ = 13;

        Behavior_state_ Strategy_Control(Cost_Weight weight_,int now_state,Robot_State R1_state,Robot_State R2_state);
        int Set_State(int person,int robot,int behavior);//设置状态，比如红蓝方，机器人   

        void Print_ALL_Behavior();
        Game_Strategy(){
            std::srand(static_cast<unsigned int>(std::time(nullptr)));
            GenerateAllBehaviors();//记录所有可能执行行为
        }
    };

    
    class Best_Weigt_Finding :public Game_Strategy{
        private:
            static std::vector<Cost_Weight> ALL_Weight_State;
            
            int Winning_Num = 0;
            int Sim_num;
            
            int Our_now_State = 0;
            int Opp_now_State = 0;

            Robot_State  Our_R1;
            Robot_State  Our_R2;
            Robot_State  Opp_R1;
            Robot_State  Opp_R2;

            int Our_R1_Black_Num;
            int Our_R2_Black_Num;
            int Our_weapon_Num;
            int Opp_R1_Black_Num;
            int Opp_R2_Black_Num;
            int Opp_weapon_Num;

            int Reset_Our_R1_Black_Num;
            int Reset_Our_R2_Black_Num;
            int Reset_Our_weapon_Num;
            int Reset_Opp_R1_Black_Num;
            int Reset_Opp_R2_Black_Num;
            int Reset_Opp_weapon_Num;

            Range Accel_randon_range;
            Range Max_vel_randon_range;
            Range DeadLine_randon_range;
            Range Start_Pos_randon_range;

            float Opp_R1_accel_offset = 0.0f;
            float Opp_R2_accel_offset = 0.0f;
            float Opp_R1_max_vel_offset = 0.0f;
            float Opp_R2_max_vel_offset = 0.0f;

            Position  Our_R1_start_Pos;
            Position  Our_R2_start_Pos;
            Position  Opp_R1_start_Pos;
            Position  Opp_R2_start_Pos;

            Position  Our_R1_start_Offset_Pos;
            Position  Our_R2_start_Offset_Pos;
            Position  Opp_R1_start_Offset_Pos;
            Position  Opp_R2_start_Offset_Pos;

            float DeadLine_Time = 0.0f;//一局的截止时间
            float DeadLine_Time_offset = 0.0f;//一局截至时间的随机噪声

            bool is_init_robot = 0;

            Cost_Weight Our_best_weight;

            Cost_Weight Opp_weight[3]; //得分激进对手 速度激进对手 均衡对手 
        public:
            int Process_winner_jungle(int Nine_Square_state_);
            int jungle_winner(int Nine_Square_state_);
            Behavior_Using_State Find_mining_time_behavior(Behavior_Using_State Our_R1_,Behavior_Using_State Our_R2_,Behavior_Using_State Opp_R1_,Behavior_Using_State Opp_R2_);
            int nine_square_update(int nine_square,int person,Behavior_Using_State state);
            void Update_OPP_Robot_State();
            void State_clear(); 
            void Set_Black_Num(int Our_R1_Num,int Our_R2_Num,int our_weapon,int Opp_R1_Num,int Opp_R2_Num,int Opp_weapon);
            void Main_Runing();
            Best_Weigt_Finding(float Our_R1_max_vel,float Our_R1_accel,
                                float Our_R2_max_vel,float Our_R2_accel,
                                float Opp_Base_R1_max_vel,float Opp_Base_R1_accel,
                                float Opp_Base_R2_max_vel,float Opp_Base_R2_accel,float DeadLine_Time_,int Sim_num_,
                                Range Accel_randon_range_,Range Max_vel_randon_range_,Range DeadLine_randon_range_,Range Start_Pos_randon_range_);
    };

}
#endif


