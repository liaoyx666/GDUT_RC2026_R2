#include "../../include/Nine_Square/Game_Strategy.h"
#include <iostream>
//获取当前状态
//列出所有可以执行的行为，以及对应的目标位置点
//求出每个行为的代价函数得分
//找出最高得分的行为

//第一步：就是实现输入一个行为求出对应的分数，以及遍历所有行为找出最优行为
/*
*九宫格序号0~8，0~2为最下面，3~5为中间，6~8为最上面，小为左，大为右
*方块理想：R1 1个,武器 1个，R2 2个
————————————————————————————————————————————————————————————————————————————————————————————————
*全部行为（目前当作放R1和武器捅对面独立）：
1.  放1个 R1：0~2   共3种行为                                          

2. 放1个 R1，1个 R2  R1 0~2   R2 3~8            共 3*6 = 18种行为     

3.  放1个 R1，用 1个武器  R1放0 武器 1~8 8种
                        R1放1 武器 0 2~8 8种
                        R1放2 武器 0~1 3~8 8种  共24种行为
4.  放1个 R1，2个 R2，用1个武器    共3*15*6 = 270 种行为




5. 放1个 R2 3~8                                 共6种行为



6.  放2个 R2       34 35 36 37 38  45 46 47 48 56 57 58 67 68 78  共15种行为
7.  放1个 R2 用 1个武器  共 6*8 = 48 种行为
8.  放2个 R2 用 1个武器  共15*7 = 105种行为
9.  用1个武器  0~8 共9种行为

共 3+18+ 24+ 270+ 6+15+48+105+9 = 498种情况
*** 若放R1和用武器同时只能执行一个 那么3.，4.不算  498-270-24 = 204种

遍历过程性淘汰：
    1.若用到武器，武器执行的位置不是对方方块（我方方块或空），当成一个愚蠢行为，不去进行后续计算直接跳过
    2.若放置我方方块，这个位置已经有方块了，跳过

由此得到所有有效行为
——————————————————————————————————————————————————————————————————————————————————————————————————
*时间预估：
1.若R1与R2执行区块位置重合，时间 = R1和R2中执行时间最短的时间+占位机器人移开时间+移开后时另一机器人位置到该位置及执行时间之和
2.若1机器人挡住另1要执行操作的机器人，时间 = 无执行操作机器人避让时间+执行机器人执行时间

将两车看成圆柱体，三次B样条进行路径规划
——————————————————————————————————————————————————————————————————————————————————————————————————
*行为选取
位置得分：
        放中间10分 
        放角上8分 
        放边上5分
对方失分：
        武器捅去对方 0~2方块 40分
                    3~5方块 70分
                    6~8方块 100分
方块得分：
        0~2方块 40分
        3~5方块 70分
        6~8方块 100分
消耗时间：由运动到对应位置路径规划求得的预估时间+行为执行的时间+时间噪声
得分 = 放置位置总得分*位置weight + 对方失分*失分weight + 我方方块总得分*得分weight - 我方消耗时间*时间weight

*对所有有效行为求得分 得分最高的为本次执行操作
——————————————————————————————————————————————————————————————————————————————————————————————————
*/
namespace Strategy{
    const float Game_Strategy::Black_Score[9] = {R1_BLACK_SCORE_SCORE,R1_BLACK_SCORE_SCORE,R1_BLACK_SCORE_SCORE,
                                                R2_MIDDLE_BLACK_SCORE_SCORE,R2_MIDDLE_BLACK_SCORE_SCORE,R2_MIDDLE_BLACK_SCORE_SCORE,
                                                R2_UPPER_BLACK_SCORE_SCORE,R2_UPPER_BLACK_SCORE_SCORE,R2_UPPER_BLACK_SCORE_SCORE};
    const float Game_Strategy::Black_pos_Score[9] = {DIAGON_SCORE,LINE_SCORE,DIAGON_SCORE,
                                                    LINE_SCORE,MIDDLE_SCORE,LINE_SCORE,
                                                    DIAGON_SCORE,LINE_SCORE,DIAGON_SCORE};

    void Game_Strategy::GenerateAllBehaviors() {
        std::vector<int> behaviors; 
        for(int i = 0;i<3;i++){
            End_pos[PERSON_RED-1][ROBOT_R1-1][i] = {5.84f,(1.81f-0.5f*i)};
        }
        for(int i = 0;i<3;i++){
            End_pos[PERSON_RED-1][ROBOT_R2-1][i] = {5.84f,(1.81f-0.5f*i)};
        }
        
        for(int i = 0;i<3;i++){
            End_pos[PERSON_BLUE-1][ROBOT_R1-1][i] = {6.16f,(1.81f-0.5f*i)};
        }
        
        for(int i = 0;i<3;i++){
            End_pos[PERSON_BLUE-1][ROBOT_R2-1][i] = {6.16f,(1.81f-0.5f*i)};
        }



        for (int r1 = 0; r1 < 3; ++r1) {
                int behavior = 0;
                behavior = State_Change(behavior, R1_USED, 1);
                behavior = State_Change(behavior, r1, Place_Block);
                behaviors.push_back(behavior);
        }

        for (int r1 = 0; r1 < 3; ++r1) {
            for (int r2 = 3; r2 < 9; ++r2) {
                int behavior = 0;
                behavior = State_Change(behavior, R1_USED, 1);
                behavior = State_Change(behavior, R2_USED, 1);
                behavior = State_Change(behavior, r1, Place_Block);
                behavior = State_Change(behavior, r2, Place_Block);
                behaviors.push_back(behavior);
            }
        }

        for (int r2 = 3; r2 < 9; ++r2) {
            int behavior = 0;
            behavior = State_Change(behavior, R2_USED, 1);
            behavior = State_Change(behavior, r2, Place_Block);
            behaviors.push_back(behavior);
        }

        int r2_pos[] = {3,4,5,6,7,8};
        for (int i = 0; i < 6; ++i) {
            for (int j = i+1; j < 6; ++j) {
                int behavior = 0;
                behavior = State_Change(behavior, R2_USED, 2);
                behavior = State_Change(behavior, r2_pos[i], Place_Block);
                behavior = State_Change(behavior, r2_pos[j], Place_Block);
                behaviors.push_back(behavior);
            }
        }

        for (int r2 = 3; r2 < 9; ++r2) {
            for (int weapon = 0; weapon < 9; ++weapon) {
                if (weapon == r2) continue; // 武器不能打自己放R2的位置
                int behavior = 0;
                behavior = State_Change(behavior, R2_USED, 1);
                behavior = State_Change(behavior, WEAPON_USED, 1);
                behavior = State_Change(behavior, r2, Place_Block);
                behavior = State_Change(behavior, weapon, Push_Block);
                behaviors.push_back(behavior);
            }
        }

        for (int i = 0; i < 6; ++i) {
            for (int j = i+1; j < 6; ++j) {
                int r2_1 = r2_pos[i];
                int r2_2 = r2_pos[j];
                for (int weapon = 0; weapon < 9; ++weapon) {
                    if (weapon == r2_1 || weapon == r2_2) continue;
                    int behavior = 0;
                    behavior = State_Change(behavior, R2_USED, 2);
                    behavior = State_Change(behavior, WEAPON_USED, 1);
                    
                    behavior = State_Change(behavior, r2_1, Place_Block);
                    behavior = State_Change(behavior, r2_2, Place_Block);
                    behavior = State_Change(behavior, weapon, Push_Block);
                    behaviors.push_back(behavior);
                }
            }
        }

        for (int weapon = 0; weapon < 9; ++weapon) {
            int behavior = 0;
            behavior = State_Change(behavior, WEAPON_USED, 1);
            behavior = State_Change(behavior, weapon, Push_Block);
            behaviors.push_back(behavior);
        }

        if (behaviors.size() != ALL_BEHAVIOR_SIZE) {
            printf("warning:real number is %d\n", (int)behaviors.size());
        }
        // 拷贝到数组
        std::copy(behaviors.begin(), behaviors.end(), All_Behavior);
        // 剩余位置初始化0
        for (int i = behaviors.size(); i < ALL_BEHAVIOR_SIZE; ++i) {
            All_Behavior[i] = 0;
        }
    }
    
    bool Game_Strategy::Behavior_Jungle(int now_state,int behavior){
        int valid_grid_count = 0;//合法格子数
        int grid_state;
        int grid_behavior;
        //检测当前持有状态是否满足该行为使用方块或武器数
        if(State_Search(now_state,R1_USED) < State_Search(behavior,R1_USED) || State_Search(now_state,R2_USED) < State_Search(behavior,R2_USED) || State_Search(now_state,WEAPON_USED) < State_Search(behavior,WEAPON_USED)){
            return false;
        }
        for (int i = 0; i < 9; ++i){
            grid_state = State_Search(now_state, i);   // 当前格子状态
            grid_behavior = State_Search(behavior, i); // 对当前格子执行的行为
            // 根据状态和行为判断是否合法
            switch (grid_state) {
                case Empty:
                    if (grid_behavior == Nothing || grid_behavior == Place_Block) {
                        valid_grid_count++;
                    }
                    break;                    
                case OurBlack:
                    if (grid_behavior == Nothing) {
                        valid_grid_count++;
                    }
                    break;
                case OppBlack:
                    if (grid_behavior == Nothing || grid_behavior == Push_Block) {
                        valid_grid_count++;
                    }
                    break;
                default:
                    break;
            }
        }
        if(valid_grid_count == 9){
            return true;
        }
        else{
            return false;
        }
    }

    float Game_Strategy::Total_Black_Scores_Calc(int behavior){
        float Total_Score = 0.0f;
        for (int i = 0;i < 9;++i){
            if(State_Search(behavior,i) == Place_Block){
                Total_Score += Black_Score[i];
            }
        }
        return Total_Score;
    }
    float Game_Strategy::Total_BlackSite_Scores_Calc(int behavior){
        float Total_Score = 0.0f;
        for(int i = 0;i<9;++i){
            if(State_Search(behavior,i) == Place_Block){
                Total_Score += Black_pos_Score[i];
            }
        }
        return Total_Score;
    }
    float Game_Strategy::Total_LostScore_Calc(int behavior)
    {
        float Total_Score = 0.0f;

        if(State_Search(behavior,WEAPON_USED)){
            for(int i = 0;i<9;++i){
                if(State_Search(behavior,i) == Push_Block){
                    Total_Score += Black_Score[i];
                }
            }
        }
        return Total_Score;
    }

    float Distance_Calc(Position start_pos,Position end_pos){
        return (sqrt((start_pos.x- end_pos.x)*(start_pos.x- end_pos.x)+ (start_pos.y- end_pos.y)*(start_pos.y- end_pos.y)));
    }

    std::vector<Behavior_Using_State> Game_Strategy::Whole_Behavior_To_Step(int person,int Robot,Position position,int behavior,float Accel,float Max_vel){
        // if(person != PERSON_RED){
        //     printf("whole_behavior_to_step error\n");
        // }

        std::vector<Position> end_point;
        std::vector<Behavior_Using_State> behavior_using;
        if(Robot == ROBOT_R1){
            //放
            for(int i = 0;i<3;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }
            //推
            for(int i = 0;i<9;++i){
                if(State_Search(behavior,i) == Push_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }            
            //等待合体
            for(int i = 6;i<9;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }

            if(end_point.size() == 0){
                behavior_using.push_back({0,0,0,0,EMPTY});
            }
            else if(end_point.size() == 1){
                //放置/推/等待合体都有可能
                for(int i=0;i<3;i++){
                    if(State_Search(behavior,i) == Place_Block){
                        behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_PLACE});
                    }
                }
                for(int i=0;i<9;i++){
                    if(State_Search(behavior,i) == Push_Block){
                        behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_PUSH});
                    }
                }
                for(int i=6;i<9;i++){
                    if(State_Search(behavior,i) == Place_Block){
                        behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_WAITING_FIT});
                    }
                }
                //可以加个放置时间
            }
            else if(end_point.size() == 2){
                int flag_R1_size2 = 0;
                for(int i=0;i<3;i++){
                    if(State_Search(behavior,i) == Place_Block){
                        behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_PLACE});
                        flag_R1_size2++;
                    }
                }
                if(flag_R1_size2 == 1){
                    for(int i=0;i<9;i++){//(放|推)
                        if(State_Search(behavior,i) == Push_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R1_PUSH});
                        }
                    }
                    for(int i=6;i<9;i++){//（放|合）
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R1_WAITING_FIT});
                        }
                    }
                }
                else{//(推|合)
                    for(int i=0;i<9;i++){
                        if(State_Search(behavior,i) == Push_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_PUSH});
                        }
                    }
                    for(int i=6;i<9;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R1_WAITING_FIT});
                        }
                    }
                }
                //这里可以加上一次合体和放置时间
            }
            else if(end_point.size() == 3){//正常顺序(放|推|合)，最优路径点可能是（放|推|合）|（推|放|合）
                if(Distance_Calc(position,end_point[0]) > Distance_Calc(position,end_point[1])){//运动到放置大于推时间（推|放|合）
                    for(int i=0;i<9;i++){
                        if(State_Search(behavior,i) == Push_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R1_PUSH});
                        }
                    }
                    for(int i=0;i<3;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[1],end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_PLACE});
                        }
                    }
                    for(int i=6;i<9;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[2]),Accel,Max_vel),end_point[2].x,end_point[2].y,i+1,R1_WAITING_FIT});
                        }
                    }
                }
                else{//(放|推|合)
                    for(int i=0;i<3;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R1_PLACE});
                        }
                    }
                    for(int i=0;i<9;i++){
                        if(State_Search(behavior,i) == Push_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R1_PUSH});
                        }
                    }
                    for(int i=6;i<9;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[1],end_point[2]),Accel,Max_vel),end_point[2].x,end_point[2].y,i+1,R1_WAITING_FIT});
                        }
                    }
                }
            }
        }
        else if(Robot == ROBOT_R2)
        {   
            int flag = 0;
            //放
            for(int i = 3;i<6;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                    flag++;
                }
            }
            for(int i = 6;i<9;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }

            if(end_point.size() == 0){//不执行
                behavior_using.push_back({0,position.x,position.y,0,EMPTY});
            }
            else if(end_point.size() == 1){
                if(flag == 0){//上
                    for(int i=6;i<9;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R2_PLACE_UPPER1});
                        }
                    }
                }
                else{//中
                    for(int i=3;i<6;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R2_PLACE_MIDDLE});
                        }
                    }
                }
            }
            else if(end_point.size() == 2){
                int flag_jungle_up1_or_up2 = 0;
                if(flag == 0){//上上
                    for(int i=6;i<9;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            if(flag_jungle_up1_or_up2==0){
                                behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R2_PLACE_UPPER1});
                                flag_jungle_up1_or_up2++;
                            }
                            else{
                                behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R2_PLACE_UPPER2});
                            }
                        }
                    }
                }
                else if(flag == 1){//中|上
                    for(int i=3;i<6;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R2_PLACE_MIDDLE});
                        }
                    }
                    for(int i=6;i<9;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R2_PLACE_UPPER1});
                        }
                    }
                }
                else if(flag == 2){//中|中
                    for(int i=3;i<6;i++){
                        if(State_Search(behavior,i) == Place_Block){
                            if(flag_jungle_up1_or_up2 == 0){
                                behavior_using.push_back({Path_To_Time(Distance_Calc(position,end_point[0]),Accel,Max_vel),end_point[0].x,end_point[0].y,i+1,R2_PLACE_MIDDLE});
                                flag_jungle_up1_or_up2++;
                            }
                            else{
                                behavior_using.push_back({Path_To_Time(Distance_Calc(end_point[0],end_point[1]),Accel,Max_vel),end_point[1].x,end_point[1].y,i+1,R2_PLACE_MIDDLE});                                
                            }
                        }
                    }                    
                }
            }
        }
        else{
            printf("EEE error!");
            behavior_using.push_back({0,0,0,0,EMPTY});
        }
        return behavior_using;
    }

    float Game_Strategy::Total_UsingTime_Calc(int behavior,Robot_State R1_state,Robot_State R2_state){
        float using_time = 0.0f;
        float using_time2 = 0.0f;
       
        int person;
        int Robot;

        if(R1_state.Person != R2_state.Person || R1_state.Robot == R2_state.Robot){
            printf("Robot not same Person or there are same robot\n");
        }
        else{
            person = R1_state.Person;
            Robot = ROBOT_R1;
        }
        std::vector<Position> end_point;

        if(Robot == ROBOT_R1){
            //放
            for(int i = 0;i<3;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }
            //推
            for(int i = 0;i<9;++i){
                if(State_Search(behavior,i) == Push_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }            

            /*以上为纯R1自身决策目标位置坐标点，目前只可能同一时间存在一个*/
            
            /*以下为R2放置上方方块需要R1移动到对应目标位置合体，考虑到R2可能决策放两块因此可能R1受R2影响而多0/1/2 个目标位置*/
            //等待合体
            for(int i = 6;i<9;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }

            //因此R1可能有0/1/2/3 个目标位置点
            /*枚举（有顺序）：//设位置点0，1，2
                0：返回0
                1：直接计算
                2：00 01 02 10 11 12 20 21 22 //必定存在一次合体，先移动到执行点位等待R2合体，然后运动到对应点位  
                3：001 002 010 012 020 021   |   101 102 110  112 120 121 |   201 202 210 212 220 221 //直接执行到R1单独操作的位置，然后合体，合体后R1带着R2运动到剩下两个点位
            */
            if(end_point.size() == 0){
                using_time = 0.0f;
            }
            else if(end_point.size() == 1){
                using_time = Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                //可以加个放置时间
            }
            else if(end_point.size() == 2){
                using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);
                //这里可以加上一次合体和放置时间
            }

            else if(end_point.size() == 3){
                if(end_point[0].y == End_pos[person-1][Robot-1][0].y){
                    if (end_point[1].y == end_point[0].y)
                    {
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[2]),R1_state.Accel,R1_state.Max_vel);
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[2].y == end_point[0].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);                        
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[1].y == End_pos[person-1][Robot-1][1].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[1],end_point[2]),R1_state.Accel,R1_state.Max_vel);                          
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[1].y == End_pos[person-1][Robot-1][2].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[2]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[2],end_point[1]),R1_state.Accel,R1_state.Max_vel);                          
                        //这里可以加上一次合体和放置时间
                    }                    
                }
                else if(end_point[0].y == End_pos[person-1][Robot-1][1].y){
                    if (end_point[1].y == end_point[0].y)
                    {
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[2]),R1_state.Accel,R1_state.Max_vel);
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[2].y == end_point[0].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);                        
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[1].y == End_pos[person-1][Robot-1][0].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[1],end_point[2]),R1_state.Accel,R1_state.Max_vel);                          
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[1].y == End_pos[person-1][Robot-1][2].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[2]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[2],end_point[1]),R1_state.Accel,R1_state.Max_vel);                          
                        //这里可以加上一次合体和放置时间
                    }    
                }
                else if(end_point[0].y == End_pos[person-1][Robot-1][2].y){
                    if (end_point[1].y == end_point[0].y)
                    {
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[2]),R1_state.Accel,R1_state.Max_vel);
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[2].y == end_point[0].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);                        
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[1].y == End_pos[person-1][Robot-1][0].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[2]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[2],end_point[1]),R1_state.Accel,R1_state.Max_vel);                          
                        //这里可以加上一次合体和放置时间
                    }
                    else if(end_point[1].y == End_pos[person-1][Robot-1][1].y){
                        using_time+= Path_To_Time(Distance_Calc(R1_state.now_Position,end_point[0]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R1_state.Accel,R1_state.Max_vel);
                        using_time+= Path_To_Time(Distance_Calc(end_point[1],end_point[2]),R1_state.Accel,R1_state.Max_vel);                          
                        //这里可以加上一次合体和放置时间
                    }                    
                }

            }
            Robot = ROBOT_R2;
        }
        else if (Robot == ROBOT_R2){
            end_point.clear();

            int flag=0;

            //放
            for(int i = 3;i<6;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                    flag++;
                }
            }
            for(int i = 6;i<9;++i){
                if(State_Search(behavior,i) == Place_Block){
                    end_point.push_back(End_pos[person-1][Robot-1][i]);
                }
            }            
            //____________________________________________________________________

            if(end_point.size() == 0){
                using_time = 0.0f;
            }
            else if(end_point.size() == 1){
                using_time2 = Path_To_Time(Distance_Calc(R2_state.now_Position,end_point[0]),R2_state.Accel,R2_state.Max_vel);
                //可以加合体等时间，直接计算R2运动到对应点的使用时间，因为R1使用时间考虑了合体后的运动时间，这个是单机器人运动时间计算，最后模拟训练时间按用时最长的机器人计算
            }
            else if(end_point.size() == 2){
                if(Distance_Calc(R2_state.now_Position,end_point[0]) > Distance_Calc(R2_state.now_Position,end_point[1])){
                    using_time2 += Path_To_Time(Distance_Calc(R2_state.now_Position,end_point[1]),R2_state.Accel,R2_state.Max_vel);
                    using_time2 += Path_To_Time(Distance_Calc(end_point[1],end_point[0]),R2_state.Accel,R2_state.Max_vel);
                }
                else{
                    using_time2 += Path_To_Time(Distance_Calc(R2_state.now_Position,end_point[0]),R2_state.Accel,R2_state.Max_vel);
                    using_time2 += Path_To_Time(Distance_Calc(end_point[0],end_point[1]),R2_state.Accel,R2_state.Max_vel);
                }
                //不管合体了，有合体基本上就是R1主导时间，最好消耗时间基本上是R1消耗的时间
            }
            //R2可能有0/1/2 个目标位置点
            /*枚举（有顺序）：//设位置点0，1，2
                0：返回0
                1：如果是3/4/5则直接计算，如果是6/7/8则看R1有无运动点，如果R1有运动点则时间等于（R1运动到第一个点的时间与R2运动到R1第一个点的时间取最大）+合体时间+ R1运动到这个点的时间
                2： 两中 34（01） 35（02） 45（12）  //不用合体 时间= pos 到最短运动距离点时间+到另一个点的时间
                    一中一上 36（00） 37（01） 38（02）  46（10）  47（11）  48（12）   56（20）  57（21）   58（22）//直接合体  再根据R1点位位置到最短点位移点时间+到另外点的时间
                    两上 67（01）  68（02）  78（12） //和一中一上一样
            */
        }
        return using_time > using_time2 ? using_time : using_time2;
    }

    int Game_Strategy::Set_State(int person,int robot,int behavior){
        int state = behavior;
        state = State_Change(state,ROBOT_POS_,robot);
        state = State_Change(state,PERSON_POS_,person);
        return state;
    }
    //这里的now_state是九宫格的情况
    Behavior_state_ Game_Strategy::Strategy_Control(Cost_Weight weight_,int now_state,Robot_State R1_state,Robot_State R2_state){
        Behavior_state_  state;
        int my_behavior = 0;
        int true_num = 0;//合理行为的数量
        float black_score = 0.0f;
        float site_score = 0.0f;
        float lost_score = 0.0f;
        float using_time = 0.0f;
        float my_using_time = 0.0f;
        float my_Score = 0.0f;
        float this_score = 0.0f;


        for (int i = 0;i<ALL_BEHAVIOR_SIZE;++i){
            if(Behavior_Jungle(now_state,All_Behavior[i])){
                if(true_num == 0){//第一次直接写入不进行比较
                    black_score = Total_Black_Scores_Calc(All_Behavior[i]);
                    site_score = Total_BlackSite_Scores_Calc(All_Behavior[i]);
                    lost_score = Total_LostScore_Calc(All_Behavior[i]);

                    using_time = Total_UsingTime_Calc(All_Behavior[i],R1_state, R2_state);

                    this_score = Calc_Score(weight_,black_score,site_score,lost_score,using_time);
                    
                    my_using_time = using_time;
                    my_Score = this_score;
                    my_behavior = All_Behavior[i];
                    
                    true_num++;
                }
                else
                {
                    black_score = Total_Black_Scores_Calc(All_Behavior[i]);
                    site_score = Total_BlackSite_Scores_Calc(All_Behavior[i]);
                    lost_score = Total_LostScore_Calc(All_Behavior[i]);
                    using_time = Total_UsingTime_Calc(All_Behavior[i],R1_state, R2_state);

                    this_score = Calc_Score(weight_,black_score,site_score,lost_score,using_time);

                    float delta = this_score - my_Score;

                    //视为相等，按照时间得分取行为
                    if (delta < EPS && delta > -EPS) {
                        if(Total_UsingTime_Calc(my_behavior,R1_state, R2_state) > using_time){
                            my_behavior = All_Behavior[i];
                            my_Score = this_score;
                            my_using_time = using_time;
                        }
                    }
                    //不等
                    else{
                        if(my_Score > this_score){
                            break;
                        }
                        else if(my_Score <= this_score){
                            my_Score = this_score;
                            my_behavior = All_Behavior[i];
                            my_using_time = using_time;
                        }

                    }
                    true_num++;
                }
            }
        }
        state.Behavior = my_behavior;
        state.sim_using_time = my_using_time;
        return state;
    }
    
    Position Game_Strategy::Time_To_Position(Position start_point,Position end_point,float time,float accel,float max_vel){
        float distance = Distance_Calc(start_point,end_point);
        float real_distance;
        
        float whole_time = Path_To_Time(distance,accel,max_vel);
        float time1 = max_vel/accel;

        float accel_distance = max_vel*max_vel/(2*accel);
        
        Position arrive_point;
        if (distance <= 0 || accel <= 0 || max_vel <= 0)
        {
            return {0.0f,0.0f};
        }

        if(accel_distance*2 < distance){//梯形
            real_distance = 1/2 * accel *time*time;

            if(real_distance< accel_distance){
                arrive_point.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                arrive_point.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);
            }
            else if(real_distance >accel_distance){
                float real_distance =  (time - time1)*max_vel+accel_distance;
                if(real_distance<distance-accel_distance){
                    arrive_point.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                    arrive_point.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);                    
                }
                else if(real_distance > distance-accel_distance){
                    real_distance = distance - ((whole_time - time)/time1)*accel_distance;
                    arrive_point.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                    arrive_point.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);   
                }
            }
        }
        else{//三角形
            if(time < whole_time/2){
                real_distance = (time/time1)*accel_distance;
                arrive_point.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                arrive_point.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);
            }
            else{
                real_distance = distance - (distance/2)*((whole_time-time)/(whole_time/2));
                arrive_point.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                arrive_point.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);            
            }
        }
        return arrive_point; 
    }

    float Game_Strategy::Path_To_Time(float distance,float accel,float max_vel){
        if (distance <= 0 || accel <= 0 || max_vel <= 0)
        {
            return 0.0f;
        }
        
        float t,t1; 
        t1 = max_vel/accel;

        if(t1*max_vel < distance){//足够梯形速度规划
            t = distance/max_vel + t1;
        }
        else {
            t = sqrt((2.0f * distance) / accel);
        }
        return t;
    }

    float Game_Strategy::Calc_Score(Cost_Weight weight,float Total_Black_Scores,float Total_BlackSite_Scores,float Total_LostScore,float Total_UsingTime){        
        return (weight.Q1*Total_Black_Scores + weight.Q2*Total_BlackSite_Scores+weight.Q3*Total_LostScore - weight.Q4*Total_UsingTime)/100.0f;
    }

    void Game_Strategy::Print_ALL_Behavior(){
        for(int i = 0; i < ALL_BEHAVIOR_SIZE; ++i){ 
            Print_binary_System(All_Behavior[i]);
        }
    }


    void Best_Weigt_Finding::Set_Black_Num(int Our_R1_Num,int Our_R2_Num,int our_weapon,int Opp_R1_Num,int Opp_R2_Num,int Opp_weapon){
        this->Our_R1_Black_Num = Our_R1_Num;
        this->Our_R2_Black_Num = Our_R2_Num;
        this->Our_weapon_Num = our_weapon;
        this->Opp_R1_Black_Num = Opp_R1_Num;
        this->Opp_R2_Black_Num = Opp_R2_Num;
        this->Opp_weapon_Num = Opp_weapon;
    }



    std::vector<Strategy::Cost_Weight> Best_Weigt_Finding::ALL_Weight_State;
    Best_Weigt_Finding::Best_Weigt_Finding(float Our_R1_max_vel,float Our_R1_accel,
                                            float Our_R2_max_vel,float Our_R2_accel,
                                            float Opp_Base_R1_max_vel,float Opp_Base_R1_accel,
                                            float Opp_Base_R2_max_vel,float Opp_Base_R2_accel,float DeadLine_Time_,int Sim_num_,
                                            Range Accel_randon_range_,Range Max_vel_randon_range_,Range DeadLine_randon_range_,Range Start_Pos_randon_range_){
        this->Opp_R1.Max_vel = Opp_Base_R1_max_vel;
        this->Opp_R1.Accel = Opp_Base_R1_accel;
        this->Opp_R1.now_Position = {2.75f,1.25f};
        this->Opp_R1.Person = PERSON_BLUE;
        this->Opp_R1.Robot = ROBOT_R1;

        this->Opp_R2.Max_vel = Opp_Base_R2_max_vel;
        this->Opp_R2.Accel = Opp_Base_R2_accel;
        this->Opp_R2.now_Position = {1.75f,1.25f};
        this->Opp_R2.Person = PERSON_BLUE;
        this->Opp_R2.Robot = ROBOT_R2;

        this->Our_R1.Max_vel = Our_R1_max_vel;
        this->Our_R1.Accel = Our_R1_accel;
        this->Our_R1.now_Position = {9.25f,1.25f};
        this->Our_R1.Person = PERSON_RED;
        this->Our_R1.Robot = ROBOT_R1;

        this->Our_R2.Max_vel = Our_R2_max_vel;
        this->Our_R2.Accel = Our_R2_accel;
        this->Our_R2.now_Position = {10.25f,1.25f};
        this->Our_R2.Person = PERSON_RED;
        this->Our_R2.Robot = ROBOT_R2;

        Sim_num = 100;
        
        DeadLine_Time = DeadLine_Time_;//截至时间秒数

        Accel_randon_range = Accel_randon_range_;
        Max_vel_randon_range = Max_vel_randon_range_;
        DeadLine_randon_range = DeadLine_randon_range_;
        Start_Pos_randon_range = Start_Pos_randon_range_;
        
        Opp_weight[0] = {100,5,5,5}; // 得分更激进
        Opp_weight[1] = {5,5,5,100}; // 速度更激进
        Opp_weight[2] = {40,40,40,40}; // 均衡但更强

        //存储所有weight，用于寻找最优weight
        for(int a = 1; a <= 97;++a){
            for(int b = 1;b<=97-a;++b){
                for(int c = 1;c<= 97-a-b;++c){
                    int d = 100 - a - b - c;
                    if(d>=1){
                        ALL_Weight_State.push_back(Strategy::Cost_Weight{a,b,c,d});
                    }
                }
            }
        }
    }
    


    void Best_Weigt_Finding::State_clear(){
        Our_now_State = 0;
        Opp_now_State = 0;
    }

    void Best_Weigt_Finding::Update_OPP_Robot_State(){
        State_clear();
        //更新
        Our_now_State = State_Change(Our_now_State,R1_USED,Reset_Our_R1_Black_Num);
        Our_now_State = State_Change(Our_now_State,R2_USED,Reset_Our_R2_Black_Num);
        Our_now_State = State_Change(Our_now_State,WEAPON_USED,Reset_Our_weapon_Num);       
        Opp_now_State = State_Change(Opp_now_State,R1_USED,Reset_Opp_R1_Black_Num);
        Opp_now_State = State_Change(Opp_now_State,R2_USED,Reset_Opp_R2_Black_Num);
        Opp_now_State = State_Change(Opp_now_State,WEAPON_USED,Reset_Opp_weapon_Num);

        //随机噪声更新
        this->Our_R1_start_Offset_Pos.x = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);
        this->Our_R1_start_Offset_Pos.y = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);
        this->Our_R2_start_Offset_Pos.x = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);
        this->Our_R2_start_Offset_Pos.y = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);

        this->Opp_R1_start_Offset_Pos.x = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);
        this->Opp_R1_start_Offset_Pos.y = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);
        this->Opp_R2_start_Offset_Pos.x = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);
        this->Opp_R2_start_Offset_Pos.y = RandFloat(this->Start_Pos_randon_range.min,this->Start_Pos_randon_range.max);

        this->DeadLine_Time_offset = RandFloat(this->DeadLine_randon_range.min,this->DeadLine_randon_range.max);

        this->Opp_R1_accel_offset = RandFloat(this->Accel_randon_range.min,this->Accel_randon_range.max);
        this->Opp_R2_accel_offset = RandFloat(this->Accel_randon_range.min,this->Accel_randon_range.max);
        this->Opp_R1_max_vel_offset = RandFloat(this->Max_vel_randon_range.min,this->Max_vel_randon_range.max);
        this->Opp_R2_max_vel_offset = RandFloat(this->Max_vel_randon_range.min,this->Max_vel_randon_range.max);

    }
    
    
    Behavior_Using_State Best_Weigt_Finding::Find_mining_time_behavior(
        Behavior_Using_State  Our_R1_,
        Behavior_Using_State  Our_R2_,
        Behavior_Using_State  Opp_R1_,
        Behavior_Using_State  Opp_R2_) 
    {   
        
        std::vector<Behavior_Using_State> candidates = {Our_R1_, Our_R2_, Opp_R1_, Opp_R2_};
        float min_time = 1e9f;
        Behavior_Using_State best = {0,0,0,0,EMPTY};
        bool has_valid = false;

        for (auto &b : candidates) {
            // 只有非空行为且时间大于0才考虑
            if (b.using_time > 1e-6f) {
                has_valid = true;
                if (b.using_time < min_time) {
                    min_time = b.using_time;
                    best = b;
                }
            }
        }

        if (!has_valid) {
            // 所有机器人均无有效行为，游戏应结束
            best.using_time = 0.0f;
        }
        return best;
    }
    int Best_Weigt_Finding::nine_square_update(int nine_square,int person,Behavior_Using_State state){
        if(person == PERSON_RED){            
            if(state.Behavior == EMPTY){
                return nine_square;
            }
            else if(state.Behavior == R1_PLACE){
                Our_now_State = State_Change(Our_now_State,R1_USED,State_Search(Our_now_State,R1_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OurBlack);
            }
            else if(state.Behavior == R1_PUSH){
                Our_now_State = State_Change(Our_now_State,WEAPON_USED,State_Search(Our_now_State,WEAPON_USED)-1);
                return State_Change(nine_square,state.doing_place-1,Empty);
            }
            else if(state.Behavior == R1_WAITING_FIT){
                return nine_square;
            }
            else if(state.Behavior == R2_PLACE_MIDDLE){
                Our_now_State = State_Change(Our_now_State,R2_USED,State_Search(Our_now_State,R2_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OurBlack);
            }
            else if(state.Behavior == R2_PLACE_UPPER1){
                Our_now_State = State_Change(Our_now_State,R2_USED,State_Search(Our_now_State,R2_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OurBlack);
            }
            else if(state.Behavior == R2_PLACE_UPPER2){
                Our_now_State = State_Change(Our_now_State,R2_USED,State_Search(Our_now_State,R2_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OurBlack);
            }        
        }
        else if(person == PERSON_BLUE)
        {
            if(state.Behavior == EMPTY){
                return nine_square;
            }
            else if(state.Behavior == R1_PLACE){
                Opp_now_State = State_Change(Opp_now_State,R1_USED,State_Search(Opp_now_State,R1_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OppBlack);
            }
            else if(state.Behavior == R1_PUSH){
                Opp_now_State = State_Change(Opp_now_State,WEAPON_USED,State_Search(Opp_now_State,WEAPON_USED)-1);
                return State_Change(nine_square,state.doing_place-1,Empty);
            }
            else if(state.Behavior == R1_WAITING_FIT){
                return nine_square;
            }
            else if(state.Behavior == R2_PLACE_MIDDLE){
                Opp_now_State = State_Change(Opp_now_State,R2_USED,State_Search(Opp_now_State,R2_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OppBlack);
            }
            else if(state.Behavior == R2_PLACE_UPPER1){
                Opp_now_State = State_Change(Opp_now_State,R2_USED,State_Search(Opp_now_State,R2_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OppBlack);
            }
            else if(state.Behavior == R2_PLACE_UPPER2){
                Opp_now_State = State_Change(Opp_now_State,R2_USED,State_Search(Opp_now_State,R2_USED)-1);
                return State_Change(nine_square,state.doing_place-1,OppBlack);
            } 
        }
        return nine_square;
    }

    int Best_Weigt_Finding::Process_winner_jungle(int Nine_Square_state_){
        if(State_Search(Nine_Square_state_,0) == OurBlack && State_Search(Nine_Square_state_,3) == OurBlack && State_Search(Nine_Square_state_,6) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,0) == OppBlack && State_Search(Nine_Square_state_,3) == OppBlack && State_Search(Nine_Square_state_,6) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,1) == OurBlack && State_Search(Nine_Square_state_,4) == OurBlack && State_Search(Nine_Square_state_,7) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,1) == OppBlack && State_Search(Nine_Square_state_,4) == OppBlack && State_Search(Nine_Square_state_,7) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,2) == OurBlack && State_Search(Nine_Square_state_,5) == OurBlack && State_Search(Nine_Square_state_,8) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,2) == OppBlack && State_Search(Nine_Square_state_,5) == OppBlack && State_Search(Nine_Square_state_,8) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,0) == OurBlack && State_Search(Nine_Square_state_,4) == OurBlack && State_Search(Nine_Square_state_,8) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,0) == OppBlack && State_Search(Nine_Square_state_,4) == OppBlack && State_Search(Nine_Square_state_,8) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,2) == OurBlack && State_Search(Nine_Square_state_,4) == OurBlack && State_Search(Nine_Square_state_,6) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,2) == OppBlack && State_Search(Nine_Square_state_,4) == OppBlack && State_Search(Nine_Square_state_,6) == OppBlack){
            return PERSON_BLUE;
        }
        else{
            return 0;
        }

    }
    int Best_Weigt_Finding::jungle_winner(int Nine_Square_state_){
        if(State_Search(Nine_Square_state_,0) == OurBlack && State_Search(Nine_Square_state_,3) == OurBlack && State_Search(Nine_Square_state_,6) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,0) == OppBlack && State_Search(Nine_Square_state_,3) == OppBlack && State_Search(Nine_Square_state_,6) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,1) == OurBlack && State_Search(Nine_Square_state_,4) == OurBlack && State_Search(Nine_Square_state_,7) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,1) == OppBlack && State_Search(Nine_Square_state_,4) == OppBlack && State_Search(Nine_Square_state_,7) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,2) == OurBlack && State_Search(Nine_Square_state_,5) == OurBlack && State_Search(Nine_Square_state_,8) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,2) == OppBlack && State_Search(Nine_Square_state_,5) == OppBlack && State_Search(Nine_Square_state_,8) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,0) == OurBlack && State_Search(Nine_Square_state_,4) == OurBlack && State_Search(Nine_Square_state_,8) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,0) == OppBlack && State_Search(Nine_Square_state_,4) == OppBlack && State_Search(Nine_Square_state_,8) == OppBlack){
            return PERSON_BLUE;
        }
        else if(State_Search(Nine_Square_state_,2) == OurBlack && State_Search(Nine_Square_state_,4) == OurBlack && State_Search(Nine_Square_state_,6) == OurBlack){
            return PERSON_RED;
        }
        else if(State_Search(Nine_Square_state_,2) == OppBlack && State_Search(Nine_Square_state_,4) == OppBlack && State_Search(Nine_Square_state_,6) == OppBlack){
            return PERSON_BLUE;
        }

        int Red_total_score = 0;
        int blue_total_score = 0;
        for(int i = 0;i<3;++i){
            if(State_Search(Nine_Square_state_,i) == OurBlack){
                Red_total_score+=40;
            }
            else if(State_Search(Nine_Square_state_,i) == OppBlack){
                blue_total_score+=40;
            }
        }
        for(int i = 3;i<6;++i){
            if(State_Search(Nine_Square_state_,i) == OurBlack){
                Red_total_score+=70;
            }
            else if(State_Search(Nine_Square_state_,i) == OppBlack){
                blue_total_score+=70;
            }
        }
        for(int i = 6;i<9;++i){
            if(State_Search(Nine_Square_state_,i) == OurBlack){
                Red_total_score+=100;
            }
            else if(State_Search(Nine_Square_state_,i) == OppBlack){
                blue_total_score+=100;
            }
        }
        return Red_total_score>blue_total_score ? PERSON_RED:PERSON_BLUE;
        
    }

    void Best_Weigt_Finding::Main_Runing(){
        this->Reset_Our_R1_Black_Num = this->Our_R1_Black_Num;
        this->Reset_Our_R2_Black_Num = this->Our_R2_Black_Num;
        this->Reset_Our_weapon_Num = this->Our_weapon_Num;
        this->Reset_Opp_R1_Black_Num = this->Opp_R1_Black_Num;
        this->Reset_Opp_R2_Black_Num = this->Opp_R2_Black_Num;
        this->Reset_Opp_weapon_Num = this->Opp_weapon_Num;
        Position reset_Our_R1_pos = this->Our_R1.now_Position;
        Position reset_Our_R2_pos = this->Our_R2.now_Position; 
        Position reset_Opp_R1_pos = this->Opp_R1.now_Position;
        Position reset_Opp_R2_pos = this->Opp_R2.now_Position;
        
        is_init_robot = 1;
        float wining_NUM = 0.0f;
        int flag1 = 0;
        int Nine_Square_state = 0;
        float total_time = 0;
        float last_total_time = 0;
        float wining_Rate = 0;
        float Last_wining_Rate = 0;
        float best_winning_rate = 0.0f;
        Cost_Weight best_weight = {0,0,0,0};

        Behavior_state_ Our_state;
        Behavior_state_ Opp_state;
        int num = 0;
        Update_OPP_Robot_State();
        for(int i=0; i < ALL_Weight_State.size();++i){
            for (int k =0; k<this->Sim_num;++k){
                for(int j=0;j<sizeof(Opp_weight)/sizeof(Cost_Weight);++j){
                    while(total_time < DeadLine_Time + DeadLine_Time_offset){
                        int person = 0;
                        Our_now_State = (Our_now_State & ~0x3FFFF) | (Nine_Square_state & 0x3FFFF);
                        Opp_now_State = (Opp_now_State & ~0x3FFFF) | (Nine_Square_state & 0x3FFFF);

                        Our_state = Strategy_Control(ALL_Weight_State[i],Our_now_State,Our_R1,Our_R2);
                        Opp_state = Strategy_Control(Opp_weight[j],Opp_now_State,Opp_R1,Opp_R2);
                        std::vector<Behavior_Using_State> Our_R1_ = Whole_Behavior_To_Step(Our_R1.Person,Our_R1.Robot,Our_R1.now_Position,Our_state.Behavior,Our_R1.Accel,Our_R1.Max_vel);
                        std::vector<Behavior_Using_State> Our_R2_ = Whole_Behavior_To_Step(Our_R2.Person,Our_R2.Robot,Our_R2.now_Position,Our_state.Behavior,Our_R2.Accel,Our_R2.Max_vel);
                        std::vector<Behavior_Using_State> Opp_R1_ = Whole_Behavior_To_Step(Opp_R1.Person,Opp_R1.Robot,Opp_R1.now_Position,Opp_state.Behavior,Opp_R1.Accel,Opp_R1.Max_vel);
                        std::vector<Behavior_Using_State> Opp_R2_ = Whole_Behavior_To_Step(Opp_R2.Person,Opp_R2.Robot,Opp_R2.now_Position,Opp_state.Behavior,Opp_R2.Accel,Opp_R2.Max_vel);
                        
                        int size = Our_R1_.size() + Our_R2_.size()+Opp_R1_.size() + Opp_R2_.size();
                        int size1,size2,size3,size4;
                        Behavior_Using_State min_time_behavior;
                        size1 = 0;
                        size2 = 0;
                        size3 = 0;
                        size4 = 0;
                        for (int m = 0; m < size; ++m)
                        {   
                            min_time_behavior = Find_mining_time_behavior(Our_R1_[size1],Our_R2_[size2],Opp_R1_[size3],Opp_R2_[size4]);
                    
                            Our_R1.now_Position = Time_To_Position(Our_R1.now_Position,{Our_R1_[size1].x,Our_R1_[size1].y},min_time_behavior.using_time,Our_R1.Accel,Our_R1.Max_vel);
                            Our_R2.now_Position = Time_To_Position(Our_R2.now_Position,{Our_R2_[size2].x,Our_R2_[size2].y},min_time_behavior.using_time,Our_R2.Accel,Our_R2.Max_vel);
                            Opp_R1.now_Position = Time_To_Position(Opp_R1.now_Position,{Opp_R1_[size3].x,Opp_R1_[size3].y},min_time_behavior.using_time,Opp_R1.Accel,Opp_R1.Max_vel);
                            Opp_R2.now_Position = Time_To_Position(Opp_R2.now_Position,{Opp_R2_[size4].x,Opp_R2_[size4].y},min_time_behavior.using_time,Opp_R2.Accel,Opp_R2.Max_vel);                            
                            if(min_time_behavior.using_time == Our_R1_[size1].using_time && min_time_behavior.x == Our_R1_[size1].x && min_time_behavior.y == Our_R1_[size1].y){
                                person = Our_R1.Person;
                                if(size1<Our_R1_.size()-1){
                                    size1++;
                                }
                            }
                            else if(min_time_behavior.using_time == Our_R2_[size2].using_time && min_time_behavior.x == Our_R2_[size2].x && min_time_behavior.y == Our_R2_[size2].y){
                                person = Our_R2.Person;
                                if(size2<Our_R2_.size()-1){
                                    size2++;
                                }
                            }
                            else if(min_time_behavior.using_time == Opp_R1_[size3].using_time && min_time_behavior.x == Opp_R1_[size3].x && min_time_behavior.y == Opp_R1_[size3].y){
                                person = Opp_R1.Person;
                                if(size3<Opp_R1_.size()-1){
                                    size3++;
                                }
                            }
                            else if(min_time_behavior.using_time == Opp_R2_[size4].using_time && min_time_behavior.x == Opp_R2_[size4].x && min_time_behavior.y == Opp_R2_[size4].y){
                                person = Opp_R2.Person;
                                if(size4<Opp_R2_.size()-1){
                                    size4++;
                                }
                            }
                            Nine_Square_state = nine_square_update(Nine_Square_state,person,min_time_behavior);
                                                        
                            total_time += min_time_behavior.using_time;
                            if(Process_winner_jungle(Nine_Square_state) == PERSON_RED){
                                wining_NUM++;
                                flag1 = 1;
                                //跳出循环
                                break;
                            }
                            else if(Process_winner_jungle(Nine_Square_state) == PERSON_BLUE){
                                //跳出循环
                                flag1 =1;
                                break;
                            }
                            else if(State_Search(Our_now_State,R1_USED) == 0 && State_Search(Our_now_State,R2_USED) == 0 
                                    && State_Search(Opp_now_State,R1_USED) == 0 && State_Search(Opp_now_State,R2_USED) == 0)
                            {
                                flag1 = 1;
                                break;
                            }
                        }
                        if(Process_winner_jungle(Nine_Square_state) == PERSON_RED){
                            //跳出循环
                            break;
                        }
                        else if(Process_winner_jungle(Nine_Square_state) == PERSON_BLUE){
                            //跳出循环
                            break;
                        }
                        else if(State_Search(Our_now_State,R1_USED) == 0 && State_Search(Our_now_State,R2_USED) == 0 
                                && State_Search(Opp_now_State,R1_USED) == 0 && State_Search(Opp_now_State,R2_USED) == 0)
                        {
                            break;
                        }
                    }
                    if(flag1 != 1){
                        if(jungle_winner(Nine_Square_state) == PERSON_RED){
                            wining_NUM++;
                        }
                        flag1 = 0;
                    }
                    else{
                        flag1 = 0;
                    }
                    Nine_Square_state = 0;
                    this->Our_R1_Black_Num = this->Reset_Our_R1_Black_Num;
                    this->Our_R2_Black_Num = this->Reset_Our_R2_Black_Num;
                    this->Our_weapon_Num = this->Reset_Our_weapon_Num ;
                    this->Opp_R1_Black_Num = this->Reset_Opp_R1_Black_Num;
                    this->Opp_R2_Black_Num = this->Reset_Opp_R2_Black_Num;
                    this->Opp_weapon_Num = this->Reset_Opp_weapon_Num;

                    this->Our_R1.now_Position = reset_Our_R1_pos;
                    this->Our_R2.now_Position = reset_Our_R2_pos; 
                    this->Opp_R1.now_Position = reset_Opp_R1_pos;
                    this->Opp_R2.now_Position = reset_Opp_R2_pos;
                    total_time = 0.0f;
                    Update_OPP_Robot_State();
                    num++;
                }
            }
            wining_Rate = wining_NUM/(Sim_num*sizeof(Opp_weight)/sizeof(Cost_Weight));
            
            // 调试打印
            printf("weight %d: wining_NUM=%d, Sim_num=%d, Opp_types=%d, wining_Rate=%f\n", i, (int)wining_NUM, Sim_num, (int)(sizeof(Opp_weight)/sizeof(Cost_Weight)), wining_Rate);
            
            if(wining_Rate >= Last_wining_Rate && wining_Rate!= 1.0f){
                best_winning_rate = wining_Rate;
                best_weight = ALL_Weight_State[i];
            }
            Last_wining_Rate = best_winning_rate;
            wining_NUM = 0;
            printf("now param:%f,%f,%f,%f\n",best_weight.Q1,best_weight.Q2,best_weight.Q3,best_weight.Q4);
            printf("now win rate:%f\n",best_winning_rate);

        }
        printf("best param:%d,%d,%d,%d\n",best_weight.Q1,best_weight.Q2,best_weight.Q3,best_weight.Q4);
        printf("best win rate:%f\n",best_winning_rate);
    
    }
    
    //左闭右开
    float RandFloat(float min,float max){
        static std::mt19937 gen(std::random_device{}());
        return std::uniform_real_distribution<float>(min,max)(gen);
    }

    //索引对应九宫格状态
    int State_Search(int num,int pos){
        return ((0x03 << (2*pos)) & num) >> (2*pos);
    }

    //更改对应九宫格状态
    int State_Change(int num, int pos, int input) {
        int mask = ~(0x03 << (2 * pos));
        int out = (num & mask) | ((input & 0x03) << (2 * pos));
        return out;
    }

    //二进制打印数据
    template<typename Num_Type>
    void Print_binary_System(Num_Type num){
        for (int i = 31; i>=0;--i){
            printf("%d",(num >> i)&1);
        }
        printf("\n");
    }
    


    /*
    新想法：
    九宫格用一个数组来表示
    不考虑整体了，就两个机器人，各自决策自己的行为，最后发送合体指令再合体
    整体同步进行的部分就写死就可以了
    
    */

}


