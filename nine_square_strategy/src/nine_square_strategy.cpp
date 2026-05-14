#include "../include/nine_square_strategy/nine_square_strategy.h"
#include "ros/ros.h"

namespace Nine_Square_Strategy{
    //生成随机浮点数,左闭右开
    float Random_Float(float min,float max){
         std::mt19937 gen(std::random_device{}());
        return std::uniform_real_distribution<float>(min,max)(gen);
    }
    //向量归一化
    Position Dir_Normalization(Position dir){
        float length = sqrt(dir.x*dir.x + dir.y*dir.y);
        if(length == 0){
            return {0.0f,0.0f};
        }
        Position norm_dir;
        norm_dir.x = dir.x/length;
        norm_dir.y = dir.y/length;
        return norm_dir;
    }
    //根据起始位置终点位置计算直线距离
    float Line_Distance_Calc(Position start_pos,Position end_pos){
        return (sqrt((start_pos.x- end_pos.x)*(start_pos.x- end_pos.x)+ (start_pos.y- end_pos.y)*(start_pos.y- end_pos.y)));
    }
    //根据路径总路程，按照梯形速度规划计算时间
    float LinePath_To_Time(float distance,float accel,float max_vel){
        if (!isfinite(distance) || distance < 0) distance = 0.0f;
        if (!isfinite(accel) || accel <= 0) accel = 0.1f;
        if (!isfinite(max_vel) || max_vel <= 0) max_vel = 0.1f;
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
    //已知起始位置、终点位置、加速度和最大速度，根据花费时间计算对应位置，速度大小，方向
    Pos_Vel_Dir Line_Time_To_Position(Position start_point,Position end_point,float time,float accel,float max_vel){
        Pos_Vel_Dir state = {{0.0f,0.0f},{0.0f,0.0f},0.0f};
        Pos_Vel_Dir output;
        float distance = Line_Distance_Calc(start_point,end_point);
        float real_distance;    
        float whole_time = LinePath_To_Time(distance,accel,max_vel);
        float time1 = max_vel/accel;
        float accel_distance = max_vel*max_vel/(2*accel);
        if (distance <= 0 || accel <= 0 || max_vel <= 0)
        {
            LOG_ERROR("Time_To_Position: Invalid input parameters.");
            output = state;
            return output;
        }

        if(accel_distance*2 < distance){//梯形
            real_distance = 1.0f/2.0f * accel *time*time;

            //上升沿
            if(real_distance< accel_distance){
                
                state.pos.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                state.pos.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);
                state.vel = accel*time;
                state.dir = Dir_Normalization(end_point - start_point);
            }
            else if(real_distance >accel_distance){
                float real_distance =  (time - time1)*max_vel+accel_distance;
                //平稳沿
                if(real_distance<distance-accel_distance){
                    
                    state.pos.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                    state.pos.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);                    
                    state.vel = max_vel;
                    state.dir = Dir_Normalization(end_point - start_point);
                
                }
                //下降沿
                else if(real_distance > distance-accel_distance){
                    real_distance = distance - ((whole_time - time)/time1)*accel_distance;
                    
                    state.pos.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                    state.pos.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);   
                    state.vel = accel*(whole_time - time);
                    state.dir = Dir_Normalization(end_point - start_point);
                }
            }
        }
        else{//三角形
            //上升沿
            if(time < whole_time/2){
                real_distance = (time/time1)*accel_distance;
                
                state.pos.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                state.pos.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);
                state.vel = accel*time;
                state.dir = Dir_Normalization(end_point - start_point);
            }
            //下降沿
            else{
                real_distance = distance - (distance/2.0f)*((whole_time-time)/(whole_time/2.0f));
                
                state.pos.x = start_point.x + (real_distance/distance)*(end_point.x -start_point.x);
                state.pos.y = start_point.y + (real_distance/distance)*(end_point.y -start_point.y);            
                state.vel = accel*(whole_time - time);
                state.dir = Dir_Normalization(end_point - start_point);
            }
        }
        output = state;
        return output; 
    }
    //随机生成九宫格状态
    int CLASS_Nine_Square_Tool::Function_Random_Nine_Square_State(){
        int state = 0;
        for (int i = 0; i < 9; ++i)
        {
            int cell_state = std::rand() % 5; // 0-4分别对应五种状态
            state |= (cell_state << (3 * i)); // 每三位表示一个格子的状态
        }
        return state;
    }
    //直观打印九宫格状况
    void CLASS_Nine_Square_Tool::Function_Print_Nine_Square_State(int nine_square_state){
        printf("-------------------\n");
        for (int i = 2; i >= 0; --i)
        {
            for(int j = 2; j >= 0; --j){
                printf("|");
                int cell_state = Function_State_Search(&nine_square_state,i*3+j);
                switch (cell_state)
                {
                case SQUARE_INVALID:
                    printf("INVALID");
                    break;                
                case SQUARE_EMPTY:
                    printf("EMPTY");
                    break;
                case SQUARE_RED_R1_BLACK:
                    printf("R__R1");
                    break;
                case SQUARE_RED_R2_BLACK:
                    printf("R__R2");
                    break;
                case SQUARE_BLUE_R1_BLACK:
                    printf("B__R1");
                    break;
                case SQUARE_BLUE_R2_BLACK:
                    printf("B__R2");
                    break;
                default:
                    LOG_ERROR("Function_Print_Nine_Square_State: Invalid cell state.");
                    printf(" ? ");
                    break;
                }
                if(j == 0){
                    printf("|\n");
                }
            }
        }
        printf("-------------------\n");
    }
    //3*3二维数组转int类型九宫格状态
    int CLASS_Nine_Square_Tool::Function_2DArray_To_Int(int nine_square_array[3][3]){
        int state = 0;
        for (int i = 2; i >= 0; --i)
        {
            for (int j = 2; j >= 0; --j)
            {
                if(nine_square_array[i][j]>=0 && nine_square_array[i][j] <= 5){
                    state |= (nine_square_array[2-i][j] << (3 * (i*3+2-j))); // 每三位表示一个格子的状态
                }
                else{
                    LOG_ERROR("Function_2DArray_To_Int: Invalid cell state in array at position (" + std::to_string(i) + "," + std::to_string(j) + ")");
                    return -1; // 返回-1表示输入数组中有无效状态
                }
            }
        }
        return state;
    }
    //9位数组转int类型九宫格状态，逆序
    int CLASS_Nine_Square_Tool::Function_Reverse_Vector_To_Int(int nine_square_vector[9]){
        int state = 0;
        for (int i = 8; i >= 0; --i)
        {
            if(nine_square_vector[i]>=0 && nine_square_vector[i] <= 5){
                state |= (nine_square_vector[i] << (3 * (8 - i))); // 每三位表示一个格子的状态
            }
            else{
                LOG_ERROR("Function_Reverse_Vector_To_Int: Invalid cell state in vector at index " + std::to_string(i));
                return -1; // 返回-1表示输入向量中有无效状态
            }
        }
        return state;
    }
    //9位数组转int类型九宫格状态，正序
    int CLASS_Nine_Square_Tool::Function_Vector_To_Int(int nine_square_vector[9]){
        int state = 0;
        for (int i = 0; i < 9; ++i)
        {
            if(nine_square_vector[i]>=0 && nine_square_vector[i] <= 5){
                state |= (nine_square_vector[i] << (3 * i)); // 每三位表示一个格子的状态
            }
            else{
                LOG_ERROR("Function_Vector_To_Int: Invalid cell state in vector at index " + std::to_string(i));
                return -1; // 返回-1表示输入向量中有无效状态
            }
        }
        return state;
    }
    //KFC持有数量判断是否合理
    bool CLASS_Nine_Square_Tool::Function_KFC_Having_Judge(TYPE_KFC_Having_State having_state){
        if(having_state.R1_KFC_Num < 0 || having_state.R1_KFC_Num > MAX_R1_KFC_NUM){
            LOG_ERROR("Function_KFC_Having_Judge: Invalid R1 KFC number.");
            return false;
        }
        if(having_state.R2_KFC_Num < 0 || having_state.R2_KFC_Num > MAX_R2_KFC_NUM){
            LOG_ERROR("Function_KFC_Having_Judge: Invalid R2 KFC number.");
            return false;
        }
        return true;
    }
    //九宫格红蓝双方各自总得分
    TYPE_Score_State CLASS_Nine_Square_Tool::Function_Nine_Square_Score_State(int nine_square_state){
        int nine_square_vector[9];
        for(int i = 0; i < 9; ++i){
            nine_square_vector[i] = Function_Cell_Player_Jungle(&nine_square_state,i);
        }
        TYPE_Score_State score_state = {0,0};
        for(int i = 0; i < 9; ++i){
            if(nine_square_vector[i] == PLAYER_RED_PLAYER){
                if(i/3 == 2){
                    score_state.red_score += SCORE_BOTTOM_R1_KFC;
                }
                else if(i/3 == 1){
                    score_state.red_score += SCORE_MIDDLE_R1_KFC;
                }
                else if(i/3 == 0){
                    score_state.red_score += SCORE_UPPER_R1_KFC;
                }
            }
            else if(nine_square_vector[i] == PLAYER_BLUE_PLAYER){
                if(i/3 == 2){
                    score_state.blue_score += SCORE_BOTTOM_R1_KFC;
                }
                else if(i/3 == 1){
                    score_state.blue_score += SCORE_MIDDLE_R1_KFC;
                }
                else if(i/3 == 0){
                    score_state.blue_score += SCORE_UPPER_R1_KFC;
                }
            }
        }
        return score_state;
    }
    //判断九宫格即将大胜,返回值包含是否即将大胜，谁将赢，再下哪个位置他就会赢(可能有多个位置)
    TYPE_Will_Win_State CLASS_Nine_Square_Tool::Function_Will_Win(int nine_square_state){
        int nine_square_vector[9];
        for(int i = 0; i < 9; ++i){
            nine_square_vector[i] = Function_Cell_Player_Jungle(&nine_square_state,i);
        }
        TYPE_Will_Win_State state;
        //竖将大胜
        for (int i = 0; i < 3; i++)
        {
            //第一种情况，顶部为空
            if(nine_square_vector[i] == PLAYER_RED_PLAYER && nine_square_vector[i+3] == PLAYER_RED_PLAYER && nine_square_vector[i+6] == SQUARE_EMPTY){
                state.player = PLAYER_RED_PLAYER;
                state.pos.push_back(i+6);
            }
            else if(nine_square_vector[i] == PLAYER_BLUE_PLAYER && nine_square_vector[i+3] == PLAYER_BLUE_PLAYER && nine_square_vector[i+6] == SQUARE_EMPTY){
                state.player = PLAYER_BLUE_PLAYER;
                state.pos.push_back(i+6);
            }
            //第二种情况，中部为空
            else if(nine_square_vector[i] == PLAYER_RED_PLAYER && nine_square_vector[i+3] == SQUARE_EMPTY && nine_square_vector[i+6] == PLAYER_RED_PLAYER){
                state.player = PLAYER_RED_PLAYER;
                state.pos.push_back(i+3);
            }
            else if(nine_square_vector[i] == PLAYER_BLUE_PLAYER && nine_square_vector[i+3] == SQUARE_EMPTY && nine_square_vector[i+6] == PLAYER_BLUE_PLAYER){
                state.player = PLAYER_BLUE_PLAYER;
                state.pos.push_back(i+3);
            }
            //第三种情况，底部为空
            else if(nine_square_vector[i] == SQUARE_EMPTY && nine_square_vector[i+3] == PLAYER_RED_PLAYER && nine_square_vector[i+6] == PLAYER_RED_PLAYER){
                state.player = PLAYER_RED_PLAYER;
                state.pos.push_back(i);
            }
            else if(nine_square_vector[i] == SQUARE_EMPTY && nine_square_vector[i+3] == PLAYER_BLUE_PLAYER && nine_square_vector[i+6] == PLAYER_BLUE_PLAYER){
                state.player = PLAYER_BLUE_PLAYER;
                state.pos.push_back(i);
            }
        }
        //斜将大胜
        //左上到右下
        //第一种情况，左上为空
        if(nine_square_vector[0] == PLAYER_RED_PLAYER && nine_square_vector[4] == PLAYER_RED_PLAYER && nine_square_vector[8] == SQUARE_EMPTY){
            state.player = PLAYER_RED_PLAYER;
            state.pos.push_back(8);
        }
        else if(nine_square_vector[0] == PLAYER_BLUE_PLAYER && nine_square_vector[4] == PLAYER_BLUE_PLAYER && nine_square_vector[8] == SQUARE_EMPTY){
            state.player = PLAYER_BLUE_PLAYER;
            state.pos.push_back(8);
        }
        //第二种情况，中部为空
        else if(nine_square_vector[0] == PLAYER_RED_PLAYER && nine_square_vector[4] == SQUARE_EMPTY && nine_square_vector[8] == PLAYER_RED_PLAYER){
            state.player = PLAYER_RED_PLAYER;
            state.pos.push_back(4);
        }
        else if(nine_square_vector[0] == PLAYER_BLUE_PLAYER && nine_square_vector[4] == SQUARE_EMPTY && nine_square_vector[8] == PLAYER_BLUE_PLAYER){
            state.player = PLAYER_BLUE_PLAYER;
            state.pos.push_back(4);
        }
        //第三种情况，右下为空
        else if(nine_square_vector[0] == SQUARE_EMPTY && nine_square_vector[4] == PLAYER_RED_PLAYER && nine_square_vector[8] == PLAYER_RED_PLAYER){
            state.player = PLAYER_RED_PLAYER;
            state.pos.push_back(0);
        }
        else if(nine_square_vector[0] == SQUARE_EMPTY && nine_square_vector[4] == PLAYER_BLUE_PLAYER && nine_square_vector[8] == PLAYER_BLUE_PLAYER){
            state.player = PLAYER_BLUE_PLAYER;
            state.pos.push_back(0);
        }
        //右上到左下
        //第一种情况，右上为空
        if(nine_square_vector[2] == PLAYER_RED_PLAYER && nine_square_vector[4] == PLAYER_RED_PLAYER && nine_square_vector[6] == SQUARE_EMPTY){
            state.player = PLAYER_RED_PLAYER;
            state.pos.push_back(6);
        }
        else if(nine_square_vector[2] == PLAYER_BLUE_PLAYER && nine_square_vector[4] == PLAYER_BLUE_PLAYER && nine_square_vector[6] == SQUARE_EMPTY){
            state.player = PLAYER_BLUE_PLAYER;
            state.pos.push_back(6);
        }
        //第二种情况，中部为空
        else if(nine_square_vector[2] == PLAYER_RED_PLAYER && nine_square_vector[4] == SQUARE_EMPTY && nine_square_vector[6] == PLAYER_RED_PLAYER){
            state.player = PLAYER_RED_PLAYER;
            state.pos.push_back(4);
        }
        else if(nine_square_vector[2] == PLAYER_BLUE_PLAYER && nine_square_vector[4] == SQUARE_EMPTY && nine_square_vector[6] == PLAYER_BLUE_PLAYER){
            state.player = PLAYER_BLUE_PLAYER;
            state.pos.push_back(4);
        }
        //第三种情况，右下为空
        else if(nine_square_vector[2] == SQUARE_EMPTY && nine_square_vector[4] == PLAYER_RED_PLAYER && nine_square_vector[6] == PLAYER_RED_PLAYER){
            state.player = PLAYER_RED_PLAYER;
            state.pos.push_back(2);
        }
        else if(nine_square_vector[2] == SQUARE_EMPTY && nine_square_vector[4] == PLAYER_BLUE_PLAYER && nine_square_vector[6] == PLAYER_BLUE_PLAYER){
            state.player = PLAYER_BLUE_PLAYER;
            state.pos.push_back(2);
        } 
        if(!state.pos.empty()){
            state.will_win = true;
        } else{
            state.will_win = false;
        }            
        return state;            
    }
    //九宫格判断是否大胜
    bool CLASS_Nine_Square_Tool::Function_IS_Win(int num,int player){
        if(player != PLAYER_RED_PLAYER && player != PLAYER_BLUE_PLAYER){
            LOG_ERROR("Function_IS_Win: Invalid player identifier.");
            return false;
        }
        //竖大胜
        for(int i = 0; i < 3; ++i){
            if(Function_Cell_Player_Jungle(&num,i) == player && Function_Cell_Player_Jungle(&num,i+3) == player && Function_Cell_Player_Jungle(&num,i+6) == player){
                return true;
            }
        }
        //斜大胜
        if((Function_Cell_Player_Jungle(&num,0) == player && Function_Cell_Player_Jungle(&num,4) == player && Function_Cell_Player_Jungle(&num,8) == player) ||
            (Function_Cell_Player_Jungle(&num,2) == player && Function_Cell_Player_Jungle(&num,4) == player && Function_Cell_Player_Jungle(&num,6) == player)){
            return true;
        }
        return false;
    }
    //索引对应九宫格状态,pos为0-8，num为存储状态的整数，每三位表示一个状态
    int CLASS_Nine_Square_Tool::Function_State_Search(int* num,int pos){
        if(pos<0 || pos >8){
            LOG_ERROR("Function_State_Search: Invalid position index.");
            return -1;
        }
        return (*num >> (3*pos)) & 0x07;
    }
    //更改对应九宫格状态
    void CLASS_Nine_Square_Tool::Function_State_Change(int* num, int pos, int input) {
        if(pos<0 || pos >8){
            LOG_ERROR("Function_State_Change: Invalid position index.");
            return;
        }
        int mask = ~(0x07 << (3 * pos));
        *num = (*num & mask) | ((input & 0x07) << (3 * pos));
    }
    //根据格子状态判断玩家类型
    int CLASS_Nine_Square_Tool::Function_Cell_Player_Jungle(int* state,int pos){
        if(pos<0 || pos >8){
            LOG_ERROR("Player_Jungle: Invalid position index.");
            return -1;
        }
        int cell_state = Function_State_Search(state,pos);
        if(cell_state == SQUARE_RED_R1_BLACK || cell_state == SQUARE_RED_R2_BLACK){
            return PLAYER_RED_PLAYER;
        }
        else if(cell_state == SQUARE_BLUE_R1_BLACK || cell_state == SQUARE_BLUE_R2_BLACK){
            return PLAYER_BLUE_PLAYER;
        }
        else if(cell_state == SQUARE_EMPTY){
            return SQUARE_EMPTY;
        }
        else{
            LOG_ERROR("Player_Jungle: Invalid cell state.");
            return -1;
        }
    }
    //根据决策,KFC数量减少
    void CLASS_R2_Decision::Function_KFC_Num_Decrease(TYPE_KFC_Having_State* having_state, TYPE_Decision decision,int Player){
        if(Player == PLAYER_RED_PLAYER){
            if(decision.Black_Type == SQUARE_RED_R1_BLACK){
                if(having_state->R1_KFC_Num > 0){
                    having_state->R1_KFC_Num -= 1;
                }
                else{
                    LOG_ERROR("Function_KFC_Num_Decrease: No R1 KFC left to decrease.");
                }
            }
            else if(decision.Black_Type == SQUARE_RED_R2_BLACK){
                if(having_state->R2_KFC_Num > 0){
                    having_state->R2_KFC_Num -= 1;
                }
                else{
                    LOG_ERROR("Function_KFC_Num_Decrease: No R2 KFC left to decrease.");
                }
            }
            else{
                LOG_ERROR("Function_KFC_Num_Decrease: Invalid black type for red player.");
            }
        }
        else if(Player == PLAYER_BLUE_PLAYER){
            if(decision.Black_Type == SQUARE_BLUE_R1_BLACK){
                if(having_state->R1_KFC_Num > 0){
                    having_state->R1_KFC_Num -= 1;
                }
                else{
                    LOG_ERROR("Function_KFC_Num_Decrease: No R1 KFC left to decrease.");
                }
            }
            else if(decision.Black_Type == SQUARE_BLUE_R2_BLACK){
                if(having_state->R2_KFC_Num > 0){
                    having_state->R2_KFC_Num -= 1;
                }
                else{
                    LOG_ERROR("Function_KFC_Num_Decrease: No R2 KFC left to decrease.");
                }
            }
            else{
                LOG_ERROR("Function_KFC_Num_Decrease: Invalid black type for blue player.");
            }
        }
        else{
            LOG_ERROR("Function_KFC_Num_Decrease: Invalid player identifier.");
        }
    }
    //决策打印
    void CLASS_R2_Decision::Function_Print_Decision(TYPE_Decision decision){
        switch (decision.Black_Type)
        {
        case SQUARE_EMPTY:
            /* code */
            printf("Place_position = %d, Black_Type = EMPTY\n", decision.Place_position);
            break;
        case SQUARE_RED_R1_BLACK:
            printf("Place_position = %d, Black_Type = R__R1\n", decision.Place_position);
            break;
        case SQUARE_RED_R2_BLACK:
            printf("Place_position = %d, Black_Type = R__R2\n", decision.Place_position);
            break;
        case SQUARE_BLUE_R1_BLACK:
            printf("Place_position = %d, Black_Type = B__R1\n", decision.Place_position);
            break;
        case SQUARE_BLUE_R2_BLACK:
            printf("Place_position = %d, Black_Type = B__R2\n", decision.Place_position);
            break;
        default:
            break;
        }
    }      
    //伪随机放置决策(纯R2版),如果能直接赢会下直接赢的位置，以及堵对面赢，否则随机放一个位置，放置的方块类型根据持有状态随机放置
    TYPE_Decision CLASS_R2_Decision::Function_R2_Randon_Decision(int nine_square_state,TYPE_KFC_Having_State having_state){
        TYPE_Decision decision = {0,0};
        std::vector<int> empty_positions;
        std::vector<int> will_win_positions;
        if (Function_KFC_Having_Judge(having_state))
        {   
            TYPE_Will_Win_State will_win_state = Function_Will_Win(nine_square_state);
            //如果即将大胜，无论哪方都要下在即将大胜的可执行位置上(3,4,5)
            if(will_win_state.will_win){
                for (int pos : will_win_state.pos)
                {
                    if(pos >= 3 && pos <= 5){
                        will_win_positions.push_back(pos);
                        break;
                    }
                }
                if (!will_win_positions.empty()) {
                    decision.Place_position = will_win_positions[std::rand() % will_win_positions.size()];
                }
            }
            else{
                //3~5空位随机放一个方块
                for (int i = 3; i < 6; i++)
                {
                    if(Function_State_Search(&nine_square_state,i) == SQUARE_EMPTY){
                        empty_positions.push_back(i);
                        break;
                    }
                }
                if (!empty_positions.empty()) {
                    decision.Place_position = empty_positions[std::rand() % empty_positions.size()];
                }
                else{
                    LOG_ERROR("Function_R2_Randon_Decision: No empty positions available for placement.");
                }
            }
            //放置方块类型应该根据持来随机放置
            if(having_state.R1_KFC_Num!= 0 && having_state.R2_KFC_Num != 0){
                decision.Black_Type = (std::rand()%2)+1;//1或2，分别对应红R1和R2的方块
            }
            else if(having_state.R1_KFC_Num != 0 && having_state.R2_KFC_Num == 0){
                decision.Black_Type = 1;//只能放红R1的方块
            }
            else if(having_state.R1_KFC_Num == 0 && having_state.R2_KFC_Num != 0){
                decision.Black_Type = 2;//只能放红R2的方块
            }
            else{
                LOG_ERROR("Function_R2_Randon_Decision: No KFC available for placement.");
            }
        }
        else{
            LOG_ERROR("Function_R2_Randon_Decision: Invalid KFC having state.");
        }
        return decision;
    }
    //伪随机放置决策(R1-R2合体,合体时R2决策),如果能直接赢会下直接赢的位置，以及堵对面赢，否则随机放一个位置，放置的方块类型根据持有状态随机放置
    TYPE_Decision CLASS_R2_Decision::Function_R1_R2_Combined_Random_Decision(int nine_square_state,TYPE_KFC_Having_State having_state){
        TYPE_Decision decision = {0,0};
        std::vector<int> will_win_positions;
        std::vector<int> empty_positions;
        if(Function_KFC_Having_Judge(having_state))
        {   
            TYPE_Will_Win_State will_win_state = Function_Will_Win(nine_square_state);
            //如果即将大胜，无论哪方都要下在即将大胜的可执行位置上(3,4,5)
            if(will_win_state.will_win){
                //3~8位置上即将大胜的可执行位置
                for (int pos : will_win_state.pos)                    {
                    if(pos >= 3 && pos <= 8){
                        will_win_positions.push_back(pos);
                        break;
                    }
                }
                if (!will_win_positions.empty()) {
                    decision.Place_position = will_win_positions[std::rand() % will_win_positions.size()];
                }
            }
            else{
                //3~8空位随机放一个方块
                for (int i = 3; i < 9; i++)
                {
                    if(Function_State_Search(&nine_square_state,i) == SQUARE_EMPTY){
                        empty_positions.push_back(i);
                        break;
                    }
                }
                if (!empty_positions.empty()) {
                    decision.Place_position = empty_positions[std::rand() % empty_positions.size()];
                }
                else{
                    LOG_ERROR("Function_R1_R2_Combined_Random_Decision: No empty positions available for placement.");
                }
            }
            //放置方块类型应该根据持来随机放置
            if(having_state.R1_KFC_Num!= 0 && having_state.R2_KFC_Num != 0){
                decision.Black_Type = (std::rand()%2)+1;//1或2，分别对应红R1和R2的方块
            }
            else if(having_state.R1_KFC_Num != 0 && having_state.R2_KFC_Num == 0){
                decision.Black_Type = 1;//只能放红R1的方块
            }
            else if(having_state.R1_KFC_Num == 0 && having_state.R2_KFC_Num != 0){
                decision.Black_Type = 2;//只能放红R2的方块
            }
            else{
                LOG_ERROR("Function_R1_R2_Combined_Random_Decision: No KFC available for placement.");
            }
        }
        else{
            LOG_ERROR("Function_R1_R2_Combined_Random_Decision: Invalid KFC having state.");
        }
        return decision;
    }
    //根据得分,时间和是否能大胜,或对方是否能大胜进行决策(纯R2版)
    TYPE_Decision CLASS_R2_Decision::Function_R2_Time_Decision(int nine_square_state,TYPE_KFC_Having_State having_state,Pos_Vel_Dir start_state){
        //最高优先级是大胜和堵住对面大胜
        TYPE_Decision decision = {0,0};
        std::vector<int> empty_positions;
        std::vector<int> will_win_positions;
        std::vector<float> using_time;
        float distance;
        if (Function_KFC_Having_Judge(having_state))
        {   
            TYPE_Will_Win_State will_win_state = Function_Will_Win(nine_square_state);
            //如果即将大胜，无论哪方都要下在即将大胜的可执行位置上(3,4,5)
            if(will_win_state.will_win){
                for (int pos : will_win_state.pos)
                {
                    if(pos >= 3 && pos <= 5){
                        will_win_positions.push_back(pos);
                        break;
                    }
                }
                if (!will_win_positions.empty()){
                    //能堵或大胜以运动时间最短为决策
                    if(will_win_positions.size() == 1){
                        decision.Place_position = will_win_positions[0];
                    }
                    else{
                        for (size_t i = 0; i < will_win_positions.size(); ++i)
                        {
                            distance = Line_Distance_Calc(start_state.pos,PARAM_end_state[will_win_positions[i]%3].pos);
                            using_time.push_back(LinePath_To_Time(distance,PARAM_R2_accel,PARAM_R2_max_vel));
                        }
                        if(will_win_positions.size()>=3){
                            for (size_t j = 0; j < will_win_positions.size()-1; ++j)
                            {
                                decision.Place_position = using_time[j] > using_time[j+1] ? will_win_positions[j+1]:will_win_positions[j];
                            }
                        }
                        else{
                            decision.Place_position = using_time[0] > using_time[1] ? will_win_positions[1]:will_win_positions[0];
                        }
                    }
                }
                else{
                    goto normal_placement;
                }
            }
            else{
            normal_placement:
                //3~5空位放方块
                for (int i = 3; i < 6; i++)
                {
                    if(Function_State_Search(&nine_square_state,i) == SQUARE_EMPTY){
                        empty_positions.push_back(i);
                    }
                }
                if (!empty_positions.empty()){
                    //优先放正中心
                    //次优先根据运动时间决定放置位置
                    int flag = 0;
                    for (size_t i = 0; i < empty_positions.size(); ++i)
                    {
                        //中心
                        if(empty_positions[i] == 4){
                            decision.Place_position = empty_positions[i];
                            flag = 1;
                            break;
                        }
                        else{
                            distance = Line_Distance_Calc(start_state.pos,PARAM_end_state[empty_positions[i]%3].pos);
                            using_time.push_back(LinePath_To_Time(distance,PARAM_R2_accel,PARAM_R2_max_vel));
                        }
                    }
                    if(!flag){
                        if(empty_positions.size() == 1){
                            decision.Place_position = empty_positions[0];
                        }
                        else{
                            if(empty_positions.size()>=3){
                                for (size_t j = 0; j < empty_positions.size()-1; ++j)
                                {
                                    decision.Place_position = using_time[j] > using_time[j+1] ? empty_positions[j+1]:empty_positions[j];
                                }
                            }
                            else{
                                decision.Place_position = using_time[0] > using_time[1] ? empty_positions[1]:empty_positions[0];
                            }
                        }
                    }
                }
                else{
                    decision.Place_position = -1;
                    LOG_ERROR("Function_R2_Randon_Decision: No empty positions available for placement.");
                }
            }
            //放置方块类型应该根据持来随机放置
            if(having_state.R1_KFC_Num!= 0 && having_state.R2_KFC_Num != 0){
                /*因为不管哪层，R1和R2只差十分，在地面也差10分，所以理论上对没合体时R2来说
                放R1和R2最终都是一样的(默认在机器人手上也有分，而不是非要在地上有分)
                哪个快放哪个，先随机*/
                decision.Black_Type = (std::rand()%2)+1;;
            }
            else if(having_state.R1_KFC_Num != 0 && having_state.R2_KFC_Num == 0){
                decision.Black_Type = 1;//只能放红R1的方块
            }
            else if(having_state.R1_KFC_Num == 0 && having_state.R2_KFC_Num != 0){
                decision.Black_Type = 2;//只能放红R2的方块
            }
            else{
                LOG_ERROR("Function_R2_Randon_Decision: No KFC available for placement.");
            }
        }
        else{
            LOG_ERROR("Function_R2_Randon_Decision: Invalid KFC having state.");
        }
        return decision;
    }
    //根据得分和是否能大胜,或对方是否能大胜进行决策,不考虑时间使用长短(R1-R2合体,合体时R2决策)
    TYPE_Decision CLASS_R2_Decision::Function_R1_R2_Time_Decision(int nine_square_state,TYPE_KFC_Having_State having_state,Pos_Vel_Dir start_state){
        //最高优先级是大胜和堵住对面大胜
        TYPE_Decision decision = {0,0};
        std::vector<int> empty_positions;
        std::vector<int> will_win_positions;
        std::vector<float> using_time;
        float distance;
        if (Function_KFC_Having_Judge(having_state))
        {   
            TYPE_Will_Win_State will_win_state = Function_Will_Win(nine_square_state);
            //如果即将大胜，无论哪方都要下在即将大胜的可执行位置上(3,4,5,6,7,8)
            if(will_win_state.will_win){
                for (int pos : will_win_state.pos)
                {
                    if(pos >= 3 && pos <= 8){
                        will_win_positions.push_back(pos);
                        break;
                    }
                }
                if (!will_win_positions.empty()){
                    //能堵或大胜以运动时间最短为决策
                    if(will_win_positions.size() == 1){
                        decision.Place_position = will_win_positions[0];
                    }
                    else{
                        for (size_t i = 0; i < will_win_positions.size(); ++i)
                        {
                            distance = Line_Distance_Calc(start_state.pos,PARAM_end_state[will_win_positions[i]%3].pos);
                            using_time.push_back(LinePath_To_Time(distance,PARAM_R2_accel,PARAM_R2_max_vel));
                        }
                        if(will_win_positions.size()>=3){
                            for (size_t j = 0; j < will_win_positions.size()-1; ++j)
                            {
                                decision.Place_position = using_time[j] > using_time[j+1] ? will_win_positions[j+1]:will_win_positions[j];
                            }
                        }
                        else{
                            decision.Place_position = using_time[0] > using_time[1] ? will_win_positions[1]:will_win_positions[0];
                        }
                    }
                }
                else{
                    goto normal_placement2;
                }
            }
            else{
            normal_placement2:
                //3~8是否空
                for (int i = 3; i < 9; i++)
                {
                    if(Function_State_Search(&nine_square_state,i) == SQUARE_EMPTY){
                        empty_positions.push_back(i);
                    }
                }
                if (!empty_positions.empty()){
                    //优先放正中心
                    //次优先根据运动时间决定放置位置
                    int flag = 0;
                    for (size_t i = 0; i < empty_positions.size(); ++i)
                    {
                        //中心
                        if(empty_positions[i] == 4){
                            decision.Place_position = empty_positions[i];
                            flag = 1;
                            break;
                        }
                        else{
                            distance = Line_Distance_Calc(start_state.pos,PARAM_end_state[empty_positions[i]%3].pos);
                            using_time.push_back(LinePath_To_Time(distance,PARAM_R2_accel,PARAM_R2_max_vel));
                        }
                    }
                    if(!flag){
                        if(empty_positions.size() == 1){
                            decision.Place_position = empty_positions[0];
                        }
                        else{
                            if(empty_positions.size()>=3){
                                for (size_t j = 0; j < empty_positions.size()-1; ++j)
                                {
                                    decision.Place_position = using_time[j] > using_time[j+1] ? empty_positions[j+1]:empty_positions[j];
                                }
                            }
                            else{
                                decision.Place_position = using_time[0] > using_time[1] ? empty_positions[1]:empty_positions[0];
                            }
                        }
                    }
                    else{
                        decision.Place_position = -1;
                    }
                }
                else{
                    decision.Place_position = -1;
                }

            }
            //放置方块类型应该根据持来随机放置
            if(having_state.R1_KFC_Num!= 0 && having_state.R2_KFC_Num != 0){
                /*因为不管哪层，R1和R2只差十分，在地面也差10分，所以理论上对没合体时R2来说
                放R1和R2最终都是一样的(默认在机器人手上也有分，而不是非要在地上有分)
                哪个快放哪个，先随机*/
                decision.Black_Type = (std::rand()%2)+1;;
            }
            else if(having_state.R1_KFC_Num != 0 && having_state.R2_KFC_Num == 0){
                decision.Black_Type = 1;//只能放红R1的方块
            }
            else if(having_state.R1_KFC_Num == 0 && having_state.R2_KFC_Num != 0){
                decision.Black_Type = 2;//只能放红R2的方块
            }
            else{
                LOG_ERROR("Function_R2_Randon_Decision: No KFC available for placement.");
                decision.Black_Type = SQUARE_EMPTY;
                decision.Place_position = -1; // 标记无效位置，后续跳过处理
                return decision; // 提前返回，避免后续错误
            }
        }
        else{
            LOG_ERROR("Function_R2_Randon_Decision: Invalid KFC having state.");
        }
        return decision;           
    }    

}


