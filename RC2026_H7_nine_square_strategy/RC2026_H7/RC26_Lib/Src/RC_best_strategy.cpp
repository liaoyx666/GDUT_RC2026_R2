#include "RC_best_strategy.h"

namespace strategy
{
	//R1 1000*1000  
	//R2 800*800
	//三区场地宽度2500 440*2+1620
	BestStrategy::BestStrategy(ros::Nine_Square& nine_square_nh,data::RobotPose& robot_pose):
	nine_square_nh_(&nine_square_nh),robot_pose_(&robot_pose),ManagedTask("test", 33, 256, task::TASK_PERIOD, 1)
	{

	}
	//计算机器人当前位置到放置位置的距离
	float BestStrategy::distance_calculation(int place_pos,vector2d::Vector2D robot_pos){
			return robot_pos.distanceSquared(robot_pos,PlaceKFC_location[place_pos%3]);
	}
	//检测下对应位置能否大胜
	bool BestStrategy::check_will_win(int pos, int player_block) const {
			int temp[9];
			for(int i=0; i<9; i++) temp[i] = now_state.nine_square_state[i];
			if(temp[pos] != this->EMPTY) return false;
			temp[pos] = player_block;
			// 遍历所有连线
			for(int i=0; i<5; i++){
					int a = win_lines[i][0];
					int b = win_lines[i][1];
					int c = win_lines[i][2];
					if(temp[a] == player_block && temp[b] == player_block && temp[c] == player_block){
							return true;
					}
			}
			return false;
	}
	
	//更新数据
	void BestStrategy::update_state(){
			//是否处于合体状态 false未合体 true合体
			now_state.is_combine = true;
			//九宫格状态数据
			for(int i=0;i<9;++i){
					now_state.nine_square_state[i] = nine_square_nh_->Get_nine_square()[i];
			}			
			//九宫格状态数据
//			for(int i = 0; i<9;++i){
//				now_state.nine_square_state[i] = this->UNKNOWN;
//			}
//			now_state.nine_square_state[2] = this->OUR_BLACK;
//			now_state.nine_square_state[4] = this->OUR_BLACK;
//			now_state.nine_square_state[6] = this->EMPTY;
			
			//机器人位置坐标
			now_state.pos = vector2d::Vector2D(0.0f,0.0f);//(robot_pose_->X(),robot_pose_->Y());
			
			if(!now_state.is_combine)
			{			
				for(int i= 0;i<3;++i){
					this->pos_distance_incombine[i].place_pos = i+3;
					this->pos_distance_incombine[i].distance = distance_calculation(i+3,now_state.pos);
				}
				distance_decide_bubble_sort(this->pos_distance_incombine,3);
			}
			else{
				for(int i= 0;i<6;++i){
					this->pos_distance_combine[i].place_pos = i+3;
					this->pos_distance_combine[i].distance = distance_calculation(i+3,now_state.pos);
				}
				distance_decide_bubble_sort(this->pos_distance_combine,6);
			}
	}
	
	
	//检查对应位置是否为空
	bool BestStrategy::check_is_empty(int pos) const{
		//pos 0~8
		if(this->now_state.nine_square_state[pos] == this->EMPTY){
			return true;
		}
		else{
			return false;
		}
	}
	
	//根据距离的冒泡排序，从小到大排
	void BestStrategy::distance_decide_bubble_sort(place_distance arg[],int size){
			int i,j;
			for(i=0;i<size-1;i++){
					for(j=0;j<size-1-i;j++){
							if(arg[j].distance>arg[j+1].distance){
									place_distance temp = arg[j];
									arg[j] = arg[j+1];
									arg[j+1] = temp;
							}
					}
			}
	}
	//未合体决策
	void BestStrategy::decision_process_no_combine(int(&can_place_decide)[3]){//3 4 5
			int result[3] = {-1, -1, -1};
			// 分类收集
			int win_positions[3] = {0};   // 能大胜的位置
			int win_count = 0;
			bool has_empty4 = false;       // 位置4是否为可放置的空位
			int other_empty[3] = {0};      // 除4以外的可放置空位
			int other_count = 0;

			for (int i = 0; i < 3; ++i) {
					int pos = i + 3;               // 实际位置值 3,4,5
					int status = can_place_decide[i];
					if (status == pos) {
							win_positions[win_count++] = pos;
					} else if (status == 0) {
							if (pos == 4)
									has_empty4 = true;
							else
									other_empty[other_count++] = pos;
					}
			}
			int idx = 0;  // 结果数组当前写入位置
			//按距离从小到大输出所有大胜位置
			for (int i = 0; i < 3 && idx < 3; ++i) {
					int target = pos_distance_incombine[i].place_pos;
					for (int j = 0; j < win_count; ++j) {
							if (win_positions[j] == target) {
									result[idx++] = target;
									break;
							}
					}
			}
			//如果位置4是空位,插入4（在大胜之后、其他空位之前）
			if (has_empty4 && idx < 3) {
					result[idx++] = 4;
			}

			// 3. 按距离顺序输出其他空位
			for (int i = 0; i < 3 && idx < 3; ++i) {
					int target = pos_distance_incombine[i].place_pos;
					for (int j = 0; j < other_count; ++j) {
							if (other_empty[j] == target) {
									result[idx++] = target;
									break;
							}
					}
			}
			for(int i =0;i<3;++i){
				this->decision_square_incombine[i] = result[i];
			}
	}
	//合体决策
	void BestStrategy::decision_process_combine(int(&can_place_decide)[6]){
    int result[6] = {-1, -1, -1, -1, -1, -1};
		
    int win_positions[6] = {0};
    int win_count = 0;
    int high_empty[3] = {0};   // 位置 6,7,8
    int high_count = 0;
    int low_empty[3] = {0};    // 位置 3,4,5
    int low_count = 0;

    for (int i = 0; i < 6; ++i) {
        int pos = i + 3;   
        int status = can_place_decide[i];
        if (status == pos) {
            win_positions[win_count++] = pos;
        } else if (status == 0) {
            if (pos >= 6) {   //空位 6,7,8
                high_empty[high_count++] = pos;
            } else {          //空位 3,4,5
                low_empty[low_count++] = pos;
            }
        }
    }

    int idx = 0;
    //按距离顺序输出大胜位置
    for (int i = 0; i < 6 && idx < 6; ++i) {
        int target = pos_distance_combine[i].place_pos;
        for (int j = 0; j < win_count; ++j) {
            if (win_positions[j] == target) {
                result[idx++] = target;
                break;
            }
        }
    }
    // 按距离顺序输出6~8空
    for (int i = 0; i < 6 && idx < 6; ++i) {
        int target = pos_distance_combine[i].place_pos;
        for (int j = 0; j < high_count; ++j) {
            if (high_empty[j] == target) {
                result[idx++] = target;
                break;
            }
        }
    }

    //按距离顺序输出3~5空
    for (int i = 0; i < 6 && idx < 6; ++i) {
        int target = pos_distance_combine[i].place_pos;
        for (int j = 0; j < low_count; ++j) {
            if (low_empty[j] == target) {
                result[idx++] = target;
                break;
            }
        }
    }
		for(int i =0;i<6;++i){
			this->decision_square_combine[i] = result[i];
		}
	}
	
	
	void BestStrategy::Task_Process(){
			update_state();
			//判断有没有合体
			if(!now_state.is_combine){
				//没合体
				//首先判断3,4,5是否存在直接大胜的位置
				int decide[3] = {1,1,1};//1为未知,无效或占据
				//3,4,5判断是否空以及是否能大胜
				for(int i= 3;i<6;++i){
					if(check_is_empty(i)){
						if(check_will_win(i,this->OUR_BLACK) || check_will_win(i,this->OPP_BLACK)){
							decide[i-3] = i;
						}
						else{
							decide[i-3] = 0;
						}
					}				
				}
				if(decide[0]==1 && decide[1]==1 && decide[2]==1){
					this->decision_square_incombine[0] = -1;
					this->decision_square_incombine[1] = -1;
					this->decision_square_incombine[2] = -1;
				}
				else{
					decision_process_no_combine(decide);
				}
			} 
			//合体
			else{
				int decide[6] = {1,1,1,1,1,1};
				for(int i= 3;i<9;++i){
					if(check_is_empty(i)){
						if(check_will_win(i,this->OUR_BLACK) || check_will_win(i,this->OPP_BLACK)){
							decide[i-3] = i;
						}
						else{
							decide[i-3] = 0;
						}
					}				
				}
				if(decide[0]==1 && decide[1]==1 && decide[2]==1 && decide[3]==1 && decide[4]==1 && decide[5]==1){
					this->decision_square_combine[0] = -1;
					this->decision_square_combine[1] = -1;
					this->decision_square_combine[2] = -1;
					this->decision_square_combine[3] = -1;
					this->decision_square_combine[4] = -1;
					this->decision_square_combine[5] = -1;
				}
				else{
					decision_process_combine(decide);
				}
			}
	}
	
	
}


