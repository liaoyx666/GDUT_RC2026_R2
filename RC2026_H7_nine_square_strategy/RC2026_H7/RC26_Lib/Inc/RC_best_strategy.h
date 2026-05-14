#pragma once
#include "RC_path2.h"
#include "RC_nine_square.h"
#include "RC_data_pool.h"
#include "RC_task.h"

#ifdef __cplusplus

namespace strategy
{
	/*
	-1：未知
	0：无效
	1：空位
	2：我方方块
	3：敌方方块
	
	九宫格位置标记
		8 7 6
	  5 4 3
	  2 1 0
	*/
	struct place_distance{
		int place_pos;
		float distance;
	};
	 struct state_data
   {
			bool is_combine;//是否是合体状态
			vector2d::Vector2D pos;//机器人所处位置
			int nine_square_state[9];//九宫格状态
   };
		class BestStrategy: public task::ManagedTask
		{
		public:
		BestStrategy(ros::Nine_Square& nine_square_nh,data::RobotPose& robot_pose);
		~BestStrategy() = default;
		int* Get_incombine_decision(){return this->decision_square_incombine;}
		int* Get_combine_decision(){return this->decision_square_combine;}
		protected:
		place_distance pos_distance_incombine[3];
		place_distance pos_distance_combine[6];
		//三个点位放置KFC,0,1,2
		const vector2d::Vector2D PlaceKFC_location[3] = 
		{
			vector2d::Vector2D(7.091f+2.750f+0.54f*2, 0.371f-1.190f), vector2d::Vector2D(7.091f+2.750f+0.54f, 0.371f-1.190f), vector2d::Vector2D(7.091f+2.750f, 0.371f-1.190f)
		};
		//大胜连线
		const int win_lines[5][3] = {
			{2,5,8},
			{0,3,6},
			{1,4,7},
			{0,4,8},
			{2,4,6}
		};		
		int decision_square_incombine[3] = {-1,-1,-1};
		int decision_square_combine[6] = {-1,-1,-1,-1,-1,-1};		
		state_data now_state;//当前状态
		
		ros::Nine_Square* nine_square_nh_;
		data::RobotPose* robot_pose_;
		
		//距离计算
		float distance_calculation(int place_pos,vector2d::Vector2D robot_pos);
		//检查放对应位置是否能赢
		bool check_will_win(int pos, int player_block) const;
		bool check_is_empty(int pos) const;
		void update_state();
		//三位数组分别对应3,4,5格状态，数字等于格子就是能大胜或对面大胜的可放置位置，0则是空，1则是已经被占据或无效未知
		virtual void decision_process_no_combine(int(&can_place_decide)[3]);
		//六位数组分别对应3,4,5,6,7,8格状态，数字等于格子就是能大胜或对面大胜的可放置位置，0则是空，1则是已经被占据或无效未知
		virtual void decision_process_combine(int(&can_place_decide)[6]);
		void distance_decide_bubble_sort(place_distance arg[],int size);
		void Task_Process() override;
		const int UNKNOWN = -1;//未知
		const int INVALID = 0;//无效
		const int EMPTY = 1;//空
		const int OUR_BLACK = 2;//我方方块
		const int OPP_BLACK = 3;//敌方方块
		private:
			
		};
		
		

}
#endif