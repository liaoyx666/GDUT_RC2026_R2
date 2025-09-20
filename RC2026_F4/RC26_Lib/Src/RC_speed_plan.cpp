#include "RC_speed_plan.h"

namespace speed_plan {
	void Trapezoid_speed_test() {
        // 实例化参数：起点，终点，加速度，最大速度，减速度，误差容忍
        SimpleTrapezoid<float> planner(0.0f, 100.0f, 2.0f, 10.0f, 2.0f, 0.01f);
        int step = 0;
        while (!planner.get_arrived()&&step<100) {
            planner.step(1);
    //		printf("Step %d:",step);
            planner.print_status();
            ++step;
        }
    //    printf("到达,目的地");
    }
			
    void Trapezoid2D_speedplan::start_plan(Vector2D::Vector2D start_point,Vector2D::Vector2D end_point,float32_t max_a,float32_t max_d,float32_t V_max,float32_t epsilon){
        this-> start_point = start_point;//起点
        this->end_point = end_point;//终点
        this->V_current = Vector2D::Vector2D(0, 0);//初始速度为0
        this->now_point = start_point;//当前位置初始化为起点    
        this->is_arrive = false;//未到达
        this->max_a = max_a;
        this->max_d = max_d;
        this->V_max = V_max;
        // 计算方向向量和总距离
        Vector2D::Vector2D delta = this->end_point - this->start_point;
		
		this->total_distance = delta.magnitude();//取模
		if(total_distance==0){
			return ;
		}
		this->direction = delta.normalize();//单位化方向向量
        this->a_s = (V_max * V_max) / (2 * max_a);//加速距离
        this->d_s = (V_max * V_max) / (2 * max_d);//减速距离
		this->epsilon = epsilon;
        // 判断是否需要匀速阶段
        if (a_s + d_s > total_distance) {
            // 三角规划，重新计算最大速度
            this->V_max = sqrt((2 * max_a * max_d * total_distance) / (max_a + max_d));
            a_s = (this->V_max * this->V_max) / (2 * max_a);
            d_s = (this->V_max * this->V_max) / (2 * max_d);
            this->current_state = ACCELERATING;//初始状态为加速阶段
        } else {
            // 梯形规划
            this->current_state = ACCELERATING;//初始状态为加速阶段
        }
    }
	
    void Trapezoid2D_speedplan::step(float32_t dt) {
        if (is_arrive) return;
		this->dt = dt;  // 更新当前时间步长
		
        // 计算已移动距离
        moved_distance = (now_point - start_point).magnitude();
        // 根据当前状态更新速度
        switch (current_state) {
            case ACCELERATING:
                // 加速阶段
                if (moved_distance < a_s) {
                    // 继续加速
                    V_current = V_current + direction * max_a*dt;
                    if (V_current.magnitude() > this->V_max) {
                            V_current = direction * this->V_max;
                    }
                } else {
                    // 进入匀速阶段
                    current_state = CRUISING;
                    V_current = direction * V_current.magnitude();
                }
                break;
            case CRUISING:
                // 匀速阶段
                if (moved_distance >= total_distance - d_s) {
                    // 进入减速阶段
                    current_state = DECELERATING;
                }
                break;
            case DECELERATING:
				{//内部定义了变量要用大括号
                // 减速阶段
                float32_t remaining_distance = total_distance - moved_distance;
                    
                // 计算所需减速度
                float32_t required_deceleration = (V_current.magnitude() * V_current.magnitude()) / (2 * remaining_distance);
                float32_t deceleration = max_d < required_deceleration ? max_d : required_deceleration;
						
			    // 计算理论减速后的速度
				Vector2D::Vector2D new_velocity = V_current - direction * deceleration * dt;
    
				// 检查是否即将到达终点（考虑微小误差）
				if (remaining_distance < V_current.magnitude() * dt || new_velocity.magnitude() < 0.1f) {
				// 平滑过渡到终点
					V_current = direction * (remaining_distance / dt); // 刚好到达终点的速度
				} else {
					V_current = new_velocity;
				}
				// 检查是否到达终点
				if (remaining_distance < epsilon) {
					V_current = Vector2D::Vector2D(0, 0);
					current_state = FINISHED;
					is_arrive = true;
				}
                break;
				}
            case FINISHED:
                // 已完成
                V_current = Vector2D::Vector2D(0, 0);
                is_arrive = true;
                break;
            default:
                break;
        }
        // 更新位置
        now_point = now_point + V_current*dt;
        // 检查是否到达终点
        if ((now_point - end_point).magnitude() < this->epsilon) {
            now_point = end_point;
            V_current = Vector2D::Vector2D(0, 0);
            is_arrive = true;
            current_state = FINISHED;
        }
    }


}



