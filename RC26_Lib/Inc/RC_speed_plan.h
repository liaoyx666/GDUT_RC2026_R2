#pragma once
#include "arm_math.h"
#include "Vector2D.h"

#ifdef __cplusplus
namespace speed_plan {
    template <typename speed_t>
	// 通用速度规划基类
    class SpeedPlanBase {
    protected:
        speed_t start;      // 起点
        speed_t end;        // 终点
        speed_t epsilon;    // 误差容忍
		speed_t dt;
    public:
        SpeedPlanBase(speed_t start, speed_t end, speed_t epsilon)
            : start(start), end(end), epsilon(epsilon) {}
        virtual void step(speed_t dt) = 0; // 纯虚函数，子类实现
        virtual speed_t get_position() const = 0;
        virtual speed_t get_velocity() const = 0;
        virtual bool get_arrived() const = 0;
        virtual void print_status() const = 0;
        virtual ~SpeedPlanBase() {}
    };
	
    // 单轴梯形速度规划
    template <typename speed_t>
    class SimpleTrapezoid :public SpeedPlanBase<speed_t>{
    protected:
        speed_t pos;        // 当前位置信息
        speed_t v;          // 当前速度
        speed_t a;          // 加速度
        speed_t d;          // 减速度
        speed_t vmax;       // 最大速度
        bool arrived;     // 是否到达终点

        // 方向（+1或-1）
        inline speed_t direction() const { return (this->end >= this->start) ? 1.0f : -1.0f; }
        // 总距离,绝对值
        inline speed_t total_dist() const { return fabs(this->end - this->start); }
        // 已运动距离
        inline speed_t moved_dist() const { return (this->pos - this->start) * this->direction(); }
        // 加速段距离
        inline speed_t accel_dist() const { return (this->vmax * this->vmax) / (2 * this->a); }
        // 减速段距离
        inline speed_t decel_dist() const { return (this->vmax * this->vmax) / (2 * this->d); }
        // 速度限幅
        inline speed_t limit_v(speed_t v_in) const {
            speed_t v_abs = fabs(v_in);
            return (v_abs > this->vmax) ? this->direction() * this->vmax : v_in;
        }
        // 防止终点速度反向
        inline speed_t prevent_reverse(speed_t v_in) const {
            if (this->direction() > 0 && v_in < 0) return 0;
            if (this->direction() < 0 && v_in > 0) return 0;
            return v_in;
        }
        // 到达判定，带误差范围
        inline bool is_arrive() const {
            return (fabs(this->pos - this->end) < this->epsilon) && (fabs(this->v) < this->epsilon);
        }
        // 预测下一步是否过冲
        inline bool will_overflow(speed_t next_pos) const {
            return (this->direction() > 0 && next_pos > this->end) || (this->direction() < 0 && next_pos < this->end);
        }
    public:
        //构造函数，主要是判断两个点间距离是否大于加速和减速距离之和，生成一个最大速度
        SimpleTrapezoid(speed_t start, speed_t end, speed_t acceleration, speed_t max_speed, speed_t deceleration, speed_t epsilon)
            : SpeedPlanBase<speed_t>(start, end, epsilon), pos(start), v(0), a(fabs(acceleration)), d(fabs(deceleration)), vmax(fabs(max_speed)), arrived(false)
        {
            speed_t adist = this->accel_dist();
            speed_t ddist = this->decel_dist();
            if (adist + ddist > this->total_dist()) {
                this->vmax = sqrt((2 * this->a * this->d * this->total_dist()) / (this->a + this->d));
            }
        }
        // 步进函数，更新位置和速度
        void step(speed_t dt) {
            if (this->arrived) return;
            speed_t adist = this->accel_dist();//构造函数生成Vmax后的加速距离
            speed_t ddist = this->decel_dist();//构造函数生成Vmax后的减速距离
            speed_t dist = this->total_dist();//总距离
            speed_t moved = this->moved_dist();//已运动距离
            if (moved < adist) {//加速阶段
                this->v += this->a * this->direction()*dt;
                this->v = this->limit_v(this->v);
            } else if (moved >= adist && moved < (dist - ddist)) {//匀速阶段
                this->v = this->direction() * this->vmax;
            } else if (moved >= (dist - ddist) && moved < dist) {//减速阶段
                this->v -= this->d * this->direction()*dt;
                this->v = this->prevent_reverse(this->v);
            }
            speed_t next_pos = this->pos + this->v*dt;
            if (this->will_overflow(next_pos)) {//预测下一步是否会过冲，过冲清零位置和速度，并标记到达
                this->pos = this->end;
                this->v = 0;
                this->arrived = true;
            } else {//正常更新位置
                this->pos = next_pos;
                if (this->is_arrive()) {
                    this->pos = this->end;
                    this->v = 0;
                    this->arrived = true;
                }
            }
        }

        inline speed_t get_position() const { return this->pos; }//当前位置信息
        inline speed_t get_velocity() const { return this->v; }//当前速度
        inline bool get_arrived() const { return this->arrived; }//到达判定

        inline void print_status() const {//调试用，打印当前状态
    //  	printf("Pos: %.3f | V: %.3f | Arrive: %s\n", this->pos, this->v, this->arrived ? "Yes" : "No");
        }
	};
	
	
	//二维梯形速度规划
	class Trapezoid2D_speedplan {
	private:
        Vector2D::Vector2D  now_point;//当前位置
		Vector2D::Vector2D  V_current;//当前速度
		Vector2D::Vector2D  start_point;//起点
		Vector2D::Vector2D  end_point;//终点
        Vector2D::Vector2D  direction;//方向单位向量
        Vector2D::Vector2D a;//加速度
        Vector2D::Vector2D d;//减速度
        float32_t max_a;                 // 最大加速度（标量）
        float32_t max_d;                 // 最大减速度（标量）
        float32_t V_max;//最大速度
		float32_t a_s;//加速距离
		float32_t d_s;//减速距离
		bool is_arrive;//是否到达终点
		float32_t total_distance;//起点到终点的距离
        float32_t moved_distance;// 已移动距离
		float32_t dt;//时间步长（单位s）
	
		float32_t epsilon;

        // 内部状态
        enum State {
            ACCELERATING,  // 加速阶段
            CRUISING,      // 匀速阶段
            DECELERATING,  // 减速阶段
            FINISHED       // 完成
        } current_state;

	public:
		Trapezoid2D_speedplan(): V_current(0,0), start_point(0,0), end_point(0,0),
        V_max(0), a(0,0), d(0,0), a_s(0), d_s(0), is_arrive(false),total_distance(0),direction(0,0),moved_distance(0),current_state(FINISHED),dt(0),epsilon(0.01f){}
        //最大加速度（正值），最大减速度（正值），最大允许速度，起始时的速度，目标点的速度，起始坐标，目标坐标，用户设置的 PID 控制距离阈值，若为 0 则不使用 PID 控制
		void start_plan(Vector2D::Vector2D start_point,Vector2D::Vector2D end_point,float32_t max_a,float32_t max_d,float32_t V_max,float32_t epsilon = 0.01f);
        // 步进函数，更新位置和速度
        void step(float32_t dt);
		//写入当前位置
		inline void wirte_Point(Vector2D::Vector2D  point){
			this->now_point = point;
		}
        // 获取当前位置
        inline Vector2D::Vector2D get_position() const {return this->now_point;}
        // 获取当前速度
        inline Vector2D::Vector2D get_velocity() const {return this->V_current;}
        // 检查是否到达终点
        inline bool arrived() const {return this->is_arrive;}
        // 获取当前状态
        inline State get_state() const {return this->current_state;}
        // 获取总距离
        inline float32_t get_total_distance() const {return this->total_distance;}
        // 获取已移动距离
        inline float32_t get_moved_distance() const {return this->moved_distance;}
	};


}//namespace
#endif 
