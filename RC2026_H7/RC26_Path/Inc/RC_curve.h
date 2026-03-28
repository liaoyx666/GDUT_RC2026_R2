#pragma once
#include "RC_vector2d.h"
#include "RC_pid.h"	

#ifdef __cplusplus
namespace curve
{	
	class Curve2D
    {
    public:
		Curve2D();
		virtual ~Curve2D() {}
	
		virtual void Get_Point_On_T(float t, vector2d::Vector2D* p) const = 0;
		virtual bool Get_Point_On_Len(float len, vector2d::Vector2D* p) const = 0;
		virtual void Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan, vector2d::Vector2D* nor) const = 0;
		virtual void Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const = 0;
		virtual float Get_Len_On_T(float t) const = 0;
		
		/*曲线是否初始化*/
		bool Is_Init() const {return is_init;}
		float Len() const {return len;}
		float Cur() const {return cur;}
		float End_Vel() const {return end_vel;}
		void Set_End_Vel(float end_vel_) {end_vel = end_vel_;}
		void Update_End_Vel(float end_vel_) {end_vel = (end_vel_ < end_vel ? end_vel_ : end_vel);}

		float Vel_On_Len(float l_, float a) const;
		
    protected:
		float len;/*总长度*/
		bool is_init;
		float cur;/*无符号曲率*/
		
	private:
		float end_vel;/*结束速度*/
    };
	
	class Line2D : public Curve2D
    {
    public:
		Line2D();
		virtual ~Line2D() {}
		
		bool Init(vector2d::Vector2D start_, vector2d::Vector2D end_);	
		void Reset();
		
		void Get_Point_On_T(float t, vector2d::Vector2D* p) const override;
		void Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const override;
		void Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan_, vector2d::Vector2D* nor_) const override;
		bool Get_Point_On_Len(float len_, vector2d::Vector2D* p) const override;
		float Get_Len_On_T(float t) const override;

    private:
		vector2d::Vector2D start;
		vector2d::Vector2D dir;
		vector2d::Vector2D tan;
		vector2d::Vector2D nor;
		float len_sq;
    };
	
	class Arc2D : public Curve2D
    {
    public:
		Arc2D();
		virtual ~Arc2D() {}
		
		bool Init(vector2d::Vector2D start_, vector2d::Vector2D center_, float ag_);
		bool Init(vector2d::Vector2D start_, vector2d::Vector2D end_, float radius_, bool is_counter_clockwise);
		void Reset();
			
		void Get_Point_On_T(float t, vector2d::Vector2D* p) const override;
		void Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const override;
		void Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan_, vector2d::Vector2D* nor_) const override;
		bool Get_Point_On_Len(float len_, vector2d::Vector2D* p) const override;
		float Get_Len_On_T(float t) const override;

    private:
		float start_ag;
		float end_ag;
		vector2d::Vector2D center;
		float radius;
		float delta_ag;
    };
	
}

extern float calcVel(float dis, float end_v, float a);

#endif
