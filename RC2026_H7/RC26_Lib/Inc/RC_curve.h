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
//		virtual bool Get_Point_On_Len(float len, vector2d::Vector2D* p) const = 0;
//		virtual bool Get_Point_On_Dis(float dis, vector2d::Vector2D* p) const = 0;
	
		virtual void Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan, vector2d::Vector2D* nor) const = 0;
		
		virtual void Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const = 0;
		
		
    protected:
		float len;/*总长度*/
		bool is_init;
	
    private:
    
    };
	
	
	class Line2D : public Curve2D
    {
    public:
		Line2D();
		virtual ~Line2D() {}
		
		bool Init(vector2d::Vector2D start_, vector2d::Vector2D end_);	
		
		void Get_Point_On_T(float t, vector2d::Vector2D* p) const override;
			
		void Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const override;
		
		void Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan_, vector2d::Vector2D* nor_) const override;
			
    protected:
		
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
		
		void Get_Point_On_T(float t, vector2d::Vector2D* p) const override;
			
		void Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const override;
		
		void Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan_, vector2d::Vector2D* nor_) const override;
    protected:
		
    private:
		float start_ag;
		float end_ag;
		vector2d::Vector2D center;
		float radius;
		float delta_ag;
    };
	
}
#endif
