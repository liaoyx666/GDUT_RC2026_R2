#pragma once
#include <math.h>


#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifdef __cplusplus



namespace adrc
{
	class ADRC
    {
    public:
		ADRC();
		virtual ~ADRC() {}
			
		float ADRC_Calculate(bool normalization = false, float unit = PI);
		void Update_Real(float real_){y = real_;}
		void Update_Target(float target_){v = target_;}
		float Get_Output(){return u;}
		
		void ADRC_Param_Init(
			float output_limit_,// 输出限幅
		
			float r_,// 快速跟踪因子
			float h_,// 滤波因子，系统调用步长

			float b_,// 系统系数
			float delta_,// fal函数的线性区间宽度
			float beta_01_,// 扩张状态观测器反馈增益1
			float beta_02_,// 扩张状态观测器反馈增益2
			float beta_03_,// 扩张状态观测器反馈增益3

			float alpha_1_,// 非线性因子1
			float alpha_2_,// 非线性因子2
			float beta_1_,// 跟踪输入信号增益1
			float beta_2_// 跟踪微分信号增益2
		);
			
    protected:

	
    private:
		float output_limit = 0;// 输出限幅
	
		/*---------------TD-----------------*/
		float r = 0;// 快速跟踪因子
		float h = 0;// 滤波因子，系统调用步长
	
		/*---------------ESO----------------*/
		float b = 0;// 系统系数
		float delta = 0;// fal函数的线性区间宽度
		float beta_01 = 0;// 扩张状态观测器反馈增益1
		float beta_02 = 0;// 扩张状态观测器反馈增益2
		float beta_03 = 0;// 扩张状态观测器反馈增益3
	
		/*---------------NLSEF--------------*/
		float alpha_1 = 0;// 非线性因子1
		float alpha_2 = 0;// 非线性因子2
		float beta_1 = 0;// 跟踪输入信号增益1
		float beta_2 = 0;// 跟踪微分信号增益2
		
		
		
		/*---------------状态变量------------*/
		float v = 0;// 输入目标值
	
		float v1 = 0;
		float v2 = 0;
		
		float v1_last = 0;
		float v2_last = 0;
		
		float u = 0;
		float u0 = 0;
		
		float z1 = 0;
		float z2 = 0;
		float z3 = 0;


		float e1 = 0;
        float e2 = 0;
		float e2_last  = 0;

		float u0_last = 0;
		
		float y = 0;// 反馈值

    };
	

	float fal(float e_, float alpha_, float delta_);
	float fst(float x1_, float x2_, float r_, float h_);
	float sgn(float x_);
	float fhan(float x1, float x2, float r, float h);
}
#endif
