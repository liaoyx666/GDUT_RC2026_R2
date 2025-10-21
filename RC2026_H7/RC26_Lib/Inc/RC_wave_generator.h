#pragma once
#include "tim.h"
#include "RC_pid.h"
// Header: 信号发生器
// File Name: 
// Author:
// Date:

#ifdef __cplusplus


// 方波发生器
class SquareWave
{
public:
	SquareWave(float amplitude_, uint32_t half_cycle_);
	virtual ~SquareWave() {}
		
	float Get_Signal();
	void Init();
		
	void Set_Amplitude(float amplitude_) {amplitude = amplitude_;}
	void Set_half_cycle(uint32_t half_cycle_) {half_cycle = half_cycle_;}
protected:
	
private:
	bool is_init = false;

	float amplitude;
	uint32_t half_cycle;// ms
	uint32_t start_time;


};


class SinWave
{
public:
	SinWave(float amplitude_, uint32_t cycle_);
	virtual ~SinWave() {}
		
	float Get_Signal();
	void Init();
		
	void Set_Amplitude(float amplitude_) {amplitude = amplitude_;}
	void Set_half_cycle(uint32_t cycle_) {cycle = cycle_;}
	
protected:
	


private:
	bool is_init = false;

	float amplitude;
	uint32_t cycle;// ms
	uint32_t start_time;

};


#endif
