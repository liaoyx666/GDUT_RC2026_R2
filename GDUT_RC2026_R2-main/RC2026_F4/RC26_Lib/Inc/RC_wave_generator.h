#pragma once
#include "tim.h"

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


#endif
