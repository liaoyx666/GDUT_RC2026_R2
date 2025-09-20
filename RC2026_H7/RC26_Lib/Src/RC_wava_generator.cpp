#include "RC_wave_generator.h"


SquareWave::SquareWave(float amplitude_, uint32_t half_cycle_) : amplitude(amplitude_), half_cycle(half_cycle_) {}


void SquareWave::Init()
{
	start_time = HAL_GetTick();
	is_init = true;
}


float SquareWave::Get_Signal()
{
	if (is_init == true)
	{
		uint32_t current_time = HAL_GetTick();
		uint32_t time = current_time - start_time;
		uint32_t half_cycle_num = time / half_cycle;
		
		if (half_cycle_num % 2 == 0) return amplitude;
		else return -amplitude;
	}
	else return 0;
}