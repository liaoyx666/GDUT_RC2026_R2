#include "RC_cordic.h"

#define FLOAT_TO_Q31(f)    ((int32_t)(f * 2147483648.0f))
#define Q31_TO_FLOAT(q)    ((float)q / 2147483648.0f)

#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

#define CORDIC_CSR_BASE   	(	\
    0x00    				|  	\
    CORDIC_SCALE_0			|  	\
    CORDIC_INSIZE_32BITS  	|  	\
    CORDIC_OUTSIZE_32BITS 	|  	\
    CORDIC_NBWRITE_1  		|  	\
    CORDIC_NBREAD_1  		| 	\
	CORDIC_PRECISION_15CYCLES)


float cordic_sinf(float x)
{
	int32_t x_q31 = FLOAT_TO_Q31(x / PI);

	__disable_irq();
	
	CORDIC->CSR = CORDIC_CSR_BASE | CORDIC_FUNCTION_SINE;
    CORDIC->WDATA = x_q31;	
	//while(!(CORDIC->CSR & CORDIC_CSR_RRDY)) {};
    int32_t sin_q31 = (int32_t)CORDIC->RDATA;
	
    __enable_irq();
    
    return Q31_TO_FLOAT(sin_q31);
}