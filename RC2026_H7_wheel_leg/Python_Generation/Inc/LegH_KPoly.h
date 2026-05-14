#ifndef __LEGH_KPOLY_H_
#define __LEGH_KPOLY_H_

typedef struct{
	float K1;
	float K2;
	float K3;
	float K4;
}LQR_K;
typedef LQR_K (*FitFunction)(float Leg_H);

#ifdef __cplusplus
namespace Poly_List{
LQR_K Poly_0(float Leg_H);
 LQR_K Poly_1(float Leg_H);
 LQR_K Poly_2(float Leg_H);
 LQR_K Poly_3(float Leg_H);
 LQR_K Poly_4(float Leg_H);
 LQR_K Poly_5(float Leg_H);
 LQR_K Poly_6(float Leg_H);
 LQR_K Poly_7(float Leg_H);
 LQR_K Poly_8(float Leg_H);
 LQR_K Poly_9(float Leg_H);
    class LegH_KPoly{
    public:
    LegH_KPoly(){
fitFunctions[0]=Poly_0;
 fitFunctions[1]=Poly_1;
 fitFunctions[2]=Poly_2;
 fitFunctions[3]=Poly_3;
 fitFunctions[4]=Poly_4;
 fitFunctions[5]=Poly_5;
 fitFunctions[6]=Poly_6;
 fitFunctions[7]=Poly_7;
 fitFunctions[8]=Poly_8;
 fitFunctions[9]=Poly_9;
    }
    
    FitFunction fitFunctions[10]; 
    private:

    };
}
#endif
#endif
