#include "RC_init.h"

tim::Tim tim7(htim7);
tim::Tim tim4(htim4);

can::Can can1(hcan1);
can::Can can2(hcan2);


m3508::M3508 m3508_5(5, can1, tim7);
m3508::M3508 m3508_1(1, can1, tim7);
m3508::M3508 m3508_4(4, can1, tim7);
m3508::M3508 m3508_2(2, can1, tim7);

m3508::M3508 m3508_3(3, can1, tim7);
m3508::M3508 m3508_6(6, can1, tim7);
m3508::M3508 m3508_7(7, can1, tim7);
m3508::M3508 m3508_8(8, can1, tim7);

SquareWave wave(1000, 3000);

float target = 0;
float a = 30;

void test(void *argument)
{
	wave.Init();
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		
		
		uart_printf("%f,%f,%f\n", target, m3508_1.pos, m3508_1.rpm);
		
		m3508_1.Set_Pos(target);
		m3508_2.Set_Pos(target);
		m3508_3.Set_Pos(target);
		m3508_4.Set_Pos(target);
		m3508_5.Set_Pos(target);
		m3508_6.Set_Pos(target);
		m3508_7.Set_Pos(target);
		m3508_8.Set_Pos(target);

		
		
		osDelay(1);
	}
}


task::TaskCreator test_task("test", 20, 256, test, NULL);






void All_Init()
{
	can1.Can_Filter_Init(1, CAN_RX_FIFO0, 0, 0, 0, 0);
	can1.Can_Start();
	
	tim4.Tim_It_Start();
	tim7.Tim_It_Start();
	
	
	
	
	
	
}