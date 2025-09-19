#include "RC_init.h"

tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

can::Can can1(hcan1);
can::Can can2(hcan2);

m3508::M3508 m3508_5(5, can1, tim7_1khz);
m3508::M3508 m3508_1(1, can1, tim7_1khz);
m3508::M3508 m3508_4(4, can1, tim7_1khz);
m3508::M3508 m3508_2(2, can1, tim7_1khz);
							
m3508::M3508 m3508_3(3, can1, tim7_1khz);
m3508::M3508 m3508_6(6, can1, tim7_1khz);
m3508::M3508 m3508_7(7, can1, tim7_1khz);
m3508::M3508 m3508_8(8, can1, tim7_1khz);


timer::Timer timer_us(tim4_timer);// 用于获取时间戳


flysky::FlySky remote_ctrl(GPIO_PIN_7);// 遥控

SquareWave wave(1000, 3000);// 用于调pid

float target = 0;
float a = 30;


//Vector2D::Vector2D VOUT_For_chassis;
//speed_plan::Trapezoid2D_speedplan my_speed_plan;
//Vector2D::Vector2D start_point(0,0);
//Vector2D::Vector2D end_point(5,5);
//Vector2D::Vector2D now_point(0,0);
//float32_t max_a = 1;
//float32_t max_d = 1;
//float32_t V_max = 1000;
//float32_t epsilon = 0.01f;
//RC_chassis::chassis_info my_chassis_info(0.05f, 0.5f, 0.5f, 0.3f); 
//RC_chassis::omni4_Chassis<float> my_chassis(my_chassis_info);
//float wheel_speed[4];
//void my_test(void *argument){
//	my_speed_plan.start_plan(start_point,end_point,max_a,max_d,V_max,epsilon);
//	my_speed_plan.step(1);
//	my_speed_plan.wirte_Point(now_point);
//	VOUT_For_chassis = my_speed_plan.get_velocity();
//	my_chassis.omni4_chassis_calc(wheel_speed,VOUT_For_chassis.x,VOUT_For_chassis.y,0);
//	m3508_1.Set_Rpm(wheel_speed[0]);
//	m3508_2.Set_Rpm(wheel_speed[1]);
//	m3508_3.Set_Rpm(wheel_speed[2]);
//	m3508_4.Set_Rpm(wheel_speed[3]);
//	//路径规划->速度规划->底盘
//	//底盘解算出各个电机的target
//	//传入target给各个电机，RPM
//}
//task::TaskCreator test2_task("test2", 20, 256, my_test, NULL);


void test(void *argument)
{
	wave.Init();
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		//uart_printf("%f,%f,%f\n", target, m3508_1.pos, m3508_1.rpm);
		
		uart_printf("%4d,%4d,%4d,%4d\n", remote_ctrl.left_x, remote_ctrl.left_y, remote_ctrl.right_x, remote_ctrl.right_y);// 打印遥控数据
		
		m3508_1.Set_Rpm(target);
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

//初始化
void All_Init()
{
	can1.Can_Filter_Init(1, CAN_RX_FIFO0, 0, 0, 0, 0);
	can1.Can_Filter_Init(2, CAN_RX_FIFO1, 0, 0, 0, 0);
	
	//can1.Can_Start();
	
	tim4_timer.Tim_It_Start();
	tim7_1khz.Tim_It_Start();
}