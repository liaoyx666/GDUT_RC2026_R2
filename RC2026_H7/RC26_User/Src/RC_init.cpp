#include "RC_init.h"

tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);


can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);




m3508::M3508 m3508_1(1, can1, tim7_1khz);
m3508::M3508 m3508_2(2, can1, tim7_1khz);
m3508::M3508 m3508_3(3, can1, tim7_1khz);






timer::Timer timer_us(tim4_timer);// 用于获取时间戳

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控


cdc::CDC CDC_HS(cdc::USB_CDC_HS);// 虚拟串口

ros::Radar radar(CDC_HS, 1);


chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 2, 2);



SquareWave wave(1000, 3000);// 用于调pid

float target = 0;
float a = 30;

void test(void *argument)
{
	wave.Init();
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		
		
		//uart_printf("%f,%f,%f\n", target, m3508_1.pos, m3508_1.rpm);
		
		
//		uart_printf("%4d,%4d,%4d,%4d\n", remote_ctrl.left_x, remote_ctrl.left_y, remote_ctrl.right_x, remote_ctrl.right_y);// 打印遥控数据
		
//		m3508_1.Set_Rpm(target);
//		m3508_2.Set_Pos(target);
//		m3508_3.Set_Pos(target);
//		m3508_4.Set_Pos(target);
//		m3508_5.Set_Pos(target);
//		m3508_6.Set_Pos(remote_ctrl.left_y / 100);
//		m3508_7.Set_Pos(target);
//		m3508_8.Set_Pos(target);

		
//		uint8_t ccc[8] = {1, 2, 3, 4, 5, 6, 7, 8};
//		
//		float x = 3.14f;
		
//		CDC_HS.CDC_Send_Pkg(1, (uint8_t*)&x, sizeof(x), 1);// 虚拟串口打包发送
//		CDC_HS.CDC_AddToBuf(ccc, 8, 1);// 虚拟串口发送

		
		
		omni_chassis.Set_Chassis_Spd(remote_ctrl.left_x / 100.f, remote_ctrl.left_y / 100.f, remote_ctrl.right_x / 100.f);
		
		
		osDelay(1);
	}
}


task::TaskCreator test_task("test", 20, 256, test, NULL);


void All_Init()
{
	can1.Can_Filter_Init(FDCAN_STANDARD_ID, 1, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can1.Can_Filter_Init(FDCAN_STANDARD_ID, 2, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can1.Can_Start();

	can2.Can_Filter_Init(FDCAN_STANDARD_ID, 3, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can2.Can_Filter_Init(FDCAN_STANDARD_ID, 4, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can2.Can_Start();
	
	can3.Can_Filter_Init(FDCAN_STANDARD_ID, 5, FDCAN_FILTER_TO_RXFIFO0, 0, 0);
	can3.Can_Filter_Init(FDCAN_STANDARD_ID, 6, FDCAN_FILTER_TO_RXFIFO1, 0, 0);
	can3.Can_Start();


	tim4_timer.Tim_It_Start();
	
	tim7_1khz.Tim_It_Start();
	
}