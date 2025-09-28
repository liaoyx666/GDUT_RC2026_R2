#include "RC_init.h"

tim::Tim tim7_1khz(htim7);
tim::Tim tim4_timer(htim4);

can::Can can1(hfdcan1);
can::Can can2(hfdcan2);
can::Can can3(hfdcan3);


motor::M6020 m6020_1(1, can1, tim7_1khz);

//motor::M3508 m3508_1(1, can1, tim7_1khz);
//motor::M3508 m3508_2(2, can1, tim7_1khz);
//motor::M3508 m3508_3(3, can1, tim7_1khz);


timer::Timer timer_us(tim4_timer);// 用于获取时间戳

flysky::FlySky remote_ctrl(GPIO_PIN_8);// 遥控


cdc::CDC CDC_HS(cdc::USB_CDC_HS);// 虚拟串口

ros::Radar radar(CDC_HS, 1);


//chassis::OmniChassis omni_chassis(m3508_3, m3508_1, m3508_2, 2, 2);



SquareWave wave(1000, 3000);// 用于调pid

float target = 0;
float a = 0;
float p = 400, i = 0, d = 0.01;

void test(void *argument)
{
	m6020_1.pid_pos.Pid_Mode_Init(false, false, 0);
	m6020_1.pid_pos.Pid_Param_Init(400, 0, 0.01, 0, 0.001, 0, 300, 200, 200, 200, 200);
	
	
	wave.Init();
	for (;;)
	{
		wave.Set_Amplitude(a);
		target = wave.Get_Signal();
		

		//m3508_1.Set_Pos(target);
		//omni_chassis.Set_Chassis_Spd(remote_ctrl.left_x / 100.f, remote_ctrl.left_y / 100.f, remote_ctrl.right_x / 100.f);
		
		
		uart_printf("%f,%f\n", m6020_1.angle, target);
		
		m6020_1.pid_pos.Set_Kp(p);
		m6020_1.pid_pos.Set_Ki(i);
		m6020_1.pid_pos.Set_Kd(d);
		
		
		m6020_1.Set_Angle(a);
		
		
		
		
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