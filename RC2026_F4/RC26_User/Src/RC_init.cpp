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

/*
*测试全向轮底盘和梯形速度规划测试
*/
/***************************************************************************/
Vector2D::Vector2D VOUT_For_chassis;//用于接收速度规划给的速度值
speed_plan::Trapezoid2D_speedplan my_speed_plan;//实例化速度规划对象
Vector2D::Vector2D start_point(0,0);//设置起点
Vector2D::Vector2D end_point(2500,3000);//设置终点
Vector2D::Vector2D now_point(0,0);//设置当前位置
float32_t max_a = 1;//设置加速度
float32_t max_d = 1;//设置减速度
float32_t V_max = 100;//设置最大速度
float32_t epsilon = 0.01f;//设置速度规划容忍误差大小
RC_chassis::chassis_info my_chassis_info(0, 0, 0, 5);//设置底盘参数 
RC_chassis::omni4_Chassis<float> my_chassis(my_chassis_info);//实例化底盘解算对象
float wheel_speed[4];//用于存储解算后的速度值
void my_test(void *argument){
	my_speed_plan.start_plan(start_point,end_point,max_a,max_d,V_max,epsilon);
	for (;;)
	{	
		my_speed_plan.step(0.1);//设置为0.1s执行一次，要根据实际来
		my_speed_plan.write_Point(now_point);//写入当前位置
		VOUT_For_chassis = my_speed_plan.get_velocity();//读取速度规划得到的Vx,Vy
		now_point = VOUT_For_chassis*my_speed_plan.get_dt() + now_point;
		my_chassis.omni4_chassis_calc(wheel_speed,VOUT_For_chassis.x,VOUT_For_chassis.y,0);//写入底盘，解算成电机输出
		//输入给电机
		m3508_1.Set_Rpm(wheel_speed[0]);
		m3508_2.Set_Rpm(wheel_speed[1]);
		m3508_3.Set_Rpm(wheel_speed[2]);
		m3508_4.Set_Rpm(wheel_speed[3]);
		osDelay(10);
	}
}
//task::TaskCreator test2_task("test2", 21, 256, my_test, NULL);//创建test2任务
/**************************************************************************************************/

/*
*轮足平衡测试
*/
/***************************************************************************************************/
RC_WheelFoot_Chassis::State_data my_State = {0,0,0,0};//传感器得
RC_WheelFoot_Chassis::State_U    my_Accel = {0,0};
RC_WheelFoot_Chassis::State_K    my_K = {0,0,0,0};//matlab建模计算得
RC_WheelFoot_Chassis::State_Target my_target = {0,0,0};//速度设置，以及平衡的机械零点
float my_dt = 0;//间隔时间
float my_Ratio = 0;//输出的比例系数
float my_wheel_base = 0.5f;//轮距
float my_Turn_Ratio = 1;//转动系数
RC_WheelFoot_Chassis::WheelFoot_LQR my_WheelFoot(my_State,my_K,my_target,my_dt,my_wheel_base,my_Turn_Ratio);//实例化对象

void my_test2(void *argument){
	RC_WheelFoot_Chassis::State_K    my_K = {0,0,0,0};
	RC_WheelFoot_Chassis::Output_Velocity my_Output = {0,0};//速度输出
	RC_WheelFoot_Chassis::State_U    my_Accel = {0,0};//加速度
	RC_WheelFoot_Chassis::State_Target my_target = {0,0,0};
	float h =0;
	for(;;){
		//h根据传感器计算得
		//类似这样，这个是随便搞得，实际要matlab计算生成
		my_K.K1 = 0.000687*h*h-0.247225*h+6.542786;
		my_K.K2 = 0.000005*h*h-0.010043*h-0.658127;
		my_K.K3 = -0.000000*h*h+-0.000000*h+0.000000;
		my_K.K4 = 0.000058*h*h-0.020423*h-2.622680; 
		my_WheelFoot.Set_K(my_K);//设置K，主要是为了根据轮足不同高度设置不同的反馈系数
		//遥控输入参数，或上位机输入参数
		my_target.target_Pitch = 0;
		my_target.target_speed = 100;
		my_target.target_turn = 0;
		my_WheelFoot.input_data(my_target);
		
		my_State.X_Pose += my_State.X_speed*my_dt;//两轮几何中心位移
		my_State.X_speed = 0;//底盘两轮几何中心速度，左轮速度加右轮速度之和除于2
		my_State.Pitch = 0;//俯仰角,传感器获取
		my_State.Gyro = 0;//绕轮毂的角速度
		my_WheelFoot.Update_State(my_State);//更新状态
		my_Accel = my_WheelFoot.accel_calc();//计算反馈输入
		my_Output = my_WheelFoot.Get_Robot_Out();//获取电机输入
		osDelay(10);
	}
}
//task::TaskCreator test3_task("test2", 21, 256, my_test2, NULL);//创建test3任务
/***************************************************************************************************/



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