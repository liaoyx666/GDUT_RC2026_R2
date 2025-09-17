#pragma once
#include "cmsis_os.h"
#include "main.h"

#ifdef __cplusplus

#define TASK_NAME(var) #var
#define MAX_TASK_NUM 12// 最大可管理任务数量


namespace task
{


	typedef enum TaskType
	{
		TASK_DELAY,// 延时任务
		TASK_PERIOD// 周期任务
	} TaskType;

	class TaskCreator
	{
	public:
		TaskCreator(
			const char *name, 
			uint8_t priority, 
			uint16_t stack_size, 
			osThreadFunc_t func, 
			void *argument
		);
		virtual ~TaskCreator() {}
		
	protected:
		osThreadId_t TaskHandle;// 任务句柄
	private:
		
	};




	class ManagedTask : public TaskCreator
	{
	public:
		ManagedTask(
			const char *name, 
			uint8_t priority, 
			uint16_t stack_size, 
			TaskType task_type_,
			uint8_t ticks_
		);
		virtual ~ManagedTask() {}
	protected:
		
	private:
		uint8_t ticks;
		TaskType task_type;

		virtual void Task_Process() = 0;
		uint8_t task_list_dx;
		
		static void All_Task_Process(void *argument);
		static ManagedTask *task_list[MAX_TASK_NUM];// 可管理任务列表
		static uint8_t task_num;// 可管理任务数量
	};


}

#endif
