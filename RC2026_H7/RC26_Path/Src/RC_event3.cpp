#include "RC_event3.h"

namespace path
{
	Event3* Event3::list[] = {nullptr};
	
	Event3::Event3(uint8_t id_, bool wait_finish_)
	{
		if (id_ > EVENT3_MAX_EVENT_NUM)/*超出id范围*/
		{
			id_ = 0;/*无效id*/
		}
		else
		{
			if (list[id_ - 1] == nullptr)/*检查是否有空位*/
			{
				list[id_ - 1] = this;
				id = id_;/*储存id*/
			}
		}
		
		trig_signal = false;
		finish_signal = false;
		
		wait_finish = wait_finish_;
	}
	
	void Event3::Trig_Once()
	{
		trig_signal = true;
	}
	
	bool Event3::Is_Trig()
	{
		if (trig_signal)
		{
			trig_signal = false;
			return true;
		}
		else
		{
			return false;
		}
	}
	
	void Event3::Finish()
	{
		finish_signal = true;
	}
	
	bool Event3::Is_Finish()
	{
		if (finish_signal)
		{
			finish_signal = false;
			return true;
		}
		else
		{
			return false;
		}
	}
}