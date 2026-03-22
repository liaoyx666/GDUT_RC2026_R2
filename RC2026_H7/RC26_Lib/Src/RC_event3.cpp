#include "RC_event3.h"

namespace path
{
	Event3* Event3::list[] = {nullptr};
	
	Event3::Event3(uint8_t id_)
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
	}
}