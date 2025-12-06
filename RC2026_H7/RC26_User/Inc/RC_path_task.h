#pragma once
#include "RC_task.h"
#include "RC_path.h"
#include "RC_flysky.h"
#include "RC_radar.h"
#include "RC_best_path.h"
#include "RC_timer.h"
#include "RC_omni_chassis.h"

#ifdef __cplusplus


extern path::PathPlan path_plan;// 路径规划
extern flysky::FlySky remote_ctrl;// 遥控
extern chassis::OmniChassis omni_chassis;// 三全向轮底盘
extern ros::Radar radar;// 雷达数据接收
extern ros::BestPath MF_path;// 路径数据接收
extern ros::Map map;// 地图数据接收
extern timer::Timer timer_us;// 用于获取us级时间戳

#endif
