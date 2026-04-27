#pragma once

/* api */
#include "RC_task.h"
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_serial.h"
#include "RC_cdc.h"
#include "RC_timer.h"
#include "RC_cordic.h"

/* motor */
#include "RC_m3508.h"
#include "RC_m2006.h"
#include "RC_m6020.h"
#include "RC_dm4310.h"
#include "RC_go.h"
#include "RC_rs04.h"
#include "RC_vesc.h"
#include "RC_j60.h"

/* lib */
#include "RC_wave_generator.h"
#include "RC_flysky.h"
#include "RC_data_pool.h"
#include "RC_LiDAR.h"
#include "RC_omni_chassis.h"

/* path */
#include "RC_traj_plan3.h"
#include "RC_traj_track3.h"
#include "RC_path3.h"
#include "RC_path_plan3.h"
#include "RC_map_graph.h"
#include "RC_head_check_event.h"
#include "RC_graph_plan.h"

/* ROS */
#include "RC_radar.h"
#include "RC_best_path.h"

/* HAL */
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

#ifdef __cplusplus

#endif

#ifdef __cplusplus
extern "C" {
#endif

/* C interface */
void All_Init();

#ifdef __cplusplus
}
#endif