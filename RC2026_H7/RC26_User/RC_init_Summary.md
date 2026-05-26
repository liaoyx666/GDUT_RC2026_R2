# RC_init.cpp 当前状态总结

> 编写日期：2026-05-26
> 目的：从 main 拉取更新后，init.cpp 会被覆盖。本文档记录当前 init.cpp 的逻辑和正在测试的 Aim 功能，方便重新设计。

---

## 一、硬件外设配置

### CAN 总线（3路）
| CAN | 用途 | Filter 配置 |
|-----|------|------------|
| can1 (hfdcan1) | 底盘 4×M3508 | StdID→FIFO0, ExtID→FIFO1 |
| can2 (hfdcan2) | 龙门架电机 (M2006D×2, M3508D×2, M2006×1, DM4340×1) | StdID→FIFO0, ExtID→FIFO1 |
| can3 (hfdcan3) | 抬升 M3508×2 + 辅助轮 M2006×2 | StdID→FIFO0, ExtID→FIFO1 |

### 定时中断
| 定时器 | 频率 | 用途 |
|--------|------|------|
| tim7 | 1 kHz | DM4340 电机控制 |
| tim13 | 500 Hz | 大部分电机控制（底盘/龙门架/抬升） |
| tim4 | 500 Hz | 预留（已使能中断） |

### 通信外设
| 外设 | 接口 | 用途 |
|------|------|------|
| CDC_HS | USB HS 虚拟串口 | 与上位机(ROS)通信：雷达定位、相机数据 |
| UART3 | 串口 | LiDAR 激光测距 |
| GPIO_PIN_8 | IBUS | FlySky 遥控接收 |

---

## 二、电机清单

### CAN1 — 底盘
```
M3508 ×4  → 全向轮底盘（omni_4_chassis）
```

### CAN2 — 龙门架
```
M2006D (id:3,4)  → Gantry X轴，双电机对偶，减速比 36:1
M3508D (id:1,2)  → Gantry Y轴，双电机对偶，减速比 3591/187:1
M2006  (id:5)    → Gantry Z轴
DM4340 (id:0x12) → Gantry Pitch轴
```

### CAN3 — 抬升
```
M3508 (id:5) → 抬升左，差速配置，减速比 51:1
M3508 (id:6) → 抬升右，差速配置，减速比 51:1
M2006 (id:7) → 辅助轮左
M2006 (id:8) → 辅助轮右
```

---

## 三、核心模块对象及依赖关系

```
robot_pose (RobotPose)               ← 被 radar/camera/track/head_ctrl/auto_lift 共享
radar (Radar)                         ← 依赖 CDC_HS, robot_pose
camera (Camera)                       ← 依赖 CDC_HS, robot_pose
remote_ctrl (FlySky)                  ← GPIO_PIN_8
lidar_1 (LiDAR)                       ← UART3

omni_4_chassis (Omni4Chassis)         ← 依赖 4×M3508, robot_pose
lift (LiftChassis)                    ← 依赖 M3508×2, M2006×2, omni_4_chassis

head_ctrl (HeadCtrl)                 ← 依赖 robot_pose, omni_4_chassis
track (TrajTrack3)                   ← 依赖 robot_pose, omni_4_chassis, head_ctrl, 周期 5ms
path_plan (PathPlan3)                ← 依赖 track, 约束: LonConstr3(2.0,2.1), HeadConstr3(0,3,4,false)
graph_plan (GraphPlan)               ← 依赖 path_plan
navigation (Navigation)              ← 依赖 graph_plan
auto_lift (AutoLift)                 ← 依赖 lift, track, robot_pose

gan (Gantry)                         ← 依赖 M2006D×2, M2006×1, M3508D×2, DM4340×1
suck (Suction)                       ← 依赖 GPIOG, GPIO_PIN_7
getKFS (GetKFS)                      ← 依赖 gan, suck, lidar_1

aim_ctrl (Aim_Ctrl)                  ← 依赖 camera, omni_4_chassis, gan, aim_yaw_pid, aim_z_pid, aim_y_pid
```

---

## 四、Aim 功能详解（当前测试重点）

### 4.1 是什么
`aim::Aim_Ctrl` 是**相机对准控制**：通过相机检测目标的 4 轴坐标 (X, Y, Z, Yaw)，分阶段控制底盘旋转 + 龙门架 Y/Z 轴，使末端执行器对准目标。

### 4.2 状态机流程（6 阶段）

```
Phase_Check  →  Phase_Yaw  →  Phase_YZ_Coarse  →  Phase_Z  →  Phase_Y  →  Phase_Done
```

| 阶段 | 功能 | 退出条件 | 当前状态 |
|------|------|---------|---------|
| **Phase_Check** | 等待相机检测目标，5帧判稳 X/Y/Z | 三轴均稳定 | **被跳过**（`#if 1` 直跳 Phase_YZ_Coarse） |
| **Phase_Yaw** | 底盘 yaw 角补正（PID 控制角速度） | — | **被跳过**（直接 break 到 Phase_YZ_Coarse） |
| **Phase_YZ_Coarse** | Y+Z 双轴同时粗调，低通滤波后设龙门架位置 | 20帧内误差<0.02 且 |error|<0.05 | 正常运行 |
| **Phase_Z** | Z 轴精调 | 60帧内误差<0.002 | 正常运行 |
| **Phase_Y** | Y 轴精调 | 60帧内误差<0.002 | 正常运行 |
| **Phase_Done** | 对准完成，保持位置 | — | 正常运行 |

### 4.3 关键参数

**PID（待调参）：**
```cpp
aim_yaw_pid:  Kp=0.2, Ki=0, Kd=0, 输出限幅 ±0.08
aim_z_pid:    Kp=0.2, Ki=0, Kd=0, 输出限幅 ±0.002
aim_y_pid:    Kp=0.2, Ki=0, Kd=0, 输出限幅 ±0.002
```

**低通滤波器：**
```cpp
Z 通道: 截止频率 0.60 Hz, 采样率 1000 Hz
Y 通道: 截止频率 0.60 Hz, 采样率 1000 Hz
```

**数据保护：**
- 四轴全零 = 相机未识别到目标，直接 return 不处理
- 滑动窗口判稳（Frame_Stable），仅在新帧到达时推进窗口
- `Check_New_Data()` 保证帧到场才更新，避免重复处理同一帧

### 4.4 当前 TODO 标记

| 位置 | 内容 | 说明 |
|------|------|------|
| RC_aim.cpp:54 | `#if 1 // TODO: 验证完滑窗改回 0` | Phase_Check 被跳过，验证完后恢复完整流程 |
| RC_init.cpp:191 | `if (0)  // TODO: 验证完改回 remote_ctrl.swc != 2` | 遥控龙门架调试功能被禁用 |
| RC_init.cpp:232 | `aim_ctrl.Run();  // TODO: 验证完滑动窗口改回 Reset` | 两个分支都调用 Run()，正常应该一个 Reset |

---

## 五、Main_Task 主循环逻辑

### 启动时（进循环前，仅执行一次）
```cpp
gan.Set_Defualt_Td();         // 龙门架恢复默认速度/加速度
gan.Set_Reset_Pos();          // 龙门架回零位 (X=0.03, Y=0, Z=0, P=0)
navigation.Go_To_Get_KFS(5, DIR_B);   // 导航到 KFS 点位5，方向B
navigation.Go_To_Get_KFS(6, DIR_L);   // 导航到 KFS 点位6，方向L
navigation.Go_To_Do( (10.42, -4.53), PI/2, EVENT3_NULL );  // 导航到指定坐标
```

### 主循环（1ms 周期）
```
1. path_plan.Plan()           — 始终运行路径规划
2. robot_pose.Robot_Pose_Check() — 检查位姿超时
3. getKFS.Auto_Get_KFS()      — 自动取KFS检测
4. gan.Gantry_Base()          — 龙门架互斥控制
5. camera.Send_QR_Req()       — 发送二维码识别请求

6. SWA=1 → path_plan.Enable()  启用自主路径
   SWA=0 → path_plan.Disable()
           ├─ SWC=2 → aim_ctrl.Run()  
           └─ SWC≠2 → aim_ctrl.Run() + 底盘手动(摇杆) + 抬升(上/20)
```

### 遥控器通道映射
| 通道 | 功能 |
|------|------|
| SWA | 路径规划使能（1=自主 / 0=手动） |
| SWC | 模式选择（0/1=雷达重定位 / 2=aim测试） |
| SWD(边沿) | 触发雷达重定位 |
| left_y | 底盘 X 方向速度 |
| left_x | 底盘 Y 方向速度 |
| right_x | 底盘旋转角速度 |

---

## 六、Path_Task 独立线程

优先级 31（高于 Main_Task 的 20），1ms 周期：
```cpp
track.Traj_Track()      // 轨迹跟踪
head_ctrl.Head_Ctrl()   // 航向控制
auto_lift.Auto_Lift()   // 自动上下台阶
```

---

## 七、All_Init() 初始化顺序

```
1. can1 滤波初始化 + 启动
2. can2 滤波初始化 + 启动
3. can3 滤波初始化 + 启动
4. tim4_500hz  中断使能
5. tim7_1khz   中断使能
6. tim13_500hz 中断使能
7. 系统时间戳启动
8. lidar_1 串口接收启动
9. data::Init_Side(true)  — 场地位置设为蓝方左侧
10. Motor_Config()         — 所有电机 PID 参数 + 限幅 + aim PID
```

---

## 八、重新设计时的注意事项

1. **Aim 状态机是半成品**：Phase_Check 和 Phase_Yaw 被跳过，PID 参数标注"待调参"，Yaw PID 声明了但实际未使用（Phase_Yaw 直接 break）
2. **Aim 直接操作 Gantry**：没有走 `GantryUser` 的互斥机制，可能与 `getKFS` / `gan.Gantry_Base()` 冲突
3. **导航硬编码**：启动时的三条导航指令是写死的测试路径
4. **ALL_Init 中 `Init_Side(true)` 硬编码为蓝方左侧**，实际比赛需要根据场地配置
5. **`ctrl::AutoCtrl` 是空壳**：头文件有类声明，源文件 namespace 为空，可能是预留给整体自动控制的接口
6. **lift 对象当前没有真正用到自动抬升**：主循环中 lift 相关代码被注释掉，只用手动控制
7. **can3 的滤波 ID 号与其他 CAN 有重叠**（都用 1-6 号 filter），需确认是否是 CubeMX 的默认配置
