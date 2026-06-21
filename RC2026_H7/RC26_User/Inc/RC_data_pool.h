#pragma once
#include "RC_task.h"
#include "RC_timer.h"

#ifdef __cplusplus
namespace data
{
	/*================启动区===================*/
	class BootArea
    {
    public:
		BootArea() = delete;
		~BootArea() = delete;

		// 初始化启动区
		static void Init_Is_Boot_At_Mc(bool is_boot_at_mc_)
		{
			if (!is_init)
			{
				is_boot_at_mc = is_boot_at_mc_;
				is_init = true;
			}
		}
		
		static bool Is_Boot_Area_Init() { return is_init; }
		static bool Is_Boot_At_Mc() { return is_boot_at_mc; }
    private:
		static inline bool is_boot_at_mc = true;// 是否在武馆启动（默认启动区）
		static inline bool is_init = false;
    };
	/*================启动区===================*/
	
	
	
	
	
	
	
	
	/*================左右半场===================*/
	class Side
    {
    public:
		Side() = delete;
		~Side() = delete;
		
		// 初始化场地位置
		static void Init_Is_Blue_Left_Side(bool blue_left_side_)
		{
			if (!is_init)
			{
				blue_left_side = blue_left_side_;
				is_init = true;
			}
		}
		
		static bool Is_Side_Init() { return is_init; }
		static bool Is_Blue_Left_Side() { return blue_left_side; }
	
    private:
		static inline bool blue_left_side = true;  /*场地位置*/
		static inline bool is_init = false;
	
		friend class AllData;
    };
	/*================左右半场===================*/
	
	
	
	
	
	
	
	/*==============携带KFS的数量=====================*/
	class KFSNum
    {
    public:
		KFSNum() = delete;
		~KFSNum() = delete;
    
		// 初始化KFS数量
		static void Init_KFS_Num(uint8_t kfs_num)
		{
			if (!is_init)
			{
				KFS_num = kfs_num;
				is_init = true;
			}
		}
		
		static bool Is_KFS_Num_Init() { return is_init; }
		static uint8_t Get_KFS_Num() { return KFS_num; }
		static void KFS_Add_One() { KFS_num++; }
		static void KFS_Sub_One() { KFS_num--; }
		
    private:
		static inline uint8_t KFS_num = 0;
		static inline bool is_init = false;
	
		friend class AllData;
    };
	/*==============携带KFS的数量=====================*/
	
	
	
	
	
	/*==============是否持有武器=====================*/
	class HaveWeapon
    {
    public:
		HaveWeapon() = delete;
		~HaveWeapon() = delete;
    
		static void Init_Have_Weapon(bool have)
		{
			if (!is_init)
			{
				have_weapon = have;
				is_init = true;
			}
		}
		
		static bool Is_Have_Weapon_Init() { return is_init; }
		static bool Have_Weapon() { return have_weapon; }

    private:
		static inline bool have_weapon = false;
		static inline bool is_init = false;
	
		friend class AllData;
    };
	/*==============是否持有武器=====================*/
	
	
	
	
	/*==============是否对接=====================*/
	class IsDock
    {
    public:
		IsDock() = delete;
		~IsDock() = delete;

		static void Init_Is_Dock(bool is_dock_)
		{
			if (!is_init)
			{
				is_dock = is_dock_;
				is_init = true;
			}
		}
		
		static bool Is_Dock_Init() { return is_init; }
		static bool Is_Dock() { return is_dock; }
		
    private:
		static inline bool is_dock = false;
		static inline bool is_init = false;
	
		friend class AllData;
    };
	/*==============是否对接=====================*/
	

	
	/*==============取武器编号=====================*/
	class PickWeaponNum
    {
    public:
		PickWeaponNum() = delete;
		~PickWeaponNum() = delete;

		static void Init_Pick_Num(uint8_t pick_num_)
		{
			if (!is_init)
			{
				if (pick_num_ > 6)
				{
					pick_num_ = 0;
				}
				
				pick_num = pick_num_;
				is_init = true;
			}
		}
		
		static bool Is_Pick_Num_Init() { return is_init; }
		static bool Get_Pick_Num() { return pick_num; }
		
    private:
		static inline bool pick_num = false;
		static inline bool is_init = false;
	
		friend class AllData;
    };
	/*==============取武器编号=====================*/
	
	
	class AllData
    {
    public:
		AllData() = delete;
		~AllData() = delete;

		static bool Is_All_Init()
		{
			return (
				Side::Is_Side_Init() 				&&
				KFSNum::Is_KFS_Num_Init() 			&&
				HaveWeapon::Is_Have_Weapon_Init() 	&&
				IsDock::Is_Dock_Init()				&&
				BootArea::Is_Boot_Area_Init()
			);
		}
    };
	
	
	
	
	/*==============坐标=====================*/
	#define POSITION_TIME_OUT 1000000// us
	#define ORIENTATION_TIME_OUT 1000000// us

	class RobotPose
    {
    public:
		RobotPose();
		~RobotPose() = default;
		
		void Update_Position(float * x_, float * y_, float * z_);
		void Update_Orientation(float * yaw_, float * roll_, float * pitch_);
		
		constexpr float X() const {return x;}
		constexpr float Y() const {return y;}
		constexpr float Z() const {return z;}
		
		constexpr float Yaw() const {return yaw;}
		constexpr float Roll() const {return roll;}
		constexpr float Pitch() const {return pitch;}
		
		constexpr bool Is_Position_Valid() const {return position_is_valid;}
		constexpr bool Is_Orientation_Valid() const {return orientation_is_valid;}
		
		inline void Robot_Pose_Check()
		{
			uint32_t delta_time = timer::Timer::Get_DeltaTime(position_last_time);
			
			if (delta_time > POSITION_TIME_OUT)
			{
				position_is_valid = false;
			}

			delta_time = timer::Timer::Get_DeltaTime(orientation_last_time);
			
			if (delta_time > ORIENTATION_TIME_OUT)
			{
				orientation_is_valid = false;
			}
		}

    private:

		float x = 0;
		float y = 0;
		float z = 0;
			
		float yaw = 0;
		float roll = 0;
		float pitch = 0;
	
		bool position_is_valid = false;
		uint32_t position_last_time = 0;
			
		bool orientation_is_valid = false;
		uint32_t orientation_last_time = 0;
    };
	/*==============坐标=====================*/


}
#endif
