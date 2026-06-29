#pragma once

#ifdef __cplusplus
#include "RC_cdc.h"
#include "RC_data_pool.h"

namespace ros
{
	class GetData : public cdc::CDCHandler
    {
    public:
		GetData(cdc::CDC &cdc_, uint8_t rx_id_);
		~GetData() = default;
	
    private:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override
		{
			if (len == 6)
			{
				if (!data::AllData::Is_All_Init())
				{
					is_blue_side 	= (bool)buf[0];
					kfs_num			= buf[1];
					weapon_num      = (bool)buf[2];
					is_dock         = (bool)buf[3];
					pick_weapon_num = buf[4];
					boot_area       = (bool)buf[5];
					
					if (kfs_num > 4) kfs_num = 4;
					if (pick_weapon_num > 6) pick_weapon_num = 6;
					
					data::Side::Init_Is_Blue_Left_Side(is_blue_side);
					data::KFSNum::Init_KFS_Num(kfs_num);
					data::HaveWeapon::Init_Have_Weapon(weapon_num);
					data::IsDock::Init_Is_Dock(is_dock);
					data::BootArea::Init_Is_Boot_At_Mc(boot_area);
					data::PickWeaponNum::Init_Pick_Num(pick_weapon_num);
				}
				
				// 应答
				uint8_t ack = 1;
				cdc->CDC_Send_Pkg(9, &ack, 1, 10);
			}
		}
		
		bool 	is_blue_side;
		uint8_t kfs_num;
		bool weapon_num;
		bool 	is_dock;
		uint8_t pick_weapon_num;
		bool 	boot_area;
    };
}
#endif
