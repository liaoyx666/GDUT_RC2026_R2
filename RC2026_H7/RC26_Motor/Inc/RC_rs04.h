#pragma once
#include "RC_motor.h"
#include "RC_can.h"
#include "RC_tim.h"


#define RS04_COM_TYPE_0_ID 0x0
#define RS04_COM_TYPE_1_CONTROL 0x1
#define RS04_COM_TYPE_2_FEEDBACK 0x2
#define RS04_COM_TYPE_3_ENABLE 0x3
#define RS04_COM_TYPE_4_DISABLE 0x4
#define RS04_COM_TYPE_6_RESET_POS 0x6
#define RS04_COM_TYPE_7_SET_ID 0x7
#define RS04_COM_TYPE_11 0x11
#define RS04_COM_TYPE_12 0x12
#define RS04_COM_TYPE_15 0x15
#define RS04_COM_TYPE_16 0x16
#define RS04_COM_TYPE_18 0x18


#ifdef __cplusplus
namespace motor
{
	class RS04 : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		RS04(uint8_t id_, can::Can& can_, tim::Tim& tim_, bool use_mit_ = false, float k_spd_ = 0, float k_pos_ = 0);
		virtual ~RS04() {}
		
		void Set_K_Pos(float target_k_pos_) override;
		void Set_K_Spd(float target_k_spd_) override;
			
		void Set_Can_Id(uint8_t id_);
		void Set_ZeroPos();

    protected:
		void CanHandler_Register() override;
		void Tim_It_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t* rx_data) override;
		void Can_Tx_Process() override;
		
    private:
		uint8_t id = 0;
		uint8_t master_id = 0;
		
		bool use_mit = false;
		bool is_enable = false;
	
		uint8_t target_id = 0;
		
		uint8_t tx_com_type = RS04_COM_TYPE_3_ENABLE;
    };
}
#endif
