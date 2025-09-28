#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#ifdef __cplusplus


#define CONTROL_MODE_1		10
#define CONTROL_WRITE_K 	11
#define CONTROL_READ_K 		12
#define CONTROL_MODE_2		13

#define STATUS_LOCK			0
#define STATUS_FOC			1
#define STATUS_CALIBRATE	2

namespace motor
{
	class Go : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		Go(uint8_t id_, uint8_t module_id_, can::Can &can_, tim::Tim &tim_);
		virtual ~Go() {}
		
    protected:
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;
		void CanHandler_Register() override;
	
    private:
		uint16_t k_spd = 0, k_pos = 0;
		uint8_t error_code = 0;
		uint8_t air = 0;// 气压参数
	
		uint8_t status = STATUS_FOC;
		uint8_t control_mode = CONTROL_MODE_1;
		uint8_t id;
		uint8_t module_id;
    };
}
#endif
