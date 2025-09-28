#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#include <math.h>

#ifdef __cplusplus
namespace motor
{
	class DjiMotor : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		DjiMotor(can::Can &can_, tim::Tim &tim_);
		virtual ~DjiMotor() {}
		
		pid::Pid pid_spd, pid_pos;

    protected:
		virtual void Dji_Id_Init(uint8_t id_) = 0;

		void CanHandler_Register() override;
		void Tim_It_Process() override;
		void Can_Tx_Process() override;
		void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;

		bool can_rx_is_first = true;
		uint8_t id;
	
    private:
		
    };
}
#endif