#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace motor
{
	typedef enum class M6020CtrlType
	{
		CURRENT,// 电流控制
		VOLTAGE // 电压控制
	} M6020CtrlType;
	
	class M6020 : public DjiMotor
	{
	public:
		M6020(uint8_t id_, can::Can &can_, tim::Tim &tim_, bool is_reset_pos_angle = false, M6020CtrlType control_type_ = M6020CtrlType::CURRENT);
		virtual ~M6020() {}

	protected:
		void Dji_Id_Init(uint8_t id_) override;
		void Can_Tx_Process() override;
	
	private:
		M6020CtrlType control_type = M6020CtrlType::CURRENT;
	};
}
#endif
