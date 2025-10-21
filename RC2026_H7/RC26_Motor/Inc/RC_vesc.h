//位置式非常不建议使用
#pragma once
#include "RC_motor.h"
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_pid.h"

#ifdef __cplusplus

enum vesc_mode {
    vesc_current,
    vesc_rpm,
	vesc_pos,
	vesc_duty
};
typedef enum {
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,
	CAN_PACKET_FILL_RX_BUFFER				= 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG			= 6,
	CAN_PACKET_PROCESS_RX_BUFFER			= 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER			= 8,
	CAN_PACKET_STATUS						= 9,
	CAN_PACKET_SET_CURRENT_REL				= 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL		= 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE		= 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL	= 13,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_PING							= 17,
	CAN_PACKET_PONG							= 18,
	CAN_PACKET_DETECT_APPLY_ALL_FOC			= 19,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES		= 20,
	CAN_PACKET_CONF_CURRENT_LIMITS			= 21,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS	= 22,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN		= 23,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN	= 24,
	CAN_PACKET_CONF_FOC_ERPMS				= 25,
	CAN_PACKET_CONF_STORE_FOC_ERPMS			= 26,
	CAN_PACKET_STATUS_5						= 27,
	CAN_PACKET_POLL_TS5700N8501_STATUS		= 28,
	CAN_PACKET_CONF_BATTERY_CUT				= 29,
	CAN_PACKET_CONF_STORE_BATTERY_CUT		= 30,
	CAN_PACKET_SHUTDOWN						= 31,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4			= 32,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8			= 33,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12			= 34,
	CAN_PACKET_IO_BOARD_DIGITAL_IN			= 35,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL	= 36,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM		= 37,
	CAN_PACKET_BMS_V_TOT					= 38,
	CAN_PACKET_BMS_I						= 39,
	CAN_PACKET_BMS_AH_WH					= 40,
	CAN_PACKET_BMS_V_CELL					= 41,
	CAN_PACKET_BMS_BAL						= 42,
	CAN_PACKET_BMS_TEMPS					= 43,
	CAN_PACKET_BMS_HUM						= 44,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT		= 45,
	CAN_PACKET_PSW_STAT						= 46,
	CAN_PACKET_PSW_SWITCH					= 47,
	CAN_PACKET_BMS_HW_DATA_1				= 48,
	CAN_PACKET_BMS_HW_DATA_2				= 49,
	CAN_PACKET_BMS_HW_DATA_3				= 50,
	CAN_PACKET_BMS_HW_DATA_4				= 51,
	CAN_PACKET_BMS_HW_DATA_5				= 52,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL			= 53,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL			= 54,
	CAN_PACKET_UPDATE_PID_POS_OFFSET		= 55,
	CAN_PACKET_POLL_ROTOR_POS				= 56,
	CAN_PACKET_NOTIFY_BOOT					= 57,
	CAN_PACKET_STATUS_6						= 58,
	CAN_PACKET_GNSS_TIME					= 59,
	CAN_PACKET_GNSS_LAT						= 60,
	CAN_PACKET_GNSS_LON						= 61,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP			= 62
}
CAN_PACKET_ID;

namespace vesc {
    class Vesc : public motor::Motor, public can::CanHandler, public tim::TimHandler {
    public:
        Vesc(uint8_t id_, can::Can &can_, tim::Tim &tim_);
        virtual ~Vesc() {}

        
        void Set_Rpm(float target_rpm_);
        void Set_Current(float target_c_);
		void Set_Pos(float target_pos_);
		void Set_Duty(float target_duty_);
        void UpdateTxId();
        pid::Pid pid_spd, pid_pos;

    protected:
        void CanHandler_Register() override;
        void Tim_It_Process() override;
        void Can_Tx_Process() override;
        void Can_Rx_It_Process(uint8_t *rx_data) override;

    private:
        uint8_t id;
        float gear_ratio = 1;
        int motor_polse = 7;
        float erpm = 0;
		float duty = 0;
		float target_duty = 0;
        int32_t send_current = 0;
        int32_t send_rpm = 0;
        int32_t send_pos = 0;
		int32_t send_duty = 0;
        vesc_mode vesc_motor_mode = vesc_current;  // Ĭ�ϵ���ģʽ

       
    };
}
#endif