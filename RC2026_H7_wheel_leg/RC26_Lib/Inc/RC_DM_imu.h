#pragma once
#include "RC_can.h"
#include "RC_tim.h"
#include "RC_imu.h"

#define ACCEL_CAN_MAX (235.2f)
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1

#ifdef __cplusplus

namespace imu
{
    //imu通信模式
    typedef enum
    {
        COM_USB=0,
        COM_RS485,
        COM_CAN,
        COM_VOFA

    }imu_com_port_e;

    //imu波特率
    typedef enum
    {
        CAN_BAUD_1M=0,
        CAN_BAUD_500K,
        CAN_BAUD_400K,
        CAN_BAUD_250K,
        CAN_BAUD_200K,
        CAN_BAUD_100K,
        CAN_BAUD_50K,
        CAN_BAUD_25K
        
    }imu_baudrate_e;

    //imu寄存器地址
    typedef enum 
    {
        REBOOT_IMU=0,//重启imu
        ACCEL_DATA,//加速度数据
        GYRO_DATA,//角速度数据
        EULER_DATA,//欧拉角数据
        QUAT_DATA,//四元数数据
        SET_ZERO,//角度置零
        ACCEL_CALI,//加速度计六面校准
        GYRO_CALI,//陀螺仪静态校准
        MAG_CALI,//磁计椭球校准
        CHANGE_COM,//切换通信模式
        SET_DELAY,//设置主动发送间隔
        CHANGE_ACTIVE,//切换主被动模式
        SET_BAUD,//设置CAN波特率
        SET_CAN_ID,//设置CAN ID
        SET_MST_ID,//设置主站 ID
        DATA_OUTPUT_SELECTION,//数据输出选择
        SAVE_PARAM=254,//保存参数
        RESTORE_SETTING=255//还原出厂设置
    }reg_id_e;

    class DM_imu : public can::CanHandler,public imu::imu
    {
    public:
        DM_imu(uint8_t id_,can::Can &can_);
        virtual ~DM_imu() {}

        void init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan);
        void write_reg(uint8_t reg_id_,uint32_t data_);
        void read_reg(uint8_t reg_id_);
        inline void reboot();
        inline void accel_calibration();
        inline void gyro_calibration();
        inline void change_com_port(imu_com_port_e port);
        inline void set_active_mode_delay(uint32_t delay);
        inline void change_to_active();
        inline void change_to_request();
        inline void set_baud(imu_baudrate_e baud);
        inline void set_can_id(uint8_t can_id);
        inline void set_mst_id(uint8_t mst_id);
        inline void save_parameters();
        inline void restore_settings();
        inline void request_accel();
        inline void request_gyro();
        inline void request_euler();
        inline void request_quat();


        void CanHandler_Register() override;
        void Can_Tx_Process() override;
        void Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data) override;
    

    protected:

    private:
        uint8_t id;
        uint8_t master_id;
				uint8_t reg_id;
				uint8_t ac;
				uint32_t data;
		
        int float_to_uint(float x_float, float x_min, float x_max, int bits);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);

        void IMU_UpdateAccel(uint8_t* pData);
        void IMU_UpdateGyro(uint8_t* pData);
        void IMU_UpdateEuler(uint8_t* pData);
        void IMU_UpdateQuaternion(uint8_t* pData);
        void IMU_UpdateData(uint8_t* pData);

    };
}
#endif
