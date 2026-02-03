#pragma once
#include "RC_cdc.h"
#include <arm_math.h>


#ifdef __cplusplus
namespace ros
{
	typedef enum class Diraction
	{
		F,	// 前
		B,	// 后
		L,	// 左
		R,	// 右
		N	// 无效
	} Dir;

	// -1：未知
	// 0：无效
	// 1：r1
	// 2：r2
	// 3：fake
	// 4：空位
	class Map : cdc::CDCHandler
    {
    public:
		Map(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~Map() {}
		
		int8_t Get_MF(uint8_t n);
		void Set_MF(uint8_t n, int8_t kfs);
		
		bool Is_Init() const {return is_init;}
		
		int8_t Kfs_On_Dir(uint8_t n, Dir d);

    protected:
		uint8_t map[12] = 
		{
			1, 1, 1, 2, 2, 2, 2, 3, 4, 4, 4, 4
		};
	
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;

    private:
		bool is_init = true;/// 
    };
	
	
	
	
	
	
	
}
#endif
