#pragma once
#include "RC_cdc.h"
#include <arm_math.h>


#ifdef __cplusplus
namespace ros
{
		// -1：未知
		// 0：无效
		// 1：空位
		// 2：我方方块
		// 3：敌方方块
		class Nine_Square : cdc::CDCHandler{
			public:
			Nine_Square(cdc::CDC &cdc_, uint8_t rx_id_);
			~Nine_Square() = default;
			//获取指定方格状态
			int8_t Get_Square(uint8_t n);
			int* Get_nine_square();
			//设置指定方格状态
			//n:1~9
			void Set_Square(uint8_t n, int8_t kfs);
			//是否初始化
			bool Is_Init() const {return is_init;}
			protected:
			int nine_square[9]={
				1,1,1,
				1,1,1,
				1,1,1
			};
			void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;

			private:
			bool is_init = true; 			
		};

}
#endif
