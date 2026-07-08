#pragma once

#include <stdint.h>
#include "spi.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "RC_task.h"
#include "RC_timer.h"

// 根据spi频率的不同，需要更改编码0和编码1的字节大小
//            编码 0 : 11100000
#define CODE_0      0xE0
//            编码 1 : 11111100
#define CODE_1      0xFC

// ws2812b灯珠数量
#define WS2812B_AMOUNT     28

// ws2812b 全局亮度 (0x00~0xFF)，修改此值统一控制所有颜色亮度
#define WS2812B_BRIGHTNESS  0x30//0x3F

// C 兼容颜色枚举
typedef enum
{
    WS2812B_COLOR_RED = 0,
    WS2812B_COLOR_GREEN,
    WS2812B_COLOR_BLUE,
    WS2812B_COLOR_WHITE,
    WS2812B_COLOR_YELLOW,
    WS2812B_COLOR_NONE
} WS2812B_Color;

#ifdef __cplusplus
namespace ws2812
{
    // LED颜色缓存
    struct ColorCache
    {
        uint8_t R;
        uint8_t G;
        uint8_t B;
    };

    // 灯带颜色
    enum class Color : uint8_t
    {
        RED,
        GREEN,
        BLUE,
        WHITE,
        YELLOW,
        NONE
    };

    class Ws2812B : public task::ManagedTask
    {
    public:
        Ws2812B(SPI_HandleTypeDef* hspi_);
        ~Ws2812B() = default;

        // 公共接口：设置常亮颜色
        void SetRed();
        void SetGreen();
        void SetBlue();
        void SetWhite();
        void SetYellow();
        void SetNone();

        // 公共接口：当前颜色闪烁两次（灭→恢复）
        void FlashOnce(Color color1, Color color2);

        // 公共接口：持续在两种颜色之间闪烁，设置常亮颜色后停止
        void FlashContinuous(Color color1, Color color2);

        // 设置全部灯为指定颜色
        void SetAllColor(uint8_t r, uint8_t g, uint8_t b);

    protected:
        void Task_Process() override;

    private:
        // SPI 句柄
        SPI_HandleTypeDef* hspi;

        // FreeRTOS 队列句柄
        QueueHandle_t signal_queue;

        // 常量
        static constexpr uint8_t RESET_BYTES = 100;

        // SPI 数据缓存 (DMA 需要放在 AXI SRAM)
        static uint8_t spi_buf[WS2812B_AMOUNT * 24 + RESET_BYTES];

        // 灯条颜色显存
        ColorCache color_cache[WS2812B_AMOUNT];

        // 将单颗灯珠的 RGB 值编码为 SPI 波形数据
        void EncodeLed(uint16_t led_index, uint8_t r, uint8_t g, uint8_t b);

        // 通过 SPI DMA 发送灯条数据
        void SpiSend();

        // 队列命令
        enum class CommandType : uint8_t
        {
            SET_COLOR,
            FLASH_ONCE,
            FLASH_CONTINUOUS
        };

        struct Command
        {
            CommandType type;
            Color color1;
            Color color2;
        };

        enum class FlashState : uint8_t
        {
            STEADY,
            ONCE_COLOR1_FIRST,
            ONCE_COLOR2_FIRST,
            ONCE_COLOR1_SECOND,
            ONCE_COLOR2_SECOND,
            CONTINUOUS_COLOR1,
            CONTINUOUS_COLOR2
        };

        // LED 内部函数
        void ApplyColor(Color color);
        void LedOff();
        void SetColor(Color color);
        void SendFlashCommand(CommandType type, Color color1, Color color2);

        // 状态机变量
        FlashState flash_state = FlashState::STEADY;
        uint32_t start_time = 0;
        Color current_color = Color::NONE;   // 初始颜色
        Color flash_color1 = Color::NONE;
        Color flash_color2 = Color::NONE;

        // 闪烁间隔 (系统 tick)
        static constexpr uint32_t TIME_INTERVAL = 80000;
    };

    // 全局实例指针，供 C 兼容接口使用
    extern Ws2812B* g_ws2812b_instance;
}

extern "C" {
#endif

// C 兼容接口
void WS2812B_SetRed(void);
void WS2812B_SetGreen(void);
void WS2812B_SetBlue(void);
void WS2812B_SetWhite(void);
void WS2812B_SetYellow(void);
void WS2812B_SetNone(void);
void WS2812B_FlashOnce(WS2812B_Color color1, WS2812B_Color color2);
void WS2812B_FlashContinuous(WS2812B_Color color1, WS2812B_Color color2);

#ifdef __cplusplus
}
#endif





