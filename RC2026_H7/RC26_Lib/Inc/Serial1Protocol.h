// Serial1Protocol.h
#ifndef SERIAL1_PROTOCOL_H
#define SERIAL1_PROTOCOL_H

#include "string.h"
#include <stdint.h>
#include "usart.h"
#include "RC_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 协议常量定义 */
#define SERIAL1_FRAME_HEAD0     0xFC
#define SERIAL1_FRAME_HEAD1     0xFB
#define SERIAL1_FRAME_TAIL0     0xFD
#define SERIAL1_FRAME_TAIL1     0xFE

#define SERIAL1_DATA_LEN        3
#define SERIAL1_FRAME_LEN       8

#define SERIAL1_SEND_TIMES      3          // 每批发送3次
#define SERIAL1_RETRY_INTERVAL  500        // 重发间隔500ms

/* 校验字节位定义 */
#define SERIAL1_CHECKSUM_MASK   0x3F
#define SERIAL1_PARITY_BIT_MASK 0x40

/* 状态枚举 */
typedef enum {
    SERIAL1_STATE_IDLE,           // 空闲
    SERIAL1_STATE_SENDING,        // 主动发送中
    SERIAL1_STATE_WAITING_ACK     // 等待主动发送的应答
} Serial1State_t;

/* 发送结果回调 */
typedef void (*Serial1SendResultCallback)(uint8_t* data, uint8_t parity, uint8_t success);

/* 数据接收回调（串口2主动发来的数据） */
typedef void (*Serial1DataReceiveCallback)(uint8_t* data, uint8_t parity);

class Serial1Protocol {
public:
    static Serial1Protocol& getInstance();
    
    void init(UART_HandleTypeDef* huart);
    void process(void);
    
    // 主动发送指令（等待应答）
    bool sendCommand(uint8_t* data);
    
    // 串口回调
    void onUartReceive(uint8_t* buffer, uint16_t size);
    void onUartTxComplete(void);
    
    void setSendResultCallback(Serial1SendResultCallback callback);
    void setDataReceiveCallback(Serial1DataReceiveCallback callback);
    
    // 获取接收到的数据（轮询方式）
    void getReceivedData(uint8_t* data_out, uint8_t* parity_out);
    bool hasNewData(void);

private:
    Serial1Protocol();
    
    uint8_t calculateChecksum(uint8_t* data);
    void buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out);
    int parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out);
    void sendFrame(uint8_t* data, uint8_t parity);
    uint32_t getTickMs(void);
    void startSending(void);
    void notifySendResult(uint8_t success);
    
    void startRetransmit(void);
    void stopRetransmit(void);
    
    // 奇偶位管理
    int findHistoryIndex(uint8_t* data);
    uint8_t getNextParity(uint8_t* data);
    void updateSendHistory(uint8_t* data, uint8_t parity);
    
    // 应答发送
    void sendAckFrame(void);
    
private:
    UART_HandleTypeDef* m_huart;
    
    Serial1State_t m_state;
    Serial1SendResultCallback m_resultCallback;
    Serial1DataReceiveCallback m_dataReceiveCallback;
    
    // 主动发送相关
    uint8_t m_current_send_data[SERIAL1_DATA_LEN];
    uint8_t m_current_send_parity;
    uint8_t m_uart_send_frame[SERIAL1_FRAME_LEN];
    uint32_t m_send_start_time;
    uint32_t m_send_first_start_time;
    uint8_t m_send_batch_count;
    uint32_t m_retry_count;
    volatile uint8_t m_tx_complete;
    uint8_t m_waiting_for_response;
    
    // 接收相关
    uint8_t m_rx_buffer[30];
    volatile uint8_t m_rx_ready;
    volatile uint16_t m_rx_size;
    
    uint8_t m_received_data[SERIAL1_DATA_LEN];
    uint8_t m_received_parity;
    uint8_t m_new_data_available;
    
    // 接收去重（用于主动数据）
    uint8_t m_last_rx_data[SERIAL1_DATA_LEN];
    uint8_t m_last_rx_parity;
    uint32_t m_last_rx_time;
    
    // 主动数据应答管理（用于判断相同数据重发）
    uint8_t m_last_processed_data[SERIAL1_DATA_LEN];   // 上次处理的数据
    uint8_t m_last_processed_parity;                  // 上次处理的奇偶
    
    // 发送历史记录
    #define MAX_HISTORY 10
    
    typedef struct {
        uint8_t data[SERIAL1_DATA_LEN];
        uint8_t last_parity;
        uint8_t send_count;
    } SendHistory_t;
    
    SendHistory_t m_send_history[MAX_HISTORY];
    uint8_t m_history_count;
    
    uint8_t m_command_send_data[SERIAL1_DATA_LEN];
    uint8_t m_command_send_parity;
    
    // 定时器变量
    uint32_t m_last_send_time;
};

#ifdef __cplusplus
}
#endif

#endif