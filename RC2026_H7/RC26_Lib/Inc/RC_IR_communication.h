#ifndef RC_IR_COMMUNICATION_H
#define RC_IR_COMMUNICATION_H

#include "main.h"
#include "string.h"
#include "RC_serial.h"

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
#define DATA_BUFFER_SIZE        16

typedef enum {
    DATA_TYPE_KFS = 0,
    DATA_TYPE_CMD = 1
} DataType_t;

// 数据包结构体
typedef struct {
    uint8_t type;     // 类型：KFS=0, CMD=1
    union {
        uint8_t cmd;      // 命令字节
        uint8_t kfs[3];   // KFS 3字节数据
    } data;
} DataPacket_t;

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

extern volatile uint8_t g_send_complete;
extern volatile uint8_t g_send_success;

#ifdef __cplusplus

    class Serial1Protocol : public serial::UartRx {
public:
    static Serial1Protocol& getInstance();
    
    void init(UART_HandleTypeDef* huart);
    void process(void);
    
    // 串口中断回调接口
    void Uart_Rx_It_Process(uint8_t* buffer, uint16_t size) override;

    void getReceivedData(uint8_t* data_out, uint8_t* parity_out);
    void R1_Send_KFS(uint8_t KFS1, uint8_t KFS2, uint8_t KFS3);
    void sendAckFrame(void);
    
    bool hasData() const { return m_has_latest_data; }
    bool getLatestData(DataPacket_t* packet);
    void onUartReceive(uint8_t* buffer, uint16_t size);
    
    private:
    Serial1Protocol(); 
    uint8_t calculateChecksum(uint8_t* data);
    void buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out);
    int parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out);
    void sendFrame(uint8_t* data, uint8_t parity);
    bool has_consecutive_zeros_exceed_10(const uint8_t* data); 
    
    // 奇偶位管理
    int findHistoryIndex(uint8_t* data);
    uint8_t getNextParity(uint8_t* data);
    void storeReceivedData(uint8_t* data, uint8_t parity);
    
private:
    UART_HandleTypeDef* m_huart;
    
     #if defined(__GNUC__)
    uint8_t m_rx_work_buffer[32] __attribute__((aligned(32)));
    #else
    __align(32) uint8_t m_rx_buffer[32];
    __align(32) uint8_t m_rx_work_buffer[32];
    #endif

    volatile uint8_t m_rx_ready;
    volatile uint16_t m_rx_size;
    
    // 主动发送相关
    uint8_t m_received_data[SERIAL1_DATA_LEN];
    uint8_t m_received_parity;
    volatile uint8_t m_new_data_available;
    uint8_t m_last_rx_data[SERIAL1_DATA_LEN];
    uint8_t m_last_rx_parity;
    uint32_t m_last_rx_time;
    uint32_t m_last_send_time; 
    uint8_t m_last_processed_data[SERIAL1_DATA_LEN];   
    uint8_t m_last_processed_parity;  

    
    // 发送状态管理
    Serial1State_t m_state;
    Serial1SendResultCallback m_resultCallback;
    Serial1DataReceiveCallback m_dataReceiveCallback;
    uint8_t m_current_send_data[SERIAL1_DATA_LEN];
    uint8_t m_current_send_parity;
    uint8_t m_send_batch_count;
    volatile uint8_t m_tx_complete;                  
    
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
    
    // 最新数据存储
    DataPacket_t m_latest_packet;      
    volatile bool m_has_latest_data; // 上层多线程/多任务轮询时需确保可见性
};

#endif

#endif