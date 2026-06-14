#include "RC_IR_communication.h"
#include "RC_timer.h"

// 显式将 DMA 专用接收与发送缓冲区放置在 DMA 可正常访问的 D2RAM 区域，并按照 32 字节对齐
#if defined(__GNUC__)
    static uint8_t s_serial1_tx_buffer[32] __attribute__((section(".D2RAM"))) __attribute__((aligned(32)));
    static uint8_t s_serial1_rx_buffer[32] __attribute__((section(".D2RAM"))) __attribute__((aligned(32)));
#else
    __align(32) __attribute__((section(".D2RAM"))) static uint8_t s_serial1_tx_buffer[32];
    __align(32) __attribute__((section(".D2RAM"))) static uint8_t s_serial1_rx_buffer[32];
#endif

static uint8_t m_uart_send_frame[SERIAL1_FRAME_LEN] __attribute__((section(".D2RAM"))) __attribute__((aligned(32)));

// 全局变量定义
volatile uint8_t g_recv_data[3] = {0};
volatile uint8_t g_recv_parity = 0;
volatile uint8_t g_send_complete = 0;
volatile uint8_t g_send_success = 0;

extern UART_HandleTypeDef huart6;

Serial1Protocol &Serial1Protocol::getInstance()
{
    static Serial1Protocol instance;
    return instance;
}

Serial1Protocol::Serial1Protocol() : serial::UartRx(huart6, s_serial1_rx_buffer, 32, true, true)
{
    m_huart = &huart6;
    m_state = SERIAL1_STATE_IDLE;
    m_send_batch_count = 0;
    m_current_send_parity = 0;
    m_tx_complete = 1;
    m_rx_ready = 0;
    m_rx_size = 0;
    m_resultCallback = nullptr;
    m_dataReceiveCallback = nullptr;
    m_new_data_available = 0;

    m_last_send_time = 0;
    m_last_rx_time = 0;
    m_last_rx_parity = 0;

    memset(m_last_processed_data, 0, SERIAL1_DATA_LEN);
    m_last_processed_parity = 0;
    memset(m_last_rx_data, 0, SERIAL1_DATA_LEN);
    memset(m_received_data, 0, SERIAL1_DATA_LEN);
    memset(m_rx_work_buffer, 0, sizeof(m_rx_work_buffer));
    memset(m_current_send_data, 0, SERIAL1_DATA_LEN);

    m_history_count = 0;
    memset(m_send_history, 0, sizeof(m_send_history));
    memset(m_command_send_data, 0, SERIAL1_DATA_LEN);
    m_command_send_parity = 0;

    memset(&m_latest_packet, 0, sizeof(m_latest_packet));
    m_has_latest_data = false;
}

void Serial1Protocol::init(UART_HandleTypeDef* huart)
{
    if (huart)
    {
        m_huart = huart;
    }
    m_state = SERIAL1_STATE_IDLE;
    m_tx_complete = 1;
    m_rx_ready = 0;
    m_has_latest_data = false;
    memset(m_last_processed_data, 0, SERIAL1_DATA_LEN);
    m_last_processed_parity = 0;

    this->Uart_Rx_Start();  // 调用 HAL_UARTEx_ReceiveToIdle_DMA 开启红外串口的 DMA 接收
}

void Serial1Protocol::Uart_Rx_It_Process(uint8_t* buffer, uint16_t size)
{
    onUartReceive(buffer, size);
}

// ========== 主循环处理 ==========
void Serial1Protocol::process(void) 
{

    // ========== 1. 处理串口接收数据 ==========
    if (m_rx_ready) {
        m_rx_ready = 0;
        uint8_t received_data[SERIAL1_DATA_LEN];
        uint8_t received_parity;
        
        if (parseFrame(m_rx_work_buffer, m_rx_size, received_data, &received_parity)) {

            // 检查是否是串口应答（数据全0）
            uint8_t is_ack = (received_data[0] == 0x00 && 
                              received_data[1] == 0x00 && 
                              received_data[2] == 0x00);
            
            // ========== 场景B：收到非应答数据（串口2主动发送的数据） ==========
             if (!is_ack) {
                
                // 判断是否与上次处理的数据相同（包含奇偶位）
                uint8_t is_same_as_last = (memcmp(received_data, m_last_processed_data, SERIAL1_DATA_LEN) == 0 &&
                                           received_parity == m_last_processed_parity);
                
                // 去重判断（防止同一数据重复解析）
                uint8_t is_same_data = (memcmp(received_data, m_last_rx_data, SERIAL1_DATA_LEN) == 0);
                uint8_t is_same_parity = (received_parity == m_last_rx_parity);
                
                // ? 情况1：相同数据 → 串口2没收到应答，重发应答
                if (is_same_as_last) {
                    // 重发应答，不保存数据，不调用回调
                    sendAckFrame();
                }
                // ? 情况2：新数据（与上次不同）
                else if (!is_same_as_last && (!is_same_data || (is_same_data && !is_same_parity))) {
                    // 更新上次处理记录
                    memcpy(m_last_processed_data, received_data, SERIAL1_DATA_LEN);
                    m_last_processed_parity = received_parity;
                    
                    // 保存数据供上层读取
                    memcpy(m_received_data, received_data, SERIAL1_DATA_LEN);
                    m_received_parity = received_parity;
                    m_new_data_available = 1;

                    storeReceivedData(received_data, received_parity);
                    // 回调通知上层
//                    if (m_dataReceiveCallback) {
//                        m_dataReceiveCallback(received_data, received_parity);
//                    }
//                    
                    // 发送应答
                    sendAckFrame();
                    
                    // 更新去重记录
                    memcpy(m_last_rx_data, received_data, SERIAL1_DATA_LEN);
                    m_last_rx_parity = received_parity;
                }
                // 完全相同的重复数据（已被去重过滤）：忽略
            }
        }
    }
}


void Serial1Protocol::sendFrame(uint8_t* data, uint8_t parity) {
    if (!m_huart ) return;
    
    m_tx_complete = 0;
    buildFrame(data, parity, m_uart_send_frame);
	  if((data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00) )
		{
			SCB_CleanDCache_by_Addr((uint32_t *)m_uart_send_frame, SERIAL1_FRAME_LEN);
			HAL_UART_Transmit_DMA(m_huart, m_uart_send_frame, SERIAL1_FRAME_LEN);
		}
}


// 串口发送空载荷的应答 ACK 帧，只回复协议握手应答、不带业务数据
void Serial1Protocol::sendAckFrame(void)
{
    uint8_t ack_data[SERIAL1_DATA_LEN] = {0x00, 0x00, 0x00};
    sendFrame(ack_data, 0);
}



// 自定义简易校验和计算
uint8_t Serial1Protocol::calculateChecksum(uint8_t* data)
{
    uint16_t sum = data[0] + data[1] + data[2];
    uint8_t crc = ~((sum * sum) & 0xFF);
    return crc & SERIAL1_CHECKSUM_MASK;
}

void Serial1Protocol::buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out)
{
    frame_out[0] = SERIAL1_FRAME_HEAD0;
    frame_out[1] = SERIAL1_FRAME_HEAD1;
    memcpy(&frame_out[2], data, SERIAL1_DATA_LEN);

    uint8_t checksum = calculateChecksum(data);
    uint8_t check_byte = checksum | ((parity & 0x01) << 6);

    frame_out[5] = check_byte;
    frame_out[6] = SERIAL1_FRAME_TAIL0;
    frame_out[7] = SERIAL1_FRAME_TAIL1;
}

int Serial1Protocol::parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out)
{
    if (size < SERIAL1_FRAME_LEN) return 0;

    for (uint16_t i = 0; i <= size - SERIAL1_FRAME_LEN; i++)
    {
        if (buffer[i] == SERIAL1_FRAME_HEAD0 && buffer[i + 1] == SERIAL1_FRAME_HEAD1)
        {
            if (buffer[i + 6] == SERIAL1_FRAME_TAIL0 && buffer[i + 7] == SERIAL1_FRAME_TAIL1)
            {
                memcpy(data_out, &buffer[i + 2], SERIAL1_DATA_LEN);

                uint8_t check_byte = buffer[i + 5];
                *parity_out = (check_byte & SERIAL1_PARITY_BIT_MASK) >> 6;
                uint8_t received_checksum = check_byte & SERIAL1_CHECKSUM_MASK;
                if (received_checksum == calculateChecksum(data_out)) return 1;
            }
        }
    }
    return 0;
}


void Serial1Protocol::getReceivedData(uint8_t* data_out, uint8_t* parity_out)
{
    memcpy(data_out, m_received_data, SERIAL1_DATA_LEN);
    *parity_out = m_received_parity;
    m_new_data_available = 0;
}

int Serial1Protocol::findHistoryIndex(uint8_t* data)
{
    for (int i = 0; i < m_history_count; i++)
    {
        if (memcmp(m_send_history[i].data, data, SERIAL1_DATA_LEN) == 0)
        {
            return i;
        }
    }
    return -1;
}

uint8_t Serial1Protocol::getNextParity(uint8_t* data)
{
    int index = findHistoryIndex(data);
    if (index >= 0) return (m_send_history[index].last_parity == 0) ? 1 : 0;
    return 0;
}



void Serial1Protocol::onUartReceive(uint8_t* buffer, uint16_t size) {
    if (size > 0 && size <= 30) {
        memcpy(m_rx_work_buffer, buffer, size);
        m_rx_size = size;
        m_rx_ready = 1;
        process();
    }
}


// 存储接收到的数据
void Serial1Protocol::storeReceivedData(uint8_t* data, uint8_t parity) {
    DataPacket_t packet;
    uint8_t Data_valid_flag=0;
    // 判断数据类型
    // 规则：如果 data[2] != 0 且 data[0]==0 && data[1]==0，则是 CMD
    if (data[0] == 0 && data[1] == 0 && data[2] != 0) {
        packet.type = DATA_TYPE_CMD;
        packet.data.cmd = data[2];
			  Data_valid_flag=1;
    } 
		else if(((data[1]&0x0F)==0)&&(data[2]==0)) 
		{
        packet.type = DATA_TYPE_KFS;
        packet.data.kfs[0] = (data[0]&0xF0)>>4;
        packet.data.kfs[1] = (data[0]&0x0F);
        packet.data.kfs[2] = (data[1]&0xF0)>>4;
			  Data_valid_flag=1;
    }
    else if(!has_consecutive_zeros_exceed_10(data))
		{ 
			  packet.type = DATA_TYPE_KFS;
			  memcpy(packet.data.kfs,data,3);
			  Data_valid_flag=1;
			
			
		}
		if(Data_valid_flag==1)
		{
			
			    // 只保存最新一条（覆盖）
    m_latest_packet = packet;
    m_has_latest_data = true;
			
		}
}


// 获取数据
bool Serial1Protocol::getLatestData(DataPacket_t* packet) {

	    if (!m_has_latest_data) {
        return false;
    }
    
    *packet = m_latest_packet;
    m_has_latest_data = false;  // 消费后清除标志
    return true;
}

bool Serial1Protocol::has_consecutive_zeros_exceed_10(const uint8_t* data)
{
    uint32_t value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];

    int max_consecutive_zeros = 0;
    int current_zeros = 0;

    for (int i = 23; i >= 0; i--)
    {
        if ((value >> i) & 1)
        {
            current_zeros = 0;
        }
        else
        {
            current_zeros++;
            if (current_zeros > max_consecutive_zeros)
            {
                max_consecutive_zeros = current_zeros;
            }
        }
    }
    return max_consecutive_zeros > 10;
}

