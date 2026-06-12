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
    if (size > 0 && size <= 32)
    {
        memcpy(m_rx_work_buffer, buffer, size);
        m_rx_size = size;
        m_rx_ready = 1;
    }
}

// ========== 主循环轮询处理（隔离中断） ==========
// 轮询处理串口接收缓存数据，做帧解析、重复帧判重、业务上报、应答回复
void Serial1Protocol::process(void)
{
    uint16_t local_rx_size = 0;
    bool data_to_process = false;
    uint8_t temp_work_buffer[32]; // 栈上临时缓冲区，防止解析期间被中断重写数据

    //临界区安全提取：连同数据内容一同安全复制到栈空间
    if (m_rx_ready)
    {
        __disable_irq();
        local_rx_size = m_rx_size;
        if (local_rx_size > 0 && local_rx_size <= 32)
        {
            memcpy(temp_work_buffer, m_rx_work_buffer, local_rx_size);
            data_to_process = true;
        }
        m_rx_ready = 0;
        __enable_irq();
    }

    if (!data_to_process) return;

    uint32_t now = getTickMs();
    uint8_t local_received_data[SERIAL1_DATA_LEN];
    uint8_t local_received_parity;

    // 解析临时缓冲区的数据，确保不会因突发接收中断导致数据解析冲突
    if (parseFrame(temp_work_buffer, local_rx_size, local_received_data, &local_received_parity))
    {
        bool is_ack = (local_received_data[0] == 0x00 &&
                       local_received_data[1] == 0x00 &&
                       local_received_data[2] == 0x00);

        if (!is_ack)
        {
            bool is_same_as_last = (memcmp(local_received_data, m_last_processed_data, SERIAL1_DATA_LEN) == 0 &&
                                    local_received_parity == m_last_processed_parity);

            bool is_same_data = (memcmp(local_received_data, m_last_rx_data, SERIAL1_DATA_LEN) == 0);
            bool is_same_parity = (local_received_parity == m_last_rx_parity);

            if (is_same_as_last)
            {
                sendAckFrame(); // 重发应答
            }
            else if (!is_same_as_last && (!is_same_data || (is_same_data && !is_same_parity)))
            {
                memcpy(m_last_processed_data, local_received_data, SERIAL1_DATA_LEN);
                m_last_processed_parity = local_received_parity;

                memcpy(m_received_data, local_received_data, SERIAL1_DATA_LEN);
                m_received_parity = local_received_parity;
                m_new_data_available = 1;

                storeReceivedData(local_received_data, local_received_parity);

                if (m_dataReceiveCallback)
                {
                    m_dataReceiveCallback(local_received_data, local_received_parity);
                }

                sendAckFrame();

                memcpy(m_last_rx_data, local_received_data, SERIAL1_DATA_LEN);
                m_last_rx_parity = local_received_parity;
                m_last_rx_time = now;
            }

        }
        else
        {
            // =====当收到对端的 ACK 帧（0x00, 0x00, 0x00）时置位全局标志位 =====
            g_send_success = 1;
            g_send_complete = 1;

//            // 触发可能注册的发送结果回调函数，通知发送成功
//            if (m_resultCallback)
//            {
//                m_resultCallback(true);
//            }
        }
    }
}

// 封装串口协议帧打包 + 发送
void Serial1Protocol::sendFrame(uint8_t* data, uint8_t parity)
{
    if (!m_huart) return;

    uint32_t start_time = getTickMs();
    const uint32_t timeout_ms = 10; // 对于8字节，15ms的超时时间在任何波特率下都足够

    while (!m_tx_complete)
    {
        if (getTickMs() - start_time > timeout_ms)
        {
            HAL_UART_DMAStop(m_huart);
            m_tx_complete = 1;
            break;
        }
        __NOP(); 
    }

    // 临界区原子锁保护：防止多任务并发争抢发送引起的数据包重叠及 DMA 崩溃
    __disable_irq();
    if (!m_tx_complete)
    {
        __enable_irq();
        return; // 被抢占或未能成功释放，则安全退出
    }
    m_tx_complete = 0;
    __enable_irq();

    buildFrame(data, parity, s_serial1_tx_buffer);

    SCB_CleanDCache_by_Addr((uint32_t*)s_serial1_tx_buffer, 32);

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(m_huart, s_serial1_tx_buffer, SERIAL1_FRAME_LEN);

    if (status != HAL_OK)
    {
        m_tx_complete = 1; // 启动失败时及时释放发送资源
    }
}

// 串口发送空载荷的应答 ACK 帧，只回复协议握手应答、不带业务数据
void Serial1Protocol::sendAckFrame(void)
{
    uint8_t ack_data[SERIAL1_DATA_LEN] = {0x00, 0x00, 0x00};
    sendFrame(ack_data, 0);
}

uint32_t Serial1Protocol::getTickMs(void)
{
    return timer::Timer::Get_TimeStamp() / 1000;
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

void Serial1Protocol::onUartTxComplete(void)
{
    m_tx_complete = 1;
}

void Serial1Protocol::setSendResultCallback(Serial1SendResultCallback callback)
{
    m_resultCallback = callback;
}

void Serial1Protocol::setDataReceiveCallback(Serial1DataReceiveCallback callback)
{
    m_dataReceiveCallback = callback;
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

void Serial1Protocol::updateSendHistory(uint8_t* data, uint8_t parity)
{
    int index = findHistoryIndex(data);
    if (index >= 0)
    {
        m_send_history[index].last_parity = parity;
        m_send_history[index].send_count++;
    }
    else
    {
        if (m_history_count < MAX_HISTORY)
        {
            memcpy(m_send_history[m_history_count].data, data, SERIAL1_DATA_LEN);
            m_send_history[m_history_count].last_parity = parity;
            m_send_history[m_history_count].send_count = 1;
            m_history_count++;
        }
        else
        {
            for (int i = 0; i < MAX_HISTORY - 1; i++)
            {
                m_send_history[i] = m_send_history[i + 1];
            }
            memcpy(m_send_history[MAX_HISTORY - 1].data, data, SERIAL1_DATA_LEN);
            m_send_history[MAX_HISTORY - 1].last_parity = parity;
            m_send_history[MAX_HISTORY - 1].send_count = 1;
        }
    }
}

//void Serial1Protocol::onUartReceive(uint8_t* buffer, uint16_t size) {
//    if (size > 0 && size <= 30) {
//        memcpy(m_rx_buffer, buffer, size);
//        m_rx_size = size;
//        m_rx_ready = 1;
//            process();
//    }
//} 
bool Serial1Protocol::sendCommand(uint8_t* data)
{
    // ===== 每次发起新的发送时，主动复位 ACK 确认相关的全局状态标志 =====
    g_send_complete = 0;
    g_send_success = 0;
    uint8_t parity = getNextParity(data);

    memcpy(m_command_send_data, data, SERIAL1_DATA_LEN);
    m_command_send_parity = parity;
    memcpy(m_current_send_data, data, SERIAL1_DATA_LEN);
    m_current_send_parity = parity;

    updateSendHistory(data, parity);
    sendFrame(m_current_send_data, m_current_send_parity);
    return true;
}

void Serial1Protocol::sendTestData(uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint8_t data[3] = {d1, d2, d3};
    sendCommand(data);
}

void Serial1Protocol::storeReceivedData(uint8_t* data, uint8_t parity)
{
    DataPacket_t packet;
    uint8_t data_valid_flag = 0;

    // CMD 判断规则
    if (data[0] == 0 && data[1] == 0 && data[2] != 0)
    {
        packet.type = DATA_TYPE_CMD;
        packet.data.cmd = data[2];
        data_valid_flag = 1;
    }
    // KFS 特殊压缩格式判断规则
    else if (((data[1] & 0x0F) == 0) && (data[2] == 0))
    {
        packet.type = DATA_TYPE_KFS;
        packet.data.kfs[0] = (data[0] & 0xF0) >> 4;
        packet.data.kfs[1] = (data[0] & 0x0F);
        packet.data.kfs[2] = (data[1] & 0xF0) >> 4;
        data_valid_flag = 1;
    }
    // 普通 3 字节原始数据流
    else if (!has_consecutive_zeros_exceed_10(data))
    {
        packet.type = DATA_TYPE_KFS;
        memcpy(packet.data.kfs, data, SERIAL1_DATA_LEN);
        data_valid_flag = 1;
    }

    if (data_valid_flag == 1)
    {
        __disable_irq();
        m_latest_packet = packet;
        m_has_latest_data = true;
        __enable_irq();
    }
}

//消除“检查-执行”的线程安全漏洞，使最新数据读取成为完全的原子操作
bool Serial1Protocol::getLatestData(DataPacket_t* packet)
{
    bool success = false;
    __disable_irq();
    if (m_has_latest_data)
    {
        *packet = m_latest_packet;
        m_has_latest_data = false;  // 消费后清除
        success = true;
    }
    __enable_irq();

    return success;
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

void Serial1Protocol::waitForSendComplete(void)
{
    uint32_t start_time = getTickMs();
    while (!m_tx_complete)
    {
        if (getTickMs() - start_time > 10) // 精确时间超时
        {
            break;
        }
    }
}