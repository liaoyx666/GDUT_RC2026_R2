#include "Serial1Protocol.h"

Serial1Protocol& Serial1Protocol::getInstance() {
    static Serial1Protocol instance;
    return instance;
}

Serial1Protocol::Serial1Protocol() {
    m_huart = nullptr;
    m_state = SERIAL1_STATE_IDLE;
    m_send_batch_count = 0;
    m_retry_count = 0;
    m_current_send_parity = 0;
    m_tx_complete = 1;
    m_rx_ready = 0;
    m_resultCallback = nullptr;
    m_dataReceiveCallback = nullptr;
    m_send_start_time = 0;
    m_send_first_start_time = 0;
    m_new_data_available = 0;
    
    // 初始化主动发送变量
    m_waiting_for_response = 0;
    m_last_send_time = 0;
    
    // 初始化主动数据应答管理
    memset(m_last_processed_data, 0, SERIAL1_DATA_LEN);
    m_last_processed_parity = 0;
    
    // 初始化去重变量
    m_last_rx_time = 0;
    m_last_rx_parity = 0;
    memset(m_last_rx_data, 0, SERIAL1_DATA_LEN);
    memset(m_received_data, 0, SERIAL1_DATA_LEN);
    memset(m_rx_buffer, 0, sizeof(m_rx_buffer));
    memset(m_current_send_data, 0, SERIAL1_DATA_LEN);
    
    // 初始化发送历史
    m_history_count = 0;
    memset(m_send_history, 0, sizeof(m_send_history));
    memset(m_command_send_data, 0, SERIAL1_DATA_LEN);
    m_command_send_parity = 0;
}

void Serial1Protocol::init(UART_HandleTypeDef* huart) {
    m_huart = huart;
    m_state = SERIAL1_STATE_IDLE;
    m_tx_complete = 1;
    m_waiting_for_response = 0;
    m_new_data_available = 0;
    memset(m_last_processed_data, 0, SERIAL1_DATA_LEN);
    m_last_processed_parity = 0;
    
    if (m_huart) {
        HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
        __HAL_UART_CLEAR_IDLEFLAG(m_huart);
    }
}

//////////
uint32_t Serial1Protocol::getTickMs(void) {
    //return TimeStamp::getInstance().getMilliseconds();
	return timer::Timer::Get_TimeStamp();
}

uint8_t Serial1Protocol::calculateChecksum(uint8_t* data) {
    uint16_t sum = data[0] + data[1] + data[2];
    uint8_t crc = ~((sum * sum) & 0xFF);
    return crc & SERIAL1_CHECKSUM_MASK;
}

void Serial1Protocol::buildFrame(uint8_t* data, uint8_t parity, uint8_t* frame_out) {
    frame_out[0] = SERIAL1_FRAME_HEAD0;
    frame_out[1] = SERIAL1_FRAME_HEAD1;
    memcpy(&frame_out[2], data, SERIAL1_DATA_LEN);
    
    uint8_t checksum = calculateChecksum(data);
    uint8_t check_byte = checksum | ((parity & 0x01) << 6);
    
    frame_out[5] = check_byte;
    frame_out[6] = SERIAL1_FRAME_TAIL0;
    frame_out[7] = SERIAL1_FRAME_TAIL1;
}

int Serial1Protocol::parseFrame(uint8_t* buffer, uint16_t size, uint8_t* data_out, uint8_t* parity_out) {
    if (size < SERIAL1_FRAME_LEN) return 0;
    
    for (uint16_t i = 0; i <= size - SERIAL1_FRAME_LEN; i++) {
        if (buffer[i] == SERIAL1_FRAME_HEAD0 && buffer[i+1] == SERIAL1_FRAME_HEAD1) {
            if (buffer[i+6] == SERIAL1_FRAME_TAIL0 && buffer[i+7] == SERIAL1_FRAME_TAIL1) {
                memcpy(data_out, &buffer[i+2], SERIAL1_DATA_LEN);
                
                uint8_t check_byte = buffer[i+5];
                *parity_out = (check_byte & SERIAL1_PARITY_BIT_MASK) >> 6;
                uint8_t received_checksum = check_byte & SERIAL1_CHECKSUM_MASK;
                uint8_t calc_checksum = calculateChecksum(data_out);
                
                if (received_checksum == calc_checksum) {
                    return 1;
                }
            }
        }
    }
    return 0;
}

void Serial1Protocol::sendFrame(uint8_t* data, uint8_t parity) {
    if (!m_huart || m_tx_complete == 0) return;
    
    m_tx_complete = 0;
    buildFrame(data, parity, m_uart_send_frame);
    HAL_UART_Transmit_DMA(m_huart, m_uart_send_frame, SERIAL1_FRAME_LEN);
}

// ========== 发送应答帧 ==========
void Serial1Protocol::sendAckFrame(void) {
    if (!m_huart || m_tx_complete == 0) return;
    
    uint8_t ack_data[SERIAL1_DATA_LEN] = {0x00, 0x00, 0x00};
    sendFrame(ack_data, 0);
}

void Serial1Protocol::startSending(void) {
    m_send_batch_count = 0;
    m_state = SERIAL1_STATE_SENDING;
}

void Serial1Protocol::notifySendResult(uint8_t success) {
    if (m_resultCallback) {
        m_resultCallback(m_current_send_data, m_current_send_parity, success);
    }
}

void Serial1Protocol::setSendResultCallback(Serial1SendResultCallback callback) {
    m_resultCallback = callback;
}

void Serial1Protocol::setDataReceiveCallback(Serial1DataReceiveCallback callback) {
    m_dataReceiveCallback = callback;
}

void Serial1Protocol::getReceivedData(uint8_t* data_out, uint8_t* parity_out) {
    memcpy(data_out, m_received_data, SERIAL1_DATA_LEN);
    *parity_out = m_received_parity;
    m_new_data_available = 0;
}

bool Serial1Protocol::hasNewData(void) {
    return (m_new_data_available != 0);
}

// ========== 查找历史记录 ==========
int Serial1Protocol::findHistoryIndex(uint8_t* data) {
    for (int i = 0; i < m_history_count; i++) {
        if (memcmp(m_send_history[i].data, data, SERIAL1_DATA_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

uint8_t Serial1Protocol::getNextParity(uint8_t* data) {
    int index = findHistoryIndex(data);
    
    if (index >= 0) {
        uint8_t next_parity = (m_send_history[index].last_parity == 0) ? 1 : 0;
        return next_parity;
    } else {
        return 0;
    }
}

void Serial1Protocol::updateSendHistory(uint8_t* data, uint8_t parity) {
    int index = findHistoryIndex(data);
    
    if (index >= 0) {
        m_send_history[index].last_parity = parity;
        m_send_history[index].send_count++;
    } else {
        if (m_history_count < MAX_HISTORY) {
            memcpy(m_send_history[m_history_count].data, data, SERIAL1_DATA_LEN);
            m_send_history[m_history_count].last_parity = parity;
            m_send_history[m_history_count].send_count = 1;
            m_history_count++;
        } else {
            for (int i = 0; i < MAX_HISTORY - 1; i++) {
                memcpy(m_send_history[i].data, m_send_history[i+1].data, SERIAL1_DATA_LEN);
                m_send_history[i].last_parity = m_send_history[i+1].last_parity;
                m_send_history[i].send_count = m_send_history[i+1].send_count;
            }
            memcpy(m_send_history[MAX_HISTORY - 1].data, data, SERIAL1_DATA_LEN);
            m_send_history[MAX_HISTORY - 1].last_parity = parity;
            m_send_history[MAX_HISTORY - 1].send_count = 1;
        }
    }
}

void Serial1Protocol::startRetransmit(void) {
    m_retry_count++;
    m_send_batch_count = 0;
    m_state = SERIAL1_STATE_SENDING;
    
    sendFrame(m_current_send_data, m_current_send_parity);
    m_send_batch_count = 1;
    m_last_send_time = getTickMs();
}

void Serial1Protocol::stopRetransmit(void) {
    m_waiting_for_response = 0;
    m_state = SERIAL1_STATE_IDLE;
}

// ========== 主动发送指令 ==========
bool Serial1Protocol::sendCommand(uint8_t* data) {
    if (m_waiting_for_response) {
        return false;
    }
    
    uint8_t parity = getNextParity(data);
    
    memcpy(m_command_send_data, data, SERIAL1_DATA_LEN);
    m_command_send_parity = parity;
    memcpy(m_current_send_data, data, SERIAL1_DATA_LEN);
    m_current_send_parity = parity;
    
    m_send_first_start_time = getTickMs();
    m_retry_count = 0;
    m_waiting_for_response = 1;
    
    updateSendHistory(data, parity);
    startSending();
    return true;
}

// ========== 主循环处理 ==========
void Serial1Protocol::process(void) {
    uint32_t now = getTickMs();
    
    // ========== 1. 处理串口接收数据 ==========
    if (m_rx_ready) {
        m_rx_ready = 0;
        uint8_t received_data[SERIAL1_DATA_LEN];
        uint8_t received_parity;
        
        if (parseFrame(m_rx_buffer, m_rx_size, received_data, &received_parity)) {
            
            // 检查是否是串口应答（数据全0）
            uint8_t is_ack = (received_data[0] == 0x00 && 
                              received_data[1] == 0x00 && 
                              received_data[2] == 0x00);
            
            // ========== 场景A：收到应答（串口2回复的确认，针对主动发送指令） ==========
            if (is_ack && m_waiting_for_response) {
                stopRetransmit();
                notifySendResult(1);
            }
            // ========== 场景B：收到非应答数据（串口2主动发送的数据） ==========
            else if (!is_ack) {
                
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
                    
                    // 回调通知上层
                    if (m_dataReceiveCallback) {
                        m_dataReceiveCallback(received_data, received_parity);
                    }
                    
                    // 发送应答
                    sendAckFrame();
                    
                    // 更新去重记录
                    memcpy(m_last_rx_data, received_data, SERIAL1_DATA_LEN);
                    m_last_rx_parity = received_parity;
                    m_last_rx_time = now;
                }
                // 完全相同的重复数据（已被去重过滤）：忽略
            }
        }
        
        // 重新启动接收
        if (m_huart) {
            HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_rx_buffer, 30);
            __HAL_UART_CLEAR_IDLEFLAG(m_huart);
        }
    }
    
    // ========== 2. 主动发送状态机 ==========
    switch (m_state) {
        case SERIAL1_STATE_SENDING:
            if (m_tx_complete) {
                m_send_batch_count++;
                
                if (m_send_batch_count < SERIAL1_SEND_TIMES) {
                    sendFrame(m_current_send_data, m_current_send_parity);
                } else {
                    m_state = SERIAL1_STATE_WAITING_ACK;
                    m_send_start_time = now;
                }
            }
            break;
            
        case SERIAL1_STATE_WAITING_ACK:
            if ((now - m_send_start_time) >= SERIAL1_RETRY_INTERVAL) {
                startRetransmit();
            }
            break;
            
        default:
            break;
    }
}

// ========== 串口回调 ==========
void Serial1Protocol::onUartReceive(uint8_t* buffer, uint16_t size) {
    if (size > 0 && size <= 30) {
        memcpy(m_rx_buffer, buffer, size);
        m_rx_size = size;
        m_rx_ready = 1;
    }
}

void Serial1Protocol::onUartTxComplete(void) {
    m_tx_complete = 1;
}