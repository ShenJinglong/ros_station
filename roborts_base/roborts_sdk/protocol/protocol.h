#ifndef ROBORTS_SDK_PROTOCOL_H
#define ROBORTS_SDK_PROTOCOL_H

#include <memory>
#include <cstring>
#include <map>
#include <atomic>
#include <thread>

#include "../hardware/serial_device.h"
#include "../utilities/memory_pool.h"
#include "../utilities/circular_buffer.h"
#include "../utilities/crc.h"

namespace roborts_sdk {

typedef struct Header {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t session_id : 5;
  uint32_t is_ack : 1;
  uint32_t reserved0 : 2; // Always 0
  uint32_t sender: 8;
  uint32_t receiver: 8;
  uint32_t reserved1 : 16;
  uint32_t seq_num : 16;
  uint32_t crc : 16;
} Header;

typedef struct CommandInfo {
    uint8_t cmd_set;
    uint8_t cmd_id;
    bool need_ack;
    uint8_t sender;
    uint8_t receiver;
    uint16_t length;
} CommandInfo;

typedef struct MessageHeader {
    uint16_t seq_num;
    uint8_t session_id;
    bool is_ack;
} MessageHeader;

typedef union MessageData {
    uint8_t raw_data[1024];
} MessageData;

enum class ACKSessionStatus: uint8_t {
    ACK_SESSION_IDLE = 0,
    ACK_SESSION_PROCESS = 1,
    ACK_SESSION_USING = 2,
};

enum class CMDSessionMode: uint8_t {
    CMD_SESSION_0 = 0,
    CMD_SESSION_1 = 1,
    CMD_SESSION_AUTO = 32,
};

typedef struct CMDSession {
    uint8_t session_id;
    bool usage_flag;
    MemoryBlock *memory_block_ptr;
    uint8_t cmd_set;
    uint8_t cmd_id;

    uint16_t sent;
    uint16_t retry_time;
    std::chrono::milliseconds ack_timeout;
    std::chrono::steady_clock::time_point pre_time_stamp;

    uint32_t pre_seq_num;
} CMDSession;

typedef struct ACKSession {
    uint8_t session_id;
    ACKSessionStatus session_status;
    MemoryBlock *memory_block_ptr;
} ACKSession;

typedef struct RecvStream {
    uint32_t reuse_index;
    uint32_t reuse_count;
    uint32_t recv_index;
    uint8_t *recv_buff;
} RecvStream;

typedef struct RecvContainer {
    CommandInfo command_info;
    MessageHeader message_header;
    MessageData message_data;
} RecvContainer;

class Protocol {
public:
    explicit Protocol(std::shared_ptr<SerialDevice> serial_device_ptr);
    ~Protocol();

    bool Init();
    void AutoRepeatSendCheck();
    void ReceivePool();
    bool Take(const CommandInfo *command_info,
                MessageHeader *message_header,
                void *message_data);
    bool SendResponse(const CommandInfo *command_info,
                        const MessageHeader *message_header,
                        void *message_data);
    bool SendRequest(const CommandInfo *command_info,
                        MessageHeader *message_header,
                        void *message_data);
    bool SendMessage(const CommandInfo *command_info,
                        void *message_data);
    
    bool SendCMD(uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver,
                        void *data_ptr, uint16_t data_length,
                        CMDSessionMode session_mode, MessageHeader *message_header = nullptr,
                        std::chrono::milliseconds ack_timeout = std::chrono::milliseconds(50), int retry_time = 5);
    bool SendACK(uint8_t session_id, uint16_t seq_num, uint8_t receiver,
                    void *ack_ptr, uint16_t ack_length);
    bool DeviceSend(uint8_t *buf);
    RecvContainer *Receive();
    bool ByteHandler(const uint8_t byte);
    bool StreamHandler(uint8_t byte);
    bool CheckStream();
    bool VerifyHeader();
    bool VerifyData();
    bool ContainerHandler();
    void PrepareStream();
    void ShiftStream();
    void ReuseStream();
    void SetupSession();
    CMDSession *AllocCMDSession(CMDSessionMode session_mode, uint16_t size);
    void FreeCMDSession(CMDSession *session);
    ACKSession *AllocACKSession(uint8_t receiver, uint16_t session_id, uint16_t size);
    void FreeACKSession(ACKSession *session);
    uint16_t CRC16Update(uint16_t crc, uint8_t ch);
    uint32_t CRC32Update(uint32_t crc, uint8_t ch);
    uint16_t CRC16Calc(const uint8_t *data_ptr, size_t length);
    uint32_t CRC32Calc(const uint8_t *data_ptr, size_t length);
    bool CRCHeadCheck(uint8_t *data_ptr, size_t length);
    bool CRCTailCheck(uint8_t *data_ptr, size_t length);
    
    static const size_t BUFFER_SIZE = 2048;
    static const size_t MAX_PACK_SIZE = 2048;
    static const size_t SESSION_TABLE_NUM = 32;
    static const size_t HEADER_LEN = sizeof(Header);
    static const size_t CRC_HEAD_LEN = sizeof(uint16_t);
    static const size_t CRC_DATA_LEN = sizeof(uint32_t);
    static const size_t CMD_SET_PREFIX_LEN = 2 * sizeof(uint8_t);

    static const uint8_t SOF = 0xAA;
    static const uint8_t VERSION = 0x00;
    static const uint8_t DEVICE = 0x00;
    static const uint8_t RECEIVER_NUM = 6;
private:
    std::shared_ptr<SerialDevice> serial_device_ptr_;
    std::shared_ptr<MemoryPool> memory_pool_ptr_;

    uint16_t seq_num_;
    std::chrono::milliseconds poll_tick_;

    CMDSession cmd_session_table_[SESSION_TABLE_NUM];
    ACKSession ack_session_table_[RECEIVER_NUM][SESSION_TABLE_NUM - 1];

    uint8_t *recv_buff_ptr_;
    uint16_t recv_buff_read_pos_;
    uint16_t recv_buff_read_len_;

    bool is_large_data_protocol_;
    bool reuse_buffer_;

    RecvStream *recv_stream_ptr_;
    RecvContainer *recv_container_ptr_;

    std::map<std::pair<uint8_t, uint8_t>, std::shared_ptr<CircularBuffer<RecvContainer>>> buffer_pool_map_;
    std::atomic<bool> running_;

    std::thread send_poll_thread_;
    std::thread receive_pool_thread_;
};

}

#endif