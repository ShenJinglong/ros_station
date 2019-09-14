#ifndef ROBORTS_SDK_MEMORY_POOL_H
#define ROBORTS_SDK_MEMORY_POOL_H

#include <stdint.h>
#include <cstring>
#include <mutex>

typedef struct MemoryBlock {
    bool usage_flag;
    uint8_t table_index;
    uint16_t memory_size;
    uint8_t *memory_ptr;
} MemoryBlock;

class MemoryPool {
public:
    MemoryPool(uint16_t max_block_size = 1024,
                uint16_t memory_size = 1024,
                uint16_t memory_table_num = 32):
                memory_size_(memory_size_),
                memory_table_num_(memory_table_num),
                max_block_size_(max_block_size) {};
    ~MemoryPool() {
        delete []memory_;
        delete []memory_table_;
    }

    void Init() {
        memory_table_ = new MemoryBlock[memory_table_num_];
        memory_ = new uint8_t[memory_size_];
        uint32_t i;
        memory_table_[0].table_index = 0;
        memory_table_[0].usage_flag = 1;
        memory_table_[0].memory_ptr = memory_;
        memory_table_[0].memory_size = 0;
        for (i = 1; i < (memory_table_num_ - 1); i++) {
            memory_table_[i].table_index = i;
            memory_table_[i].usage_flag = 0;
        }
        memory_table_[memory_table_num_ - 1].table_index = memory_table_num_ - 1;
        memory_table_[memory_table_num_ - 1].usage_flag = 1;
        memory_table_[memory_table_num_ - 1].memory_ptr = memory_ + memory_size_;
        memory_table_[memory_table_num_ - 1].memory_size = 0;
    }

    void FreeMemory(MemoryBlock *memory_block) {
        if (memory_block == (MemoryBlock *)0) {
            return;
        }
        if (memory_block->table_index == 0 || memory_block->table_index == (memory_table_num_ - 1)) {
            return;
        }
        memory_block->usage_flag = 0;
    }

    MemoryBlock *AllocMemory(uint16_t size) {
        uint32_t used_memory_size = 0;
        uint8_t i = 0;
        uint8_t j = 0;
        uint8_t memory_table_used_num = 0;
        uint8_t memory_table_used_index[memory_table_num_];

        uint32_t block_memory_size;
        uint32_t temp_area[2] = {0xFFFFFFFF, 0xFFFFFFFF};
        uint32_t accumulate_left_memory_size = 0;
        bool index_found = false;

        if (size > max_block_size_ || size > memory_size_) {
            return (MemoryBlock *)0;
        }

        for (i = 0; i < memory_table_num_; i++) {
            if (memory_table_[i].usage_flag == 1) {
                used_memory_size += memory_table_[i].memory_size;
                memory_table_used_index[memory_table_used_num++] = memory_table_[i].table_index;
            }
        }

        if (memory_size_ < (used_memory_size + size)) {
            return (MemoryBlock *)0;
        }

        if (used_memory_size == 0) {
            memory_table_[1].memory_ptr = memory_table_[0].memory_ptr;
            memory_table_[1].memory_size = size;
            memory_table_[1].usage_flag = 1;
            return &memory_table_[1];
        }

        for (i = 0; i < (memory_table_used_num - 1); i++) {
            for (j = 0; j < (memory_table_used_num - i - 1); j++) {
                if (memory_table_[memory_table_used_index[j]].memory_ptr >
                    memory_table_[memory_table_used_index[j + 1]].memory_ptr) {
                    memory_table_used_index[j + 1] ^= memory_table_used_index[j];
                    memory_table_used_index[j] ^= memory_table_used_index[j + 1];
                    memory_table_used_index[j + 1] ^= memory_table_used_index[j];
                }
            }
        }

        for (i = 0; i < (memory_table_used_num - 1); i++) {
            block_memory_size = static_cast<uint32_t>(memory_table_[memory_table_used_index[i + 1]].memory_ptr -
                    memory_table_[memory_table_used_index[i]].memory_ptr);
            
            if ((block_memory_size - memory_table_[memory_table_used_index[i]].memory_size) >= size) {
                if (temp_area[1] > (block_memory_size - memory_table_[memory_table_used_index[i]].memory_size)) {
                    temp_area[0] = memory_table_[memory_table_used_index[i]].table_index;
                    temp_area[1] = block_memory_size - memory_table_[memory_table_used_index[i]].memory_size;
                }
            }

            accumulate_left_memory_size += block_memory_size - memory_table_[memory_table_used_index[i]].memory_size;
            if (accumulate_left_memory_size >= size && !index_found) {
                j = i;
                index_found = true;
            }
        }

        if (temp_area[0] == 0xFFFFFFFF && temp_area[1] == 0xFFFFFFFF) {
            for ( i = 0; i < j; i++) {
                if (memory_table_[memory_table_used_index[i + 1]].memory_ptr >
                    (memory_table_[memory_table_used_index[i]].memory_ptr + 
                    memory_table_[memory_table_used_index[i]].memory_size)) {
                    memmove(memory_table_[memory_table_used_index[i]].memory_ptr +
                            memory_table_[memory_table_used_index[i]].memory_size,
                            memory_table_[memory_table_used_index[i + 1]].memory_ptr,
                            memory_table_[memory_table_used_index[i + 1]].memory_size);
                    memory_table_[memory_table_used_index[i + 1]].memory_ptr =
                        memory_table_[memory_table_used_index[i]].memory_ptr +
                        memory_table_[memory_table_used_index[i]].memory_size;
                }
            }

            for (i = 1; i < (memory_table_num_ - 1); i++) {
                if (memory_table_[i].usage_flag == 0) {
                    memory_table_[i].memory_ptr = memory_table_[memory_table_used_index[j]].memory_ptr +
                        memory_table_[memory_table_used_index[j]].memory_size;
                    memory_table_[i].memory_size = size;
                    memory_table_[i].usage_flag = 1;
                    return &memory_table_[i];
                }
            }

            return (MemoryBlock *) 0;
        }

        for (i = 1; i < (memory_table_num_ - 1); i++) {
            if (memory_table_[i].usage_flag == 0) {
                memory_table_[i].memory_ptr =
                    memory_table_[temp_area[0]].memory_ptr + memory_table_[temp_area[0]].memory_size;
                memory_table_[i].memory_size = size;
                memory_table_[i].usage_flag = 1;
                return &memory_table_[i];
            }
        }

        return (MemoryBlock *)0;        
    }

    void LockMemory() {
        memory_mutex_.lock();
    }

    void UnlockMemory() {
        memory_mutex_.unlock();
    }
private:
    std::mutex memory_mutex_;
    uint16_t memory_table_num_;
    uint16_t memory_size_;
    uint16_t max_block_size_;
    MemoryBlock *memory_table_;
    uint8_t *memory_;
};

#endif