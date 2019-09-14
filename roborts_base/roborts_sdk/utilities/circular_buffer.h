#ifndef ROBORTS_SDK_CIRCULAR_BUFFER_H
#define ROBORTS_SDK_CIRCULAR_BUFFER_H

#include <memory>
#include <mutex>

template <class T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t size): 
        buf_(std::unique_ptr<T[]>(new T[size])), 
        max_size_(size) {
    }

    void Push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        buf_[head_] = item;
        if (full_) {
            tail_ = (tail_ + 1) % max_size_;
        }
        head_ = (head_ + 1) % max_size_;
        full_ = head_ == tail_;
    }

    bool Pop(T &item) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (IsEmpty()) {
            return false;
        }

        item = buf_[tail_];
        full_ = false;
        tail_ = (tail_ + 1) % max_size_;

        return true;
    }

    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_;
        full_ = false;
    }

    bool IsEmpty() const {
        return (!full_ && (head_ == tail_));
    }

    bool IsFull() const {
        return full_;
    }

    size_t GetCapacity() const {
        return max_size_;
    }

    size_t GetSize() const {
        size_t size = max_size_;
        if (!full_) {
            if (head_ >= tail_) {
                size = head_ - tail_;
            } else {
                size = max_size_ + head_ - tail_;
            }
        }
        return size;
    }

private:
    std::mutex mutex_;
    std::unique_ptr<T[]> buf_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t max_size_;
    bool full_ = 0;
};

#endif