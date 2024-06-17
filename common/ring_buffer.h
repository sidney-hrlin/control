#pragma once

#include <cstddef>
#include <iostream>

class RingBufferTest;

template <typename T>
class RingBuffer {
   public:
    explicit RingBuffer(const size_t capacity)
        : buffer_(new T[capacity]),
          capacity_(capacity),
          size_(0),
          begin_(0),
          end_(0) {}

    ~RingBuffer() { delete[] buffer_; }

    size_t Size() const { return size_; }

    bool Empty() const { return size_ == 0; }

    bool Full() const { return !Empty() && (size_ == capacity_); }

    void Reset() {
        size_ = 0;
        begin_ = 0;
        end_ = 0;
    }

    void ResetWith(const T& val) {
        Reset();
        while (!Full()) {
            PushBack(val);
        }
    }

    const T& operator[](const size_t i) const {
        return buffer_[ModN(begin_ + i)];
    }

    T& operator[](const size_t i) { return buffer_[ModN(begin_ + i)]; }

    const T& Front() const { return ModN(begin_); }

    const T& End() const { return ModN(end_ - 1); }

    void PushFront(const T& val) {
        begin_ = ModN(begin_ - 1);
        buffer_[begin_] = val;
        if (Full()) {
            end_ = begin_;
        } else {
            size_++;
        }
    }

    void PushBack(const T& val) {
        buffer_[end_] = val;
        end_ = ModN(end_ + 1);
        if (Full()) {
            begin_ = end_;
        } else {
            size_++;
        }
    }

    void Print() const {
        for (size_t i = 0; i < size_; i++) {
            std::cout << this->operator[](i) << " ";
        }
        std::cout << std::endl;
    }

   private:
    size_t ModN(const size_t n) const { return (n + capacity_) % capacity_; }

    T* buffer_;
    size_t capacity_, size_;
    int begin_, end_;

    friend RingBufferTest;
};