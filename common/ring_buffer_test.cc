#include "ring_buffer.h"

#include <iostream>
#include <memory>
#include <vector>

class RingBufferTest {
   public:
    void BuildBuffer(const size_t capacity) {
        ring_buffer_ = std::make_unique<RingBuffer<double>>(capacity);
    }

    RingBuffer<double>* GetBuffer() { return ring_buffer_.get(); }

   private:
    std::unique_ptr<RingBuffer<double>> ring_buffer_{nullptr};
};

int main() {
    RingBufferTest test;

    test.BuildBuffer(5);
    auto* const buffer = test.GetBuffer();

    std::vector<double> inputs{1.0, 2.0, 3.0, 4.0, 5.0,
                               6.0, 7.0, 8.0, 9.0, 10.0};

    std::cout << "TEST[push front]" << std::endl;
    for (double input : inputs) {
        buffer->PushFront(input);
        buffer->Print();
    }

    buffer->Reset();

    std::cout << "TEST[push back]" << std::endl;
    for (double input : inputs) {
        buffer->PushBack(input);
        buffer->Print();
    }

    std::cout << "TEST[reset with initial value]" << std::endl;
    buffer->ResetWith(7.0);
    buffer->Print();

    return 0;
}