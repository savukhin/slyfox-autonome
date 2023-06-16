#ifndef DUMMY_RX_TX_HPP
#define DUMMY_RX_TX_HPP

#pragma once

#include "./BaseRxTx.hpp"
#include <stddef.h>
#include <cstdlib>

// class DummyRxTx {
class DummyRxTx : public BaseRxTx {
private:
    size_t max_batch_size_;
    size_t min_batch_size_;

    size_t generateSize() {
        return (rand() / RAND_MAX) * (this->max_batch_size_ - this->min_batch_size_) + this->min_batch_size_;
    }

public:
    DummyRxTx(size_t min=0, size_t max=10) : BaseRxTx(), max_batch_size_(max), min_batch_size_(min) {}

    std::vector<uint8_t> ReadBytes() override {
        size_t size = this->generateSize();
        std::vector<uint8_t> values(size);

        for (size_t i = 0; i < size; i++) {
            values[i] = (rand() / RAND_MAX) * UINT8_MAX;
        }

        return values;
    }

    void WriteBytes(const void *, const uint) override {
    };
};

#endif
