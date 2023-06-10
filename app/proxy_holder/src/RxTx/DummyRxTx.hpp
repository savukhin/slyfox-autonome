#pragma once

#include "BaseRxTx.hpp"
#include <stddef.h>
#include <cstdlib>

class DummyRxTx : public BaseRxTx {
private:
    size_t minBatchSize;
    size_t maxBatchSize;

    size_t generateSize() {
        return (rand() / RAND_MAX) * (this->maxBatchSize - this->minBatchSize) + this->minBatchSize;
    }

public:
    DummyRxTx(size_t min=0, size_t max=10) : BaseRxTx() {}

    std::vector<uint8_t> ReadBytes() override {
        size_t size = this->generateSize();
        std::vector<uint8_t> values(size);

        for (size_t i = 0; i < size; i++) {
            values[i] = (rand() / RAND_MAX) * UINT8_MAX;
        }

        return values;
    }
};