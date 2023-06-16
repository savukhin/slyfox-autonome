#pragma once

#include <stdint.h>
#include <vector>

class BaseRxTx {
public:
    BaseRxTx() {}
    virtual ~BaseRxTx() {}

    virtual std::vector<uint8_t> ReadBytes() = 0;
    virtual void WriteBytes(const void *b, const uint size) = 0;
};
