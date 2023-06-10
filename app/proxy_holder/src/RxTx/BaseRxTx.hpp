#pragma once

#include <stdint.h>
#include <vector>

class BaseRxTx {
public:
    BaseRxTx() {}

    virtual std::vector<uint8_t> ReadBytes();
};