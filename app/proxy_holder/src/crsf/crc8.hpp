#pragma once

#include <stdint.h>

class Crc8
{
public:
    Crc8(uint8_t poly) {
        init(poly);
    }
    uint8_t calc(uint8_t *data, uint8_t len) {
        uint8_t crc = 0;
    while (len--)
    {
        crc = _lut[crc ^ *data++];
    }
    return crc;
    }

protected:
    uint8_t _lut[256];
    void init(uint8_t poly) { 
        for (int idx=0; idx<256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift=0; shift<8; ++shift)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        _lut[idx] = crc & 0xff;
    }
    }
};