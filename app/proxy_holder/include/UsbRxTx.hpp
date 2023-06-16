#pragma once

#include "BaseRxTx.hpp"
#include <stddef.h>
#include <cstdlib>

#include <stdio.h>
#include <string>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "serialib.hpp"

class UsbRxTx : public BaseRxTx
{
private:
    serialib serial_;
    uint batch_size_;

public:
    UsbRxTx(const char *port, uint baudrate, uint batch_size=64) : BaseRxTx(), batch_size_(batch_size)
    {
        char errorOpening = serial_.openDevice(port, baudrate);

        if (errorOpening != 1)
            throw new std::runtime_error("cannot open port " + std::string(port));
    }

    ~UsbRxTx()
    {
        serial_.closeDevice();
    }

    std::vector<uint8_t> ReadBytes() override
    {
        uint8_t *received = new uint8_t[this->batch_size_];
        // unsigned char *received = new unsigned char[this->batch_size_];
        int n_bytes = serial_.readBytes(received, 64, 100, 1000);

        std::vector<uint8_t> res(n_bytes);
        for (int i = 0; i < n_bytes; i++) {
            res[i] = uint8_t(received[i]);
        }
        delete received;
        // free(received);

        return res;
    }

    void WriteBytes(const void *b, const uint size) override
    {
        this->serial_.writeBytes(b, size);
    }
};