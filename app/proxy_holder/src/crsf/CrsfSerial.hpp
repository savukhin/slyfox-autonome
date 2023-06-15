#pragma once

#include "crc8.hpp"
#include "CrsfProtocol.hpp"

#include <vector>

enum eFailsafeAction { fsaNoPulses, fsaHold };

class CrsfSerial
{
public:
    // Packet timeout where buffer is flushed if no data is received in this time
    static const unsigned int CRSF_PACKET_TIMEOUT_MS = 100;
    static const unsigned int CRSF_FAILSAFE_STAGE1_MS = 300;

    CrsfSerial();
    // uint8_t* queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    std::pair<uint8_t*, int> queuePacket(uint8_t addr, uint8_t type, const void *payload, uint8_t len);
    void handleByte(uint8_t b);
    void handleBytes(std::vector<uint8_t> sequence);
    void handleBytes(uint8_t *sequence, size_t len);

    // Return current channel value (1-based) in us
    int getChannel(unsigned int ch) const { return _channels[ch - 1]; }
    int* getChannels() { return _channels; }
    const crsfLinkStatistics_t *getLinkStatistics() const { return &_linkStatistics; }
    const crsf_sensor_gps_t *getGpsSensor() const { return &_gpsSensor; }
    bool isLinkUp() const { return _linkIsUp; }
    bool getPassthroughMode() const { return _passthroughMode; }
    void setPassthroughMode(bool val);

    // Event Handlers
    std::function<void()> onLinkUp;
    std::function<void()> onLinkDown;
    std::function<void()> onPacketChannels;
    std::function<void(uint8_t)> onShiftyByte;
    std::function<void(crsfLinkStatistics_t*)> onPacketLinkStatistics;
    std::function<void(crsf_sensor_gps_t*)> onPacketGps;

private:
    uint8_t _rxBuf[CRSF_MAX_PACKET_SIZE];
    uint8_t _rxBufPos;
    Crc8 _crc;
    crsfLinkStatistics_t _linkStatistics;
    crsf_sensor_gps_t _gpsSensor;
    uint32_t _lastReceive;
    uint32_t _lastChannelsPacket;
    bool _linkIsUp;
    bool _passthroughMode;
    int _channels[CRSF_NUM_CHANNELS];

    
    void handleByteReceived();
    void shiftRxBuffer(uint8_t cnt);
    void processPacketIn(uint8_t len);
    void checkPacketTimeout();
    void checkLinkDown();

    // Packet Handlers
    void packetChannelsPacked(const crsf_header_t *p);
    void packetLinkStatistics(const crsf_header_t *p);
    void packetGps(const crsf_header_t *p);
};

#include "CrsfSerial.cpp"
