package crsf

import (
	"bytes"
	"encoding/binary"
	"slyfox-payload-controller/utils"
	"time"
)

type Crsf struct {
	// rxBuf []uint8
	rxBuf              [CRSF_MAX_PACKET_LEN + 3]uint8
	rxBufPos           uint8
	crc                Crc8
	linkStatistics     *CrsfLinkStatistics
	gpsSensor          *CrsfSensorGps
	lastReceive        uint32
	lastChannelsPacket uint32
	linkIsUp           bool
	passthroughMode    bool
	// channels           [CRSF_NUM_CHANNELS]int
	channels [CRSF_NUM_CHANNELS]int32

	onLinkUp func()
	// onLinkDown             func()
	OnPacketChannels       func()
	onShiftyByte           func(b uint8)
	onPacketLinkStatistics func(ls *CrsfLinkStatistics)
	onPacketGps            func(gpsSensor *CrsfSensorGps)
}

func NewCrsf() *Crsf {
	return &Crsf{
		crc:                *NewCrc8(0xd5),
		lastReceive:        0,
		lastChannelsPacket: 0,
		linkIsUp:           false,
		passthroughMode:    false,
	}
}

func (crsf *Crsf) HandleBytes(sequence []byte) {
	for _, b := range sequence {
		crsf.HandleByte(b)
	}
}

func (crsf *Crsf) HandleByte(b uint8) {
	crsf.lastReceive = uint32(time.Now().Unix())

	if crsf.passthroughMode {
		if crsf.onShiftyByte != nil {
			crsf.onShiftyByte(b)
		}
		return
	}

	crsf.rxBuf[crsf.rxBufPos] = b
	crsf.rxBufPos++
	crsf.handleByteReceived()

	if int(crsf.rxBufPos) == len(crsf.rxBuf) {
		// Packet buffer filled and no valid packet found, dump the whole thing
		crsf.rxBufPos = 0
	}
}

func (crsf *Crsf) handleByteReceived() {
	reprocess := true
	for reprocess {
		reprocess = false
		if crsf.rxBufPos > 1 {
			var len uint8 = crsf.rxBuf[1]

			// Sanity check the declared length, can't be shorter than Type, X, CRC
			if len < 3 || len > CRSF_MAX_PACKET_LEN {
				crsf.shiftRxBuffer(1)
				reprocess = true
			} else if crsf.rxBufPos >= (len + 2) {
				var inCrc uint8 = crsf.rxBuf[2+len-1]
				var crc uint8 = crsf.crc.Calc(crsf.rxBuf[2:], len-1)

				if crc == inCrc {
					crsf.processPacketIn(len)
					crsf.shiftRxBuffer(len + 2)
					reprocess = true
				} else {
					crsf.shiftRxBuffer(1)
					reprocess = true
				}
			} // if complete packet
		} // if pos > 1
	}
}

// Shift the bytes in the RxBuf down by cnt bytes
func (crsf *Crsf) shiftRxBuffer(cnt uint8) {
	// If removing the whole thing, just set pos to 0
	if cnt >= crsf.rxBufPos {
		crsf.rxBufPos = 0
		return
	}

	if cnt == 1 && crsf.onShiftyByte != nil {
		crsf.onShiftyByte(crsf.rxBuf[0])
	}

	// Otherwise do the slow shift down
	// uint8_t * src = &_rxBuf[cnt]
	// uint8_t *dst = &_rxBuf[0];
	// _rxBufPos -= cnt;
	// uint8_t left = _rxBufPos;
	// while (left--)
	// 	*dst++ = *src++;
	right := cnt
	left := uint8(0)
	crsf.rxBufPos -= cnt
	i := crsf.rxBufPos
	for i != 0 {
		crsf.rxBuf[left] = crsf.rxBuf[right]

		left++
		right++
		i -= 1
	}
}

func (crsf *Crsf) processPacketIn(len uint8) {
	// const crsf_header_t *hdr = (crsf_header_t *)_rxBuf;
	hdr := ToCrsfHeader(crsf.rxBuf[:])
	if CrsfAddr(hdr.DeviceAddr) == CRSF_ADDRESS_FLIGHT_CONTROLLER {
		switch hdr.Type {
		case CRSF_FRAMETYPE_GPS:
			crsf.packetGps(hdr)
		case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
			crsf.packetChannelsPacked(hdr)
		case CRSF_FRAMETYPE_LINK_STATISTICS:
			crsf.packetLinkStatistics(hdr)
		}
	} // CRSF_ADDRESS_FLIGHT_CONTROLLER
}

func (crsf *Crsf) packetGps(p *CrsfHeader) {
	// const crsf_sensor_gps_t *gps = (crsf_sensor_gps_t *)p->data;
	crsf.gpsSensor = ToCrsfSensorGps(p.Data)

	if crsf.onPacketGps != nil {
		crsf.onPacketGps(crsf.gpsSensor)
	}
}

func (crsf *Crsf) packetLinkStatistics(p *CrsfHeader) {
	crsf.linkStatistics = ToCrsfLinkStatistics(p.Data)

	if crsf.onPacketLinkStatistics != nil {
		crsf.onPacketLinkStatistics(crsf.linkStatistics)
	}
}

func (crsf *Crsf) packetChannelsPacked(p *CrsfHeader) error {
	var ch CrsfChannels
	var buf = bytes.NewBuffer(make([]byte, 0, len(p.Data)))

	if err := binary.Write(buf, binary.BigEndian, &p.Data); err != nil {
		return err
	}

	if err := binary.Read(buf, binary.BigEndian, &ch); err != nil {
		return err
	}

	// crsf.channels[0] = ch.CH0
	// crsf.channels[1] = ch.CH1
	// crsf.channels[2] = ch.CH2
	// crsf.channels[3] = ch.CH3
	// crsf.channels[4] = ch.CH4
	// crsf.channels[5] = ch.CH5
	// crsf.channels[6] = ch.CH6
	// crsf.channels[7] = ch.CH7
	// crsf.channels[8] = ch.CH8
	// crsf.channels[9] = ch.CH9
	// crsf.channels[10] = ch.CH10
	// crsf.channels[11] = ch.CH11
	// crsf.channels[12] = ch.CH12
	// crsf.channels[13] = ch.CH13
	// crsf.channels[14] = ch.CH14
	// crsf.channels[15] = ch.CH15

	for i := 0; i < CRSF_NUM_CHANNELS; i++ {
		// _channels[i] = map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
		// interpolation.Cast,
		// interploated := utils.Lerp(float64(crsf.channels[i]), CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000)
		interploated := utils.Lerp(float64(ch.CH[i]), CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000)
		crsf.channels[i] = int32(interploated)

	}

	if !crsf.linkIsUp && crsf.onLinkUp != nil {
		crsf.onLinkUp()
	}
	crsf.linkIsUp = true
	crsf.lastChannelsPacket = uint32(time.Now().Unix())

	if crsf.OnPacketChannels != nil {
		crsf.OnPacketChannels()
	}

	return nil
}

func (crsf *Crsf) QueuePacket(addr, data_type uint8, payload []byte) *[CRSF_MAX_PACKET_LEN + 4]byte {
	if !crsf.linkIsUp {
		return nil
	}
	if crsf.passthroughMode {
		return nil
	}

	len := uint8(len(payload))

	if len > CRSF_MAX_PACKET_LEN {
		return nil
	}

	buf := &[CRSF_MAX_PACKET_LEN + 4]byte{}
	// var buf [CRSF_MAX_PACKET_LEN + 4]uint8
	// buf := make([CRSF_MAX_PACKET_LEN + 4]uint8)
	buf[0] = addr
	buf[1] = len + 2 // type + payload + crc
	buf[2] = data_type
	// memcpy(&buf[3], payload, len);

	for i := uint8(0); i < len; i++ {
		buf[3+i] = payload[i]
	}

	buf[len+3] = crsf.crc.Calc(buf[2:], len+1)

	// Busywait until the serial port seems free
	//while (millis() - _lastReceive < 2)
	//    loop();
	// write(buf, len+4)

	return buf
}

func (crsf *Crsf) SetPassthroughMode(val bool, baud uint) {
	crsf.passthroughMode = val
	// _port.flush();
	// if (baud != 0)
	//     _port.begin(baud);
	// else
	//     _port.begin(_baud);
}
