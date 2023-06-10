package crsf

const (
	CRSF_BAUDRATE           = 420000
	CRSF_NUM_CHANNELS       = 16
	CRSF_CHANNEL_VALUE_MIN  = 172
	CRSF_CHANNEL_VALUE_1000 = 191
	CRSF_CHANNEL_VALUE_MID  = 992
	CRSF_CHANNEL_VALUE_2000 = 1792
	CRSF_CHANNEL_VALUE_MAX  = 1811
	CRSF_CHANNEL_VALUE_SPAN = (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
	CRSF_MAX_PACKET_LEN     = 64
)

const CRSF_SYNC_BYTE = 0xC8

// Clashes with CRSF_ADDRESS_FLIGHT_CONTROLLER

type CrsfFrameLength int

const (
	CRSF_FRAME_LENGTH_ADDRESS      CrsfFrameLength = 1 // length of ADDRESS field
	CRSF_FRAME_LENGTH_FRAMELENGTH  CrsfFrameLength = 1 // length of FRAMELENGTH field
	CRSF_FRAME_LENGTH_TYPE         CrsfFrameLength = 1 // length of TYPE field
	CRSF_FRAME_LENGTH_CRC          CrsfFrameLength = 2 // length of CRC field
	CRSF_FRAME_LENGTH_TYPE_CRC     CrsfFrameLength = 2 // length of TYPE and CRC fields combined
	CRSF_FRAME_LENGTH_EXT_TYPE_CRC CrsfFrameLength = 4 // length of Extended Dest/Origin, TYPE and CRC fields combined
	CRSF_FRAME_LENGTH_NON_PAYLOAD  CrsfFrameLength = 4 // combined length of all fields except payload
)

type CrsfFrameAddress int

const (
	CRSF_FRAME_GPS_PAYLOAD_SIZE             CrsfFrameAddress = 15
	CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE  CrsfFrameAddress = 8
	CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE CrsfFrameAddress = 10
	CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE     CrsfFrameAddress = 22 // 11 bits per channel * 16 channels = 22 bytes.
	CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE        CrsfFrameAddress = 6
)

type CrsfFrameType int

const (
	CRSF_FRAMETYPE_GPS                CrsfFrameType = 0x02
	CRSF_FRAMETYPE_BATTERY_SENSOR     CrsfFrameType = 0x08
	CRSF_FRAMETYPE_LINK_STATISTICS    CrsfFrameType = 0x14
	CRSF_FRAMETYPE_OPENTX_SYNC        CrsfFrameType = 0x10
	CRSF_FRAMETYPE_RADIO_ID           CrsfFrameType = 0x3A
	CRSF_FRAMETYPE_RC_CHANNELS_PACKED CrsfFrameType = 0x16
	CRSF_FRAMETYPE_ATTITUDE           CrsfFrameType = 0x1E
	CRSF_FRAMETYPE_FLIGHT_MODE        CrsfFrameType = 0x21
	// Extended Header Frames, range: 0x28 to 0x96
	CRSF_FRAMETYPE_DEVICE_PING              CrsfFrameType = 0x28
	CRSF_FRAMETYPE_DEVICE_INFO              CrsfFrameType = 0x29
	CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY CrsfFrameType = 0x2B
	CRSF_FRAMETYPE_PARAMETER_READ           CrsfFrameType = 0x2C
	CRSF_FRAMETYPE_PARAMETER_WRITE          CrsfFrameType = 0x2D
	CRSF_FRAMETYPE_COMMAND                  CrsfFrameType = 0x32
	// MSP commands
	CRSF_FRAMETYPE_MSP_REQ   CrsfFrameType = 0x7A // response request using msp sequence as command
	CRSF_FRAMETYPE_MSP_RESP  CrsfFrameType = 0x7B // reply with 58 byte chunked binary
	CRSF_FRAMETYPE_MSP_WRITE CrsfFrameType = 0x7C // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
)

type CrsfAddr uint8

const (
	CRSF_ADDRESS_BROADCAST         CrsfAddr = 0x00
	CRSF_ADDRESS_USB               CrsfAddr = 0x10
	CRSF_ADDRESS_TBS_CORE_PNP_PRO  CrsfAddr = 0x80
	CRSF_ADDRESS_RESERVED1         CrsfAddr = 0x8A
	CRSF_ADDRESS_CURRENT_SENSOR    CrsfAddr = 0xC0
	CRSF_ADDRESS_GPS               CrsfAddr = 0xC2
	CRSF_ADDRESS_TBS_BLACKBOX      CrsfAddr = 0xC4
	CRSF_ADDRESS_FLIGHT_CONTROLLER CrsfAddr = 0xC8
	CRSF_ADDRESS_RESERVED2         CrsfAddr = 0xCA
	CRSF_ADDRESS_RACE_TAG          CrsfAddr = 0xCC
	CRSF_ADDRESS_RADIO_TRANSMITTER CrsfAddr = 0xEA
	CRSF_ADDRESS_CRSF_RECEIVER     CrsfAddr = 0xEC
	CRSF_ADDRESS_CRSF_TRANSMITTER  CrsfAddr = 0xEE
)

type CrsfHeader struct {
	DeviceAddr uint8         // from crsf_addr_e
	FrameSize  uint8         // counts size after this byte, so it must be the payload size + 2 (type and crc)
	Type       CrsfFrameType // from crsf_frame_type_e
	Data       []uint8
}

func ToCrsfHeader(buf []uint8) *CrsfHeader {
	return &CrsfHeader{
		DeviceAddr: buf[0],
		FrameSize:  buf[1],
		Type:       CrsfFrameType(buf[2]),
		Data:       buf[3:],
	}
}

// type ChannelInt uint16

type CrsfChannels struct {
	// CH0  uint32
	// CH1  uint32
	// CH2  uint32
	// CH3  uint32
	// CH4  uint32
	// CH5  uint32
	// CH6  uint32
	// CH7  uint32
	// CH8  uint32
	// CH9  uint32
	// CH10 uint32
	// CH11 uint32
	// CH12 uint32
	// CH13 uint32
	// CH14 uint32
	// CH15 uint32
	CH [CRSF_NUM_CHANNELS]uint16
}

// func ToChannels(data []byte) *CrsfChannels {
// 	result := &{}
// }

type CrsfLinkStatistics struct {
	Uplink_RSSI_1         uint8
	Uplink_RSSI_2         uint8
	Uplink_Link_quality   uint8
	Uplink_SNR            int8
	Active_antenna        uint8
	Rf_Mode               uint8
	Uplink_TX_Power       uint8
	Downlink_RSSI         uint8
	Downlink_Link_quality uint8
	Downlink_SNR          int8
}

func ToCrsfLinkStatistics(data []uint8) *CrsfLinkStatistics {
	return &CrsfLinkStatistics{
		Uplink_RSSI_1:         data[0],
		Uplink_RSSI_2:         data[1],
		Uplink_Link_quality:   data[2],
		Uplink_SNR:            int8(data[3]),
		Active_antenna:        data[4],
		Rf_Mode:               data[5],
		Uplink_TX_Power:       data[6],
		Downlink_RSSI:         data[7],
		Downlink_Link_quality: data[8],
		Downlink_SNR:          int8(data[9]),
	}
}

type CrsfSensorBattery struct {
	Voltage   uint // V * 10 big endian
	Current   uint // A * 10 big endian
	Capacity  uint // mah big endian
	Remaining uint // %
}

func NewCrsfSensorBattery() CrsfSensorBattery {
	return CrsfSensorBattery{
		Voltage:   16,
		Current:   16,
		Capacity:  24,
		Remaining: 8,
	}
}

type CrsfSensorGps struct {
	Latitude    int32  // degree / 10,000,000 big endian
	Longitude   int32  // degree / 10,000,000 big endian
	Groundspeed uint16 // km/h / 10 big endian
	Heading     uint16 // GPS heading, degree/100 big endian
	Altitude    uint16 // meters, +1000m big endian
	Satellites  uint8  // satellites
}

func ToCrsfSensorGps(data []uint8) *CrsfSensorGps {
	return &CrsfSensorGps{
		// Latitude: data[0],
	}
}
