package crsf

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"slyfox-payload-controller/utils"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCrsf(t *testing.T) {
	crsf := NewCrsf()

	addr := CRSF_ADDRESS_FLIGHT_CONTROLLER
	data_type := CRSF_FRAMETYPE_RC_CHANNELS_PACKED
	desired_channels := []uint16{210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360}
	desired_channels_lerped := make([]int32, len(desired_channels))
	for i, channel := range desired_channels {
		desired_channels_lerped[i] = int32(utils.Lerp(float64(channel), CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000))
	}

	var data_bytes []byte

	var buf = bytes.NewBuffer(make([]byte, 0, len(desired_channels)))

	if err := binary.Write(buf, binary.BigEndian, desired_channels); err != nil {
		panic("p data" + err.Error())
	}

	data_bytes = buf.Bytes()

	t.Log("Testing shifting")
	{
		crsf := NewCrsf()
		for i := 0; i < 5; i++ {
			crsf.rxBuf[i] = uint8(i + 1)
		}
		crsf.rxBufPos = 5
		crsf.shiftRxBuffer(2)

		require.ElementsMatch(t, crsf.rxBuf[:5], []uint8{3, 4, 5, 4, 5})
	}

	crsf.linkIsUp = true
	packed := crsf.QueuePacket(uint8(addr), uint8(data_type), data_bytes)[:]

	crsf.HandleBytes(packed)

	require.ElementsMatch(t, crsf.channels[:16], desired_channels_lerped)

	fmt.Println(packed)
}

func TestCrc(t *testing.T) {
	crc := NewCrc8(0xd5)
	require.NotNil(t, crc)
}
