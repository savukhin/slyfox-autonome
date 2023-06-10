package crsf

type Crc8 struct {
	lut [256]uint8
}

func NewCrc8(poly uint8) *Crc8 {
	crc8 := &Crc8{}

	for idx := 0; idx < 256; idx++ {
		crc := uint8(idx)
		for shift := 0; shift < 8; shift++ {
			// crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
			var xoring uint8 = 0

			if (crc & 0x80) != 0 {
				xoring = poly
			} else {
				xoring = 0
			}
			crc = (crc << 1) ^ xoring
		}
		crc8.lut[idx] = crc & 0xff
	}

	return crc8
}

func (c *Crc8) Calc(data []uint8, len uint8) uint8 {
	var crc uint8 = 0

	i := 0

	for len != 0 {
		len--
		crc = c.lut[crc^data[i]]
		i++
	}

	return crc
}
