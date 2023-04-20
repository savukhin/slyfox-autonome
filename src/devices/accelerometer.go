package devices

type IAccelerometer interface {
	GetMeasurements() (ax, ay, az float64)
}

type DummyAccelerometer struct {
}

func (accel *DummyAccelerometer) GetMeasurements() (ax, ay, az float64) {
	return 0, 0, 0
}
