package main

import (
	"slyfox-payload-controller/analyzers"
	"slyfox-payload-controller/devices"
	"slyfox-payload-controller/utils"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/opencv"
	"gobot.io/x/gobot/platforms/raspi"
	"gocv.io/x/gocv"
)

func main() {
	// rx_port := utils.GetEnv("RX_UART", "/dev/tty1")
	// rx_baud := utils.GetEnvInt("RX_BAUD", crsf.CRSF_BAUDRATE)
	// tx_port := utils.GetEnv("TX_UART", "/dev/tty2")
	// tx_baud := utils.GetEnvInt("TX_BAUD", crsf.CRSF_BAUDRATE)

	// rx_from_apparature, err := devices.NewCrsfSerial(rx_port, rx_baud)
	// if err != nil {
	// 	fmt.Println("Rx setting up finished with error:", err.Error())
	// 	return
	// }

	// tx_to_payload, err := devices.NewCrsfSerial(tx_port, tx_baud)
	// if err != nil {
	// 	fmt.Println("Tx setting up finished with error:", err.Error())
	// 	return
	// }

	// rx_from_apparature.SetOnPacketChannels(nil)
	// tx_to_payload.SetOnPacketChannels(nil)

	r := raspi.NewAdaptor()
	camera := opencv.NewCameraDriver(0)

	motorX1Pin := utils.GetEnv("MOTOR_X1_PIN", "1")
	motorX2Pin := utils.GetEnv("MOTOR_X2_PIN", "1")
	motorY1Pin := utils.GetEnv("MOTOR_Y1_PIN", "1")
	motorY2Pin := utils.GetEnv("MOTOR_Y2_PIN", "1")

	motorsX := devices.NewServoMotorPair(r, motorX1Pin, motorX2Pin)
	motorsY := devices.NewServoMotorPair(r, motorY1Pin, motorY2Pin)

	pidController := analyzers.NewPIDMotors(
		[]devices.IServoMotor{motorsX.IServoMotor, motorsY.IServoMotor},
		analyzers.DefaultProportionalGain,
		analyzers.DefaultIntegralGain,
		analyzers.DefaultDerivativeGain,
	)

	analyzer := analyzers.NewImageAnalyzer(10, 10000)

	lastSample := time.Now()

	work := func() {
		camera.On(opencv.Frame, func(data interface{}) {
			camera.Halt()

			img := data.(gocv.Mat)

			clusters := analyzer.Analyze(img)

			origin := clusters[0]
			values := []float64{float64(origin.X), float64(origin.Y)}

			currentSample := time.Now()

			pidController.Update(values, currentSample.Sub(lastSample))

			lastSample = currentSample

			camera.Start()
		})
	}

	robot := gobot.NewRobot("arcticFox",
		[]gobot.Connection{r},
		[]gobot.Device{camera},
		work,
	)

	robot.Start()

}
