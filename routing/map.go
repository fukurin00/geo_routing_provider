package routing

import (
	"image"
	"image/color"
	"log"
	"os"

	_ "github.com/jbuchbinder/gopnm"
)

type MapMeta struct {
	W      int
	H      int
	Origin Point
	Reso   float64
	Data   []uint8
}

// read image file of ROS format
func ReadMapImage(yamlFile, mapFile string, closeThreth int) (*MapMeta, error) {
	m := new(MapMeta)
	mapConfig := ReadImageYaml(yamlFile)
	m.Reso = mapConfig.Resolution
	m.Origin = Point{X: mapConfig.Origin[0], Y: mapConfig.Origin[1]}

	file, err := os.Open(mapFile)
	if err != nil {
		return m, err
	}
	defer file.Close()

	imageData, _, err := image.Decode(file)
	if err != nil {
		return m, err
	}

	bound := imageData.Bounds()
	m.W = bound.Dx()
	m.H = bound.Dy()

	data := make([]uint8, m.W*m.H)
	open := 0
	close := 0
	for j := m.H - 1; j >= 0; j-- {
		for i := 0; i < m.W; i++ {
			oldPix := imageData.At(i, j)
			pixel := color.GrayModel.Convert(oldPix).(color.Gray).Y
			// pixelU := color.GrayModel.Convert(pixel).(color.Gray).Y

			a := pixel
			var v uint8 = 0
			if a > uint8(closeThreth) {
				v = 100
				close += 1
			} else {
				v = 0
				open += 1
			}
			data[i+j*m.W] = v
		}
	}
	log.Printf("open: %d, close: %d", open, close)
	m.Data = data

	return m, nil
}
