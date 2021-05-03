package routing

import (
	"log"
	"math"
	"time"
)

// initialize custom resolution
// using main
func NewGridMapResoHexa(m MapMeta, maxT int, robotRadius float64, resolution float64, objMap [][2]float64) *GridMap {
	start := time.Now()
	g := new(GridMap)
	g.MapOrigin = m.Origin
	g.Resolution = resolution

	var aList []float64
	var bList []float64

	for _, obj := range objMap {
		aList = append(aList, getA(obj[0], obj[1]))
		bList = append(bList, getB(obj[0], obj[1]))
	}

	maxX := MaxFloat(aList)
	maxY := MaxFloat(bList)
	g.Origin.X = MinFloat(aList)
	g.Origin.Y = MinFloat(bList)

	g.Width = int(math.Round((maxX - g.Origin.X) / resolution))
	g.Height = int(math.Round((maxY - g.Origin.Y) / resolution))

	g.MaxT = maxT
	g.ObjectMap = make([][]bool, g.Height)
	for i := 0; i < g.Height; i++ {
		g.ObjectMap[i] = make([]bool, g.Width)
	}
	g.TW = make(TimeRobotMap, maxT)
	for i := 0; i < maxT; i++ {
		g.TW[i] = g.ObjectMap
	}

	count := 0
	for j := 0; j < g.Height; j++ {
		b := g.Origin.Y + float64(j)*g.Resolution
		for i := 0; i < g.Width; i++ {
			a := g.Origin.X + float64(i)*g.Resolution
			x := getXAB(a, b)
			y := getYAB(a, b)
			g.ObjectMap[j][i] = false
			for _, op := range objMap {
				d := math.Hypot(op[0]-x, op[1]-y)
				if d <= robotRadius {
					g.ObjectMap[j][i] = true
					count += 1
					break
				}
			}
		}
	}

	elaps := time.Since(start).Seconds()
	log.Printf("loading gridmap resolution: %f, takes: %f seconds, obj %d counts, width: %d, height: %d", resolution, elaps, count, g.Width, g.Height)
	return g
}

func getA(x, y float64) float64 {
	return x/math.Sqrt(3) + y
}

func getB(x, y float64) float64 {
	return x/math.Sqrt(3) - y
}

func getXAB(a, b float64) float64 {
	return math.Sqrt(3)/2*a + math.Sqrt(3)/2*b
}

func getYAB(a, b float64) float64 {
	return a/2 - b/2
}
