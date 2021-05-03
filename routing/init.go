package routing

import (
	"log"
	"math"
	"sort"
	"time"
)

const (
	CloseThreth   int8 = 90     //これより大きいと通れない  [0,100]
	MaxTimeLength int  = 100000 //これ以上のtを計算しない

	MaxSearchTimeStep int = 1000 //これ以上先の時間を計算しない

	TimeStep float64 = 1.0 //時間ステップの幅 (second)
)

type Point struct {
	X float64
	Y float64
}

// type CostMap map[Index]uint8
type TimeRobotMap []ObjMap
type ObjMap [][]bool

type GridMap struct {
	Resolution float64
	Origin     Point
	Width      int
	Height     int
	MapOrigin  Point

	CurrentMinTime float64

	TW        TimeRobotMap
	MaxT      int
	ObjectMap ObjMap //元からある障害物ならTrue
}

func (g GridMap) Ind2Pos(xId, yId int) (float64, float64) {
	x := g.Origin.X + float64(xId)*g.Resolution
	y := g.Origin.Y + float64(yId)*g.Resolution
	return x, y
}

func (g GridMap) Pos2Ind(x, y float64) (int, int) {
	if x < g.Origin.X || y < g.Origin.Y {
		log.Printf("position (%f,%f) is out of map", x, y)
		return 0, 0
	}
	xid := int(math.Round((x - g.Origin.X) / g.Resolution))
	yid := int(math.Round((y - g.Origin.Y) / g.Resolution))
	return xid, yid
}

// initialize grid map using map resolution
// not using now
func NewGridMap(m MapMeta, maxT int, robotRadius float64) *GridMap {
	g := new(GridMap)
	g.Resolution = m.Reso
	g.Origin = m.Origin
	g.Width = m.W
	g.Height = m.H
	g.MaxT = maxT
	g.ObjectMap = make([][]bool, m.H)
	for i := 0; i < m.H; i++ {
		g.ObjectMap[i] = make([]bool, m.W)
	}

	g.TW = make(TimeRobotMap, maxT)
	for i := 0; i < maxT; i++ {
		g.TW[i] = g.ObjectMap
	}

	width := m.W

	var objList [][2]float64
	for i, d := range m.Data {
		if d > int8(CloseThreth) {
			g.ObjectMap[i/width][i%width] = true
			objList = append(objList, [2]float64{m.Origin.X + float64(i%width)*m.Reso, m.Origin.Y + float64(i/width)*m.Reso})
		} else {
			g.ObjectMap[i/width][i%width] = false
		}
	}

	start := time.Now()
	for j := 0; j < m.H; j++ {
		if j%2 == 1 {
			continue
		}
		y := g.Origin.Y + float64(j)*m.Reso
		for i := 0; i < m.W; i++ {
			if i%2 == 1 {
				continue
			}
			x := m.Origin.X + float64(i)*m.Reso
			for _, op := range objList {
				d := math.Hypot(x-op[0], y-op[1])
				if d <= robotRadius {
					g.ObjectMap[j][i] = true
					break
				}
			}
		}
	}
	elaps := time.Since(start).Seconds()
	log.Printf("load objmap using robot radius takes %f seconds", elaps)

	return g
}

// initialize custom resolution
// using main
func NewGridMapReso(m MapMeta, maxT int, robotRadius float64, resolution float64, objMap [][2]float64) *GridMap {
	start := time.Now()
	g := new(GridMap)
	g.Resolution = resolution

	var xList []float64
	var yList []float64

	for _, obj := range objMap {
		xList = append(xList, obj[0])
		yList = append(yList, obj[1])
	}

	maxX := MaxFloat(xList)
	maxY := MaxFloat(yList)
	g.Origin.X = MinFloat(xList)
	g.Origin.Y = MinFloat(yList)

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
		y := g.Origin.Y + float64(j)*g.Resolution
		for i := 0; i < g.Width; i++ {
			x := g.Origin.X + float64(i)*g.Resolution
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

func MinFloat(a []float64) float64 {
	sort.Float64s(a)
	return a[0]
}

func MaxFloat(a []float64) float64 {
	sort.Float64s(a)
	return a[len(a)-1]
}
