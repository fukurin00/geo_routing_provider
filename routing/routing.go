package routing

import (
	"errors"
	"sort"

	"log"
	"math"
	"time"
)

const (
	CloseThreth       int8 = 90     //これより大きいと通れない  [0,100]
	MaxTimeLength     int  = 100000 //これ以上のtを計算しない
	MaxSearchTimeStep int  = 1000   //これ以上先の時間を計算しない
)

type Point struct {
	X float64
	Y float64
}

type Index struct {
	X int
	Y int
}

func nodeIndex(n *Node) Index {
	i := Index{
		X: n.XId,
		Y: n.YId,
	}
	return i
}

type IndexT struct {
	X int
	Y int
	T int
}

func nodeIndexT(n *Node) IndexT {
	i := IndexT{
		X: n.XId,
		Y: n.YId,
		T: n.T,
	}
	return i
}

func NewIndexT(t, x, y int) *IndexT {
	n := new(IndexT)
	n.T = t
	n.X = x
	n.Y = y
	return n
}

// type CostMap map[Index]uint8
type TimeCostMap []CostMap
type CostMap [][]int8
type TimeObjMap [][][]bool

type GridMap struct {
	Resolution float64
	Origin     Point
	Width      int
	Height     int

	TW        TimeObjMap
	MaxT      int
	ObjectMap [][]bool //元からある障害物ならTrue
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
	g.TW = make(TimeObjMap, maxT)

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
	log.Print("load ros data")

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
	elaps := time.Now().Sub(start).Seconds()
	log.Printf("load objmap using robot radius takes %f seconds", elaps)

	for i := 0; i < maxT; i++ {
		g.TW[i] = g.ObjectMap
	}
	return g
}

func NewGridMapReso(m MapMeta, maxT int, robotRadius float64, resolution float64, objMap [][2]float64) *GridMap {
	g := new(GridMap)
	g.Resolution = m.Reso

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

	g.Width = int(math.Round(maxX - g.Origin.X))
	g.Height = int(math.Round(maxY - g.Origin.Y))

	g.MaxT = maxT
	g.ObjectMap = make([][]bool, m.H)
	for i := 0; i < m.H; i++ {
		g.ObjectMap[i] = make([]bool, m.W)
	}
	g.TW = make(TimeObjMap, maxT)

	for j := 0; j < g.Height; j++ {
		y := g.Origin.Y + float64(j)*g.Resolution
		for i := 0; i < g.Width; i++ {
			x := g.Origin.X + float64(i)*g.Resolution
			for _, op := range objMap {
				d := math.Hypot(op[0]-x, op[1]-y)
				if d <= robotRadius {
					g.ObjectMap[j][i] = true
					break
				}
			}
		}
	}
	for i := 0; i < maxT; i++ {
		g.TW[i] = g.ObjectMap
	}
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

func (m GridMap) Plan(sx, sy, gx, gy int) (route [][3]int, oerr error) {
	startTime := time.Now()

	if m.ObjectMap[gy][gx] {
		oerr = errors.New("path planning error: goal is not verified")
		return nil, oerr
	}
	if m.ObjectMap[sy][sx] {
		oerr = errors.New("path planning error: start point is not verified")
		return nil, oerr
	}
	start := &Node{T: 0, XId: sx, YId: sy, Cost: 0, Parent: nil}
	goal := &Node{T: 0, XId: gx, YId: gy, Cost: 0, Parent: nil}

	openSet := make(map[IndexT]*Node)

	closeSet := make(map[Index]*Node)
	closeSetT := make(map[IndexT]*Node)

	openSet[nodeIndexT(start)] = start

	count := 0
	current := &Node{T: 0}
	var minTime int

	for {
		count += 1
		if len(openSet) == 0 {
			elaps := time.Now().Sub(startTime).Seconds()
			log.Print(current.T, current.XId, current.YId, count, elaps)
			oerr = errors.New("path planning error: open set is empty...")
			return nil, oerr
		}

		// get min cost node in open set
		minCost := 9999999999999999999.9
		minTime = 99999999999999999
		var minKey IndexT
		for key, val := range openSet {
			calCost := val.Cost + heuristic(goal, val)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
			if val.T < minTime {
				minTime = val.T
			}
		}
		current = openSet[minKey]
		// fmt.Print(current.T, current.XId, current.YId, current.Cost, ",")

		if current.XId == gx && current.YId == gy {
			log.Print("find goal")
			goal.Parent = current.Parent
			goal.Cost = current.Cost
			endTime := time.Now()
			elaps := endTime.Sub(startTime)
			log.Printf("takes %f seconds, count is %d", elaps.Seconds(), count)
			return m.finalPath(goal, closeSetT)
		}

		delete(openSet, minKey)
		closeSet[nodeIndex(current)] = current
		closeSetT[nodeIndexT(current)] = current

		around := current.Around(&m, minTime)
		for _, an := range around {
			indT := nodeIndexT(an)
			ind := nodeIndex(an)

			if val, ok := closeSet[ind]; ok {
				if val.XId != an.XId || val.YId != an.YId {
					continue
				}
			}

			if _, ok := openSet[indT]; !ok {
				openSet[indT] = an
			}
		}
	}
}

func (m GridMap) finalPath(goal *Node, closeSet map[IndexT]*Node) (route [][3]int, oerr error) {
	route = append(route, [3]int{m.MaxT, goal.XId, goal.YId})

	parent := goal.Parent
	for parent != nil {
		n := closeSet[nodeIndexT(parent)]
		route = append(route, [3]int{n.T, n.XId, n.YId})
		parent = n.Parent
	}
	return route, nil
}

func (g GridMap) Route2Pos(minT float64, route [][3]int) [][3]float64 {
	l := len(route)
	fRoute := make([][3]float64, l)

	for i, r := range route {
		x, y := g.Ind2Pos(r[1], r[2])
		t := minT + float64(r[0]*i)
		p := [3]float64{t, x, y}
		fRoute[i] = p
	}
	return fRoute
}

type Node struct {
	T   int //time
	XId int
	YId int

	Cost float64

	Parent *Node
	// TW     TimeCostMap
}

func (s *Node) NewNode(t, x, y int, cost float64) *Node {
	n := new(Node)
	n.T = t
	n.XId = x
	n.YId = y
	n.Parent = s
	n.Cost = cost
	return n
}

func (n *Node) Around(g *GridMap, minTime int) []*Node {
	// time, x, y, cost
	motion := [9][4]float64{
		{1.0, 0.0, 0.0, 0.0}, //stay 要修正
		{0.0, 1.0, 0.0, 1.0},
		{0.0, 0.0, 1.0, 1.0},
		{0.0, -1.0, 0.0, 1.0},
		{0.0, 0.0, -1.0, 1.0},
		{0.0, -1.0, -1.0, math.Sqrt(2)},
		{0.0, -1.0, 1.0, math.Sqrt(2)},
		{0.0, 1.0, -1.0, math.Sqrt(2)},
		{0.0, 1.0, 1.0, math.Sqrt(2)},
	}
	var around []*Node
	for i, m := range motion {
		if i == 0 {
			continue
		}
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + int(m[0]) + 1
		if aT >= g.MaxT {
			continue
		}

		//map外のノードは外す
		if aX < 0 || aX >= g.Width {
			continue
		}
		if aY < 0 || aY >= g.Height {
			continue
		}

		//元から障害物で通れないところは消す
		if g.ObjectMap[aY][aX] {
			continue
		}

		mCost := g.TW[aT][aY][aX]
		//通れないコストマップは外す
		if mCost {
			continue
		}

		node := n.NewNode(aT, aX, aY, n.Cost+m[3]) //要修正
		around = append(around, node)
	}
	return around
}

func heuristic(n1, n2 *Node) float64 {
	w := 1.0
	d := w * math.Hypot(float64(n1.XId)-float64(n2.XId), float64(n1.YId)-float64(n2.YId))
	return d
}
