package routing

import (
	"errors"
	"log"
	"math"
)

var (
	OpenThreth    uint8 = 10   //これより小さいと通れない
	MaxTimeLength       = 1000 //これ以上のtを計算しない
)

type Point struct {
	X float64
	Y float64
}

type Index struct {
	X int
	Y int
	I int
	T int
}

func (m GridMap) nodeIndex(n *Node) Index {
	i := Index{
		X: n.XId,
		Y: n.YId,
		I: n.XId + m.Width*n.YId,
		T: n.T,
	}
	return i
}

func (g GridMap) NewIndex(t, x, y int) *Index {
	n := new(Index)
	n.T = t
	n.X = x
	n.Y = y
	n.I = y*g.Width + x
	return n
}

// type CostMap map[Index]uint8
type TimeCostMap []CostMap
type CostMap [][]uint8

type GridMap struct {
	Resolution float64
	Origin     Point
	Width      int
	Height     int

	TW   TimeCostMap
	MaxT int
}

func (m GridMap) Plan(sx, sy, gx, gy int, vel float64) (route [][2]int, oerr error) {
	// if m.TW[0][sx][sy] > OpenThreth{
	// 	oerr = errors.New("Path planning error: start point i")
	// }
	start := &Node{T: 0, XId: sx, YId: sy, Cost: 0, Parent: nil, TW: m.TW}
	goal := &Node{T: 0, XId: gx, YId: gy, Cost: 0, Parent: nil, TW: m.TW}

	openSet := make(map[Index]*Node)

	closeSet := make(map[Index]*Node)

	openSet[m.nodeIndex(start)] = start

	for {
		if len(openSet) == 0 {
			oerr = errors.New("Path planning error: gopen set is empty")
			return nil, oerr
		}

		minCost := 99999999.9
		var minKey Index
		for key, val := range openSet {
			calCost := val.Cost + heuristic(goal, val)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
		}
		current := openSet[minKey]

		if current.XId == gx && current.YId == gy {
			log.Print("find goal")
			goal.Parent = current.Parent
			goal.Cost = current.Cost
			return m.finalPath(goal, closeSet)
		}

		delete(openSet, minKey)
		closeSet[minKey] = current

		around := current.Around(&m)
		for _, an := range around {
			ind := m.nodeIndex(an)
			if _, ok := closeSet[ind]; ok {
				continue
			}

			if _, ok := openSet[ind]; !ok {
				openSet[ind] = an
			}
		}
	}
}

func (m GridMap) finalPath(goal *Node, closeSet map[Index]*Node) (route [][2]int, oerr error) {
	route = append(route, [2]int{goal.XId, goal.YId})

	parent := goal.Parent
	for parent != nil {
		n := closeSet[m.nodeIndex(parent)]
		route = append(route, [2]int{n.XId, n.YId})
		parent = n.Parent
	}
	return route, nil
}

type Node struct {
	T   int //time
	XId int
	YId int

	Cost float64

	Parent *Node
	TW     TimeCostMap
}

func (s *Node) NewNode(t, x, y int, cost float64) *Node {
	n := new(Node)
	n.T = t
	n.XId = x
	n.YId = y
	n.Parent = s
	n.Cost = cost
	n.TW = s.TW
	return n
}

func (n *Node) Around(g *GridMap) []*Node {
	// time, x, y, cost
	motion := [9][4]float64{
		{1.0, 0.0, 0.0, 2}, //stay
		{0, 1.0, 0, 1.0},
		{0, 0, 1.0, 1.0},
		{0, -1.0, 0, 1.0},
		{0, -1.0, 1.0},
		{0, -1.0, -1.0, math.Sqrt(2)},
		{0, -1.0, 1.0, math.Sqrt(2)},
		{0, 1.0, -1.0, math.Sqrt(2)},
		{0, 1.0, 1.0, math.Sqrt(2)},
	}
	var around []*Node
	for _, m := range motion {
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + int(m[0]) + 1
		if aT > g.MaxT {
			continue
		}

		//map外のノードは外す
		if aX < 0 || aX > g.Width {
			continue
		}
		if aY < 0 || aY > g.Height {
			continue
		}

		mCost := n.TW[aT][aX][aY]
		//通れないコストマップは外す
		if mCost > OpenThreth {
			continue
		}

		node := n.NewNode(aT, aX, aY, n.Cost+m[2]+float64(mCost)/2) //要修正
		around = append(around, node)
	}
	return around
}

func heuristic(n1, n2 *Node) float64 {
	w := 1.0
	d := w * math.Hypot(float64(n1.XId)-float64(n2.XId), float64(n1.YId)-float64(n2.YId))
	return d
}
