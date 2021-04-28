package routing

import (
	"errors"
	"fmt"
	"log"
	"math"
)

var (
	CloseThreth   int8 = 90   //これより大きいと通れない
	MaxTimeLength      = 1000 //これ以上のtを計算しない
)

type Point struct {
	X float64
	Y float64
}

type Index struct {
	X int
	Y int
	T int
}

func (m GridMap) nodeIndex(n *Node) Index {
	i := Index{
		X: n.XId,
		Y: n.YId,
		T: n.T,
	}
	return i
}

func (g GridMap) NewIndex(t, x, y int) *Index {
	n := new(Index)
	n.T = t
	n.X = x
	n.Y = y
	return n
}

// type CostMap map[Index]uint8
type TimeCostMap []CostMap
type CostMap [][]int8

type GridMap struct {
	Resolution float64
	Origin     Point
	Width      int
	Height     int

	TW   TimeCostMap
	MaxT int
}

func NewGridMap(reso float64, origin Point, maxT, width, height int, data []int8) *GridMap {
	g := new(GridMap)
	g.Resolution = reso
	g.Origin = origin
	g.Width = width
	g.Height = height
	g.MaxT = maxT
	g.TW = []CostMap{}

	var line []int8
	var grid [][]int8
	for i, d := range data {
		line = append(line, d)
		if i%width == width-1 {
			grid = append(grid, line)
			line = []int8{}
		}
	}
	g.TW = append(g.TW, grid)

	for i := 1; i < maxT; i++ {
		g.TW = append(g.TW, grid)
	}
	return g
}

func (m GridMap) Plan(sx, sy, gx, gy int) (route [][3]int, oerr error) {
	// if m.TW[0][sx][sy] > OpenThreth{
	// 	oerr = errors.New("Path planning error: start point i")
	// }

	if m.TW[0][gy][gx] > CloseThreth {
		oerr = errors.New("path planning error: goal is not verified...")
		return nil, oerr
	}
	start := &Node{T: 0, XId: sx, YId: sy, Cost: 0, Parent: nil}
	goal := &Node{T: 0, XId: gx, YId: gy, Cost: 0, Parent: nil}

	openSet := make(map[Index]*Node)

	closeSet := make(map[Index]*Node)

	openSet[m.nodeIndex(start)] = start

	count := 0
	current := &Node{T: 0}

	for {
		count += 1
		if len(openSet) == 0 {
			log.Print(count)
			oerr = errors.New("path planning error: open set is empty...")
			return nil, oerr
		}

		minCost := 9999999999999999999.9
		var minKey Index
		for key, val := range openSet {
			calCost := val.Cost + heuristic(goal, val) + float64(val.T) - float64(current.T)
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
		}
		current = openSet[minKey]
		fmt.Print(current.T, current.XId, current.YId, current.Cost, ",")

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

func (m GridMap) finalPath(goal *Node, closeSet map[Index]*Node) (route [][3]int, oerr error) {
	route = append(route, [3]int{m.MaxT, goal.XId, goal.YId})

	parent := goal.Parent
	for parent != nil {
		n := closeSet[m.nodeIndex(parent)]
		route = append(route, [3]int{n.T, n.XId, n.YId})
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

func (n *Node) Around(g *GridMap) []*Node {
	// time, x, y, cost
	motion := [9][4]float64{
		{1.0, 0.0, 0.0, 2}, //stay 要修正
		{0, 1.0, 0, 1.0},
		{0, 0, 1.0, 1.0},
		{0, -1.0, 0, 1.0},
		{0, 0, -1.0, 1.0},
		{0, -1.0, -1.0, math.Sqrt(2)},
		{0, -1.0, 1.0, math.Sqrt(2)},
		{0, 1.0, -1.0, math.Sqrt(2)},
		{0, 1.0, 1.0, math.Sqrt(2)},
	}
	var around []*Node
	for i, m := range motion {
		if i == 0 {
			continue
		}
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + int(m[0])
		if aT >= g.MaxT {
			continue
		}

		//map外のノードは外す
		if aX < 0 || aX > g.Width {
			continue
		}
		if aY < 0 || aY > g.Height {
			continue
		}

		mCost := g.TW[aT+1][aY][aX]
		//通れないコストマップは外す
		if mCost > CloseThreth {
			continue
		}

		node := n.NewNode(aT+1, aX, aY, n.Cost+m[3]+float64(mCost)/10) //要修正
		around = append(around, node)
	}
	return around
}

func heuristic(n1, n2 *Node) float64 {
	w := 1.0
	d := w * math.Hypot(float64(n1.XId)-float64(n2.XId), float64(n1.YId)-float64(n2.YId))
	return d
}
