package routing

import (
	"math"
)

func (g *GridMap) Update() {
	for i := 0; i < g.MaxT-1; i++ {
		g.TW[i+1] = g.TW[i]
	}
	g.TW[0] = g.ObjectMap
}

func (g *GridMap) UpdateTimeObjMap(route [][3]int, robotRadius float64) {
	around8 := [8][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, -1}, {-1, 1}, {1, 1}, {1, -1}}
	around4 := [4][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}}

	for i := 0; i < len(route); i++ {
		it := route[i][0]
		ix := route[i][1]
		iy := route[i][2]
		g.TW[it][iy][ix] = true
		if robotRadius < g.Resolution {
			continue
		} else if robotRadius <= math.Sqrt(2)*g.Resolution {
			for _, v := range around4 {
				ny := iy + v[1]
				nx := ix + v[0]
				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
					continue
				}
				g.TW[it][ny][nx] = true
			}
		} else {
			for _, v := range around8 {
				ny := iy + v[1]
				nx := ix + v[0]
				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
					continue
				}
				g.TW[it][ny][nx] = true
			}
		}

	}
}
