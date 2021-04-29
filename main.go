/*
	briaf:
		routing multiple robots using time scale consideration

	subscribe:
		position
		routeRequest

	publish:
		path
*/

package main

import (
	"log"
	"math/rand"

	grid "github.com/fukurin00/geo_routing_provider/routing"
)

func main() {
	reso := 0.05
	w := 1216
	h := 1120
	// w := 100
	// h := 100
	origin := grid.Point{X: -16.2, Y: -38.6}
	maxT := grid.MaxTimeLength

	data := make([]int8, w*h)
	for i := 0; i < w; i++ {
		for j := 0; j < h; j++ {
			// r := 0
			// if i == 0 || i == w-1 || j == 0 || j == h-1 {
			// 	r = 100
			// } else {
			// 	if i%25 == 0 {
			// 		if j%2 == 0 {
			// 			r = 100
			// 		}
			// 	}
			// 	if j%4 == 1 {
			// 		if i%3 == 2 {
			// 			r = 95
			// 		}
			// 	}
			// }
			r := rand.Intn(100)
			if r > 80 {
				r = 100
			} else {
				r = 0
			}
			data[i+j*w] = int8(r)
		}
	}

	sx, sy := 3, 1
	gx, gy := 98, 558
	// for i, r := range data {
	// 	if i%w == sx && i/w == sy {
	// 		fmt.Print("S")
	// 	} else if i%w == gx && i/w == gy {
	// 		fmt.Print("G")
	// 	} else {
	// 		if r > grid.CloseThreth {
	// 			fmt.Print("*")
	// 		} else {
	// 			fmt.Print(".")
	// 		}
	// 	}
	// 	if i%w == w-1 {
	// 		fmt.Println()
	// 	}
	// }

	g := grid.NewGridMap(reso, origin, maxT, w, h, data)

	// for i := 0; i < g.Height; i++ {
	// 	for j := 0; j < g.Width; j++ {
	// 		if g.ObjectMap[i][j] {
	// 			fmt.Print("*")
	// 		} else {
	// 			fmt.Print(".")
	// 		}
	// 		if j == g.Width-1 {
	// 			fmt.Println()
	// 		}
	// 	}
	// }

	route, err := g.Plan(sx, sy, gx, gy)
	if err != nil {
		log.Print(err)
	}
	log.Print(route)
}
