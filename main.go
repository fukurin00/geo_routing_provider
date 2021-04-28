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
	origin := grid.Point{X: -16.2, Y: -38.6}
	maxT := grid.MaxTimeLength

	var data []int8
	for i := 0; i < w; i++ {
		for j := 0; j < h; j++ {
			r := rand.Intn(100)
			if r < 20 {
				r = 100
			} else {
				r = 0
			}
			data = append(data, int8(r))
		}
	}

	g := grid.NewGridMap(reso, origin, maxT, w, h, data)
	route, err := g.Plan(100, 100, 122, 111)
	if err != nil {
		log.Print(err)
	}
	log.Print(route)
}
