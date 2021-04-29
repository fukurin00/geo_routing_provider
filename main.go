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

	grid "github.com/fukurin00/geo_routing_provider/routing"
)

func main() {
	mapFile := "map/willow_garage_v_edited.pgm"
	yamlFile := "map/willow_garage_v_edited.yaml"

	m, err := grid.ReadMapImage(yamlFile, mapFile, 230)

	maxT := grid.MaxTimeLength

	g := grid.NewGridMap(m.Reso, m.Origin, maxT, m.W, m.H, m.Data)

	sx, sy := 10, 10
	gx, gy := 409, 867

	route, err := g.Plan(sx, sy, gx, gy)
	if err != nil {
		log.Print(err)
	}
	log.Print(route)
}
