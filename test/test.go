package main

import (
	"log"

	grid "github.com/fukurin00/geo_routing_provider/routing"
)

func main() {
	mapFile := "../map/willow_garage_v_edited.pgm"
	yamlFile := "../map/willow_garage_v_edited.yaml"

	m, err := grid.ReadMapImage(yamlFile, mapFile, 100, int(grid.CloseThreth))
	log.Print(m.Reso, m.Origin, m.H, m.W, len(m.Data))
	log.Print(err)
}
