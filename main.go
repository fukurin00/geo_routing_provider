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
	"flag"
	"fmt"
	"log"
	"sync"

	grid "github.com/fukurin00/geo_routing_provider/routing"

	synerex "github.com/fukurin00/provider_api"

	cav "github.com/synerex/proto_cav"
	sxmqtt "github.com/synerex/proto_mqtt"
	api "github.com/synerex/synerex_api"
	sxutil "github.com/synerex/synerex_sxutil"
	"google.golang.org/protobuf/proto"

	msg "github.com/fukurin00/geo_routing_provider/msg"
)

var (
	synerexConfig *synerex.SynerexConfig
	mapFile       string = "map/willow_garage_v_edited.pgm"
	yamlFile      string = "map/willow_garage_v_edited.yaml"

	mapMeta *grid.MapMeta
	gridMap *grid.GridMap
)

func routeCallback(clt *sxutil.SXServiceClient, sp *api.Supply) {
	if sp.SupplyName == "DestDemand" {
		rcd := &cav.DestinationRequest{}
		err := proto.Unmarshal(sp.Cdata.Entity, rcd)
		if err != nil {
			log.Print(err)
		}
		isx, isy := gridMap.Pos2Ind(float64(rcd.Current.X), float64(rcd.Current.Y))
		igx, igy := gridMap.Pos2Ind(float64(rcd.Destination.X), float64(rcd.Destination.Y))

		routei, err := gridMap.Plan(isx, isy, igx, igy)
		if err != nil {
			log.Print(err)
		}
		route := gridMap.Route2Pos(0, routei)

		jsonPayload, err := msg.MakePathMsg(route)
		if err != nil {
			log.Print(err)
		}
		topic := fmt.Sprintf("/robot/path/%d", rcd.RobotId)
		mqttProt := sxmqtt.MQTTRecord{
			Topic:  topic,
			Record: jsonPayload,
		}
		out, err := proto.Marshal(&mqttProt)
		if err != nil {
			log.Print(err)
		}
		_, err = synerexConfig.NotifySupply(out, synerex.MQTT_GATEWAY_SVC, "Path")
		if err != nil {
			log.Print(err)
		}

	} else {
		return
	}

}

func main() {
	wg := sync.WaitGroup{}
	flag.Parse()

	channels := []uint32{synerex.MQTT_GATEWAY_SVC, synerex.ROUTING_SERVICE}
	names := []string{"GEO_Routing_MQTT", "GEO_Routing_ROUTING"}
	synerexConfig, err := synerex.NewSynerexConfig("GeoRoutingNode", channels, names)
	if err != nil {
		log.Print(err)
	}
	synerexConfig.SubscribeSupply(synerex.ROUTING_SERVICE, routeCallback)

	mapMeta, err = grid.ReadMapImage(yamlFile, mapFile, 230)
	if err != nil {
		log.Print(err)
	}
	maxT := grid.MaxTimeLength
	gridMap = grid.NewGridMap(mapMeta.Reso, mapMeta.Origin, maxT, mapMeta.W, mapMeta.H, mapMeta.Data)

	wg.Add(1)
	wg.Wait()
	// sx, sy := 10, 10
	// gx, gy := 20, 20
	// route, err := g.Plan(sx, sy, gx, gy)
	// if err != nil {
	// 	log.Print(err)
	// }
	// r := g.Route2Pos(0.0, route)
	// log.Print(r)
}
