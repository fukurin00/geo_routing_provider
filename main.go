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
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"strings"
	"sync"
	"time"

	"github.com/fukurin00/geo_routing_provider/msg"
	grid "github.com/fukurin00/geo_routing_provider/routing"

	cav "github.com/synerex/proto_cav"
	sxmqtt "github.com/synerex/proto_mqtt"
	api "github.com/synerex/synerex_api"
	sxutil "github.com/synerex/synerex_sxutil"
	"google.golang.org/protobuf/proto"

	// msg "github.com/fukurin00/geo_routing_provider/msg"
	ros "github.com/fukurin00/go_ros_msg"
	pbase "github.com/synerex/synerex_proto"
)

var (
	// synerexConfig *synerex.SynerexConfig
	mapFile  string = "map/willow_garage_v_edited.pgm"
	yamlFile string = "map/willow_garage_v_edited.yaml"

	mapMetaUpdate               = false
	mapMeta       *grid.MapMeta = nil
	gridMap       *grid.GridMap = nil

	mu              sync.Mutex
	nodesrv         = flag.String("nodesrv", "127.0.0.1:9990", "node serv address")
	sxServerAddress string

	mqttClient  *sxutil.SXServiceClient
	routeClient *sxutil.SXServiceClient
)

func routeCallback(clt *sxutil.SXServiceClient, sp *api.Supply) {
	rcd := &cav.DestinationRequest{}
	err := proto.Unmarshal(sp.Cdata.Entity, rcd)
	if err != nil {
		log.Print(err)
	}
	log.Printf("receive dest request robot%d", rcd.RobotId)
	if gridMap == nil {
		log.Print("not receive gridMap yet ...")
		return
	}
	isx, isy := gridMap.Pos2Ind(float64(rcd.Current.X), float64(rcd.Current.Y))
	igx, igy := gridMap.Pos2Ind(float64(rcd.Destination.X), float64(rcd.Destination.Y))

	routei, err := gridMap.Plan(isx, isy, igx, igy)
	if err != nil {
		log.Print(err)
	} else {
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
		cout := api.Content{Entity: out}
		smo := sxutil.SupplyOpts{
			Name:  "robotRoute",
			Cdata: &cout,
		}
		_, err = mqttClient.NotifySupply(&smo)
		if err != nil {
			log.Print(err)
		}
	}
}

func subsclibeRouteSupply(client *sxutil.SXServiceClient) {
	ctx := context.Background()
	for {
		client.SubscribeSupply(ctx, routeCallback)
		reconnectClient(client)
	}
}

func mqqtCallback(clt *sxutil.SXServiceClient, sp *api.Supply) {
	if sp.SenderId == uint64(clt.ClientID) {
		return
	}

	rcd := &sxmqtt.MQTTRecord{}
	err := proto.Unmarshal(sp.Cdata.Entity, rcd)
	if err != nil {
		log.Print(err)
	}
	if strings.HasPrefix(rcd.Topic, "/global_costmap") {
		if !mapMetaUpdate {
			var occupancy ros.OccupancyGrid
			merr := json.Unmarshal(rcd.Record, &occupancy)
			if merr != nil {
				log.Print(merr)
			} else {
				mapMeta = grid.LoadROSMap(occupancy, 50)
				maxT := grid.MaxTimeLength
				gridMap = grid.NewGridMap(mapMeta.Reso, mapMeta.Origin, maxT, mapMeta.W, mapMeta.H, mapMeta.Data)
				log.Print("global costmap updated")
				mapMetaUpdate = true
			}
		}
	}

}

func subsclibeMQQTSupply(client *sxutil.SXServiceClient) {
	ctx := context.Background()
	for {
		client.SubscribeSupply(ctx, mqqtCallback)
		reconnectClient(client)
	}
}

func reconnectClient(client *sxutil.SXServiceClient) {
	mu.Lock()
	if client.SXClient != nil {
		client.SXClient = nil
		log.Printf("Client reset \n")
	}
	mu.Unlock()
	time.Sleep(5 * time.Second) // wait 5 seconds to reconnect
	mu.Lock()
	if client.SXClient == nil {
		newClt := sxutil.GrpcConnectServer(sxServerAddress)
		if newClt != nil {
			// log.Printf("Reconnect server [%s]\n", s.SxServerAddress)
			client.SXClient = newClt
		}
	}
	mu.Unlock()
}

func main() {
	go sxutil.HandleSigInt()
	wg := sync.WaitGroup{}
	flag.Parse()
	sxutil.RegisterDeferFunction(sxutil.UnRegisterNode)

	channels := []uint32{pbase.MQTT_GATEWAY_SVC, pbase.ROUTING_SERVICE}
	srv, err := sxutil.RegisterNode(*nodesrv, "GeoRoutingProvider", channels, nil)
	if err != nil {
		log.Fatal("can not registar node")
	}
	log.Printf("connectiong server [%s]", srv)

	sxServerAddress = srv

	synerexClient := sxutil.GrpcConnectServer(srv)
	argJson1 := "{Client: GeoMQTT}"
	mqttClient = sxutil.NewSXServiceClient(synerexClient, pbase.MQTT_GATEWAY_SVC, argJson1)
	argJson2 := "{Client: GeoRoute}"
	routeClient = sxutil.NewSXServiceClient(synerexClient, pbase.ROUTING_SERVICE, argJson2)

	// mapMeta, err = grid.ReadStaticMapImage(yamlFile, mapFile, 230)
	// if err != nil {
	// 	log.Print(err)
	// }
	// maxT := grid.MaxTimeLength
	// gridMap = grid.NewGridMap(mapMeta.Reso, mapMeta.Origin, maxT, mapMeta.W, mapMeta.H, mapMeta.Data)

	log.Print("start subscribing")
	go subsclibeRouteSupply(routeClient)
	go subsclibeMQQTSupply(mqttClient)
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
