package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"sync"
	"time"

	"github.com/fukurin00/geo_routing_provider/msg"
	grid "github.com/fukurin00/geo_routing_provider/routing"
	"github.com/fukurin00/glot"

	cav "github.com/synerex/proto_cav"
	sxmqtt "github.com/synerex/proto_mqtt"
	api "github.com/synerex/synerex_api"
	sxutil "github.com/synerex/synerex_sxutil"
	"google.golang.org/protobuf/proto"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	astar "github.com/fukurin00/astar_golang"
	ros "github.com/fukurin00/go_ros_msg"
	pbase "github.com/synerex/synerex_proto"
)

const (
	robotRadius      float64 = 0.25
	closeThresh      float64 = 0.85
	robotVelocity    float64 = 1.0 // [m/sec]
	robotRotVelocity float64 = 1.0 // [rad/sec]

	mapFile  string = "map/willow_garage_v_edited2.pgm"
	yamlFile string = "map/willow_garage_v_edited2.yaml"
)

var (
	mode Mode = ASTAR3DHEXA

	resolution      = flag.Float64("reso", 0.4, "path planning resolution")
	vizroute        = flag.Bool("visualize", true, "whether visualize route")
	mqttsrv         = flag.String("mqtt", "localhost", "MQTT Broker address")
	nodesrv         = flag.String("nodesrv", "127.0.0.1:9990", "node serv address")
	sxServerAddress string

	mapMetaUpdate                   = false
	mapMeta       *grid.MapMeta     = nil
	gridMap       *grid.GridMap     = nil
	astarPlanner  *astar.Astar      //if 2d mode
	timeCostMap   grid.TimeRobotMap = nil

	mu sync.Mutex

	//synerex client
	mqttClient  *sxutil.SXServiceClient
	routeClient *sxutil.SXServiceClient

	msgCh    chan mqtt.Message
	vizCh    chan vizOpt
	pathUpCh chan [][3]int

	plot2d *glot.Plot
	plot3d *glot.Plot

	timeStep int //計算に使う1stepの秒数
	reso     float64
)

func init() {
	msgCh = make(chan mqtt.Message)
	vizCh = make(chan vizOpt)
	pathUpCh = make(chan [][3]int)

	flag.Parse()
	reso = *resolution
	//timeStep = reso/robotVelocity + 2*math.Pi/3/robotRotVelocity // L/v + 2pi/3w  120度回転したときの一番かかる時間
	timeStep = 3 * int(math.Ceil(reso/robotVelocity)) //切り上げ整数
}

type vizOpt struct {
	id    int
	route [][3]float64
}

type Mode int

const (
	ASTAR2D     Mode = iota //normal astar
	ASTAR3D                 //original astar
	ASTAR3DHEXA             //original hexa astar
)

func (m Mode) String() string {
	s := [3]string{"Astar2D", "Astar3D", "HexaAstar3d"}
	return s[m]
}

func routing(rcd *cav.DestinationRequest) {
	var jsonPayload []byte
	if mode == ASTAR3DHEXA {
		if gridMap == nil {
			log.Print("not receive gridMap yet ...")
			return
		}
		isa, isb := gridMap.Pos2IndHexa(float64(rcd.Current.X), float64(rcd.Current.Y))
		iga, igb := gridMap.Pos2IndHexa(float64(rcd.Destination.X), float64(rcd.Destination.Y))

		routei, err := gridMap.PlanHexa(int(rcd.RobotId), isa, isb, iga, igb, robotVelocity, robotRotVelocity, float64(timeStep), append(timeCostMap[:0:0], timeCostMap...))
		if err != nil {
			log.Print(err)
		} else {
			pathUpCh <- routei
			route := gridMap.Route2PosHexa(float64(rcd.Ts.Seconds), float64(timeStep), routei)
			if *vizroute {
				vOpt := vizOpt{id: int(rcd.RobotId), route: route}
				vizCh <- vOpt
			}
			jsonPayload, err = msg.MakePathMsg(route)
			if err != nil {
				log.Print(err)
			}
			sendPath(jsonPayload, int(rcd.RobotId))
		}

	} else if mode == ASTAR3D {
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
			if *vizroute {
				vOpt := vizOpt{id: int(rcd.RobotId), route: route}
				vizCh <- vOpt
			}
			jsonPayload, err = msg.MakePathMsg(route)
			if err != nil {
				log.Print(err)
			}
			sendPath(jsonPayload, int(rcd.RobotId))
		}
	} else if mode == ASTAR2D {
		route, err := astarPlanner.Plan(float64(rcd.Current.X), float64(rcd.Current.Y), float64(rcd.Destination.X), float64(rcd.Destination.Y))
		if err != nil {
			log.Print(err)
		} else {
			jsonPayload, err = msg.MakePathMsg2D(route)
			if err != nil {
				log.Print(err)
			}
			sendPath(jsonPayload, int(rcd.RobotId))
		}
	}

}

func sendPath(jsonPayload []byte, id int) {
	topic := fmt.Sprintf("robot/path/%d", id)
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
	} else {
		log.Printf("send path robot %d", id)
	}
}

func routeCallback(clt *sxutil.SXServiceClient, sp *api.Supply) {
	rcd := &cav.DestinationRequest{}
	err := proto.Unmarshal(sp.Cdata.Entity, rcd)
	if err != nil {
		log.Print(err)
	}
	log.Printf("receive dest request robot%d", rcd.RobotId)
	go routing(rcd)

}

func vizualizeHandler() {
	counter := make(map[int]int)
	for {
		opt := <-vizCh
		if val, ok := counter[opt.id]; ok {
			counter[opt.id] = val + 1
		} else {
			counter[opt.id] = 1
		}
		plot2d.AddPointGroup(fmt.Sprintf("route%d_%d", opt.id, counter[opt.id]), "points", grid.Convert32DPoint(opt.route))
		plot3d.AddPointGroup(fmt.Sprintf("route%d_%d", opt.id, counter[opt.id]), "points", grid.Convert3DPoint(opt.route))

		// if counter[opt.id] >= 2 {
		// 	plot2d.RemovePointGroup(fmt.Sprintf("route%d_%d", opt.id, counter[opt.id]-1))
		// 	plot3d.RemovePointGroup(fmt.Sprintf("route%d_%d", opt.id, counter[opt.id]-1))
		// }

		plot2d.SavePlot(fmt.Sprintf("route/robot%d_%d_route2D.png", opt.id, counter[opt.id]))
		plot3d.SavePlot(fmt.Sprintf("route/robot%d_%d_route3D.png", opt.id, counter[opt.id]))
	}
}

func subsclibeRouteSupply(client *sxutil.SXServiceClient) {
	ctx := context.Background()
	for {
		client.SubscribeSupply(ctx, routeCallback)
		reconnectClient(client)
	}
}

//synerex recconect to client
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

func handleMqttMessage() {
	for {
		msg := <-msgCh
		if !mapMetaUpdate {
			log.Print("updating global costmap..")
			mu.Lock()
			var occupancy ros.OccupancyGrid
			merr := json.Unmarshal(msg.Payload(), &occupancy)
			if merr != nil {
				log.Print(merr)
			} else {
				mapMeta = grid.LoadROSMap(occupancy, 50)
				maxT := grid.MaxTimeLength
				gridMap = grid.NewGridMap(*mapMeta, maxT, robotRadius)
				log.Print("global costmap updated")
				plot2d.AddPointGroup("costmap", "dots", gridMap.ConvertObjMap2Point())
				mapMetaUpdate = true
				plot2d.SavePlot("map/global_costmap.png")
			}
			mu.Unlock()
		}

	}
}

// listening MQTT topics.
func listenMQTTBroker() {
	var myHandler mqtt.MessageHandler = func(client mqtt.Client, msg mqtt.Message) {
		msgCh <- msg
	}
	opts := mqtt.NewClientOptions()
	opts.AddBroker("tcp://" + *mqttsrv + ":1883") // currently only 1883 port.

	clt := mqtt.NewClient(opts)

	if token := clt.Connect(); token.Wait() && token.Error() != nil {
		log.Fatalf("MQTT connection error: %s", token.Error())
	}

	if subscribeToken := clt.Subscribe("map/global_costmap", 0, myHandler); subscribeToken.Wait() && subscribeToken.Error() != nil {
		log.Fatalf("MQTT subscribe error: %s", subscribeToken.Error())
	}
}

func LoggingSettings(logFile string) {
	logfile, _ := os.OpenFile(logFile, os.O_RDWR|os.O_CREATE|os.O_APPEND, 0666)
	multiLogFile := io.MultiWriter(os.Stdout, logfile)
	log.SetFlags(log.Ldate | log.Ltime)
	log.SetOutput(multiLogFile)
}

func SetupSynerex() {
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
}

func SetupStaticMap() {
	mapMeta, err := grid.ReadStaticMapImage(yamlFile, mapFile, closeThresh)
	if err != nil {
		log.Print("read map file errore: ", err)
	}
	objMap := mapMeta.GetObjectMap()
	reso := *resolution
	if mode == ASTAR2D {
		plot2d.AddPointGroup("map", "dots", grid.Convert2DPoint(objMap))
		plot2d.SavePlot("map/raw_static_map.png")
		astarPlanner = astar.NewAstar(objMap, robotRadius, reso)
		log.Print("load astar obj map")
	} else if mode == ASTAR3D {
		maxT := grid.MaxTimeLength
		gridMap = grid.NewGridMapReso(*mapMeta, maxT, robotRadius, reso, objMap)
		plot2d.AddPointGroup("objmap", "dots", gridMap.ConvertObjMap2Point())
		plot2d.SavePlot("map/static_obj_map.png")
		plot3d.AddPointGroup("objmap", "dots", gridMap.ConvertObjMap3Point())
	} else if mode == ASTAR3DHEXA {
		maxT := grid.MaxTimeLength
		gridMap = grid.NewGridMapResoHexa(*mapMeta, maxT, robotRadius, reso, objMap)
		timeCostMap = append(gridMap.TW[:0:0], gridMap.TW...) // copy slice
		plot2d.AddPointGroup("objmap", "dots", gridMap.ConvertObjMap2PointHexa())
		plot2d.SavePlot("map/static_obj_map_hexa.png")
		// plot3d.AddPointGroup("objmap", "dots", gridMap.ConvertObjMap3PointHexa())
	}
}

func testPath() {
	isx, isy := gridMap.Pos2Ind(0, 0)
	igx, igy := gridMap.Pos2Ind(30, 5)

	routei, err := gridMap.Plan(isx, isy, igx, igy)
	if err != nil {
		log.Print(err)
	} else {
		route := gridMap.Route2Pos(0, routei)
		plot2d.AddPointGroup("route", "points", grid.Convert32DPoint(route))
		plot2d.SavePlot("route/test_route2D.png")
		plot3d.AddPointGroup("route", "points", grid.Convert3DPoint(route))
		//plot3d.SetZrange(0, routei[len(routei)-1][0])
		plot3d.SavePlot("route/test_route3D.png")
	}
}

func updateTimeObjMapHandler() {
	log.Printf("start updating costmap timestep is %d", timeStep)
	timer := time.NewTicker(time.Duration(timeStep))
	defer timer.Stop()
	for {
		select {
		case <-timer.C:
			gridMap.Update(timeCostMap)
		case route := <-pathUpCh:
			gridMap.UpdateTimeObjMapHexa(timeCostMap, route, robotRadius)
		}
	}
}

func main() {
	log.Printf("start geo-routing server mode:%s, timestep:%d, resolution:%f", mode.String(), timeStep, reso)
	go sxutil.HandleSigInt()
	wg := sync.WaitGroup{}
	sxutil.RegisterDeferFunction(sxutil.UnRegisterNode)

	//logging configuration
	now := time.Now()
	LoggingSettings("log/" + now.Format("2006-01-02-15") + ".log")

	// connect to mqtt broker
	// listenMQTTBroker()
	// go handleMqttMessage()

	// Synerex Configuration
	SetupSynerex()

	// visualization configuration
	plot2d, _ = glot.NewPlot(2, false, false)
	plot3d, _ = glot.NewPlot(3, false, false)

	// load static map data
	SetupStaticMap()

	// testPath()

	//start main function
	log.Print("start subscribing")

	go subsclibeRouteSupply(routeClient)
	//go updateTimeObjMapHandler()
	if *vizroute {
		go vizualizeHandler()
	}
	wg.Add(1)
	wg.Wait()
}
