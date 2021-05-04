package routing

// func (g GridMap) Update(TW TimeRobotMap) {
// 	for i := 0; i < g.MaxT-1; i++ {
// 		TW[i+1] = TW[i]
// 	}
// 	TW[0] = g.ObjectMap
// }

// func (g GridMap) UpdateTimeObjMapHexa(TW TimeRobotMap, route [][3]int, robotRadius float64) {
// 	around := [6][2]int{{-1, 0}, {0, -1}, {1, -1}, {1, 0}, {0, 1}, {-1, 1}}
// 	aroundMore := [6][2]int{{2, -1}, {1, -2}, {-1, -1}, {-2, 1}, {2, -1}, {1, 1}}

// 	for i := 0; i < len(route); i++ {
// 		it := route[i][0]
// 		ix := route[i][1]
// 		iy := route[i][2]
// 		TW[it][iy][ix] = true
// 		if robotRadius < g.Resolution {
// 			continue
// 		} else if robotRadius <= 2*g.Resolution {
// 			for _, v := range around {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TW[it][ny][nx] = true
// 			}
// 		} else { //周囲18マス
// 			for _, v := range around {
// 				for d := 1; d <= 2; d++ {
// 					ny := iy + v[1]*d
// 					nx := ix + v[0]*d
// 					if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 						continue
// 					}
// 					TW[it][ny][nx] = true
// 				}
// 			}
// 			for _, v := range aroundMore {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TW[it][ny][nx] = true
// 			}
// 		}

// 	}
// }

// func (g GridMap) UpdateTimeRobotMap(route [][3]int, robotRadius float64, TRW TimeRobotMap) {
// 	around8 := [8][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, -1}, {-1, 1}, {1, 1}, {1, -1}}
// 	around4 := [4][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}}

// 	for i := 0; i < len(route); i++ {
// 		it := route[i][0]
// 		ix := route[i][1]
// 		iy := route[i][2]
// 		TRW[it][iy][ix] = true
// 		if robotRadius < g.Resolution {
// 			continue
// 		} else if robotRadius <= math.Sqrt(2)*g.Resolution {
// 			for _, v := range around4 {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TRW[it][ny][nx] = true
// 			}
// 		} else {
// 			for _, v := range around8 {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TRW[it][ny][nx] = true
// 			}
// 		}

// 	}
// }
