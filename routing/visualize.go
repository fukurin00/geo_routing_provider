package routing

func Convert2DPoint(obj [][2]float64) (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for _, o := range obj {
		xs = append(xs, o[0])
		ys = append(ys, o[1])
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}

func Convert3DPoint(obj [][3]float64) (obj3d [][]float64) {
	var xs []float64
	var ys []float64
	var ts []float64
	for _, o := range obj {
		xs = append(xs, o[1])
		ys = append(ys, o[2])
		ts = append(ts, o[0])
	}
	obj3d = append(obj3d, xs)
	obj3d = append(obj3d, ys)
	obj3d = append(obj3d, ts)
	return obj3d
}

func (m GridMap) ConvertObjMap2Point() (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for j := 0; j < m.Height; j++ {
		for i := 0; i < m.Width; i++ {
			if m.ObjectMap[j][i] {
				xs = append(xs, m.Origin.X+m.Resolution*float64(i))
				ys = append(ys, m.Origin.Y+m.Resolution*float64(j))
			}
		}
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}
