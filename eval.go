package polar

import "github.com/cnkei/gospline"

type Eval struct {
	*PolarTable
	Splines    []gospline.Spline
	windSpeeds []int
}

func (p *PolarTable) Eval() *Eval {
	sps := make([]gospline.Spline, len(p.Rows))
	wss := make([]int, len(p.Rows))
	for i, row := range p.Rows {
		var xs []float64
		var ys []float64
		for _, e := range row.Entries {
			xs = append(xs, e.Angle)
			ys = append(ys, e.Speed)
		}
		sps[i] = gospline.NewCubicSpline(xs, ys)
		wss[i] = row.Wind
	}
	return &Eval{
		PolarTable: p,
		Splines:    sps,
		windSpeeds: wss,
	}
}

func (e *Eval) SogAtIndex(idx int, twa float64) float64 {
	return e.Splines[idx].At(twa)
}

func (e *Eval) SogAtWindSpeed(windSpeed float64, twa float64) float64 {
	return linearInter(windSpeed, e.windSpeeds, func(idx int) float64 { return e.SogAtIndex(idx, twa) })
}

func (e *Eval) BeatAngle(windSpeed float64) float64 {
	return linearInter(windSpeed, e.windSpeeds, func(idx int) float64 { return e.PolarTable.Rows[idx].OptUp.Angle })
}

func (e *Eval) GybeAngle(windSpeed float64) float64 {
	return linearInter(windSpeed, e.windSpeeds, func(idx int) float64 { return e.PolarTable.Rows[idx].OptDn.Angle })
}

func linearInter(x float64, table []int, f func(int) float64) float64 {
	for i := 0; i+1 < len(table); i++ {
		if x <= float64(table[i+1]) {
			// y=((y2-y1)/(x2-x1))(x-x1)+y1
			y1 := f(i)
			y2 := f(i + 1)
			m := (y2 - y1) / float64(table[i+1]-table[i])
			y := m*(x-float64(table[i])) + y1
			return y
		}
	}
	i := len(table) - 2
	y1 := f(i)
	y2 := f(i + 1)
	m := (y2 - y1) / float64(table[i+1]-table[i])
	y := m*(x-float64(table[i])) + y1
	return y

}
