package polar

import (
	"errors"
	"math"
	"sort"
)

const (
	MaxWindRows = 26 // Wind boundaries: 0 up to 25 knots discrete rows
	MinTwaDeg   = 30 // Lower bound truncation: ignore dead-zone luffing space
	MaxTwaDeg   = 180
	TwaColumns  = (MaxTwaDeg - MinTwaDeg) + 1 // Exactly 151 discrete angular columns
	ScaleFactor = 100.0
	scale       = 1.0 / ScaleFactor
	Deg2Rad     = math.Pi / 180.0
)

// -------------------------------------------------------------------------
// 1. Core Structural Layouts
// -------------------------------------------------------------------------

// RawEntry defines a single point directly parsed from dynamic boat certificate files
type rawEntry struct {
	Angle float64
	Speed float64
}

// RawRow acts as an intake vessel grouping data per true wind speed row
type rawRow struct {
	WindSpeed int
	Entries   []rawEntry
	OptUp     rawEntry // Target VMG Upwind (Angle & VMG Speed)
	OptDn     rawEntry // Target VMG Downwind (Angle & VMG Speed)
}

// FastPolarTable is a high-density, cache-aligned, zero-allocation runtime lookup table.
// Total payload size is approx 8.3 KB. Sits natively on top of fast CPU registers.
type FastPolarTable struct {
	MaxWind int
	Grid    [MaxWindRows][TwaColumns]uint16 // [TWS_KNOTS][TWA - 30]

	// Target tracks calculated for immediate navigation dashboards
	OptUpAngle [MaxWindRows]uint16
	OptUpSpeed [MaxWindRows]uint16
	OptDnAngle [MaxWindRows]uint16
	OptDnSpeed [MaxWindRows]uint16
}

// -------------------------------------------------------------------------
// 2. Shape-Preserving Math Engine (PCHIP 1D)
// -------------------------------------------------------------------------

type pchipSegment struct {
	xi, a, b, c, d float64
}

type pchipInterpolator struct {
	segments   []pchipSegment
	xMin, xMax float64
}

// newPchipEngine fits a non-linear shape-preserving Hermite curve through coordinates.
// Guarantees monotonic behavior (speed will never decrease while accelerating down curves).
func newPchipEngine(x, y []float64) *pchipInterpolator {
	n := len(x)
	if n < 2 {
		panic("PCHIP requires a minimum of 2 coordinates")
	}

	h := make([]float64, n-1)
	m := make([]float64, n-1)
	for i := 0; i < n-1; i++ {
		h[i] = x[i+1] - x[i]
		m[i] = (y[i+1] - y[i]) / h[i]
	}

	d := make([]float64, n)
	for i := 1; i < n-1; i++ {
		if m[i-1]*m[i] <= 0 {
			d[i] = 0.0
		} else {
			w1 := 2*h[i] + h[i-1]
			w2 := h[i] + 2*h[i-1]
			d[i] = (w1 + w2) / (w1/m[i-1] + w2/m[i])
		}
	}
	d[0] = m[0]
	d[n-1] = m[n-2]

	segments := make([]pchipSegment, n-1)
	for i := 0; i < n-1; i++ {
		segments[i] = pchipSegment{
			xi: x[i],
			a:  y[i],
			b:  d[i],
			c:  (3*m[i] - 2*d[i] - d[i+1]) / h[i],
			d:  (d[i] + d[i+1] - 2*m[i]) / (h[i] * h[i]),
		}
	}

	return &pchipInterpolator{segments: segments, xMin: x[0], xMax: x[n-1]}
}

func (p *pchipInterpolator) evaluate(targetX float64) float64 {
	if targetX <= p.xMin {
		return p.segments[0].a
	}
	if targetX >= p.xMax {
		return p.segments[len(p.segments)-1].a + p.segments[len(p.segments)-1].b*(p.xMax-p.segments[len(p.segments)-1].xi)
	}

	idx := 0
	for i, seg := range p.segments {
		if targetX >= seg.xi {
			idx = i
		} else {
			break
		}
	}
	dx := targetX - p.segments[idx].xi
	return p.segments[idx].a + p.segments[idx].b*dx + p.segments[idx].c*dx*dx + p.segments[idx].d*dx*dx*dx
}

// -------------------------------------------------------------------------
// 3. Matrix Builder & Normalization (Ingestion Pipeline)
// -------------------------------------------------------------------------

// IngestAndBake converts arbitrary certificate rows into a static, high-frequency table.
func ingestAndBake(rawRows []rawRow) (*FastPolarTable, error) {
	if len(rawRows) == 0 {
		return nil, errors.New("empty dataset supplied for baking procedure")
	}

	// 1. Force strict sorting of raw rows by wind speed first
	sort.Slice(rawRows, func(i, j int) bool {
		return rawRows[i].WindSpeed < rawRows[j].WindSpeed
	})

	minWind := rawRows[0].WindSpeed
	maxWind := rawRows[len(rawRows)-1].WindSpeed
	if maxWind >= MaxWindRows {
		maxWind = MaxWindRows - 1
	}

	// 2. Pre-process arrays inside the ingestion pipeline
	twsValues := make([]float64, len(rawRows))
	twaModelsPerRow := make([]*pchipInterpolator, len(rawRows))
	upAngles := make([]float64, len(rawRows))
	dnAngles := make([]float64, len(rawRows))

	for i, row := range rawRows {
		twsValues[i] = float64(row.WindSpeed)
		upAngles[i] = row.OptUp.Angle
		dnAngles[i] = row.OptDn.Angle

		// Collect and safeguard angular data nodes
		var xs []float64
		var ys []float64

		// Guarantee core structural boundaries exist even in sparse mock tests
		hasZero := false
		has180 := false
		for _, ent := range row.Entries {
			if ent.Angle == 0 {
				hasZero = true
			}
			if ent.Angle == 180 {
				has180 = true
			}
			xs = append(xs, ent.Angle)
			ys = append(ys, ent.Speed)
		}
		if !hasZero {
			xs = append(xs, 0.0)
			ys = append(ys, 0.0)
		}
		if !has180 && len(row.Entries) > 0 {
			xs = append(xs, 180.0)
			// Fallback to the last available speed if 180 is missing
			ys = append(ys, row.Entries[len(row.Entries)-1].Speed*0.8)
		}

		// Enforce absolute monotonic sorting across the raw angular axis
		type pair struct{ x, y float64 }
		pairs := make([]pair, len(xs))
		for k := range xs {
			pairs[k] = pair{xs[k], ys[k]}
		}
		sort.Slice(pairs, func(i, j int) bool { return pairs[i].x < pairs[j].x })

		// Deduplicate points that are mathematically identical inside the same row
		var cleanXs []float64
		var cleanYs []float64
		for k, p := range pairs {
			if k > 0 && math.Abs(p.x-pairs[k-1].x) < 0.0001 {
				continue
			}
			cleanXs = append(cleanXs, p.x)
			cleanYs = append(cleanYs, p.y)
		}

		// If a row still lacks points, pad it to keep PCHIP engine alive
		if len(cleanXs) < 2 {
			cleanXs = append(cleanXs, 0.0, 180.0)
			cleanYs = append(cleanYs, 0.0, 0.0)
		}

		twaModelsPerRow[i] = newPchipEngine(cleanXs, cleanYs)
	}

	// Dynamic fallback for the TWS axis if test payload has too few wind columns
	var upAngleModel, dnAngleModel, twsCrossModelFactory func([]float64) *pchipInterpolator

	if len(rawRows) < 2 {
		// Single row fallback scenario
		upAngleModel = func(x []float64) *pchipInterpolator {
			return &pchipInterpolator{xMin: 0, xMax: 25, segments: []pchipSegment{{xi: 0, a: upAngles[0]}}}
		}
		dnAngleModel = func(x []float64) *pchipInterpolator {
			return &pchipInterpolator{xMin: 0, xMax: 25, segments: []pchipSegment{{xi: 0, a: dnAngles[0]}}}
		}
		twsCrossModelFactory = func(speeds []float64) *pchipInterpolator {
			return &pchipInterpolator{xMin: 0, xMax: 25, segments: []pchipSegment{{xi: 0, a: speeds[0]}}}
		}
	} else {
		// Standard production execution tracking multiple wind speeds
		upM := newPchipEngine(twsValues, upAngles)
		dnM := newPchipEngine(twsValues, dnAngles)
		upAngleModel = func(x []float64) *pchipInterpolator { return upM }
		dnAngleModel = func(x []float64) *pchipInterpolator { return dnM }
		twsCrossModelFactory = func(speeds []float64) *pchipInterpolator {
			return newPchipEngine(twsValues, speeds)
		}
	}

	table := &FastPolarTable{MaxWind: maxWind}

	// 3. Bake continuous physics grids down into the fixed static arrays
	for angle := MinTwaDeg; angle <= MaxTwaDeg; angle++ {
		tempSpeedsAtAngle := make([]float64, len(rawRows))
		for i := range rawRows {
			tempSpeedsAtAngle[i] = twaModelsPerRow[i].evaluate(float64(angle))
		}

		twsCrossModel := twsCrossModelFactory(tempSpeedsAtAngle)

		for w := 0; w <= maxWind; w++ {
			var bakedSpeed float64
			if w < minWind {
				if minWind > 0 {
					factor := float64(w) / float64(minWind)
					bakedSpeed = twsCrossModel.evaluate(float64(minWind)) * factor
				} else {
					bakedSpeed = 0
				}
			} else {
				bakedSpeed = twsCrossModel.evaluate(float64(w))
			}

			if bakedSpeed < 0 {
				bakedSpeed = 0
			}

			shiftedCol := angle - MinTwaDeg
			table.Grid[w][shiftedCol] = uint16(math.Round(bakedSpeed * ScaleFactor))
		}
	}

	// 4. Finalize target overlays
	for w := 0; w <= maxWind; w++ {
		var targetUpA, targetDnA float64
		if w < minWind {
			targetUpA = upAngleModel(twsValues).evaluate(float64(minWind))
			targetDnA = dnAngleModel(twsValues).evaluate(float64(minWind))
		} else {
			targetUpA = upAngleModel(twsValues).evaluate(float64(w))
			targetDnA = dnAngleModel(twsValues).evaluate(float64(w))
		}

		table.OptUpAngle[w] = uint16(math.Round(targetUpA * ScaleFactor))
		table.OptDnAngle[w] = uint16(math.Round(targetDnA * ScaleFactor))

		upAInt := int(math.Round(targetUpA))
		if upAInt < MinTwaDeg {
			upAInt = MinTwaDeg
		}
		table.OptUpSpeed[w] = table.Grid[w][upAInt-MinTwaDeg]

		dnAInt := int(math.Round(targetDnA))
		if dnAInt > MaxTwaDeg {
			dnAInt = MaxTwaDeg
		}
		table.OptDnSpeed[w] = table.Grid[w][dnAInt-MinTwaDeg]
	}

	return table, nil
}

// GetTargets resolves live target performance overlays for helm track displays
func (d *FastPolarTable) GetTargets(tws float64) (upAngle, upSpeed, dnAngle, dnSpeed float64) {
	if tws < 0 {
		tws = 0
	}
	if tws > float64(d.MaxWind) {
		tws = float64(d.MaxWind)
	}

	wMin := int(math.Floor(tws))
	wMax := int(math.Ceil(tws))
	frac := tws - float64(wMin)

	interp := func(a, b uint16) float64 {
		return (float64(a)*scale)*(1.0-frac) + (float64(b)*scale)*frac
	}

	return interp(d.OptUpAngle[wMin], d.OptUpAngle[wMax]),
		interp(d.OptUpSpeed[wMin], d.OptUpSpeed[wMax]),
		interp(d.OptDnAngle[wMin], d.OptDnAngle[wMax]),
		interp(d.OptDnSpeed[wMin], d.OptDnSpeed[wMax])
}

// Speed returns the non-linear predicted target boat speed.
func (d *FastPolarTable) Speed(tws, twa float64) float64 {
	if twa < 0 {
		twa = math.Abs(twa)
	}
	if twa > 180 {
		twa = 360.0 - twa
	}
	if math.IsNaN(twa) || math.IsNaN(tws) {
		return 0.0
	}

	if twa < float64(MinTwaDeg) || tws <= 0 {
		return 0.0
	}

	if tws > float64(d.MaxWind) {
		tws = float64(d.MaxWind)
	}

	twsMin := int(math.Floor(tws))
	twsMax := int(math.Ceil(tws))
	twaMin := int(math.Floor(twa))
	twaMax := int(math.Ceil(twa))

	twsFrac := tws - float64(twsMin)
	twaFrac := twa - float64(twaMin)

	twaMinShifted := twaMin - MinTwaDeg
	twaMaxShifted := twaMax - MinTwaDeg

	c00 := float64(d.Grid[twsMin][twaMinShifted]) * scale
	c10 := float64(d.Grid[twsMax][twaMinShifted]) * scale
	c01 := float64(d.Grid[twsMin][twaMaxShifted]) * scale
	c11 := float64(d.Grid[twsMax][twaMaxShifted]) * scale

	r1 := c00*(1.0-twsFrac) + c10*twsFrac
	r2 := c01*(1.0-twsFrac) + c11*twsFrac

	return r1*(1.0-twaFrac) + r2*twaFrac
}
