package polar

import (
	"math"
	"slices"

	"github.com/taruti/go-orcdata"
)

func OrcToPolar(b *orcdata.Data) (*PolarTable, error) {
	const deg2rad = math.Pi / 180.0
	const rad2deg = 1.0 / deg2rad
	a := b.Allowances
	p := PolarTable{
		Rows: make([]PolarRow, len(a.WindSpeeds)),
	}
	for i, tws := range a.WindSpeeds {
		bangle := a.BeatAngle[i]
		rangle := a.GybeAngle[i]
		up := PolarEntry{Angle: bangle, Speed: orcdata.AllowanceToSpeed(a.Beat[i]) / math.Cos(deg2rad*bangle)}
		dn := PolarEntry{Angle: rangle, Speed: orcdata.AllowanceToSpeed(a.Run[i]) / math.Cos(math.Pi-deg2rad*rangle)}
		p.Rows[i] = PolarRow{
			Wind:  int(tws),
			OptUp: up,
			OptDn: dn,
			Entries: []PolarEntry{
				PolarEntry{Angle: 0, Speed: 0},
				up,
				PolarEntry{Angle: 52, Speed: orcdata.AllowanceToSpeed(a.R52[i])},
				PolarEntry{Angle: 60, Speed: orcdata.AllowanceToSpeed(a.R60[i])},
				PolarEntry{Angle: 75, Speed: orcdata.AllowanceToSpeed(a.R75[i])},
				PolarEntry{Angle: 90, Speed: orcdata.AllowanceToSpeed(a.R90[i])},
				PolarEntry{Angle: 110, Speed: orcdata.AllowanceToSpeed(a.R110[i])},
				PolarEntry{Angle: 120, Speed: orcdata.AllowanceToSpeed(a.R120[i])},
				PolarEntry{Angle: 135, Speed: orcdata.AllowanceToSpeed(a.R135[i])},
				PolarEntry{Angle: 150, Speed: orcdata.AllowanceToSpeed(a.R150[i])},
				PolarEntry{Angle: 165, Speed: orcdata.AllowanceToSpeed(a.DW165[i]) / math.Cos(math.Pi-deg2rad*165)},
				PolarEntry{Angle: 180, Speed: orcdata.AllowanceToSpeed(a.DW180[i])},
				dn,
			},
		}
		slices.SortFunc(p.Rows[i].Entries, func(x PolarEntry, y PolarEntry) int {
			if x.Angle < y.Angle {
				return -1
			}
			if x.Angle > y.Angle {
				return 1
			}
			return 0
		})
	}
	return &p, nil
}
