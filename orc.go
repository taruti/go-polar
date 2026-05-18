package polar

import (
	"errors"
	"math"
	"sort"

	"github.com/taruti/go-orcdata"
)

// OrcToFastPolar parses raw ORC certificate data and directly bakes it
// into the optimized, zero-allocation 2D FastPolarTable layout using PCHIP physics.
func OrcToFastPolar(b *orcdata.Data) (*FastPolarTable, error) {
	if b == nil || len(b.Allowances.WindSpeeds) == 0 {
		return nil, errors.New("invalid or empty ORC data payload")
	}

	a := b.Allowances
	rawRows := make([]rawRow, len(a.WindSpeeds))

	// 1. Map ORC data arrays into our intermediate RawRow pipeline
	for i, tws := range a.WindSpeeds {
		bangle := a.BeatAngle[i]
		rangle := a.GybeAngle[i]

		upSTW := orcdata.AllowanceToSpeed(a.Beat[i]) / math.Cos(Deg2Rad*bangle)
		dnSTW := orcdata.AllowanceToSpeed(a.Run[i]) / math.Cos(math.Pi-(Deg2Rad*rangle))

		// Create the distinct target entries
		upEntry := rawEntry{Angle: bangle, Speed: upSTW}
		dnEntry := rawEntry{Angle: rangle, Speed: dnSTW}

		rawRows[i] = rawRow{
			WindSpeed: int(tws),
			OptUp:     upEntry,
			OptDn:     dnEntry,
			// Ingest both fixed columns AND optimal targets into the same coordinate sweep
			Entries: []rawEntry{
				{Angle: 0, Speed: 0.0}, // Dead stop head-to-wind boundary
				upEntry,                // Optimal Upwind point injected as a live physics node
				{Angle: 52, Speed: orcdata.AllowanceToSpeed(a.R52[i])},
				{Angle: 60, Speed: orcdata.AllowanceToSpeed(a.R60[i])},
				{Angle: 75, Speed: orcdata.AllowanceToSpeed(a.R75[i])},
				{Angle: 90, Speed: orcdata.AllowanceToSpeed(a.R90[i])},
				{Angle: 110, Speed: orcdata.AllowanceToSpeed(a.R110[i])},
				{Angle: 120, Speed: orcdata.AllowanceToSpeed(a.R120[i])},
				{Angle: 135, Speed: orcdata.AllowanceToSpeed(a.R135[i])},
				{Angle: 150, Speed: orcdata.AllowanceToSpeed(a.R150[i])},
				{Angle: 165, Speed: orcdata.AllowanceToSpeed(a.DW165[i])}, // FIXED: No cosine fix needed
				{Angle: 180, Speed: orcdata.AllowanceToSpeed(a.DW180[i])},
				dnEntry, // Optimal Downwind point injected as a live physics node
			},
		}
	}

	// 2. Enforce absolute sorting for wind speed keys to prevent out-of-order VPP issues
	sort.Slice(rawRows, func(i, j int) bool {
		return rawRows[i].WindSpeed < rawRows[j].WindSpeed
	})

	// 3. Hand over sorted intermediate rows directly to our pre-calculated baking algorithm
	return ingestAndBake(rawRows)
}
