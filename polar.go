package polar

type PolarTable struct {
	Rows []PolarRow
}

type PolarEntry struct {
	Angle, Speed float64
}
type PolarRow struct {
	Wind    int
	Entries []PolarEntry
	OptUp   PolarEntry
	OptDn   PolarEntry
}
