include(FlightGearComponent)

set(SOURCES
	controls.cxx
	replay.cxx
	flightrecorder.cxx
	FlightHistory.cxx
	initialstate.cxx
	AircraftPerformance.cxx
	replay-internal.cxx
	continuous.cxx
	)

set(HEADERS
	controls.hxx
	replay.hxx
	flightrecorder.hxx
	FlightHistory.hxx
	initialstate.hxx
	AircraftPerformance.hxx
	continuous.hxx    
	replay-internal.hxx    
	)


flightgear_component(Aircraft "${SOURCES}" "${HEADERS}")
