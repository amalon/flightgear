include(FlightGearComponent)

set(SOURCES
	atc_mgr.cxx
	trafficcontrol.cxx
	CommStation.cxx
        ATISEncoder.cxx
        MetarPropertiesATISInformationProvider.cxx
        CurrentWeatherATISInformationProvider.cxx
        ATCController.cxx
        ApproachController.cxx
        GroundController.cxx
        StartupController.cxx
        TowerController.cxx
	)

set(HEADERS
	atc_mgr.hxx
	trafficcontrol.hxx
	CommStation.hxx
        ATISEncoder.hxx
        MetarPropertiesATISInformationProvider.hxx
        CurrentWeatherATISInformationProvider.hxx
        ATCController.hxx
        ApproachController.hxx
        GroundController.hxx
        StartupController.hxx
        TowerController.hxx
	)
    	
flightgear_component(ATC "${SOURCES}" "${HEADERS}")
