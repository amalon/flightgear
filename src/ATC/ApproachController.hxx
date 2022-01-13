// Extracted from trafficcontrol.hxx - classes to manage AIModels based air traffic control
// Written by Durk Talsma, started September 2006.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id$

#ifndef APPROACH_CONTROLLER_HXX
#define APPROACH_CONTROLLER_HXX

#include <Airports/airports_fwd.hxx>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Shape>

#include <simgear/compiler.h>
// There is probably a better include than sg_geodesy to get the SG_NM_TO_METER...
#include <simgear/math/sg_geodesy.hxx>
#include <simgear/debug/logstream.hxx>
#include <simgear/structure/SGReferenced.hxx>
#include <simgear/structure/SGSharedPtr.hxx>

#include <ATC/ATCController.hxx>
#include <ATC/trafficcontrol.hxx>

/******************************************************************************
 * class FGTowerControl
 *****************************************************************************/
class FGApproachController : public FGATCController
{
private:
    TrafficVector activeTraffic;
    ActiveRunwayVec activeRunways;
    FGAirportDynamics *parent;

public:
    FGApproachController(FGAirportDynamics * parent);
    virtual ~FGApproachController();
    
    virtual void announcePosition(int id, FGAIFlightPlan *intendedRoute, int currentRoute,
                                  double lat, double lon,
                                  double hdg, double spd, double alt, double radius, int leg,
                                  FGAIAircraft *aircraft);
    virtual void             signOff(int id);
    virtual void             updateAircraftInformation(int id, double lat, double lon,
            double heading, double speed, double alt, double dt);
    virtual bool             hasInstruction(int id);
    virtual FGATCInstruction getInstruction(int id);

    virtual void render(bool);
    virtual std::string getName();
    virtual void update(double dt);

    ActiveRunway* getRunway(const std::string& name);

    bool hasActiveTraffic() {
        return ! activeTraffic.empty();
    };
    TrafficVector &getActiveTraffic() {
        return activeTraffic;
    };
};

#endif
