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

#ifndef ATC_CONTROLLER_HXX
#define ATC_CONTROLLER_HXX

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

#include <ATC/trafficcontrol.hxx>

/**
 * class FGATCController
 * NOTE: this class serves as an abstraction layer for all sorts of ATC controllers.
 *************************************************************************************/
class FGATCController
{
private:


protected:
    bool initialized;
    bool available;
    time_t lastTransmission;

    double dt_count;
    osg::Group* group;

    std::string formatATCFrequency3_2(int );
    std::string genTransponderCode(const std::string& fltRules);
    bool isUserAircraft(FGAIAircraft*);
    void clearTrafficControllers(TrafficVector& vec);
    TrafficVectorIterator searchActiveTraffic(TrafficVector& vec, int id);
    void eraseDeadTraffic(TrafficVector& vec);
public:
    typedef enum {
        MSG_ANNOUNCE_ENGINE_START,
        MSG_REQUEST_ENGINE_START,
        MSG_PERMIT_ENGINE_START,
        MSG_DENY_ENGINE_START,
        MSG_ACKNOWLEDGE_ENGINE_START,
        MSG_REQUEST_PUSHBACK_CLEARANCE,
        MSG_PERMIT_PUSHBACK_CLEARANCE,
        MSG_HOLD_PUSHBACK_CLEARANCE,
        MSG_ACKNOWLEDGE_SWITCH_GROUND_FREQUENCY,
        MSG_INITIATE_CONTACT,
        MSG_ACKNOWLEDGE_INITIATE_CONTACT,
        MSG_REQUEST_TAXI_CLEARANCE,
        MSG_ISSUE_TAXI_CLEARANCE,
        MSG_ACKNOWLEDGE_TAXI_CLEARANCE,
        MSG_HOLD_POSITION,
        MSG_ACKNOWLEDGE_HOLD_POSITION,
        MSG_RESUME_TAXI,
        MSG_ACKNOWLEDGE_RESUME_TAXI,
        MSG_REPORT_RUNWAY_HOLD_SHORT,
        MSG_ACKNOWLEDGE_REPORT_RUNWAY_HOLD_SHORT,
        MSG_SWITCH_TOWER_FREQUENCY,
        MSG_ACKNOWLEDGE_SWITCH_TOWER_FREQUENCY
    } AtcMsgId;

    typedef enum {
        ATC_AIR_TO_GROUND,
        ATC_GROUND_TO_AIR
    } AtcMsgDir;
    FGATCController();
    virtual ~FGATCController();
    void init();

    virtual void announcePosition(int id, FGAIFlightPlan *intendedRoute, int currentRoute,
                                  double lat, double lon,
                                  double hdg, double spd, double alt, double radius, int leg,
                                  FGAIAircraft *aircraft) = 0;
    virtual void             signOff(int id) = 0;
    virtual void             updateAircraftInformation(int id, double lat, double lon,
            double heading, double speed, double alt, double dt) = 0;
    virtual bool             hasInstruction(int id) = 0;
    virtual FGATCInstruction getInstruction(int id) = 0;

    double getDt() {
        return dt_count;
    };
    void   setDt(double dt) {
        dt_count = dt;
    };
    void transmit(FGTrafficRecord *rec, FGAirportDynamics *parent, AtcMsgId msgId, AtcMsgDir msgDir, bool audible);
    std::string getGateName(FGAIAircraft *aircraft);
    virtual void render(bool) = 0;
    virtual std::string getName()  = 0;

    virtual void update(double) = 0;


protected:
    // guard variable to avoid modifying state during destruction
    bool _isDestroying = false;

private:

    AtcMsgDir lastTransmissionDirection;
};

#endif