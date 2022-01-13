// Extracted from trafficrecord.cxx - Implementation of AIModels ATC code.
//
// Written by Durk Talsma, started September 2006.
//
// Copyright (C) 2006 Durk Talsma.
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

#include <config.h>

#include <algorithm>
#include <cstdio>
#include <random>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Shape>

#include <simgear/scene/material/EffectGeode.hxx>
#include <simgear/scene/material/matlib.hxx>
#include <simgear/scene/material/mat.hxx>
#include <simgear/scene/util/OsgMath.hxx>
#include <simgear/timing/sg_time.hxx>

#include <Scenery/scenery.hxx>

#include "trafficcontrol.hxx"
#include "atc_mgr.hxx"
#include <AIModel/AIAircraft.hxx>
#include <AIModel/AIFlightPlan.hxx>
#include <AIModel/performancedata.hxx>
#include <Traffic/TrafficMgr.hxx>
#include <Airports/groundnetwork.hxx>
#include <Airports/dynamics.hxx>
#include <Airports/airport.hxx>
#include <Radio/radio.hxx>
#include <signal.h>

#include <ATC/atc_mgr.hxx>
#include <ATC/ATCController.hxx>
#include <ATC/trafficcontrol.hxx>
#include <ATC/TowerController.hxx>

using std::sort;
using std::string;

/***************************************************************************
 * class FGTowerController
s * subclass of FGATCController
 **************************************************************************/
 
FGTowerController::FGTowerController(FGAirportDynamics *par) :
        FGATCController()
{
    parent = par;
}

FGTowerController::~FGTowerController()
{
    _isDestroying = true;
    clearTrafficControllers(activeTraffic);
}

//
void FGTowerController::announcePosition(int id,
        FGAIFlightPlan * intendedRoute,
        int currentPosition, double lat,
        double lon, double heading,
        double speed, double alt,
        double radius, int leg,
        FGAIAircraft * ref)
{
    init();
	
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    // Add a new TrafficRecord if no one exsists for this aircraft.
    if (i == activeTraffic.end() || (activeTraffic.empty())) {
        FGTrafficRecord rec;
        rec.setId(id);

        rec.setPositionAndHeading(lat, lon, heading, speed, alt);
        rec.setRunway(intendedRoute->getRunway());
        rec.setLeg(leg);
        //rec.setCallSign(callsign);
        rec.setRadius(radius);
        rec.setAircraft(ref);
        activeTraffic.push_back(rec);
        // Don't just schedule the aircraft for the tower controller, also assign if to the correct active runway.
        ActiveRunwayVecIterator rwy = activeRunways.begin();
        if (! activeRunways.empty()) {
            while (rwy != activeRunways.end()) {
                if (rwy->getRunwayName() == intendedRoute->getRunway()) {
                    break;
                }
                rwy++;
            }
        }
        if (rwy == activeRunways.end()) {
            ActiveRunway aRwy(intendedRoute->getRunway(), id);
            aRwy.addToDepartureQueue(ref);
            activeRunways.push_back(aRwy);
            rwy = (activeRunways.end()-1);
        } else {
            rwy->addToDepartureQueue(ref);
        }

        SG_LOG(SG_ATC, SG_DEBUG, ref->getTrafficRef()->getCallSign() << " You are number " << rwy->getdepartureQueueSize() << " for takeoff ");
    } else {
        i->setPositionAndHeading(lat, lon, heading, speed, alt);
    }
}

void FGTowerController::updateAircraftInformation(int id, double lat, double lon,
        double heading, double speed, double alt,
        double dt)
{
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    setDt(getDt() + dt);

    if (i == activeTraffic.end() || (activeTraffic.empty())) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: updating aircraft without traffic record at " <<
               SG_ORIGIN);
        return;
    }

    // Update the position of the current aircraft
    FGTrafficRecord& current = *i;
    current.setPositionAndHeading(lat, lon, heading, speed, alt);

    // see if we already have a clearance record for the currently active runway
    // NOTE: dd. 2011-08-07: Because the active runway has been constructed in the announcePosition function, we may safely assume that is
    // already exists here. So, we can simplify the current code.
    
    ActiveRunwayVecIterator rwy = activeRunways.begin();
    //if (parent->getId() == fgGetString("/sim/presets/airport-id")) {
    //    for (rwy = activeRunways.begin(); rwy != activeRunways.end(); rwy++) {
    //        rwy->printdepartureQueue();
    //    }
    //}
    
    rwy = activeRunways.begin();
    while (rwy != activeRunways.end()) {
        if (rwy->getRunwayName() == current.getRunway()) {
            break;
        }
        rwy++;
    }

    // only bother running the following code if the current aircraft is the
    // first in line for depature
    /* if (current.getAircraft() == rwy->getFirstAircraftIndepartureQueue()) {
        if (rwy->getCleared()) {
            if (id == rwy->getCleared()) {
                current.setHoldPosition(false);
            } else {
                current.setHoldPosition(true);
            }
        } else {
            // For now. At later stages, this will probably be the place to check for inbound traffc.
            rwy->setCleared(id);
        }
    } */
    // only bother with aircraft that have a takeoff status of 2, since those are essentially under tower control
    auto ac = rwy->getFirstAircraftInDepartureQueue();
    if (ac) {
        if (ac->getTakeOffStatus() == 1) {
            // transmit takeoff clearance
            ac->setTakeOffStatus(2);
        }
    }
    
    if (current.getAircraft()->getTakeOffStatus() == 2) {
        current.setHoldPosition(false);
    } else {
        current.setHoldPosition(true);
    }
    int clearanceId = rwy->getCleared();
    if (clearanceId) {
        if (id == clearanceId) {
            current.setHoldPosition(false);
        }
    } else {
        if (current.getAircraft() == rwy->getFirstAircraftInDepartureQueue()) {
            rwy->setCleared(id);
            auto ac = rwy->getFirstOfStatus(1);
            if (ac)
                ac->setTakeOffStatus(2);
                // transmit takeoff clearacne? But why twice?
        }
    }
} 


void FGTowerController::signOff(int id)
{
    // ensure we don't modify activeTraffic during destruction
    if (_isDestroying)
        return;

    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    if (i == activeTraffic.end() || (activeTraffic.empty())) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: Aircraft without traffic record is signing off from tower at " << SG_ORIGIN);
        return;
    }

    const auto trafficRunway = i->getRunway();
    auto runwayIt = std::find_if(activeRunways.begin(), activeRunways.end(),
                                 [&trafficRunway](const ActiveRunway& ar) {
                                     return ar.getRunwayName() == trafficRunway;
                                 });

    if (runwayIt != activeRunways.end()) {
        runwayIt->setCleared(0);
        runwayIt->updateDepartureQueue();
    } else {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: Attempting to erase non-existing runway clearance record in FGTowerController::signoff at " << SG_ORIGIN);
    }

    i->getAircraft()->resetTakeOffStatus();
    activeTraffic.erase(i);
    SG_LOG(SG_ATC, SG_DEBUG, "Signing off from tower controller");
}

// NOTE:
// IF WE MAKE TRAFFICRECORD A MEMBER OF THE BASE CLASS
// THE FOLLOWING THREE FUNCTIONS: SIGNOFF, HAS INSTRUCTION AND GETINSTRUCTION CAN
// BECOME DEVIRTUALIZED AND BE A MEMBER OF THE BASE ATCCONTROLLER CLASS
// WHICH WOULD SIMPLIFY CODE MAINTENANCE.
// Note that this function is probably obsolete
bool FGTowerController::hasInstruction(int id)
{
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: checking ATC instruction for aircraft without traffic record at " << SG_ORIGIN);
    } else {
        return i->hasInstruction();
    }
    return false;
}


FGATCInstruction FGTowerController::getInstruction(int id)
{
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: requesting ATC instruction for aircraft without traffic record at " << SG_ORIGIN);
    } else {
        return i->getInstruction();
    }
    return FGATCInstruction();
}

void FGTowerController::render(bool visible) {
    // this should be bulk, since its called quite often
    SG_LOG(SG_ATC, SG_BULK, "FGTowerController::render function not yet implemented");
}

string FGTowerController::getName() {
    return string(parent->getId() + "-tower");
}

void FGTowerController::update(double dt)
{
    FGATCController::eraseDeadTraffic(activeTraffic);
}
