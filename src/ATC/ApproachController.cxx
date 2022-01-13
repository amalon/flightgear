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
#include <ATC/trafficcontrol.hxx>
#include <ATC/ATCController.hxx>
#include <ATC/ApproachController.hxx>

using std::sort;
using std::string;

/***************************************************************************
 * class FGApproachController
 * subclass of FGATCController
 **************************************************************************/
 
FGApproachController::FGApproachController(FGAirportDynamics *par):
        FGATCController()
{
    parent = par;
}

FGApproachController::~FGApproachController()
{
    _isDestroying = true;
    clearTrafficControllers(activeTraffic);
}


void FGApproachController::announcePosition(int id,
        FGAIFlightPlan * intendedRoute,
        int currentPosition,
        double lat, double lon,
        double heading, double speed,
        double alt, double radius,
        int leg, FGAIAircraft * ref)
{
    init();
    
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    // Add a new TrafficRecord if no one exsists for this aircraft.
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        FGTrafficRecord rec;
        rec.setId(id);

        rec.setPositionAndHeading(lat, lon, heading, speed, alt);
        rec.setRunway(intendedRoute->getRunway());
        rec.setLeg(leg);
        //rec.setCallSign(callsign);
        rec.setAircraft(ref);
        activeTraffic.push_back(rec);
    } else {
        i->setPositionAndHeading(lat, lon, heading, speed, alt);
    }
}

void FGApproachController::updateAircraftInformation(int id, double lat, double lon,
        double heading, double speed, double alt,
        double dt)
{
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    TrafficVectorIterator current;
	
    // update position of the current aircraft
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: updating aircraft without traffic record at " << SG_ORIGIN);
    } else {
        i->setPositionAndHeading(lat, lon, heading, speed, alt);
        current = i;
        SG_LOG(SG_ATC, SG_BULK, "ApproachController: checking for speed");
        time_t time_diff =
            current->getAircraft()->
            checkForArrivalTime(string("final001"));
        if (time_diff > 15) {
            current->setSpeedAdjustment(current->getAircraft()->
                                        getPerformance()->vDescent() *
                                        1.35);
        } else if (time_diff > 5) {
            current->setSpeedAdjustment(current->getAircraft()->
                                        getPerformance()->vDescent() *
                                        1.2);
        } else if (time_diff < -15) {
            current->setSpeedAdjustment(current->getAircraft()->
                                        getPerformance()->vDescent() *
                                        0.65);
        } else if (time_diff < -5) {
            current->setSpeedAdjustment(current->getAircraft()->
                                        getPerformance()->vDescent() *
                                        0.8);
        } else {
            current->clearSpeedAdjustment();
        }
        //current->setSpeedAdjustment(current->getAircraft()->getPerformance()->vDescent() + time_diff);
    }
    setDt(getDt() + dt);
}

/* Search for and erase traffic record with a specific id */
void FGApproachController::signOff(int id)
{
    // ensure we don't modify activeTraffic during destruction
    if (_isDestroying)
        return;

    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: Aircraft without traffic record is signing off from approach at " << SG_ORIGIN);
    } else {
        i = activeTraffic.erase(i);
    }
}

/* Periodically check for and remove dead traffic records */
void FGApproachController::update(double dt)
{
    FGATCController::eraseDeadTraffic(activeTraffic);
}

bool FGApproachController::hasInstruction(int id)
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


FGATCInstruction FGApproachController::getInstruction(int id)
{
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    if (i == activeTraffic.end() || (activeTraffic.size() == 0)) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: requesting ATC instruction for aircraft without traffic record at " << SG_ORIGIN);
    } else {
        return i->getInstruction();
    }
    return FGATCInstruction();
}


ActiveRunway *FGApproachController::getRunway(const string& name)
{
    ActiveRunwayVecIterator rwy = activeRunways.begin();
    if (activeRunways.size()) {
        while (rwy != activeRunways.end()) {
            if (rwy->getRunwayName() == name) {
                break;
            }
            rwy++;
        }
    }
    if (rwy == activeRunways.end()) {
        ActiveRunway aRwy(name, 0);
        activeRunways.push_back(aRwy);
        rwy = activeRunways.end() - 1;
    }
    return &(*rwy);
}

void FGApproachController::render(bool visible) {
    // Must be BULK in order to prevent it being called each frame
    SG_LOG(SG_ATC, SG_BULK, "FGApproachController::render function not yet implemented");
}

string FGApproachController::getName() {
    return string(parent->getId() + "-approach");
}
