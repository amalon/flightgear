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
#include <ATC/StartupController.hxx>

using std::sort;
using std::string;

/***************************************************************************
 * class FGStartupController
 * subclass of FGATCController
 **************************************************************************/
FGStartupController::FGStartupController(FGAirportDynamics *par):
        FGATCController()
{
    parent = par;
}

FGStartupController::~FGStartupController()
{
    _isDestroying = true;
    clearTrafficControllers(activeTraffic);
}

void FGStartupController::announcePosition(int id,
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
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        FGTrafficRecord rec;
        rec.setId(id);

        rec.setPositionAndHeading(lat, lon, heading, speed, alt);
        rec.setRunway(intendedRoute->getRunway());
        rec.setLeg(leg);
        rec.setPositionAndIntentions(currentPosition, intendedRoute);
        //rec.setCallSign(callsign);
        rec.setAircraft(ref);
        rec.setHoldPosition(true);
        activeTraffic.push_back(rec);
    } else {
        i->setPositionAndIntentions(currentPosition, intendedRoute);
        i->setPositionAndHeading(lat, lon, heading, speed, alt);

    }
}

// NOTE:
// IF WE MAKE TRAFFICRECORD A MEMBER OF THE BASE CLASS
// THE FOLLOWING THREE FUNCTIONS: SIGNOFF, HAS INSTRUCTION AND GETINSTRUCTION CAN
// BECOME DEVIRTUALIZED AND BE A MEMBER OF THE BASE ATCCONTROLLER CLASS
// WHICH WOULD SIMPLIFY CODE MAINTENANCE.
// Note that this function is probably obsolete
bool FGStartupController::hasInstruction(int id)
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


FGATCInstruction FGStartupController::getInstruction(int id)
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

void FGStartupController::signOff(int id)
{
    // ensure we don't modify activeTraffic during destruction
    if (_isDestroying)
        return;

    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
    
    if (i == activeTraffic.end() || activeTraffic.empty()) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: Aircraft without traffic record is signing off from tower at " << SG_ORIGIN);
    } else {
        SG_LOG(SG_ATC, SG_DEBUG, i->getAircraft()->getCallSign() << " signing off from startupcontroller");
        i = activeTraffic.erase(i);
    }
}

bool FGStartupController::checkTransmissionState(int st, time_t now, time_t startTime, TrafficVectorIterator i, AtcMsgId msgId,
        AtcMsgDir msgDir)
{
    int state = i->getState();
    if ((state == st) && available) {
        if ((msgDir == ATC_AIR_TO_GROUND) && isUserAircraft(i->getAircraft())) {

            SG_LOG(SG_ATC, SG_BULK, "Checking state " << st << " for " << i->getAircraft()->getCallSign());
            SGPropertyNode_ptr trans_num = globals->get_props()->getNode("/sim/atc/transmission-num", true);
            int n = trans_num->getIntValue();
            if (n == 0) {
                trans_num->setIntValue(-1);
                // PopupCallback(n);
                SG_LOG(SG_ATC, SG_BULK, "Selected transmission message " << n);
                //FGATCDialogNew::instance()->removeEntry(1);
            } else {
                SG_LOG(SG_ATC, SG_BULK, "Creating message for " << i->getAircraft()->getCallSign());
                transmit(&(*i), &(*parent), msgId, msgDir, false);
                return false;
            }
        }
        if (now > startTime) {
            SG_LOG(SG_ATC, SG_BULK, "Transmitting startup msg");
            transmit(&(*i), &(*parent), msgId, msgDir, true);
            i->updateState();
            lastTransmission = now;
            available = false;
            return true;
        }
    }
    return false;
}

void FGStartupController::updateAircraftInformation(int id, double lat, double lon,
        double heading, double speed, double alt,
        double dt)
{
    // Search activeTraffic for a record matching our id
    TrafficVectorIterator i = searchActiveTraffic(activeTraffic, id);
	TrafficVectorIterator current, closest;
	
    if (i == activeTraffic.end() || (activeTraffic.size() == 0)) {
        SG_LOG(SG_ATC, SG_ALERT,
               "AI error: updating aircraft without traffic record at " << SG_ORIGIN);
        return;
    } else {
        i->setPositionAndHeading(lat, lon, heading, speed, alt);
        current = i;
    }
    setDt(getDt() + dt);

    int state = i->getState();

    // Sentry FLIGHTGEAR-2Q : don't crash on null TrafficRef
    // Sentry FLIGHTGEAR-129: don't crash on null aircraft
    if (!i->getAircraft() || !i->getAircraft()->getTrafficRef()) {
        SG_LOG(SG_ATC, SG_ALERT, "AI traffic: updating aircraft without traffic ref");
        return;
    }

    // The user controlled aircraft should have crased here, because it doesn't have a traffic reference.
    // NOTE: if we create a traffic schedule for the user aircraft, we can use this to plan a flight.
    time_t startTime = i->getAircraft()->getTrafficRef()->getDepartureTime();
    time_t now = globals->get_time_params()->get_cur_time();

    SG_LOG(SG_ATC, SG_BULK, i->getAircraft()->getTrafficRef()->getCallSign()
         << " is scheduled to depart in " << startTime-now << " seconds. Available = " << available
         << " at parking " << getGateName(i->getAircraft()));

    if ((now - lastTransmission) > 3 + (rand() % 15)) {
        available = true;
    }

    checkTransmissionState(0, now, (startTime + 0  ), i, MSG_ANNOUNCE_ENGINE_START,                     ATC_AIR_TO_GROUND);
    checkTransmissionState(1, now, (startTime + 60 ), i, MSG_REQUEST_ENGINE_START,                      ATC_AIR_TO_GROUND);
    checkTransmissionState(2, now, (startTime + 80 ), i, MSG_PERMIT_ENGINE_START,                       ATC_GROUND_TO_AIR);
    checkTransmissionState(3, now, (startTime + 100), i, MSG_ACKNOWLEDGE_ENGINE_START,                  ATC_AIR_TO_GROUND);
    if (checkTransmissionState(4, now, (startTime + 130), i, MSG_ACKNOWLEDGE_SWITCH_GROUND_FREQUENCY,       ATC_AIR_TO_GROUND)) {
        i->nextFrequency();
    }
    checkTransmissionState(5, now, (startTime + 140), i, MSG_INITIATE_CONTACT,                          ATC_AIR_TO_GROUND);
    checkTransmissionState(6, now, (startTime + 150), i, MSG_ACKNOWLEDGE_INITIATE_CONTACT,              ATC_GROUND_TO_AIR);
    checkTransmissionState(7, now, (startTime + 180), i, MSG_REQUEST_PUSHBACK_CLEARANCE,                ATC_AIR_TO_GROUND);



    if ((state == 8) && available) {
        if (now > startTime + 200) {
            if (i->pushBackAllowed()) {
                i->allowRepeatedTransmissions();
                transmit(&(*i), &(*parent), MSG_PERMIT_PUSHBACK_CLEARANCE,
                         ATC_GROUND_TO_AIR, true);
                i->updateState();
            } else {
                transmit(&(*i), &(*parent), MSG_HOLD_PUSHBACK_CLEARANCE,
                         ATC_GROUND_TO_AIR, true);
                i->suppressRepeatedTransmissions();
            }
            lastTransmission = now;
            available = false;
        }
    }
    if ((state == 9) && available) {
        i->setHoldPosition(false);
    }
}

// Note that this function is copied from simgear. for maintanance purposes, it's probabtl better to make a general function out of that.
static void WorldCoordinate(osg::Matrix& obj_pos, double lat,
                            double lon, double elev, double hdg, double slope)
{
    SGGeod geod = SGGeod::fromDegM(lon, lat, elev);
    obj_pos = makeZUpFrame(geod);
    // hdg is not a compass heading, but a counter-clockwise rotation
    // around the Z axis
    obj_pos.preMult(osg::Matrix::rotate(hdg * SGD_DEGREES_TO_RADIANS,
                                        0.0, 0.0, 1.0));
    obj_pos.preMult(osg::Matrix::rotate(slope * SGD_DEGREES_TO_RADIANS,
                                        0.0, 1.0, 0.0));
}


void FGStartupController::render(bool visible)
{
    SG_LOG(SG_ATC, SG_DEBUG, "Rendering startup controller");
    SGMaterialLib *matlib = globals->get_matlib();
    if (group) {
        //int nr = ;
        globals->get_scenery()->get_scene_graph()->removeChild(group);
        //while (group->getNumChildren()) {
        // SG_LOG(SG_ATC, SG_BULK, "Number of children: " << group->getNumChildren());
        //simgear::EffectGeode* geode = (simgear::EffectGeode*) group->getChild(0);
        //osg::MatrixTransform *obj_trans = (osg::MatrixTransform*) group->getChild(0);
        //geode->releaseGLObjects();
        //group->removeChild(geode);
        //delete geode;
        group = 0;
    }
    if (visible) {
        group = new osg::Group;
        FGScenery * local_scenery = globals->get_scenery();
        //double elevation_meters = 0.0;
        //double elevation_feet = 0.0;

        FGGroundNetwork* groundNet = parent->parent()->groundNetwork();

        //for ( FGTaxiSegmentVectorIterator i = segments.begin(); i != segments.end(); i++) {
        double dx = 0;
        time_t now = globals->get_time_params()->get_cur_time();

        for   (TrafficVectorIterator i = activeTraffic.begin(); i != activeTraffic.end(); i++) {
            if (i->isActive(300)) {
                // Handle start point
                int pos = i->getCurrentPosition();
                SG_LOG(SG_ATC, SG_BULK, "rendering for " << i->getAircraft()->getCallSign() << "pos = " << pos);
                if (pos > 0) {
                    FGTaxiSegment *segment = groundNet->findSegment(pos);
                    SGGeod start(SGGeod::fromDeg((i->getLongitude()), (i->getLatitude())));
                    SGGeod end  (segment->getEnd()->geod());

                    double length = SGGeodesy::distanceM(start, end);
                    //heading = SGGeodesy::headingDeg(start->geod(), end->geod());

                    double az2, heading; //, distanceM;
                    SGGeodesy::inverse(start, end, heading, az2, length);
                    double coveredDistance = length * 0.5;
                    SGGeod center;
                    SGGeodesy::direct(start, heading, coveredDistance, center, az2);
                    SG_LOG(SG_ATC, SG_BULK, "Active Aircraft : Centerpoint = (" << center.getLatitudeDeg() << ", " << center.getLongitudeDeg() << "). Heading = " << heading);
                    ///////////////////////////////////////////////////////////////////////////////
                    // Make a helper function out of this
                    osg::Matrix obj_pos;
                    osg::MatrixTransform *obj_trans = new osg::MatrixTransform;
                    obj_trans->setDataVariance(osg::Object::STATIC);
                    // Experimental: Calculate slope here, based on length, and the individual elevations
                    double elevationStart;
                    if (isUserAircraft((i)->getAircraft())) {
                        elevationStart = fgGetDouble("/position/ground-elev-m");
                    } else {
                        elevationStart = ((i)->getAircraft()->_getAltitude() * SG_FEET_TO_METER);
                    }
                    double elevationEnd   = segment->getEnd()->getElevationM();
                    if ((elevationEnd == 0) || (elevationEnd == parent->getElevation())) {
                        SGGeod center2 = end;
                        center2.setElevationM(SG_MAX_ELEVATION_M);
                        if (local_scenery->get_elevation_m( center2, elevationEnd, NULL )) {
                            //elevation_feet = elevationEnd * SG_METER_TO_FEET + 0.5;
                            //elevation_meters += 0.5;
                        }
                        else {
                            elevationEnd = parent->getElevation();
                        }
                        segment->getEnd()->setElevation(elevationEnd);
                    }

                    double elevationMean  = (elevationStart + elevationEnd) / 2.0;
                    double elevDiff       = elevationEnd - elevationStart;

                    double slope = atan2(elevDiff, length) * SGD_RADIANS_TO_DEGREES;

                    SG_LOG(SG_ATC, SG_BULK, "1. Using mean elevation : " << elevationMean << " and " << slope);

                    WorldCoordinate( obj_pos, center.getLatitudeDeg(), center.getLongitudeDeg(), elevationMean + 0.5 + dx, -(heading), slope );
                    ;

                    obj_trans->setMatrix( obj_pos );
                    //osg::Vec3 center(0, 0, 0)

                    float width = length /2.0;
                    osg::Vec3 corner(-width, 0, 0.25f);
                    osg::Vec3 widthVec(2*width + 1, 0, 0);
                    osg::Vec3 heightVec(0, 1, 0);
                    osg::Geometry* geometry;
                    geometry = osg::createTexturedQuadGeometry(corner, widthVec, heightVec);
                    simgear::EffectGeode* geode = new simgear::EffectGeode;
                    geode->setName("test");
                    geode->addDrawable(geometry);
                    //osg::Node *custom_obj;
                    SGMaterial *mat;
                    if (segment->hasBlock(now)) {
                        mat = matlib->find("UnidirectionalTaperRed", center);
                    } else {
                        mat = matlib->find("UnidirectionalTaperGreen", center);
                    }
                    if (mat)
                        geode->setEffect(mat->get_effect());
                    obj_trans->addChild(geode);
                    // wire as much of the scene graph together as we can
                    //->addChild( obj_trans );
                    group->addChild( obj_trans );
                    /////////////////////////////////////////////////////////////////////
                } else {
                    SG_LOG(SG_ATC, SG_DEBUG, "BIG FAT WARNING: current position is here : " << pos);
                }
                for (intVecIterator j = (i)->getIntentions().begin(); j != (i)->getIntentions().end(); j++) {
                    osg::Matrix obj_pos;
                    int k = (*j);
                    if (k > 0) {
                        SG_LOG(SG_ATC, SG_BULK, "rendering for " << i->getAircraft()->getCallSign() << "intention = " << k);
                        osg::MatrixTransform *obj_trans = new osg::MatrixTransform;
                        obj_trans->setDataVariance(osg::Object::STATIC);
                        FGTaxiSegment *segment  = groundNet->findSegment(k);

                        double elevationStart = segment->getStart()->getElevationM();
                        double elevationEnd   = segment->getEnd  ()->getElevationM();
                        if ((elevationStart == 0) || (elevationStart == parent->getElevation())) {
                            SGGeod center2 = segment->getStart()->geod();
                            center2.setElevationM(SG_MAX_ELEVATION_M);
                            if (local_scenery->get_elevation_m( center2, elevationStart, NULL )) {
                                //elevation_feet = elevationStart * SG_METER_TO_FEET + 0.5;
                                //elevation_meters += 0.5;
                            }
                            else {
                                elevationStart = parent->getElevation();
                            }
                            segment->getStart()->setElevation(elevationStart);
                        }
                        if ((elevationEnd == 0) || (elevationEnd == parent->getElevation())) {
                            SGGeod center2 = segment->getEnd()->geod();
                            center2.setElevationM(SG_MAX_ELEVATION_M);
                            if (local_scenery->get_elevation_m( center2, elevationEnd, NULL )) {
                                //elevation_feet = elevationEnd * SG_METER_TO_FEET + 0.5;
                                //elevation_meters += 0.5;
                            }
                            else {
                                elevationEnd = parent->getElevation();
                            }
                            segment->getEnd()->setElevation(elevationEnd);
                        }

                        double elevationMean  = (elevationStart + elevationEnd) / 2.0;
                        double elevDiff       = elevationEnd - elevationStart;
                        double length         = segment->getLength();
                        double slope = atan2(elevDiff, length) * SGD_RADIANS_TO_DEGREES;

                        SG_LOG(SG_ATC, SG_BULK, "2. Using mean elevation : " << elevationMean << " and " << slope);

                        SGGeod segCenter(segment->getCenter());
                        WorldCoordinate( obj_pos, segCenter.getLatitudeDeg(),
                                        segCenter.getLongitudeDeg(), elevationMean + 0.5 + dx, -(segment->getHeading()), slope );

                        //WorldCoordinate( obj_pos, segment->getLatitude(), segment->getLongitude(), parent->getElevation()+8+dx, -(segment->getHeading()) );

                        obj_trans->setMatrix( obj_pos );
                        //osg::Vec3 center(0, 0, 0)

                        float width = segment->getLength() /2.0;
                        osg::Vec3 corner(-width, 0, 0.25f);
                        osg::Vec3 widthVec(2*width + 1, 0, 0);
                        osg::Vec3 heightVec(0, 1, 0);
                        osg::Geometry* geometry;
                        geometry = osg::createTexturedQuadGeometry(corner, widthVec, heightVec);
                        simgear::EffectGeode* geode = new simgear::EffectGeode;
                        geode->setName("test");
                        geode->addDrawable(geometry);
                        //osg::Node *custom_obj;
                        SGMaterial *mat;
                        if (segment->hasBlock(now)) {
                            mat = matlib->find("UnidirectionalTaperRed", segCenter);
                        } else {
                            mat = matlib->find("UnidirectionalTaperGreen", segCenter);
                        }
                        if (mat)
                            geode->setEffect(mat->get_effect());
                        obj_trans->addChild(geode);
                        // wire as much of the scene graph together as we can
                        //->addChild( obj_trans );
                        group->addChild( obj_trans );
                    } else {
                        SG_LOG(SG_ATC, SG_DEBUG, "BIG FAT WARNING: k is here : " << pos);
                    }
                }
                dx += 0.2;
            }
        }
        globals->get_scenery()->get_scene_graph()->addChild(group);
    }
}

string FGStartupController::getName() {
    return string(parent->getId() + "-startup");
}

void FGStartupController::update(double dt)
{
    FGATCController::eraseDeadTraffic(activeTraffic);
}

