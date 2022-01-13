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

using std::sort;
using std::string;

/***************************************************************************
 * FGATCController
 *
 **************************************************************************/

FGATCController::FGATCController()
{
    dt_count = 0;
    available = true;
    lastTransmission = 0;
    initialized = false;
    lastTransmissionDirection = ATC_AIR_TO_GROUND;
    group = NULL;
}

FGATCController::~FGATCController()
{
    if (initialized) {
        auto mgr = globals->get_subsystem<FGATCManager>();
        mgr->removeController(this);
    }
}

void FGATCController::init()
{
    if (!initialized) {
        auto mgr = globals->get_subsystem<FGATCManager>();
        mgr->addController(this);
        initialized = true;
    }
}

string FGATCController::getGateName(FGAIAircraft * ref)
{
    return ref->atGate();
}

bool FGATCController::isUserAircraft(FGAIAircraft* ac)
{
    return (ac->getCallSign() == fgGetString("/sim/multiplay/callsign")) ? true : false;
};

void FGATCController::transmit(FGTrafficRecord * rec, FGAirportDynamics *parent, AtcMsgId msgId,
                               AtcMsgDir msgDir, bool audible)
{
    string sender, receiver;
    int stationFreq = 0;
    int taxiFreq = 0;
    int towerFreq = 0;
    int freqId = 0;
    string atisInformation;
    string text;
    string taxiFreqStr;
    string towerFreqStr;
    double heading = 0;
    string activeRunway;
    string fltType;
    string rwyClass;
    string SID;
    string transponderCode;
    FGAIFlightPlan *fp;
    string fltRules;
    string instructionText;
    int ground_to_air=0;

    //double commFreqD;
    sender = rec->getAircraft()->getTrafficRef()->getCallSign();
    if (rec->getAircraft()->getTaxiClearanceRequest()) {
        instructionText = "push-back and taxi";
    } else {
        instructionText = "taxi";
    }
   
    SG_LOG(SG_ATC, SG_DEBUG, "transmitting for: " << sender << "Leg = " << rec->getLeg());

    auto depApt = rec->getAircraft()->getTrafficRef()->getDepartureAirport();

    //FIXME Must be transferred to controller and fallback for GROUND -> TOWER if GROUND not set
    switch (rec->getLeg()) {
    case 1:
    case 2:
        // avoid crash FLIGHTGEAR-ER
        if (!depApt) {
            SG_LOG(SG_ATC, SG_DEV_ALERT, "TrafficRec has empty departure airport, can't transmit");
            return;
        }

        freqId = rec->getNextFrequency();
        stationFreq = depApt->getDynamics()->getGroundFrequency(rec->getLeg() + freqId);
        taxiFreq = depApt->getDynamics()->getGroundFrequency(2);
        towerFreq = depApt->getDynamics()->getTowerFrequency(2);
        receiver = depApt->getName() + "-Ground";
        atisInformation = depApt->getDynamics()->getAtisSequence();
        break;
    case 3:
        if (!depApt) {
            SG_LOG(SG_ATC, SG_DEV_ALERT, "TrafficRec has empty departure airport, can't transmit");
            return;
        }

        receiver = depApt->getName() + "-Tower";
        break;
    }
    
    // Swap sender and receiver value in case of a ground to air transmission
    if (msgDir == ATC_GROUND_TO_AIR) {
        string tmp = sender;
        sender = receiver;
        receiver = tmp;
        ground_to_air = 1;
    }
    
    switch (msgId) {
    case MSG_ANNOUNCE_ENGINE_START:
        text = sender + ". Ready to Start up.";
        break;
    case MSG_REQUEST_ENGINE_START:
        text =
            receiver + ", This is " + sender + ". Position " +
            getGateName(rec->getAircraft()) + ". Information " +
            atisInformation + ". " +
            rec->getAircraft()->getTrafficRef()->getFlightRules() +
            " to " +
            rec->getAircraft()->getTrafficRef()->getArrivalAirport()->
            getName() + ". Request start-up.";
        break;
        // Acknowledge engine startup permission
        // Assign departure runway
        // Assign SID, if necessery (TODO)
    case MSG_PERMIT_ENGINE_START:
        taxiFreqStr = formatATCFrequency3_2(taxiFreq);

        heading = rec->getAircraft()->getTrafficRef()->getCourse();
        fltType = rec->getAircraft()->getTrafficRef()->getFlightType();
        rwyClass =
            rec->getAircraft()->GetFlightPlan()->
            getRunwayClassFromTrafficType(fltType);

        rec->getAircraft()->getTrafficRef()->getDepartureAirport()->
        getDynamics()->getActiveRunway(rwyClass, 1, activeRunway,
                                       heading);
        rec->getAircraft()->GetFlightPlan()->setRunway(activeRunway);
        fp = NULL;
        rec->getAircraft()->GetFlightPlan()->setSID(fp);
        if (fp) {
            SID = fp->getName() + " departure";
        } else {
            SID = "fly runway heading ";
        }
        //snprintf(buffer, 7, "%3.2f", heading);
        fltRules = rec->getAircraft()->getTrafficRef()->getFlightRules();
        transponderCode = genTransponderCode(fltRules);
        rec->getAircraft()->SetTransponderCode(transponderCode);
        text =
            receiver + ". Start-up approved. " + atisInformation +
            " correct, runway " + activeRunway + ", " + SID + ", squawk " +
            transponderCode + ". " +
            "For "+ instructionText + " clearance call " + taxiFreqStr + ". " +
            sender + " control.";
        break;
    case MSG_DENY_ENGINE_START:
        text = receiver + ". Standby.";
        break;
    case MSG_ACKNOWLEDGE_ENGINE_START:
        fp = rec->getAircraft()->GetFlightPlan()->getSID();
        if (fp) {
            SID =
                rec->getAircraft()->GetFlightPlan()->getSID()->getName() +
                " departure";
        } else {
            SID = "fly runway heading ";
        }
        taxiFreqStr = formatATCFrequency3_2(taxiFreq);
        activeRunway = rec->getAircraft()->GetFlightPlan()->getRunway();
        transponderCode = rec->getAircraft()->GetTransponderCode();

        text =
            receiver + ". Start-up approved. " + atisInformation +
            " correct, runway " + activeRunway + ", " + SID + ", squawk " +
            transponderCode + ". " +
            "For " + instructionText + " clearance call " + taxiFreqStr + ". " +
            sender + ".";
        break;
    case MSG_ACKNOWLEDGE_SWITCH_GROUND_FREQUENCY:
        taxiFreqStr = formatATCFrequency3_2(taxiFreq);
        text = receiver + ". Switching to " + taxiFreqStr + ". " + sender + ".";
        break;
    case MSG_INITIATE_CONTACT:
        text = receiver + ". With you. " + sender + ".";
        break;
    case MSG_ACKNOWLEDGE_INITIATE_CONTACT:
        text = receiver + ". Roger. " + sender + ".";
        break;
    case MSG_REQUEST_PUSHBACK_CLEARANCE:
        if (rec->getAircraft()->getTaxiClearanceRequest()) {
            text = receiver + ". Request push-back. " + sender + ".";
        } else {
            text = receiver + ". Request Taxi clearance. " + sender + ".";
        }
        break;
    case MSG_PERMIT_PUSHBACK_CLEARANCE:
        if (rec->getAircraft()->getTaxiClearanceRequest()) {
            text = receiver + ". Push-back approved. " + sender + ".";
        } else {
            text = receiver + ". Cleared to Taxi. " + sender + "."; 
        }
        break;
    case MSG_HOLD_PUSHBACK_CLEARANCE:
        text = receiver + ". Standby. " + sender + ".";
        break;
    case MSG_REQUEST_TAXI_CLEARANCE:
        text = receiver + ". Ready to Taxi. " + sender + ".";
        break;
    case MSG_ISSUE_TAXI_CLEARANCE:
        text = receiver + ". Cleared to taxi. " + sender + ".";
        break;
    case MSG_ACKNOWLEDGE_TAXI_CLEARANCE:
        text = receiver + ". Cleared to taxi. " + sender + ".";
        break;
    case MSG_HOLD_POSITION:
        text = receiver + ". Hold Position. " + sender + ".";
        break;
    case MSG_ACKNOWLEDGE_HOLD_POSITION:
        text = receiver + ". Holding Position. " + sender + ".";
        break;
    case MSG_RESUME_TAXI:
        text = receiver + ". Resume Taxiing. " + sender + ".";
        break;
    case MSG_ACKNOWLEDGE_RESUME_TAXI:
        text = receiver + ". Continuing Taxi. " + sender + ".";
        break;
    case MSG_REPORT_RUNWAY_HOLD_SHORT:
        activeRunway = rec->getAircraft()->GetFlightPlan()->getRunway();
        //activeRunway = "test";
        text = receiver + ". Holding short runway "
               + activeRunway
               + ". " + sender + ".";
        //text = "test1";
        SG_LOG(SG_ATC, SG_DEBUG, "1 Currently at leg " << rec->getLeg());
        break;
    case MSG_ACKNOWLEDGE_REPORT_RUNWAY_HOLD_SHORT:
        activeRunway = rec->getAircraft()->GetFlightPlan()->getRunway();
        text = receiver + " Roger. Holding short runway "
               //                + activeRunway
               + ". " + sender + ".";
        //text = "test2";
        SG_LOG(SG_ATC, SG_DEBUG, "2 Currently at leg " << rec->getLeg());
        break;
    case MSG_SWITCH_TOWER_FREQUENCY:
        towerFreqStr = formatATCFrequency3_2(towerFreq);
        text = receiver + " Contact Tower at " + towerFreqStr + ". " + sender + ".";
        //text = "test3";
        SG_LOG(SG_ATC, SG_DEBUG, "3 Currently at leg " << rec->getLeg());
        break;
    case MSG_ACKNOWLEDGE_SWITCH_TOWER_FREQUENCY:
        towerFreqStr = formatATCFrequency3_2(towerFreq);
        text = receiver + " Roger, switching to tower at " + towerFreqStr + ". " + sender + ".";
        //text = "test4";
        SG_LOG(SG_ATC, SG_DEBUG, "4 Currently at leg " << rec->getLeg());
        break;
    default:
        //text = "test3";
        text = text + sender + ". Transmitting unknown Message.";
        break;
    }
    
    const bool atcAudioEnabled = fgGetBool("/sim/sound/atc/enabled", false);
    if (audible && atcAudioEnabled) {
        double onBoardRadioFreq0 =
            fgGetDouble("/instrumentation/comm[0]/frequencies/selected-mhz");
        double onBoardRadioFreq1 =
            fgGetDouble("/instrumentation/comm[1]/frequencies/selected-mhz");
        int onBoardRadioFreqI0 = (int) floor(onBoardRadioFreq0 * 100 + 0.5);
        int onBoardRadioFreqI1 = (int) floor(onBoardRadioFreq1 * 100 + 0.5);
        SG_LOG(SG_ATC, SG_DEBUG, "Using " << onBoardRadioFreq0 << ", " << onBoardRadioFreq1 << " and " << stationFreq << " for " << text << endl);

        // Display ATC message only when one of the radios is tuned
        // the relevant frequency.
        // Note that distance attenuation is currently not yet implemented
                
        if ((stationFreq > 0)&&
            ((onBoardRadioFreqI0 == stationFreq)||
             (onBoardRadioFreqI1 == stationFreq))) {
            if (rec->allowTransmissions()) {
                
                if( fgGetBool( "/sim/radio/use-itm-attenuation", false ) ) {
                    SG_LOG(SG_ATC, SG_DEBUG, "Using ITM radio propagation");
                    FGRadioTransmission* radio = new FGRadioTransmission();
                    SGGeod sender_pos;
                    double sender_alt_ft, sender_alt;
                    if(ground_to_air) {
                  sender_pos = parent->parent()->geod();
                     }
                    else {
                          sender_alt_ft = rec->getAltitude();
                          sender_alt = sender_alt_ft * SG_FEET_TO_METER;
                          sender_pos= SGGeod::fromDegM( rec->getLongitude(),
                                 rec->getLatitude(), sender_alt );
                    }
                    double frequency = ((double)stationFreq) / 100;
                    radio->receiveATC(sender_pos, frequency, text, ground_to_air);
                    delete radio;
                }
                else {
                    fgSetString("/sim/messages/atc", text.c_str());
                }
            }
        }
    } else {
        //FGATCDialogNew::instance()->addEntry(1, text);
    }
}

/*
* Format integer frequency xxxyy as xxx.yy
* @param freq - integer value
* @return the formatted string
*/
string FGATCController::formatATCFrequency3_2(int freq)
{
    char buffer[7]; // does this ever need to be freed?
    snprintf(buffer, 7, "%3.2f", ((float) freq / 100.0));
    return string(buffer);
}

// TODO: Set transponder codes according to real-world routes.
// The current version just returns a random string of four octal numbers.
string FGATCController::genTransponderCode(const string& fltRules)
{
    if (fltRules == "VFR")
        return string("1200");

    std::default_random_engine generator;
    std::uniform_int_distribution<unsigned> distribution(0, 7);

    unsigned val = (
        distribution(generator) * 1000 +
        distribution(generator) * 100 +
        distribution(generator) * 10 +
        distribution(generator));

    return std::to_string(val);
}

void FGATCController::eraseDeadTraffic(TrafficVector& vec)
{
    auto it = std::remove_if(vec.begin(), vec.end(), [](const FGTrafficRecord& traffic)
    {
        if (!traffic.getAircraft()) {
            return true;
        }
        return traffic.getAircraft()->getDie();
    });
    vec.erase(it, vec.end());
}

void FGATCController::clearTrafficControllers(TrafficVector& vec)
{
    for (const auto& traffic : vec) {
        if (!traffic.getAircraft()) {
            continue;
        }

        traffic.getAircraft()->clearATCController();
    }
}

TrafficVectorIterator FGATCController::searchActiveTraffic(TrafficVector& vec, int id)
{
    return std::find_if(vec.begin(), vec.end(), [id] (const FGTrafficRecord& rec)
                 { return rec.getId() == id; });
}

