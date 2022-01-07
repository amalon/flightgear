// TimeManager.hxx -- simulation-wide time management
//
// Written by James Turner, started July 2010.
//
// Copyright (C) 2010  Curtis L. Olson  - http://www.flightgear.org/~curt
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

#ifndef FG_TIME_TIMEMANAGER_HXX
#define FG_TIME_TIMEMANAGER_HXX

#include <simgear/props/props.hxx>
#include <simgear/structure/subsystem_mgr.hxx>
#include <simgear/math/SGVec3.hxx>

// forward decls
class SGTime;

class TimeManager : public SGSubsystem,
                    public SGPropertyChangeListener
{
public:
    TimeManager();
    virtual ~TimeManager();

    // Subsystem API.
    void init() override;
    void postinit() override;
    void reinit() override;
    void shutdown() override;
    void unbind() override;
    void update(double dt) override;

    void reposition();

    // Subsystem identification.
    static const char* staticSubsystemClassId() { return "time"; }

    void computeTimeDeltas(double& simDt, double& realDt);
    
    void computeTimeDeltasSimple(double& simDt, double& realDt);

    // SGPropertyChangeListener overrides
    void valueChanged(SGPropertyNode *) override;

    void setTimeOffset(const std::string& offset_type, long int offset);

    inline double getMPProtocolClockSec() const { return _mpProtocolClock; }
    inline double getSteadyClockSec() const { return _steadyClock; }

    double getSimSpeedUpFactor() const;

private:
    // test class is a friend so we can fake elapsed system time
    friend class TimeManagerTests;

    /**
     * Ensure a consistent update-rate using a combination of
     * sleep()-ing and busy-waiting.  
     */
    void throttleUpdateRate();

    /**
     * Compute frame (update) rate and write it to a property
     */
    void computeFrameRate();

    void updateLocalTime();

    void updateLocalTimeString();

    // set up a time offset (aka warp) if one is specified
    void initTimeOffset();

    bool _inited = false;
    SGTime* _impl = nullptr;
    SGTimeStamp _lastStamp;
    SGTimeStamp _systemStamp;
    bool _firstUpdate = true;
    double _dtRemainder = 0;
    SGPropertyNode_ptr _maxDtPerFrame;
    SGPropertyNode_ptr _clockFreeze;
    SGPropertyNode_ptr _timeOverride;
    SGPropertyNode_ptr _warp;
    SGPropertyNode_ptr _warpDelta;
    SGPropertyNode_ptr _simTimeFactor;
    SGPropertyNode_ptr _mpProtocolClockNode;
    SGPropertyNode_ptr _steadyClockNode;
    SGPropertyNode_ptr _frameTimeOffsetNode;
    SGPropertyNode_ptr _dtRemainderNode;
    SGPropertyNode_ptr _mpClockOffset;
    SGPropertyNode_ptr _steadyClockDrift;
    SGPropertyNode_ptr _computeDrift;
    SGPropertyNode_ptr _frameWait;
    SGPropertyNode_ptr _maxFrameRate;
    SGPropertyNode_ptr _localTimeStringNode;
    SGPropertyNode_ptr _localTimeZoneNode;
    SGPropertyNode_ptr _frameNumber;
    SGPropertyNode_ptr _simFixedDt;

    bool _lastClockFreeze = false;
    bool _adjustWarpOnUnfreeze = false;

    // frame-rate / worst-case latency / update-rate counters
    SGPropertyNode_ptr _frameRate;
    SGPropertyNode_ptr _frameRateWorst;
    SGPropertyNode_ptr _frameLatency;
    time_t _lastFrameTime = 0;
    double _frameLatencyMax = 0;
    double _mpProtocolClock = 0;
    double _steadyClock = 0;
    int _frameCount = 0;

    // we update TZ after moving more than a threshold distance
    SGVec3d _lastTimeZoneCheckPosition;

    SGPropertyNode_ptr _sceneryLoaded;
    SGPropertyNode_ptr _modelHz;
    SGPropertyNode_ptr _timeDelta;
    SGPropertyNode_ptr _simTimeDelta;
    
    bool    _simpleTimeEnabledPrev = false;
    SGPropertyNode_ptr _simpleTimeEnabled;
    SGPropertyNode_ptr _simpleTimeUtc;
    SGPropertyNode_ptr _simpleTimeFdm;
    double _simple_time_utc = 0;
    double _simple_time_fdm = 0;
};

#endif // of FG_TIME_TIMEMANAGER_HXX
