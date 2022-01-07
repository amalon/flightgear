// replay.hxx - a system to record and replay FlightGear flights
//
// Written by Curtis Olson, started July 2003.
//
// Copyright (C) 2003  Curtis L. Olson  - http://www.flightgear.org/~curt
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

#pragma once


#include <mutex>
#include <simgear/compiler.h>

#include <simgear/math/sg_types.hxx>
#include <simgear/props/props.hxx>
#include <simgear/structure/subsystem_mgr.hxx>
#include <simgear/io/iostreams/gzcontainerfile.hxx>
#include <simgear/io/HTTPFileRequest.hxx>

#include <MultiPlayer/multiplaymgr.hxx>

#include <deque>
#include <vector>


class FGFlightRecorder;

/** Magic string to verify valid FG flight recorder tapes. */
extern const char* const FlightRecorderFileMagic;

/* Data for a single frame. */
struct FGReplayData
{
    bool load_signals;
    bool load_multiplayer;
    bool load_extra_properties;
    
    double sim_time;
    
    // Our aircraft state.
    std::vector<char>   raw_data;
    
    // Incoming multiplayer messages, if any.
    std::vector<std::shared_ptr<std::vector<char>>> multiplayer_messages;
    
    // Serialised information about extra property changes, only used when
    // making a Continuous recording - we write this raw data into frame data
    // in the Continuous recording file.
    std::vector<char>   extra_properties;
    
    // Information about extra property changes, only used when replaying. We
    // populate these when loading a frame.
    std::map<std::string, std::string>  replay_extra_property_changes;
    std::vector<std::string>            replay_extra_property_removals;
    
    // Updates static statistics defined below. 
    void UpdateStats();

    // Resets out static property nodes; to be called by fgStartNewReset().
    static void resetStatisticsProperties();
    
    FGReplayData();
    ~FGReplayData();
    
    size_t  m_bytes_raw_data = 0;
    size_t  m_bytes_multiplayer_messages = 0;
    size_t  m_num_multiplayer_messages = 0;
    
    // Statistics about replay data, also properties /sim/replay/datastats_*.
    static size_t   s_num;
    static size_t   s_bytes_raw_data;
    static size_t   s_bytes_multiplayer_messages;
    static size_t   s_num_multiplayer_messages;
    static SGPropertyNode_ptr   s_prop_num;
    static SGPropertyNode_ptr   s_prop_bytes_raw_data;
    static SGPropertyNode_ptr   s_prop_bytes_multiplayer_messages;
    static SGPropertyNode_ptr   s_prop_num_multiplayer_messages;
};

typedef struct
{
    double sim_time;
    std::string message;
    std::string speaker;
} FGReplayMessages;

enum FGTapeType
{
    FGTapeType_NORMAL,
    FGTapeType_CONTINUOUS,
    FGTapeType_RECOVERY,
};


/* Index entry when replaying Continuous recording. */
struct FGFrameInfo
{
    size_t  offset;
    bool    has_signals = false;
    bool    has_multiplayer = false;
    bool    has_extra_properties = false;
};

std::ostream& operator << (std::ostream& out, const FGFrameInfo& frame_info);


struct FGReplayInternal
{
    FGReplayInternal();
    virtual ~FGReplayInternal();
    
    /* Methods that implement the FGReplay API. */

    void bind();
    void init();
    void reinit() ;
    void unbind();
    void update(double dt);

    static const char* staticSubsystemClassId() { return "replay"; }

    bool start(bool NewTape=false);

    bool saveTape(const SGPropertyNode* ConfigData);
    bool loadTape(const SGPropertyNode* ConfigData);
    
    static int loadContinuousHeader(
            const std::string& path,
            std::istream* in,
            SGPropertyNode* properties
            );

    bool loadTape(
            const SGPath& filename,
            bool preview,
            bool create_video,
            double fixed_dt,
            SGPropertyNode& meta_meta,
            simgear::HTTP::FileRequestRef file_request=nullptr
            );
    
    static std::string  makeTapePath(const std::string& tape_name);
    

    /* Callback for SGPropertyChangeListener. */
    //void valueChanged(SGPropertyNode * node) override;
    
    /* Internal state. */
    
    double  m_sim_time;
    
    double  m_last_mt_time;
    double  m_last_lt_time;
    double  m_last_msg_time;
    int     m_last_replay_state;
    bool    m_was_finished_already;
    
    std::deque<FGReplayData*>   m_short_term;
    std::deque<FGReplayData*>   m_medium_term;
    std::deque<FGReplayData*>   m_long_term;
    std::deque<FGReplayData*>   m_recycler;
    
    std::vector<FGReplayMessages>           m_replay_messages;
    std::vector<FGReplayMessages>::iterator m_current_msg;

    SGPropertyNode_ptr  m_disable_replay;
    SGPropertyNode_ptr  m_replay_master;
    SGPropertyNode_ptr  m_replay_master_eof;
    SGPropertyNode_ptr  m_replay_time;
    SGPropertyNode_ptr  m_replay_time_str;
    SGPropertyNode_ptr  m_replay_looped;
    SGPropertyNode_ptr  m_replay_duration_act;
    SGPropertyNode_ptr  m_speed_up;
    SGPropertyNode_ptr  m_replay_multiplayer;
    SGPropertyNode_ptr  m_recovery_period;
    SGPropertyNode_ptr  m_replay_error;
    SGPropertyNode_ptr  m_record_normal_begin;   // Time of first in-memory recorded frame.
    SGPropertyNode_ptr  m_record_normal_end;
    SGPropertyNode_ptr  m_log_frame_times;
    
    SGPropertyNode_ptr  m_sim_startup_xpos;
    SGPropertyNode_ptr  m_sim_startup_ypos;
    SGPropertyNode_ptr  m_sim_startup_xsize;
    SGPropertyNode_ptr  m_sim_startup_ysize;
    SGPropertyNode_ptr  m_simple_time_enabled;
    
    double  m_replay_time_prev; // Used to detect jumps while replaying.

    /* short term sample rate is as every frame. */
    double  m_high_res_time;        // default: 60 secs of high res data
    double  m_medium_res_time;      // default: 10 mins of 1 fps data
    double  m_low_res_time;         // default: 1 hr of 10 spf data
    double  m_medium_sample_rate;   // medium term sample rate (sec)
    double  m_long_sample_rate;     // long term sample rate (sec)

    std::shared_ptr<FGFlightRecorder>   m_flight_recorder;
    
    /* Things for Continuous recording/replay support. */
    std::unique_ptr<struct Continuous>  m_continuous;
    
    FGMultiplayMgr* m_MultiplayMgr;
};


/* Sets things up for writing to a normal or continuous fgtape file.

    extra:
        NULL or extra information when we are called from fgdata gui, e.g. with
        the flight description entered by the user in the save dialogue.
    path:
        Path of fgtape file. We return nullptr if this file already exists.
    duration:
        Duration of recording. Zero if we are starting a continuous recording.
    tape_type:
        .
    continuous_compression:
        Whether to use compression if tape_type is FGTapeType_CONTINUOUS.
    Returns:
        A new SGPropertyNode suitable as prefix of recording. If
        extra:user-data exists, it will appear as meta/user-data.
*/
SGPropertyNode_ptr saveSetup(
        const SGPropertyNode*   extra,
        const SGPath&           path,
        double                  duration,
        FGTapeType              tape_type,
        int                     continuous_compression=0
        );

/* Returns a path using different formats depending on <type>:

    FGTapeType_NORMAL:      <tape-directory>/<aircraft-type>-<date>-<time>.fgtape
    FGTapeType_CONTINUOUS:  <tape-directory>/<aircraft-type>-<date>-<time>-continuous.fgtape
    FGTapeType_RECOVERY:    <tape-directory>/<aircraft-type>-recovery.fgtape
*/
SGPath makeSavePath(FGTapeType type, SGPath* path_timeless=nullptr);
