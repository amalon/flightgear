// replay.cxx - a system to record and replay FlightGear flights
//
// Written by Curtis Olson, started July 2003.
// Updated by Thorsten Brehm, September 2011 and November 2012.
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


#include "continuous.hxx"
#include "flightrecorder.hxx"
#include "replay.hxx"

#include <Main/fg_props.hxx>
#include <MultiPlayer/mpmessages.hxx>
#include <Time/TimeManager.hxx>
#include <Viewer/viewmgr.hxx>

#include <simgear/misc/sg_dir.hxx>
#include <simgear/props/props_io.hxx>
#include <simgear/structure/commands.hxx>
#include <simgear/structure/exception.hxx>
#include <simgear/structure/subsystem_mgr.hxx>

#include <float.h>
#include <locale>
#include <string.h>


const char* const FlightRecorderFileMagic = "FlightGear Flight Recorder Tape";

void FGReplayData::UpdateStats()
{
    size_t  bytes_raw_data_old = m_bytes_raw_data;
    size_t  bytes_multiplayer_messages_old = m_bytes_multiplayer_messages;
    size_t  num_multiplayer_messages_old = m_num_multiplayer_messages;

    m_bytes_raw_data = raw_data.size();
    m_bytes_multiplayer_messages = 0;
    for ( auto m: multiplayer_messages)
    {
        m_bytes_multiplayer_messages += m->size();
    }
    m_num_multiplayer_messages = multiplayer_messages.size();

    /* Update global stats by adding how much we have changed since last time
    UpdateStats() was called. */
    s_bytes_raw_data += m_bytes_raw_data - bytes_raw_data_old;
    s_bytes_multiplayer_messages += m_bytes_multiplayer_messages - bytes_multiplayer_messages_old;
    s_num_multiplayer_messages += m_num_multiplayer_messages - num_multiplayer_messages_old;

    if (!s_prop_num)
    {
        s_prop_num = fgGetNode("/sim/replay/datastats_num", true);
    }
    if (!s_prop_bytes_raw_data)
    {
        s_prop_bytes_raw_data = fgGetNode("/sim/replay/datastats_bytes_raw_data", true);
    }
    if (!s_prop_bytes_multiplayer_messages)
    {
        s_prop_bytes_multiplayer_messages = fgGetNode("/sim/replay/datastats_bytes_multiplayer_messages", true);
    }
    if (!s_prop_num_multiplayer_messages)
    {
        s_prop_num_multiplayer_messages = fgGetNode("/sim/replay/datastats_num_multiplayer_messages", true);
    }

    s_prop_num->setLongValue(s_num);
    s_prop_bytes_raw_data->setLongValue(s_bytes_raw_data);
    s_prop_bytes_multiplayer_messages->setLongValue(s_bytes_multiplayer_messages);
    s_prop_num_multiplayer_messages->setLongValue(s_num_multiplayer_messages);
}

FGReplayData::FGReplayData()
{
    s_num += 1;
}

FGReplayData::~FGReplayData()
{
    s_bytes_raw_data -= m_bytes_raw_data;
    s_bytes_multiplayer_messages -= m_bytes_multiplayer_messages;
    s_num_multiplayer_messages -= m_num_multiplayer_messages;
    s_num -= 1;
}

void FGReplayData::resetStatisticsProperties()
{
    s_prop_num.reset();
    s_prop_bytes_raw_data.reset();
    s_prop_bytes_multiplayer_messages.reset();
    s_prop_num_multiplayer_messages.reset();
}

size_t   FGReplayData::s_num = 0;
size_t   FGReplayData::s_bytes_raw_data = 0;
size_t   FGReplayData::s_bytes_multiplayer_messages = 0;
size_t   FGReplayData::s_num_multiplayer_messages = 0;

SGPropertyNode_ptr   FGReplayData::s_prop_num;
SGPropertyNode_ptr   FGReplayData::s_prop_bytes_raw_data;
SGPropertyNode_ptr   FGReplayData::s_prop_bytes_multiplayer_messages;
SGPropertyNode_ptr   FGReplayData::s_prop_num_multiplayer_messages;


namespace ReplayContainer
{
    enum Type
    {
        Invalid    = -1,
        Header     = 0, /**< Used for initial file header (fixed identification string). */
        MetaData   = 1, /**< XML data / properties with arbitrary data, such as description, aircraft type, ... */
        Properties = 2, /**< XML data describing the recorded flight recorder properties.
                             Format is identical to flight recorder XML configuration. Also contains some
                             extra data to verify flight recorder consistency. */
        RawData    = 3  /**< Actual binary data blobs (the recorder's tape).
                             One "RawData" blob is used for each resolution. */
    };
}

/* Constructor */
FGReplayInternal::FGReplayInternal()
    :
    m_sim_time(0),
    m_last_mt_time(0.0),
    m_last_lt_time(0.0),
    m_last_msg_time(0),
    m_last_replay_state(0),
    m_replay_error(fgGetNode("sim/replay/replay-error", true)),
    m_record_normal_begin(fgGetNode("sim/replay/record-normal-begin", true)),
    m_record_normal_end(fgGetNode("sim/replay/record-normal-end", true)),
    m_sim_startup_xpos(fgGetNode("sim/startup/xpos", true)),
    m_sim_startup_ypos(fgGetNode("sim/startup/ypos", true)),
    m_sim_startup_xsize(fgGetNode("sim/startup/xsize", true)),
    m_sim_startup_ysize(fgGetNode("sim/startup/ysize", true)),
    m_simple_time_enabled(fgGetNode("/sim/time/simple-time/enabled", true)),
    m_replay_time_prev(-1.0),
    m_high_res_time(60.0),
    m_medium_res_time(600.0),
    m_low_res_time(3600.0),
    m_medium_sample_rate(0.5), // medium term sample rate (sec)
    m_long_sample_rate(5.0),   // long term sample rate (sec)
    m_flight_recorder(new FGFlightRecorder("replay-config")),
    m_continuous(new Continuous(m_flight_recorder)),
    m_MultiplayMgr(globals->get_subsystem<FGMultiplayMgr>())
{
}

/* Reads binary data from a stream into an instance of a type. */
template<typename T>
static void readRaw(std::istream& in, T& data)
{
    in.read(reinterpret_cast<char*>(&data), sizeof(data));
}

/* Writes instance of a type as binary data to a stream. */
template<typename T>
static void writeRaw(std::ostream& out, const T& data)
{
    out.write(reinterpret_cast<const char*>(&data), sizeof(data));
}

/* Read properties from stream into *node. */
static int PropertiesRead(std::istream& in, SGPropertyNode* node)
{
    uint32_t    buffer_len;
    readRaw(in, buffer_len);
    std::vector<char>   buffer( buffer_len);
    in.read(&buffer.front(), buffer.size());
    readProperties(&buffer.front(), buffer.size()-1, node);
    return 0;
}

static void popupTip(const char* message, int delay)
{
    SGPropertyNode_ptr args(new SGPropertyNode);
    args->setStringValue("label", message);
    args->setIntValue("delay", delay);
    globals->get_commands()->execute("show-message", args);
}


SGPath makeSavePath(FGTapeType type, SGPath* path_timeless)
{
    std::string tape_directory = fgGetString("/sim/replay/tape-directory", "");
    std::string aircraftType  = fgGetString("/sim/aircraft", "unknown");
    if (path_timeless) *path_timeless = "";
    SGPath  path = SGPath(tape_directory);
    path.append(aircraftType);
    if (type == FGTapeType_RECOVERY)
    {
        path.concat("-recovery");
    }
    else
    {
        if (path_timeless) *path_timeless = path;
        time_t calendar_time = time(NULL);
        struct tm *local_tm;
        local_tm = localtime( &calendar_time );
        char time_str[256];
        strftime( time_str, 256, "-%Y%m%d-%H%M%S", local_tm);
        path.concat(time_str);
    }
    if (type == FGTapeType_CONTINUOUS)
    {
        path.concat("-continuous");
        if (path_timeless) path_timeless->concat("-continuous");
    }
    path.concat(".fgtape");
    if (path_timeless) path_timeless->concat(".fgtape");
    return path;
}

/* Clear all internal buffers. */
static void clear(FGReplayInternal& self)
{
    while ( !self.m_short_term.empty() )
    {
        delete self.m_short_term.front();
        self.m_short_term.pop_front();
    }
    while ( !self.m_medium_term.empty() )
    {
        delete self.m_medium_term.front();
        self.m_medium_term.pop_front();
    }
    while ( !self.m_long_term.empty() )
    {
        delete self.m_long_term.front();
        self.m_long_term.pop_front();
    }
    while ( !self.m_recycler.empty() )
    {
        delete self.m_recycler.front();
        self.m_recycler.pop_front();
    }

    // clear messages belonging to old replay session
    fgGetNode("/sim/replay/messages", 0, true)->removeChildren("msg");
}

/* Destructor */
FGReplayInternal::~FGReplayInternal()
{
    if (m_continuous->m_out.is_open())
    {
        m_continuous->m_out.close();
    }
    clear(*this);
}

/* Initialize the data structures */
void
FGReplayInternal::init()
{
    m_disable_replay        = fgGetNode("/sim/replay/disable",              true);
    m_replay_master         = fgGetNode("/sim/replay/replay-state",         true);
    m_replay_master_eof     = fgGetNode("/sim/replay/replay-state-eof",     true);
    m_replay_time           = fgGetNode("/sim/replay/time",                 true);
    m_replay_time_str       = fgGetNode("/sim/replay/time-str",             true);
    m_replay_looped         = fgGetNode("/sim/replay/looped",               true);
    m_replay_duration_act   = fgGetNode("/sim/replay/duration-act",         true);
    m_speed_up              = fgGetNode("/sim/speed-up",                    true);
    m_replay_multiplayer    = fgGetNode("/sim/replay/record-multiplayer",   true);
    m_recovery_period       = fgGetNode("/sim/replay/recovery-period",      true);
    m_log_frame_times       = fgGetNode("/sim/replay/log-frame-times",      true);

    // alias to keep backward compatibility
    fgGetNode("/sim/freeze/replay-state", true)->alias(m_replay_master);

    reinit();
}

/* Loads messages from /sim/replay/messages. */
static void loadMessages(FGReplayInternal& self)
{
    // load messages
    self.m_replay_messages.clear();
    simgear::PropertyList msgs = fgGetNode("/sim/replay/messages", true)->getChildren("msg");

    for (auto it = msgs.begin(); it != msgs.end();++it)
    {
        std::string msgText = (*it)->getStringValue("text", "");
        const double msgTime = (*it)->getDoubleValue("time", -1.0);
        std::string msgSpeaker = (*it)->getStringValue("speaker", "pilot");
        if (!msgText.empty() && msgTime >= 0)
        {
            FGReplayMessages data;
            data.sim_time = msgTime;
            data.message = msgText;
            data.speaker = msgSpeaker;
            self.m_replay_messages.push_back(data);
        }
    }
    self.m_current_msg = self.m_replay_messages.begin();
}

void fillRecycler(FGReplayInternal& self)
{
    // Create an estimated nr of required ReplayData objects
    // 120 is an estimated maximum frame rate.
    int estNrObjects = (int) (
            self.m_high_res_time * 120
            + self.m_medium_res_time * self.m_medium_sample_rate
            + self.m_low_res_time * self.m_long_sample_rate
            );
    for (int i = 0; i < estNrObjects; i++)
    {
        FGReplayData* r = new FGReplayData;
        if (r)
            self.m_recycler.push_back(r);
        else
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "ReplaySystem: Out of memory!");
        }
    }
}

/* Reset replay queues. */
void
FGReplayInternal::reinit()
{
    m_sim_time = 0.0;
    m_last_mt_time = 0.0;
    m_last_lt_time = 0.0;
    m_last_msg_time = 0.0;

    // Flush queues
    clear(*this);
    m_flight_recorder->reinit();

    m_high_res_time   = fgGetDouble("/sim/replay/buffer/high-res-time",    60.0);
    m_medium_res_time = fgGetDouble("/sim/replay/buffer/medium-res-time", 600.0); // 10 mins
    m_low_res_time    = fgGetDouble("/sim/replay/buffer/low-res-time",   3600.0); // 1 h
    // short term sample rate is as every frame
    m_medium_sample_rate = fgGetDouble("/sim/replay/buffer/medium-res-sample-dt", 0.5); // medium term sample rate (sec)
    m_long_sample_rate   = fgGetDouble("/sim/replay/buffer/low-res-sample-dt",    5.0); // long term sample rate (sec)

    fillRecycler(*this);
    loadMessages(*this);

    m_replay_master->setIntValue(0);
    m_disable_replay->setBoolValue(0);
    m_replay_time->setDoubleValue(0);
    m_replay_time_str->setStringValue("");
}

/* Bind to the property tree */
void
FGReplayInternal::bind()
{
}

/*  Unbind from the property tree */
void
FGReplayInternal::unbind()
{
    // nothing to unbind
}

/* Returns string such as HH:MM:SS. */
static void
printTimeStr(char* pStrBuffer,double _Time, bool ShowDecimal=true)
{
    if (_Time<0)
        _Time = 0;
    unsigned int Time = _Time*10;
    unsigned int h = Time/36000;
    unsigned int m = (Time % 36000)/600;
    unsigned int s = (Time % 600)/10;
    unsigned int d = Time % 10;

    int len;
    if (h>0)
        len = sprintf(pStrBuffer,"%u:%02u:%02u",h,m,s);
    else
        len = sprintf(pStrBuffer,"%u:%02u",m,s);

    if (len < 0)
    {
        *pStrBuffer = 0;
        return;
    }

    if (ShowDecimal)
        sprintf(&pStrBuffer[len],".%u",d);
}

/* Sets specified property to representation of the specified time. */
static void setTimeStr(const char* property_path, double t, bool show_decimal=false)
{
    char buffer[30];
    printTimeStr(buffer, t, show_decimal);
    fgSetString(property_path, buffer);
}

/* Sets /sim/messages/copilot to specified message. */
static void guiMessage(const char* message)
{
    fgSetString("/sim/messages/copilot", message);
}

double get_start_time(FGReplayInternal& self)
{
    double ret;
    std::lock_guard<std::mutex> lock(self.m_continuous->m_in_time_to_frameinfo_lock);
    if (!self.m_continuous->m_in_time_to_frameinfo.empty())
    {
        ret = self.m_continuous->m_in_time_to_frameinfo.begin()->first;
        SG_LOG(SG_SYSTEMS, SG_DEBUG,
                "ret=" << ret
                << " m_in_time_to_frameinfo is "
                << self.m_continuous->m_in_time_to_frameinfo.begin()->first
                << ".."
                << self.m_continuous->m_in_time_to_frameinfo.rbegin()->first
                );
        // We don't set /sim/replay/end-time here - it is updated when indexing
        // in the background.
        return ret;
    }
    
    if (!self.m_long_term.empty())          ret = self.m_long_term.front()->sim_time;
    else if (!self.m_medium_term.empty())   ret = self.m_medium_term.front()->sim_time;
    else if (!self.m_short_term.empty())    ret = self.m_short_term.front()->sim_time;
    else                                    ret = 0.0;
    fgSetDouble("/sim/replay/start-time", ret);
    setTimeStr("/sim/replay/start-time-str", ret);
    return ret;
}

double get_end_time(FGReplayInternal& self)
{
    std::lock_guard<std::mutex> lock(self.m_continuous->m_in_time_to_frameinfo_lock);
    if (!self.m_continuous->m_in_time_to_frameinfo.empty())
    {
        double ret = self.m_continuous->m_in_time_to_frameinfo.rbegin()->first;
        SG_LOG(SG_SYSTEMS, SG_DEBUG,
                "ret=" << ret
                << " m_in_time_to_frameinfo is "
                << self.m_continuous->m_in_time_to_frameinfo.begin()->first
                << ".."
                << self.m_continuous->m_in_time_to_frameinfo.rbegin()->first
                );
        // We don't set /sim/replay/end-time here - it is updated when indexing
        // in the background.
        return ret;
    }
    double ret = self.m_short_term.empty() ? 0 : self.m_short_term.back()->sim_time;
    fgSetDouble("/sim/replay/end-time", ret);
    setTimeStr("/sim/replay/end-time-str", ret);
    return ret;
}

/** Start replay session
 */
bool
FGReplayInternal::start(bool new_tape)
{
    // freeze the fdm, resume from sim pause
    if (m_log_frame_times->getBoolValue())
    {
        // Clear existing log.
        m_log_frame_times->removeAllChildren();
    }
    double StartTime = get_start_time(*this);
    double EndTime = get_end_time(*this);
    m_was_finished_already = false;
    char StrBuffer[30];
    printTimeStr(StrBuffer, StartTime, false);
    fgSetString("/sim/replay/start-time-str", StrBuffer);
    printTimeStr(StrBuffer, EndTime, false);
    fgSetString("/sim/replay/end-time-str",   StrBuffer);

    unsigned long buffer_elements =  m_short_term.size()+m_medium_term.size()+m_long_term.size();
    fgSetDouble("/sim/replay/buffer-size-mbyte",
                buffer_elements*m_flight_recorder->getRecordSize() / (1024*1024.0));
    if ( fgGetBool("/sim/freeze/master") || !m_replay_master->getIntValue())
    {
        guiMessage("Replay active. 'Esc' to stop.");
    }
    fgSetBool("/sim/freeze/master", 0);
    fgSetBool("/sim/freeze/clock", 0);
    if (!m_replay_master->getIntValue())
    {
        m_replay_master->setIntValue(1);
        m_replay_master_eof->setIntValue(0);
        if (new_tape)
        {
            // start replay at initial time, when loading a new tape
            m_replay_time->setDoubleValue(StartTime);
        }
        else
        {
            // start replay at "loop interval" when starting instant replay
            m_replay_time->setDoubleValue(-1);
        }
        m_replay_time_str->setStringValue("");
    }
    loadMessages(*this);

    m_MultiplayMgr->ClearMotion();

    return true;
}

static const char* MultiplayerMessageCallsign(const std::vector<char>& raw_message)
{
    const T_MsgHdr* header = reinterpret_cast<const T_MsgHdr*>(&raw_message.front());
    return header->Callsign;
}

static bool CallsignsEqual(const std::vector<char>& a_message, const std::vector<char>& b_message)
{
    const char* a_callsign = MultiplayerMessageCallsign(a_message);
    const char* b_callsign = MultiplayerMessageCallsign(b_message);
    return 0 == strncmp(a_callsign, b_callsign, MAX_CALLSIGN_LEN);
}

/* Moves all multiplayer packets from first item in <list> to second item in
<list>, but only for callsigns that are not already mentioned in the second
item's list.

To be called before first item is discarded, so that we don't end up with large
gaps in multiplayer information when replaying - this can cause multiplayer
aircraft to spuriously disappear and reappear. */
static void MoveFrontMultiplayerPackets(std::deque<FGReplayData*>& list)
{
    auto it = list.begin();
    FGReplayData* a = *it;
    ++it;
    if (it == list.end())
    {
        return;
    }
    FGReplayData* b = *it;

    // Copy all multiplayer packets in <a> that are for multiplayer aircraft
    // that are not in <b>, into <b>'s multiplayer_messages.
    //
    for (auto a_message: a->multiplayer_messages)
    {
        bool found = false;
        for (auto b_message: b->multiplayer_messages)
        {
            if (CallsignsEqual(*a_message, *b_message))
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            b->multiplayer_messages.push_back(a_message);
        }
    }
}


/** Save raw replay data in a separate container */
static bool saveRawReplayData(
        simgear::gzContainerWriter& output,
        const std::deque<FGReplayData*>& replay_data,
        size_t record_size,
        SGPropertyNode* meta
        )
{
    // get number of records in this stream
    size_t count = replay_data.size();

    // write container header for raw data
    if (!output.writeContainerHeader(ReplayContainer::RawData, count * record_size))
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to save replay data."
                << " Cannot write data container. Disk full?"
                );
        return false;
    }

    // read the raw data (all records in the given list)
    auto it = replay_data.begin();
    size_t check_count = 0;
    while (it != replay_data.end() && !output.fail())
    {
        const FGReplayData* frame = *it++;
        assert(record_size == frame->raw_data.size());
        writeRaw(output, frame->sim_time);
        output.write(&frame->raw_data.front(), frame->raw_data.size());

        for (auto data: meta->getNode("meta")->getChildren("data"))
        {
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "data->getStringValue()=" << data->getStringValue());
            if (data->getStringValue() == "multiplayer")
            {
                uint32_t length = 0;
                for (auto message: frame->multiplayer_messages)
                {
                    length += sizeof(uint16_t) + message->size();
                }
                writeRaw(output, length);
                for (auto message: frame->multiplayer_messages)
                {
                    uint16_t message_size = message->size();
                    writeRaw(output, message_size);
                    output.write(&message->front(), message_size);
                }
                break;
            }
        }
        check_count++;
    }

    // Did we really write as much as we intended?
    if (check_count != count)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to save replay data."
                " Expected to write " << count << " records"
                << ", but wrote " << check_count);
        return false;
    }

    SG_LOG(SG_SYSTEMS, SG_DEBUG, "Saved " << check_count << " records"
            << " of size " << record_size);
    return !output.fail();
}


SGPropertyNode_ptr saveSetup(
        const SGPropertyNode*   extra,
        const SGPath&           path,
        double                  duration,
        FGTapeType              tape_type,
        int                     continuous_compression
        )
{
    SGPropertyNode_ptr  config;
    if (path.exists())
    {
        // same timestamp!?
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Error, flight recorder tape file with same name already exists: " << path);
        return config;
    }
    
    config = new SGPropertyNode;
    SGPropertyNode* meta = config->getNode("meta", 0 /*index*/, true /*create*/);

    // add some data to the file - so we know for which aircraft/version it was recorded
    meta->setStringValue("aircraft-type",           fgGetString("/sim/aircraft", "unknown"));
    meta->setStringValue("aircraft-description",    fgGetString("/sim/description", ""));
    meta->setStringValue("aircraft-fdm",            fgGetString("/sim/flight-model", ""));
    meta->setStringValue("closest-airport-id",      fgGetString("/sim/airport/closest-airport-id", ""));
    meta->setStringValue("aircraft-version",        fgGetString("/sim/aircraft-version", "(undefined)"));
    if (tape_type == FGTapeType_CONTINUOUS)
    {
        meta->setIntValue("continuous-compression", continuous_compression);
    }
    // add information on the tape's recording duration
    meta->setDoubleValue("tape-duration", duration);
    char buffer[30];
    printTimeStr(buffer, duration, false);
    meta->setStringValue("tape-duration-str", buffer);

    // add simulator version
    copyProperties(fgGetNode("/sim/version", 0, true), meta->getNode("version", 0, true));
    if (extra && extra->getNode("user-data"))
    {
        copyProperties(extra->getNode("user-data"), meta->getNode("user-data", 0, true));
    }

    if (tape_type != FGTapeType_CONTINUOUS || fgGetBool("/sim/replay/record-signals", true))
    {
        config->addChild("data")->setStringValue("signals");
    }
    
    if (tape_type == FGTapeType_CONTINUOUS)
    {
        if (fgGetBool("/sim/replay/record-multiplayer", false))
        {
            config->addChild("data")->setStringValue("multiplayer");
        }
        if (0
                || fgGetBool("/sim/replay/record-extra-properties", false)
                || fgGetBool("/sim/replay/record-main-window", false)
                || fgGetBool("/sim/replay/record-main-view", false)
                )
        {
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "Adding data[]=extra-properties."
                    << " record-extra-properties=" << fgGetBool("/sim/replay/record-extra-properties", false)
                    << " record-main-window=" << fgGetBool("/sim/replay/record-main-window", false)
                    << " record-main-view=" << fgGetBool("/sim/replay/record-main-view", false)
                    );
            config->addChild("data")->setStringValue("extra-properties");
        }
    }

    // store replay messages
    copyProperties(fgGetNode("/sim/replay/messages", 0, true), meta->getNode("messages", 0, true));

    return config;
}

std::ostream& operator << (std::ostream& out, const FGFrameInfo& frame_info)
{
    return out << "{"
            << " offset=" << frame_info.offset
            << " has_multiplayer=" << frame_info.has_multiplayer
            << " has_extra_properties=" << frame_info.has_extra_properties
            << "}";
}

static void replayMessage(FGReplayInternal& self, double time)
{
    if (time < self.m_last_msg_time)
    {
        self.m_current_msg = self.m_replay_messages.begin();
    }

    // check if messages have to be triggered
    while ( self.m_current_msg != self.m_replay_messages.end()
            && time >= self.m_current_msg->sim_time
            )
    {
        // don't trigger messages when too long ago (fast-forward/skipped replay)
        if (time - self.m_current_msg->sim_time < 3.0)
        {
            fgGetNode("/sim/messages", true)
                    ->getNode(self.m_current_msg->speaker, true)
                    ->setStringValue(self.m_current_msg->message);
        }
        ++self.m_current_msg;
    }
    self.m_last_msg_time = time;
}

/** 
 * given two FGReplayData elements and a time, interpolate between them
 */
static void replayNormal2(
        FGReplayInternal& self,
        double time,
        FGReplayData* current_frame,
        FGReplayData* old_frame=nullptr,
        int* xpos=nullptr,
        int* ypos=nullptr,
        int* xsize=nullptr,
        int* ysize=nullptr
        )
{
    self.m_flight_recorder->replay(time, current_frame, old_frame, xpos, ypos, xsize, ysize);
}

/** 
 * interpolate a specific time from a specific list
 */
static void interpolate( FGReplayInternal& self, double time, const std::deque<FGReplayData*>& list)
{
    // sanity checking
    if (list.empty())
    {
        return;
    }
    else if (list.size() == 1)
    {
        replayNormal2(self, time, list[0]);
        return;
    }

    unsigned int last = list.size() - 1;
    unsigned int first = 0;
    unsigned int mid = ( last + first ) / 2;

    bool done = false;
    while (!done)
    {
        // cout << "  " << first << " <=> " << last << endl;
        if ( last == first )
        {
            done = true;
        }
        else if (list[mid]->sim_time < time && list[mid+1]->sim_time < time)
        {
            // too low
            first = mid;
            mid = ( last + first ) / 2;
        }
        else if (list[mid]->sim_time > time && list[mid+1]->sim_time > time)
        {
            // too high
            last = mid;
            mid = ( last + first ) / 2;
        }
        else
        {
            done = true;
        }
    }

    replayNormal2(self, time, list[mid+1], list[mid]);
}

bool replayNormal(FGReplayInternal& self, double time)
{
    if ( !self.m_short_term.empty() )
    {
        // We use /sim/replay/end-time instead of m_short_term.back()->sim_time
        // because if we are recording multiplayer aircraft, new items will
        // be appended to m_short_term while we replay, and we don't want to
        // continue replaying into them. /sim/replay/end-time will remain
        // pointing to the last frame at the time we started replaying.
        //
        double t1 = fgGetDouble("/sim/replay/end-time");
        double t2 = self.m_short_term.front()->sim_time;
        if ( time > t1 )
        {
            // replay the most recent frame
            replayNormal2( self, time, self.m_short_term.back() );
            // replay is finished now
            return true;
        }
        else if ( time <= t1 && time >= t2 )
        {
            interpolate( self, time, self.m_short_term );
        }
        else if ( ! self.m_medium_term.empty() )
        {
            t1 = self.m_short_term.front()->sim_time;
            t2 = self.m_medium_term.back()->sim_time;
            if ( time <= t1 && time >= t2 )
            {
                replayNormal2(self, time, self.m_medium_term.back(), self.m_short_term.front());
            }
            else
            {
                t1 = self.m_medium_term.back()->sim_time;
                t2 = self.m_medium_term.front()->sim_time;
                if ( time <= t1 && time >= t2 )
                {
                    interpolate( self, time, self.m_medium_term );
                }
                else if ( ! self.m_long_term.empty() )
                {
                    t1 = self.m_medium_term.front()->sim_time;
                    t2 = self.m_long_term.back()->sim_time;
                    if ( time <= t1 && time >= t2 )
                    {
                        replayNormal2(self, time, self.m_long_term.back(), self.m_medium_term.front());
                    }
                    else
                    {
                        t1 = self.m_long_term.back()->sim_time;
                        t2 = self.m_long_term.front()->sim_time;
                        if ( time <= t1 && time >= t2 )
                        {
                            interpolate( self, time, self.m_long_term );
                        }
                        else
                        {
                            // replay the oldest long term frame
                            replayNormal2(self, time, self.m_long_term.front());
                        }
                    }
                }
                else
                {
                    // replay the oldest medium term frame
                    replayNormal2(self, time, self.m_medium_term.front());
                }
            }
        }
        else
        {
            // replay the oldest short term frame
            replayNormal2(self, time, self.m_short_term.front());
        }
    }
    else
    {
        // nothing to replay
        return true;
    }
    return false;
}

/** 
 *  Replay a saved frame based on time, interpolate from the two
 *  nearest saved frames.
 *  Returns true when replay sequence has finished, false otherwise.
 */
bool replay(FGReplayInternal& self, double time)
{
    // cout << "replay: " << time << " ";
    // find the two frames to interpolate between
    replayMessage(self, time);
    
    std::lock_guard<std::mutex> lock(self.m_continuous->m_in_time_to_frameinfo_lock);
    
    if (!self.m_continuous->m_in_time_to_frameinfo.empty())
    {
        // We are replaying a continuous recording.
        //
        return replayContinuous(self, time);
    }
    else
    {
        return replayNormal(self, time);
    }
}


static FGReplayData* record(FGReplayInternal& self, double sim_time)
{
    FGReplayData* r = NULL;

    if (!self.m_recycler.empty())
    {
        r = self.m_recycler.front();
        self.m_recycler.pop_front();
    }

    return self.m_flight_recorder->capture(sim_time, r);
}

void
FGReplayInternal::update( double dt )
{
    int current_replay_state = m_last_replay_state;
    
    if ( m_disable_replay->getBoolValue() )
    {
        if (fgGetBool("/sim/freeze/master") || fgGetBool("/sim/freeze/clock"))
        {
            // unpause - disable the replay system in next loop
            fgSetBool("/sim/freeze/master", false);
            fgSetBool("/sim/freeze/clock", false);
            m_last_replay_state = 1;
        }
        else if (m_replay_master->getIntValue() != 3 || m_last_replay_state == 3)
        {
            // disable the replay system
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "End replay");
            current_replay_state = m_replay_master->getIntValue();
            m_replay_master->setIntValue(0);
            m_replay_time->setDoubleValue(0);
            m_replay_time_str->setStringValue("");
            m_disable_replay->setBoolValue(0);
            m_speed_up->setDoubleValue(1.0);
            m_speed_up->setDoubleValue(1.0);
            if (fgGetBool("/sim/replay/mute"))
            {
                fgSetBool("/sim/sound/enabled", true);
                fgSetBool("/sim/replay/mute", false);
            }

            // Close any continuous replay file that we have open.
            //
            // This allows the user to use the in-memory record/replay system,
            // instead of replay always showing the continuous recording.
            //
            if (m_continuous->m_in.is_open())
            {
                SG_LOG(SG_SYSTEMS, SG_DEBUG, "Unloading continuous recording");
                m_continuous->m_in.close();
                m_continuous->m_in_time_to_frameinfo.clear();
            }
            assert(m_continuous->m_in_time_to_frameinfo.empty());

            continuous_replay_video_end(*m_continuous);
            guiMessage("Replay stopped. Your controls!");
        }
    }

    int replay_state = m_replay_master->getIntValue();
    if (replay_state == 0 && m_last_replay_state > 0)
    {
        if (current_replay_state == 3)
        {
            // take control at current replay position ("My controls!").
            // May need to uncrash the aircraft here :)
            fgSetBool("/sim/crashed", false);
        }
        else
        {
            // normal replay exit, restore most recent frame
            replay(*this, DBL_MAX);
        }

        // replay is finished
        m_last_replay_state = replay_state;
        return;
    }

    // remember recent state
    m_last_replay_state = replay_state;

    if (replay_state == 1)  // normal replay
    {
        if (m_log_frame_times->getBoolValue())
        {
            m_log_frame_times->addChild("dt")->setDoubleValue(dt);
        }
    }

    switch(replay_state)
    {
        case 0:
            // replay inactive, keep recording
            break;
        case 1: // normal replay
        case 3: // prepare to resume normal flight at current replay position 
        {
            // replay active
            double current_time = m_replay_time->getDoubleValue();
            bool reset_time = (current_time <= 0.0);
            if (reset_time)
            {
                // initialize start time
                double startTime = get_start_time(*this);
                double endTime = get_end_time(*this);
                double duration = 0;
                if (m_replay_duration_act->getBoolValue())
                    duration = fgGetDouble("/sim/replay/duration");
                if (duration && duration < (endTime - startTime))
                {
                    current_time = endTime - duration;
                }
                else
                {
                    current_time = startTime;
                }
            }
            if (current_time != m_replay_time_prev)
            {
                // User has skipped backwards or forward. Reset list of multiplayer aircraft.
                //
                m_MultiplayMgr->ClearMotion();
            }
            current_time += dt;
            SG_LOG(SG_GENERAL, SG_BULK, "current_time=" << std::fixed << std::setprecision(6) << current_time);
            
            bool is_finished = replay(*this, current_time);
            if (is_finished)
            {
                if (!m_was_finished_already)
                {
                    m_replay_master_eof->setIntValue(1);
                    guiMessage("End of tape. 'Esc' to return.");
                    m_was_finished_already = true;
                }
                current_time = (m_replay_looped->getBoolValue()) ? -1 : get_end_time(*this) + 0.01;
            }
            else
            {
                m_was_finished_already = false;
            }
            m_replay_time->setDoubleValue(current_time);
            m_replay_time_prev = current_time;
            char StrBuffer[30];
            printTimeStr(StrBuffer,current_time);
            m_replay_time_str->setStringValue((const char*)StrBuffer);

            // when time skipped (looped replay), trigger listeners to reset views etc
            if (reset_time)
                m_replay_master->setIntValue(replay_state);

            if (m_replay_multiplayer->getIntValue())
            {
                // Carry on recording while replaying.
                break;
            }
            else
            {
                // Don't record while replaying.
                return;
            }
        }
        case 2: // normal replay operation
            if (m_replay_multiplayer->getIntValue())
            {
                // Carry on recording while replaying.
                break;
            }
            else
            {
                // Don't record while replaying.
                return;
            }
        default:
            throw sg_range_exception("unknown FGReplayInternal state");
    }

    // flight recording

    // sanity check, don't collect data if FDM data isn't good
    if (!fgGetBool("/sim/fdm-initialized", false) || dt == 0.0)
        return;

    if (m_simple_time_enabled->getBoolValue())
    {
        m_sim_time = globals->get_subsystem<TimeManager>()->getMPProtocolClockSec();
    }
    else
    {
        double new_sim_time = m_sim_time + dt;
        // don't record multiple records with the same timestamp (or go backwards in time)
        if (new_sim_time <= m_sim_time)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "ReplaySystem: Time warp detected!");
            return;
        }
        m_sim_time = new_sim_time;
    }

    // We still record even if we are replaying, if /sim/replay/multiplayer is
    // true.
    //
    FGReplayData* r = record(*this, m_sim_time);
    if (!r)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "ReplaySystem: Out of memory!");
        return;
    }

    // update the short term list
    assert(r->raw_data.size() != 0);
    m_short_term.push_back( r );
    FGReplayData *st_front = m_short_term.front();
    
    m_record_normal_end->setDoubleValue(r->sim_time);
    if (m_record_normal_begin->getDoubleValue() == 0)
    {
        m_record_normal_begin->setDoubleValue(r->sim_time);
    }

    if (!st_front)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "ReplaySystem: Inconsistent data!");
    }
    
    if (m_continuous->m_out.is_open())
    {
        continuousWriteFrame(*m_continuous, r, m_continuous->m_out, m_continuous->m_out_config);
    }
    
    if (replay_state == 0)
    {
        // Update recovery tape.
        //
        double  recovery_period_s = m_recovery_period->getDoubleValue();
        if (recovery_period_s > 0)
        {
        
            static time_t   s_last_recovery = 0;
            time_t t = time(NULL);
            if (s_last_recovery == 0)
            {
                /* Don't save immediately. */
                s_last_recovery = t;
            }
            
            // Write recovery recording periodically.
            //
            if (t - s_last_recovery >= recovery_period_s)
            {
                s_last_recovery = t;

                // We use static variables here to avoid calculating the same
                // data each time we are called.
                //
                static SGPath               path = makeSavePath(FGTapeType_RECOVERY);
                static SGPath               path_temp = SGPath( path.str() + "-");
                static SGPropertyNode_ptr   Config;

                SG_LOG(SG_SYSTEMS, SG_BULK, "Creating recovery file: " << path);
                // We write to <path_temp> then rename to <path>, which should
                // guarantee that there is always a valid recovery tape even if
                // flightgear crashes or is killed while we are writing.
                //
                path.create_dir();
                (void) remove(path_temp.c_str());
                std::ofstream   out;
                bool ok = true;
                SGPropertyNode_ptr config = continuousWriteHeader(
                        *m_continuous,
                        m_flight_recorder.get(),
                        out,
                        path_temp,
                        FGTapeType_RECOVERY
                        );
                if (!config) ok = false;
                if (ok) ok = continuousWriteFrame(*m_continuous, r, out, config);
                out.close();
                if (ok)
                {
                    rename(path_temp.c_str(), path.c_str());
                }
                else
                {
                    std::string message = "Failed to update recovery snapshot file '" + path.str() + "';"
                            + " See File / Flight Recorder Control / 'Maintain recovery snapshot'."
                            ;
                    popupTip(message.c_str(), 10 /*delay*/);
                }
            }
        }
    }

    if ( m_sim_time - st_front->sim_time > m_high_res_time )
    {
        while ( !m_short_term.empty() && m_sim_time - st_front->sim_time > m_high_res_time )
        {
            st_front = m_short_term.front();
            MoveFrontMultiplayerPackets(m_short_term);
            m_recycler.push_back(st_front);
            m_short_term.pop_front();
        }

        // update the medium term list
        if ( m_sim_time - m_last_mt_time > m_medium_sample_rate )
        {
            m_last_mt_time = m_sim_time;
            if (!m_short_term.empty())
            {
                st_front = m_short_term.front();
                m_medium_term.push_back( st_front );
                m_short_term.pop_front();
            }

            if (!m_medium_term.empty())
            {
                FGReplayData *mt_front = m_medium_term.front();
                if ( m_sim_time - mt_front->sim_time > m_medium_res_time )
                {
                    while ( !m_medium_term.empty() && m_sim_time - mt_front->sim_time > m_medium_res_time )
                    {
                        mt_front = m_medium_term.front();
                        MoveFrontMultiplayerPackets(m_medium_term);
                        m_recycler.push_back(mt_front);
                        m_medium_term.pop_front();
                    }
                    // update the long term list
                    if ( m_sim_time - m_last_lt_time > m_long_sample_rate )
                    {
                        m_last_lt_time = m_sim_time;
                        if (!m_medium_term.empty())
                        {
                            mt_front = m_medium_term.front();
                            m_long_term.push_back( mt_front );
                            m_medium_term.pop_front();
                        }

                        if (!m_long_term.empty())
                        {
                            FGReplayData *lt_front = m_long_term.front();
                            if ( m_sim_time - lt_front->sim_time > m_low_res_time )
                            {
                                while ( !m_long_term.empty() && m_sim_time - lt_front->sim_time > m_low_res_time )
                                {
                                    lt_front = m_long_term.front();
                                    MoveFrontMultiplayerPackets(m_long_term);
                                    m_recycler.push_back(lt_front);
                                    m_long_term.pop_front();
                                }
                            }
                        }
                    }
                    
                    
                }
            }
        }
    }

#if 0
    cout << "short term size = " << m_short_term.size()
         << "  time = " << m_sim_time - m_short_term.front().sim_time
         << endl;
    cout << "medium term size = " << m_medium_term.size()
         << "  time = " << m_sim_time - m_medium_term.front().sim_time
         << endl;
    cout << "long term size = " << m_long_term.size()
         << "  time = " << m_sim_time - m_long_term.front().sim_time
         << endl;
#endif
   //stamp("point_finished");
}

/** Load raw replay data from a separate container */
static bool
loadRawReplayData(
        simgear::gzContainerReader& input,
        //FGFlightRecorder* pRecorder,
        std::deque<FGReplayData*>& replay_data,
        size_t record_size,
        bool multiplayer
        )
{
    size_t size = 0;
    simgear::ContainerType Type = ReplayContainer::Invalid;

    // write container header for raw data
    if (!input.readContainerHeader(&Type, &size))
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to load replay data. Missing data container.");
        return false;
    }
    else
    if (Type != ReplayContainer::RawData)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to load replay data. Expected data container, got " << Type);
        return false;
    }
    
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "multiplayer=" << multiplayer << " record_size=" << record_size << " Type=" << Type << " ize=" << size);

    // read the raw data
    size_t count = size / record_size;
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "Loading replay data. Container size is " << size << ", record size " << record_size <<
           ", expected record count " << count << ".");

    size_t check_count = 0;
    for (check_count=0; (check_count < count)&&(!input.eof()); ++check_count)
    {
        FGReplayData* buffer = new FGReplayData;
        readRaw(input, buffer->sim_time);
        buffer->raw_data.resize(record_size);
        input.read(&buffer->raw_data.front(), record_size);
        replay_data.push_back(buffer);

        if (multiplayer)
        {
            uint32_t    length;
            readRaw(input, length);
            uint32_t    pos = 0;
            for(;;)
            {
                if (pos == length) break;
                assert(pos < length);
                uint16_t message_size;
                readRaw(input, message_size);
                pos += sizeof(message_size) + message_size;
                if (pos > length)
                {
                    SG_LOG(SG_SYSTEMS, SG_ALERT, "Tape appears to have corrupted multiplayer data");
                    return false;
                }
                auto message = std::make_shared<std::vector<char>>(message_size);
                //std::shared_ptr<std::vector<char>>  message(new std::vector<char>(message_size));
                input.read(&message->front(), message_size);
                buffer->multiplayer_messages.push_back( message);
            }
        }
    }

    // did we get all we have hoped for?
    if (check_count != count)
    {
        if (input.eof())
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "Unexpected end of file.");
        }
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to load replay data. Expected " << count << " records, but got " << check_count);
        return false;
    }

    SG_LOG(SG_SYSTEMS, SG_DEBUG, "Loaded " << check_count << " records of size " << record_size);
    return true;
}

/** Write flight recorder tape with given filename and meta properties to disk */
static bool saveTape2(FGReplayInternal& self, const SGPath& filename, SGPropertyNode_ptr metadata)
{
    bool ok = true;

    /* open output stream *******************************************/
    simgear::gzContainerWriter output(filename, FlightRecorderFileMagic);
    
    if (!output.good())
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Cannot open file" << filename);
        return false;
    }

    SG_LOG(SG_SYSTEMS, SG_DEBUG, "writing MetaData:");
    /* write meta data **********************************************/
    ok &= output.writeContainer(ReplayContainer::MetaData, metadata.get());

    /* write flight recorder configuration **************************/
    SGPropertyNode_ptr config;
    if (ok)
    {
        config = new SGPropertyNode();
        self.m_flight_recorder->getConfig(config.get());
        ok &= output.writeContainer(ReplayContainer::Properties, config.get());
    }
    
    /* write raw data ***********************************************/
    if (config)
    {
        size_t record_size = config->getIntValue("recorder/record-size", 0);
        SG_LOG(SG_SYSTEMS, SG_DEBUG, ""
                << "Config:recorder/signal-count=" <<  config->getIntValue("recorder/signal-count", 0)
                << " RecordSize: " << record_size
                );
        if (ok)
            ok &= saveRawReplayData(output, self.m_short_term,  record_size, metadata);
        if (ok)
            ok &= saveRawReplayData(output, self.m_medium_term, record_size, metadata);
        if (ok)
            ok &= saveRawReplayData(output, self.m_long_term,   record_size, metadata);
        config = 0;
    }

    /* done *********************************************************/
    output.close();

    if (!ok) SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to write to file " << filename);
    return ok;
}

/** Write flight recorder tape to disk. User/script command. */
bool
FGReplayInternal::saveTape(const SGPropertyNode* extra)
{
    SGPath path_timeless;
    SGPath path = makeSavePath(FGTapeType_NORMAL, &path_timeless);
    SGPropertyNode_ptr metadata = saveSetup(
            extra,
            path,
            get_end_time(*this) - get_start_time(*this),
            FGTapeType_NORMAL
            );
    bool ok = false;
    if (metadata)
    {
        ok = saveTape2(*this, path, metadata);
        if (ok)
        {
            // Make a convenience link to the recording. E.g.
            // harrier-gr3.fgtape ->
            // harrier-gr3-20201224-005034.fgtape.
            //
            // Link destination is in same directory as link so we use leafname
            // path.file().
            //
            path_timeless.remove();
            ok = path_timeless.makeLink(path.file());
            if (!ok)
            {
                SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to create link " << path_timeless.c_str() << " => " << path.file());
            }
        }
    }
    if (ok)
        guiMessage("Flight recorder tape saved successfully!");
    else
        guiMessage("Failed to save tape! See log output.");

    return ok;
}


int loadContinuousHeader(const std::string& path, std::istream* in, SGPropertyNode* properties)
{
    std::ifstream   in0;
    if (!in)
    {
        in0.open(path);
        in = &in0;
    }
    if (!*in)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Failed to open path=" << path);
        return +1;
    }
    std::vector<char>   buffer(strlen( FlightRecorderFileMagic) + 1);
    in->read(&buffer.front(), buffer.size());
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "in->gcount()=" << in->gcount() << " buffer.size()=" << buffer.size());
    if ((size_t) in->gcount() != buffer.size())
    {
        // Further download is needed.
        return +1;
    }
    if (strcmp(&buffer.front(), FlightRecorderFileMagic))
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "fgtape prefix doesn't match FlightRecorderFileMagic in path: " << path);
        return -1;
    }
    bool ok = false;
    try
    {
        PropertiesRead(*in, properties);
        ok = true;
    }
    catch (std::exception& e)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Failed to read Config properties in: " << path << ": " << e.what());
    }
    if (!ok)
    {
        // Failed to read properties, so indicate that further download is needed.
        return +1;
    }
    SG_LOG(SG_SYSTEMS, SG_BULK, "properties is:\n"
            << writePropertiesInline(properties, true /*write_all*/) << "\n");
    return 0;
}

// Build up in-memory cache of simulator time to file offset, so we can handle
// random access.
//
// We also cache any frames that modify extra-properties.
//
// Can be called multiple times, e.g. if recording is being downlaoded.
//
static void indexContinuousRecording(FGReplayInternal& self, const void* data, size_t numbytes)
{
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "Indexing Continuous recording "
            << " data=" << data
            << " numbytes=" << numbytes
            << " m_indexing_pos=" << self.m_continuous->m_indexing_pos
            << " m_in_compression=" << self.m_continuous->m_in_compression
            << " m_in_time_to_frameinfo.size()=" << self.m_continuous->m_in_time_to_frameinfo.size()
            );
    time_t t0 = time(NULL);
    std::streampos original_pos = self.m_continuous->m_indexing_pos;
    size_t original_num_frames = self.m_continuous->m_in_time_to_frameinfo.size();
    
    // Reset any EOF because there might be new data.
    self.m_continuous->m_indexing_in.clear();
    
    struct data_stats_t
    {
        size_t  num_frames = 0;
        size_t  bytes = 0;
    };
    std::map<std::string, data_stats_t> stats;
    
    for(;;)
    {
        SG_LOG(SG_SYSTEMS, SG_BULK, "reading frame."
                << " m_in.tellg()=" << self.m_continuous->m_in.tellg()
                );
        self.m_continuous->m_indexing_in.seekg(self.m_continuous->m_indexing_pos);
        double sim_time;
        readRaw(self.m_continuous->m_indexing_in, sim_time);

        SG_LOG(SG_SYSTEMS, SG_BULK, ""
                << " m_indexing_pos=" << self.m_continuous->m_indexing_pos
                << " m_indexing_in.tellg()=" << self.m_continuous->m_indexing_in.tellg()
                << " sim_time=" << sim_time
                );
        
        FGFrameInfo frameinfo;
        frameinfo.offset = self.m_continuous->m_indexing_pos;
        if (self.m_continuous->m_in_compression)
        {
            // Skip compressed frame data without decompressing it.
            uint8_t    flags;
            self.m_continuous->m_indexing_in.read((char*) &flags, sizeof(flags));
            frameinfo.has_signals = flags & 1;
            frameinfo.has_multiplayer = flags & 2;
            frameinfo.has_extra_properties = flags & 4;
            
            if (frameinfo.has_signals)
            {
                stats["signals"].num_frames += 1;
            }
            if (frameinfo.has_multiplayer)
            {
                stats["multiplayer"].num_frames += 1;
                ++self.m_continuous->m_num_frames_multiplayer;
                self.m_continuous->m_in_multiplayer = true;
            }
            if (frameinfo.has_extra_properties)
            {
                stats["extra-properties"].num_frames += 1;
                ++self.m_continuous->m_num_frames_extra_properties;
                self.m_continuous->m_in_extra_properties = true;
            }
            
            uint32_t    compressed_size;
            readRaw(self.m_continuous->m_indexing_in, compressed_size);
            SG_LOG(SG_SYSTEMS, SG_BULK, "compressed_size=" << compressed_size);
            
            self.m_continuous->m_indexing_in.seekg(compressed_size, std::ios_base::cur);
        }
        else
        {
            // Skip frame data.
            auto datas = self.m_continuous->m_in_config->getChildren("data");
            SG_LOG(SG_SYSTEMS, SG_BULK, "datas.size()=" << datas.size());
            for (auto data: datas)
            {
                uint32_t    length;
                readRaw(self.m_continuous->m_indexing_in, length);
                SG_LOG(SG_SYSTEMS, SG_BULK,
                        "m_in.tellg()=" << self.m_continuous->m_indexing_in.tellg()
                        << " Skipping data_type=" << data->getStringValue()
                        << " length=" << length
                        );
                // Move forward <length> bytes.
                self.m_continuous->m_indexing_in.seekg(length, std::ios_base::cur);
                if (!self.m_continuous->m_indexing_in)
                {
                    // Dont add bogus info to <stats>.
                    break;
                }
                if (length)
                {
                    std::string data_type = data->getStringValue();
                    stats[data_type].num_frames += 1;
                    stats[data_type].bytes += length;
                    if (data_type == "signals")
                    {
                        frameinfo.has_signals = true;
                    }
                    else if (data_type == "multiplayer")
                    {
                        frameinfo.has_multiplayer = true;
                        ++self.m_continuous->m_num_frames_multiplayer;
                        self.m_continuous->m_in_multiplayer = true;
                    }
                    else if (data_type == "extra-properties")
                    {
                        frameinfo.has_extra_properties = true;
                        ++self.m_continuous->m_num_frames_extra_properties;
                        self.m_continuous->m_in_extra_properties = true;
                    }
                }
            }
        }
        SG_LOG(SG_SYSTEMS, SG_BULK, ""
                << " pos=" << self.m_continuous->m_indexing_pos
                << " sim_time=" << sim_time
                << " m_num_frames_multiplayer=" << self.m_continuous->m_num_frames_multiplayer
                << " m_num_frames_extra_properties=" << self.m_continuous->m_num_frames_extra_properties
                );

        if (!self.m_continuous->m_indexing_in)
        {
            // Failed to read a frame, e.g. because of EOF. Leave
            // m_indexing_pos unchanged so we can try again at same
            // starting position if recording is upated by background download.
            //
            SG_LOG(SG_SYSTEMS, SG_BULK, "m_indexing_in failed, giving up");
            break;
        }
        
        // We have successfully read a frame, so add it to
        // m_in_time_to_frameinfo[].
        //
        self.m_continuous->m_indexing_pos = self.m_continuous->m_indexing_in.tellg();
        std::lock_guard<std::mutex> lock(self.m_continuous->m_in_time_to_frameinfo_lock);
        self.m_continuous->m_in_time_to_frameinfo[sim_time] = frameinfo;
    }
    time_t t = time(NULL) - t0;
    auto new_bytes = self.m_continuous->m_indexing_pos - original_pos;
    auto num_frames = self.m_continuous->m_in_time_to_frameinfo.size();
    auto num_new_frames = num_frames - original_num_frames;
    if (num_new_frames)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Continuous recording: index updated:"
                << " num_frames=" << num_frames
                << " num_new_frames=" << num_new_frames
                << " new_bytes=" << new_bytes
                );
    }
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "Continuous recording indexing complete."
            << " time taken=" << t << "s."
            << " num_new_frames=" << num_new_frames
            << " m_indexing_pos=" << self.m_continuous->m_indexing_pos
            << " m_in_time_to_frameinfo.size()=" << self.m_continuous->m_in_time_to_frameinfo.size()
            << " m_num_frames_multiplayer=" << self.m_continuous->m_num_frames_multiplayer
            << " m_num_frames_extra_properties=" << self.m_continuous->m_num_frames_extra_properties
            );
    for (auto stat: stats)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "data type " << stat.first << ":"
                << " num_frames=" << stat.second.num_frames
                << " bytes=" << stat.second.bytes
                );
    }
    
    std::lock_guard<std::mutex> lock(self.m_continuous->m_in_time_to_frameinfo_lock);
    fgSetInt("/sim/replay/continuous-stats-num-frames", self.m_continuous->m_in_time_to_frameinfo.size());
    fgSetInt("/sim/replay/continuous-stats-num-frames-extra-properties", self.m_continuous->m_num_frames_extra_properties);
    fgSetInt("/sim/replay/continuous-stats-num-frames-multiplayer", self.m_continuous->m_num_frames_multiplayer);
    if (!self.m_continuous->m_in_time_to_frameinfo.empty())
    {
        double t_begin = self.m_continuous->m_in_time_to_frameinfo.begin()->first;
        double t_end = self.m_continuous->m_in_time_to_frameinfo.rbegin()->first;
        fgSetDouble("/sim/replay/start-time", t_begin);
        fgSetDouble("/sim/replay/end-time", t_end);
        setTimeStr("/sim/replay/start-time-str", t_begin);
        setTimeStr("/sim/replay/end-time-str", t_end);
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Have set /sim/replay/end-time to " << fgGetDouble("/sim/replay/end-time"));
    }
    if (!numbytes)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Continuous recording: indexing finished"
                << " m_in_time_to_frameinfo.size()=" << self.m_continuous->m_in_time_to_frameinfo.size()
                );
        self.m_continuous->m_indexing_in.close();
    }
}


bool loadTapeContinuous(
        FGReplayInternal& replay_internal,
        std::ifstream&  in,
        const SGPath& filename,
        bool preview,
        bool create_video,
        double fixed_dt,
        SGPropertyNode& meta_meta,
        simgear::HTTP::FileRequestRef file_request
        )
{
    auto continuous = replay_internal.m_continuous.get();
    SG_LOG(SG_SYSTEMS, SG_BULK, "m_in_config is:\n"
            << writePropertiesInline(continuous->m_in_config, true /*write_all*/) << "\n");
    copyProperties(continuous->m_in_config->getNode("meta", 0, true), &meta_meta);
    if (preview)
    {
        in.close();
        return true;
    }
    replay_internal.m_flight_recorder->reinit(continuous->m_in_config);
    clear(replay_internal);
    fillRecycler(replay_internal);
    continuous->m_in_time_last = -1;
    continuous->m_in_frame_time_last = -1;
    continuous->m_in_time_to_frameinfo.clear();
    continuous->m_num_frames_extra_properties = 0;
    continuous->m_num_frames_multiplayer = 0;
    continuous->m_indexing_in.open(filename.str());
    continuous->m_indexing_pos = in.tellg();
    continuous->m_in_compression = continuous
            ->m_in_config->getNode("meta/continuous-compression", true /*create*/)
            ->getIntValue()
            ;
    continuous->m_replay_create_video = create_video;
    continuous->m_replay_fixed_dt = fixed_dt;
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "m_in_compression=" << continuous->m_in_compression);
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "filerequest=" << file_request.get());

    // Make an in-memory index of the recording.
    if (file_request)
    {
        auto p_replay_internal = &replay_internal;
        file_request->setCallback(
                [p_replay_internal](const void* data, size_t numbytes)
                {
                    ::indexContinuousRecording(*p_replay_internal, data, numbytes);
                }
                );
    }
    else
    {
        ::indexContinuousRecording(replay_internal, nullptr, 0);
    }

    if (continuous->m_replay_fixed_dt)
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Replaying with fixed_dt=" << continuous->m_replay_fixed_dt);
        continuous->m_replay_fixed_dt_prev = fgGetDouble("/sim/time/fixed-dt");
        fgSetDouble("/sim/time/fixed-dt", continuous->m_replay_fixed_dt);
    }
    else
    {
        continuous->m_replay_fixed_dt_prev = -1;
    }
    
    bool ok = true;
    if (ok && continuous->m_replay_create_video)
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Replaying with create-video");
        auto view_mgr = globals->get_subsystem<FGViewMgr>();
        if (view_mgr)
        {
            ok = view_mgr->video_start(
                    "" /*name*/,
                    "" /*codec*/,
                    -1 /*quality*/,
                    -1 /*speed*/,
                    0 /*bitrate*/
                    );
        }
        else
        {
            SG_LOG(SG_GENERAL, SG_ALERT, "Cannot handle create_video=true because FGViewMgr not available");
            ok = false;
        }
    }
    if (ok) ok = replay_internal.start(true /*new_tape*/);
    return ok;
}


struct NumberCommasHelper : std::numpunct<char>
{
    virtual char do_thousands_sep() const
    {
        return ',';
    }

    virtual std::string do_grouping() const
    {
        return "\03";
    }
};

static std::locale s_comma_locale(std::locale(), new NumberCommasHelper());

/* Returns number as string with commas every 3 digits. */
static std::string numberCommas(size_t n)
{
    std::stringstream buffer;
    buffer.imbue(s_comma_locale);
    buffer << n;
    return buffer.str();
}


/** Read a flight recorder tape with given filename from disk.
 * Copies MetaData's "meta" node into MetaMeta out-param.
 * Actual data and signal configuration is not read when in "Preview" mode.
 *
 * If create_video is true we automatically encode a video while replaying. If fixed_dt
 * is not zero we also set /sim/time/fixed-dt while replaying.
 */
bool
FGReplayInternal::loadTape(
        const SGPath& filename,
        bool preview,
        bool create_video,
        double fixed_dt,
        SGPropertyNode& meta_meta,
        simgear::HTTP::FileRequestRef file_request
        )
{
    SG_LOG(SG_SYSTEMS, SG_DEBUG, "loading Preview=" << preview << " Filename=" << filename);

    /* Try to load a Continuous recording first. */
    m_replay_error->setBoolValue(false);
    std::ifstream   in_preview;
    std::ifstream&  in(preview ? in_preview : m_continuous->m_in);
    in.open(filename.str());
    if (!in)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to open"
                << " Filename=" << filename.str()
                << " in.is_open()=" << in.is_open()
                );
        return false;
    }
    size_t file_size = filename.sizeInBytes();
    meta_meta.setLongValue("tape-size", file_size);
    meta_meta.setStringValue("tape-size-str", numberCommas(file_size));
    m_continuous->m_in_config = new SGPropertyNode;
    int e = loadContinuousHeader(filename.str(), &in, m_continuous->m_in_config);
    if (e == 0)
    {
        return loadTapeContinuous(*this, in, filename, preview, create_video, fixed_dt, meta_meta, file_request);
    }
    
    // Not a continuous recording.
    in.close();
    if (file_request)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Cannot load filename=" << filename
                << " because it is download but not Continuous recording");
        return false;
    }
    bool ok = true;

    /* Open as a gzipped Normal recording. ********************************************/
    simgear::gzContainerReader input(filename, FlightRecorderFileMagic);
    if (input.eof() || !input.good())
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Cannot open file " << filename);
        ok = false;
    }

    SGPropertyNode_ptr meta_data_props = new SGPropertyNode();

    /* read meta data ***********************************************/
    if (ok)
    {
        char* meta_data = NULL;
        size_t size = 0;
        simgear::ContainerType type = ReplayContainer::Invalid;
        if (!input.readContainer(&type, &meta_data, &size) || size < 1)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "File not recognized. This is not a valid FlightGear flight recorder tape: "
                    << filename << ". Invalid meta data.");
            ok = false;
        }
        else
        if (type != ReplayContainer::MetaData)
        {
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "Invalid header. Container type " << type);
            SG_LOG(SG_SYSTEMS, SG_ALERT, "File not recognized. This is not a valid FlightGear flight recorder tape: "
                    << filename);
            ok = false;
        }
        else
        {
            try
            {
                readProperties(meta_data, size-1, meta_data_props);
                copyProperties(meta_data_props->getNode("meta", 0, true), &meta_meta);
            }
            catch (const sg_exception &e)
            {
              SG_LOG(SG_SYSTEMS, SG_ALERT, "Error reading flight recorder tape: " << filename
                     << ", XML parser message:" << e.getFormattedMessage());
              ok = false;
            }
        }

        if (meta_data)
        {
            //printf("%s\n", meta_data);
            free(meta_data);
            meta_data = NULL;
        }
    }

    /* read flight recorder configuration **************************/
    if (ok && !preview)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Loading flight recorder data...");
        char* config_xml = nullptr;
        size_t size = 0;
        simgear::ContainerType type = ReplayContainer::Invalid;
        if (!input.readContainer(&type, &config_xml, &size) || size < 1)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "File not recognized."
                    << " This is not a valid FlightGear flight recorder tape: "
                    << filename << ". Invalid configuration container.");
            ok = false;
        }
        else
        if (!config_xml || type != ReplayContainer::Properties)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "File not recognized."
                    " This is not a valid FlightGear flight recorder tape: " << filename
                   << ". Unexpected container type, expected \"properties\".");
            ok = false;
        }

        SGPropertyNode_ptr config = new SGPropertyNode();
        if (ok)
        {
            try
            {
                readProperties(config_xml, size-1, config);
            }
            catch (const sg_exception &e)
            {
              SG_LOG(SG_SYSTEMS, SG_ALERT, "Error reading flight recorder tape: " << filename
                     << ", XML parser message:" << e.getFormattedMessage());
              ok = false;
            }
            if (ok)
            {
                // reconfigure the recorder - and wipe old data (no longer matches the current recorder)
                m_flight_recorder->reinit(config);
                clear(*this);
                fillRecycler(*this);
            }
        }

        if (config_xml)
        {
            free(config_xml);
            config_xml = NULL;
        }

        /* read raw data ***********************************************/
        if (ok)
        {
            size_t record_size = m_flight_recorder->getRecordSize();
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "RecordSize=" << record_size);
            size_t original_size = config->getIntValue("recorder/record-size", 0);
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "OriginalSize=" << original_size);
            
            // check consistency - ugly things happen when data vs signals mismatch
            if (original_size != record_size && original_size != 0)
            {
                ok = false;
                SG_LOG(SG_SYSTEMS, SG_ALERT, "Error: Data inconsistency."
                        << " Flight recorder tape has record size " << record_size
                        << ", expected size was " << original_size << ".");
            }

            bool multiplayer = false;
            for (auto data: meta_meta.getChildren("data"))
            {
                if (data->getStringValue() == "multiplayer")
                {
                    multiplayer = true;
                }
            }
            SG_LOG(SG_SYSTEMS, SG_ALERT, "multiplayer=" << multiplayer);
            if (ok) ok = loadRawReplayData(input, m_short_term,  record_size, multiplayer);
            if (ok) ok = loadRawReplayData(input, m_medium_term, record_size, multiplayer);
            if (ok) ok = loadRawReplayData(input, m_long_term,   record_size, multiplayer);

            // restore replay messages
            if (ok)
            {
                copyProperties(
                        meta_data_props->getNode("messages", 0, true),
                        fgGetNode("/sim/replay/messages", 0, true)
                        );
            }
            m_sim_time = get_end_time(*this);
            // TODO we could (re)store these too
            m_last_mt_time = m_last_lt_time = m_sim_time;
        }
        /* done *********************************************************/
    }

    input.close();

    if (!preview)
    {
        if (ok)
        {
            guiMessage("Flight recorder tape loaded successfully!");
            start(true);
        }
        else
            guiMessage("Failed to load tape. See log output.");
    }

    return ok;
}

/** List available tapes in current directory.
 * Limits to tapes matching current aircraft when same_aircraft_filter is enabled.
 */
bool listTapes(bool same_aircraft_filter, const SGPath& tape_directory)
{
    const std::string aircraft_type = simgear::strutils::uppercase(fgGetString("/sim/aircraft", "unknown"));

    // process directory listing of ".fgtape" files
    simgear::Dir dir(tape_directory);
    simgear::PathList list = dir.children(simgear::Dir::TYPE_FILE, ".fgtape");

    SGPropertyNode* tape_list = fgGetNode("/sim/replay/tape-list", true);
    tape_list->removeChildren("tape");
    int index = 0;
    size_t l = aircraft_type.size();
    for (auto it = list.begin(); it != list.end(); ++it)
    {
        SGPath file(it->file());
        std::string name(file.base());
        if (!same_aircraft_filter
                || !simgear::strutils::uppercase(name).compare(0, l, aircraft_type)
                )
        {
            tape_list->getNode("tape", index++, true)->setStringValue(name);
        }
    }
    return true;
}

std::string  FGReplayInternal::makeTapePath(const std::string& tape_name)
{
    std::string path = tape_name;
    if (simgear::strutils::ends_with(path, ".fgtape"))
    {
        return path;
    }
    SGPath path2(fgGetString("/sim/replay/tape-directory", ""));
    path2.append(path + ".fgtape");
    return path2.str();
}

int FGReplayInternal::loadContinuousHeader(const std::string& path, std::istream* in, SGPropertyNode* properties)
{
    return ::loadContinuousHeader(path, in, properties);
}

/** Load a flight recorder tape from disk. User/script command. */
bool
FGReplayInternal::loadTape(const SGPropertyNode* config_data)
{
    SGPath tape_directory(fgGetString("/sim/replay/tape-directory", ""));

    // see if shall really load the file - or just obtain the meta data for preview
    bool preview = config_data->getBoolValue("preview", 0);
    
    // file/tape to be loaded
    std::string tape = config_data->getStringValue("tape", "");

    if (tape.empty())
    {
        if (preview)    return true;
        return listTapes(config_data->getBoolValue("same-aircraft", 0), tape_directory);
    }
    else
    {
        SGPropertyNode* meta_meta = fgGetNode("/sim/gui/dialogs/flightrecorder/preview", true);
        std::string path = makeTapePath(tape);
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Checking flight recorder file " << path << ", preview: " << preview);
        return loadTape(
                path,
                preview,
                config_data->getBoolValue("create-video"),
                config_data->getDoubleValue("fixed-dt", -1),
                *meta_meta
                );
    }
}
