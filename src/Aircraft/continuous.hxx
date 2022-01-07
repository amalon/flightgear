#pragma once

#include "replay-internal.hxx"

#include <simgear/props/props.hxx>

#include <fstream>

#include <mutex>
#include <thread>


struct Continuous : SGPropertyChangeListener
{
    Continuous(std::shared_ptr<FGFlightRecorder> flight_recorder);
    
    /* Callback for SGPropertyChangeListener. */
    void valueChanged(SGPropertyNode * node) override;
    
    std::shared_ptr<FGFlightRecorder>   m_flight_recorder;
    
    std::ifstream                   m_in;
    bool                            m_in_multiplayer = false;
    bool                            m_in_extra_properties = false;
    std::mutex                      m_in_time_to_frameinfo_lock;
    std::map<double, FGFrameInfo>   m_in_time_to_frameinfo;
    SGPropertyNode_ptr              m_in_config;
    double                          m_in_time_last = 0;
    double                          m_in_frame_time_last = 0;
    std::map<size_t, std::shared_ptr<FGReplayData>>
                                    m_in_pos_to_frame;
    
    std::ifstream                   m_indexing_in;
    std::streampos                  m_indexing_pos;
    
    bool                            m_replay_create_video = false;
    double                          m_replay_fixed_dt = -1;
    double                          m_replay_fixed_dt_prev = -1;
    
    // Only used for gathering statistics that are then written into
    // properties.
    //
    int m_num_frames_extra_properties = 0;
    int m_num_frames_multiplayer = 0;

    // For writing Continuous fgtape file.
    SGPropertyNode_ptr  m_out_config;
    std::ofstream       m_out;
    int                 m_out_compression = 0;
    int                 m_in_compression = 0;
};

/* Attempts to load Continuous recording header properties into
<properties>. If in is null we use internal std::fstream, otherwise we use *in.

Returns 0 on success, +1 if we may succeed after further download, or -1 if
recording is not a Continuous recording. */
int loadContinuousHeader(const std::string& path, std::istream* in, SGPropertyNode* properties);


/* Writes one frame of continuous record information. */
bool continuousWriteFrame(
        Continuous& continuous,
        FGReplayData* r,
        std::ostream& out,
        SGPropertyNode_ptr config
        );

/* Opens continuous recording file and writes header.

If MetaData is unset, we initialise it by calling saveSetup(). Otherwise should
be already set up.

If Config is unset, we make it point to a new node populated by
m_pRecorder->getConfig(). Otherwise it should be already set up to point to
such information.

If path_override is not "", we use it as the path (instead of the path
determined by saveSetup(). */
SGPropertyNode_ptr continuousWriteHeader(
        Continuous&         continuous,
        FGFlightRecorder*   m_pRecorder,
        std::ofstream&      out,
        const SGPath&       path,
        FGTapeType          tape_type
        );

/* Replays one frame from Continuous recording.

Returns true on success, otherwise we failed to read from Continuous recording.
*/
bool replayContinuous(FGReplayInternal& self, double time);

/* Stops any video recording that was started because of
continuous->m_replay_create_video. */
void continuous_replay_video_end(Continuous& continuous);
