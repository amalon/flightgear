#include "continuous.hxx"

#include <Aircraft/flightrecorder.hxx>
#include <Main/fg_props.hxx>
#include <MultiPlayer/mpmessages.hxx>
#include <Viewer/FGEventHandler.hxx>
#include <Viewer/renderer.hxx>
#include <Viewer/viewmgr.hxx>

#include <simgear/io/iostreams/zlibstream.hxx>
#include <simgear/props/props_io.hxx>
#include <simgear/structure/commands.hxx>

#include <osgViewer/ViewerBase>

#include <assert.h>
#include <string.h>


Continuous::Continuous(std::shared_ptr<FGFlightRecorder> flight_recorder)
:
m_flight_recorder(flight_recorder)
{
    SGPropertyNode* record_continuous = fgGetNode("/sim/replay/record-continuous", true);
    SGPropertyNode* fdm_initialized = fgGetNode("/sim/signals/fdm-initialized", true);
    record_continuous->addChangeListener(this, true /*initial*/);
    fdm_initialized->addChangeListener(this, true /*initial*/);
}

// Reads binary data from a stream into an instance of a type.
template<typename T>
static void readRaw(std::istream& in, T& data)
{
    in.read(reinterpret_cast<char*>(&data), sizeof(data));
}

// Writes instance of a type as binary data to a stream.
template<typename T>
static void writeRaw(std::ostream& out, const T& data)
{
    out.write(reinterpret_cast<const char*>(&data), sizeof(data));
}

// Reads uncompressed vector<char> from file. Throws if length field is longer
// than <max_length>.
template<typename SizeType>
static SizeType VectorRead(std::istream& in, std::vector<char>& out, uint32_t max_length=(1u << 31))
{
    SizeType    length;
    readRaw(in, length);
    if (sizeof(length) + length > max_length)
    {
        SG_LOG(SG_SYSTEMS, SG_ALERT, "recording data vector too long."
                << " max_length=" << max_length
                << " sizeof(length)=" << sizeof(length)
                << " length=" << length
                );
        throw std::runtime_error("Failed to read vector in recording");
    }
    out.resize(length);
    in.read(&out.front(), length);
    return sizeof(length) + length;
}

static int16_t read_int16(std::istream& in, size_t& pos)
{
    int16_t a;
    readRaw(in, a);
    pos += sizeof(a);
    return a;
}
static std::string read_string(std::istream& in, size_t& pos)
{
    int16_t length = read_int16(in, pos);
    std::vector<char>   path(length);
    in.read(&path[0], length);
    pos += length;
    std::string ret(&path[0], length);
    return ret;
}

static int PropertiesWrite(SGPropertyNode* root, std::ostream& out)
{
    stringstream buffer;
    writeProperties(buffer, root, true /*write_all*/);
    uint32_t    buffer_len = buffer.str().size() + 1;
    writeRaw(out, buffer_len);
    out.write(buffer.str().c_str(), buffer_len);
    return 0;
}

// Reads extra-property change items in next <length> bytes. Throws if we don't
// exactly read <length> bytes.
static void ReadFGReplayDataExtraProperties(std::istream& in, FGReplayData* replay_data, uint32_t length)
{
    SG_LOG(SG_SYSTEMS, SG_BULK, "reading extra-properties. length=" << length);
    size_t pos=0;
    for(;;)
    {
        if (pos == length)
        {
            break;
        }
        if (pos > length)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "Overrun while reading extra-properties:"
                    " length=" << length << ": pos=" << pos);
            in.setstate(std::ios_base::failbit);
            break;
        }
        SG_LOG(SG_SYSTEMS, SG_BULK, "length=" << length<< " pos=" << pos);
        std::string path = read_string(in, pos);
        if (path == "")
        {
            path = read_string(in, pos);
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "property deleted: " << path);
            replay_data->replay_extra_property_removals.push_back(path);
        }
        else
        {
            std::string value = read_string(in, pos);
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "property changed: " << path << "=" << value);
            replay_data->replay_extra_property_changes[path] = value;
        }
    }
}

static bool ReadFGReplayData2(
        std::istream& in,
        SGPropertyNode* config,
        bool load_signals,
        bool load_multiplayer,
        bool load_extra_properties,
        FGReplayData* ret
        )
{
    ret->raw_data.resize(0);
    for (auto data: config->getChildren("data"))
    {
        std::string data_type = data->getStringValue();
        SG_LOG(SG_SYSTEMS, SG_BULK, "in.tellg()=" << in.tellg() << " data_type=" << data_type);
        uint32_t    length;
        readRaw(in, length);
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "length=" << length);
        if (!in) break;
        if (load_signals && data_type == "signals")
        {
            ret->raw_data.resize(length);
            in.read(&ret->raw_data.front(), ret->raw_data.size());
        }
        else if (load_multiplayer && data_type == "multiplayer")
        {
            /* Multiplayer information is a vector of vectors. */
            ret->multiplayer_messages.clear();
            uint32_t pos = 0;
            for(;;)
            {
                assert(pos <= length);
                if (pos == length)  break;
                std::shared_ptr<std::vector<char>>  v(new std::vector<char>);
                ret->multiplayer_messages.push_back(v);
                pos += VectorRead<uint16_t>(in, *ret->multiplayer_messages.back(), length - pos);
                SG_LOG(SG_SYSTEMS, SG_BULK, "replaying multiplayer data"
                        << " ret->sim_time=" << ret->sim_time
                        << " length=" << length
                        << " pos=" << pos
                        << " callsign=" << ((T_MsgHdr*) &v->front())->Callsign
                        );
            }
        }
        else if (load_extra_properties && data_type == "extra-properties")
        {
            ReadFGReplayDataExtraProperties(in, ret, length);
        }
        else
        {
            SG_LOG(SG_GENERAL, SG_BULK, "Skipping unrecognised/unwanted data: " << data_type);
            in.seekg(length, std::ios_base::cur);
        }
        if (!in) break;
    }
    if (!in)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Failed to read fgtape data");
        return false;
    }
    return true;
}

/* Removes items more than <n> away from <it>. <n> can be -ve. */
template<typename Container, typename Iterator>
static void remove_far_away(Container& container, Iterator it, int n)
{
    SG_LOG(SG_GENERAL, SG_DEBUG, "container.size()=" << container.size());
    if (n > 0)
    {
        for (int i=0; i<n; ++i)
        {
            if (it == container.end())  return;
            ++it;
        }
        container.erase(it, container.end());
    }
    else
    {
        for (int i=0; i<-n-1; ++i)
        {
            if (it == container.begin())    return;
            --it;
        }
        container.erase(container.begin(), it);
    }
    SG_LOG(SG_GENERAL, SG_DEBUG, "container.size()=" << container.size());
}

/* Returns FGReplayData for frame at specified position in file. Uses
continuous.m_in_pos_to_frame as a cache, and trims this cache using
remove_far_away(). */
static std::shared_ptr<FGReplayData> ReadFGReplayData(
        Continuous& continuous,
        std::ifstream& in,
        size_t pos,
        SGPropertyNode* config,
        bool load_signals,
        bool load_multiplayer,
        bool load_extra_properties,
        int in_compression
        )
{
    std::shared_ptr<FGReplayData>   ret;
    auto it = continuous.m_in_pos_to_frame.find(pos);
    
    if (it != continuous.m_in_pos_to_frame.end())
    {
        if (0
            || (load_signals && !it->second->load_signals)
            || (load_multiplayer && !it->second->load_multiplayer)
            || (load_extra_properties && !it->second->load_extra_properties)
            )
        {
            /* This frame is in the continuous.m_in_pos_to_frame cache, but
            doesn't contain all of the required items, so we need to reload. */
            continuous.m_in_pos_to_frame.erase(it);
            it = continuous.m_in_pos_to_frame.end();
        }
    }
    if (it == continuous.m_in_pos_to_frame.end())
    {
        /* Load FGReplayData at offset <pos>.

        We need to clear any eof bit, otherwise seekg() will not work (which is
        pretty unhelpful). E.g. see:
            https://stackoverflow.com/questions/16364301/whats-wrong-with-the-ifstream-seekg
        */
        SG_LOG(SG_SYSTEMS, SG_BULK, "reading frame. pos=" << pos);
        in.clear();
        in.seekg(pos);

        ret.reset(new FGReplayData);

        readRaw(in, ret->sim_time);
        if (!in)
        {
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "Failed to read fgtape frame at offset " << pos);
            return nullptr;
        }
        bool ok;
        if (in_compression)
        {
            uint8_t     flags;
            uint32_t    compressed_size;
            in.read((char*) &flags, sizeof(flags));
            in.read((char*) &compressed_size, sizeof(compressed_size));
            simgear::ZlibDecompressorIStream    in_decompress(in, SGPath(), simgear::ZLibCompressionFormat::ZLIB_RAW);
            ok = ReadFGReplayData2(in_decompress, config, load_signals, load_multiplayer, load_extra_properties, ret.get());
        }
        else
        {
            ok = ReadFGReplayData2(in, config, load_signals, load_multiplayer, load_extra_properties, ret.get());
        }
        if (!ok)
        {
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "Failed to read fgtape frame at offset " << pos);
            return nullptr;
        }
        it = continuous.m_in_pos_to_frame.lower_bound(pos);
        it = continuous.m_in_pos_to_frame.insert(it, std::make_pair(pos, ret));
        
        /* Delete faraway items. */
        size_t size_old = continuous.m_in_pos_to_frame.size();
        int n = 2;
        size_t size_max = 2*n - 1;
        remove_far_away(continuous.m_in_pos_to_frame, it, n);
        remove_far_away(continuous.m_in_pos_to_frame, it, -n);
        size_t size_new = continuous.m_in_pos_to_frame.size();
        SG_LOG(SG_GENERAL, SG_DEBUG, ""
                << " n=" << size_old
                << " size_max=" << size_max
                << " size_old=" << size_old
                << " size_new=" << size_new
                );
        assert(size_new <= size_max);
    }
    else
    {
        ret = it->second;
    }
    return ret;
}


// streambuf that compresses using deflate().
struct compression_streambuf : std::streambuf
{
    compression_streambuf(
            std::ostream& out,
            size_t buffer_uncompressed_size,
            size_t buffer_compressed_size
            )
    :
    std::streambuf(),
    out(out),
    buffer_uncompressed(new char[buffer_uncompressed_size]),
    buffer_uncompressed_size(buffer_uncompressed_size),
    buffer_compressed(new char[buffer_compressed_size]),
    buffer_compressed_size(buffer_compressed_size)
    {
        zstream.zalloc = nullptr;
        zstream.zfree = nullptr;
        zstream.opaque = nullptr;
        
        zstream.next_in = nullptr;
        zstream.avail_in = 0;
        
        zstream.next_out = (unsigned char*) &buffer_compressed[0];
        zstream.avail_out = buffer_compressed_size;
        
        int e = deflateInit2(
                &zstream,
                Z_DEFAULT_COMPRESSION,
                Z_DEFLATED,
                -15 /*windowBits*/,
                8 /*memLevel*/,
                Z_DEFAULT_STRATEGY
                );
        if (e != Z_OK)
        {
            throw std::runtime_error("deflateInit2() failed");
        }
        // We leave space for one character to simplify overflow().
        setp(&buffer_uncompressed[0], &buffer_uncompressed[0] + buffer_uncompressed_size - 1);
    }
    
    // Flush compressed data to .out and reset zstream.next_out.
    void _flush()
    {
        // Send all data in .buffer_compressed to .out.
        size_t n = (char*) zstream.next_out - &buffer_compressed[0];
        out.write(&buffer_compressed[0], n);
        zstream.next_out = (unsigned char*) &buffer_compressed[0];
        zstream.avail_out = buffer_compressed_size;
    }
    
    // Compresses specified bytes from buffer_uncompressed into
    // buffer_compressed, flushing to .out as necessary. Returns true if we get
    // EOF writing to .out.
    bool _deflate(size_t n, bool flush)
    {
        assert(this->pbase() == &buffer_uncompressed[0]);
        zstream.next_in = (unsigned char*) &buffer_uncompressed[0];
        zstream.avail_in = n;
        for(;;)
        {
            if (!flush && !zstream.avail_in)   break;
            if (!zstream.avail_out) _flush();
            int e = deflate(&zstream, (!zstream.avail_in && flush) ? Z_FINISH : Z_NO_FLUSH);
            if (e != Z_OK && e != Z_STREAM_END)
            {
                throw std::runtime_error("zip_deflate() failed");
            }
            if (e == Z_STREAM_END)  break;
        }
        if (flush)  _flush();
        // We leave space for one character to simplify overflow().
        setp(&buffer_uncompressed[0], &buffer_uncompressed[0] + buffer_uncompressed_size - 1);
        if (!out) return true;  // EOF.
        return false;
    }
    
    int overflow(int c) override
    {
        // We've deliberately left space for one character, into which we write <c>.
        assert(this->pptr() == &buffer_uncompressed[0] + buffer_uncompressed_size - 1);
        *this->pptr() = (char) c;
        if (_deflate(buffer_uncompressed_size, false /*flush*/)) return EOF;
        return c;
    }
    
    int sync() override
    {
        _deflate(pptr() - &buffer_uncompressed[0], true /*flush*/);
        return 0;
    }
    
    ~compression_streambuf()
    {
        deflateEnd(&zstream);
    }
    
    std::ostream&           out;
    z_stream                zstream;
    std::unique_ptr<char[]> buffer_uncompressed;
    size_t                  buffer_uncompressed_size;
    std::unique_ptr<char[]> buffer_compressed;
    size_t                  buffer_compressed_size;
};


// Accepts uncompressed data via .write(), operator<< etc, and writes
// compressed data to the supplied std::ostream.
struct compression_ostream : std::ostream
{
    compression_ostream(
            std::ostream& out,
            size_t buffer_uncompressed_size,
            size_t buffer_compressed_size
            )
    :
    std::ostream(&streambuf),
    streambuf(out, buffer_uncompressed_size, buffer_compressed_size)
    {
    }
    
    compression_streambuf   streambuf;
};


static void writeFrame2(FGReplayData* r, std::ostream& out, SGPropertyNode_ptr config)
{
    for (auto data: config->getChildren("data"))
    {
        std::string data_type = data->getStringValue();
        if (data_type == "signals")
        {
            uint32_t    signals_size = r->raw_data.size();
            writeRaw(out, signals_size);
            out.write(&r->raw_data.front(), r->raw_data.size());
        }
        else if (data_type == "multiplayer")
        {
            uint32_t    length = 0;
            for (auto message: r->multiplayer_messages)
            {
                length += sizeof(uint16_t) + message->size();
            }
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "data_type=" << data_type << " out.tellp()=" << out.tellp()
                    << " length=" << length);
            writeRaw(out, length);
            for (auto message: r->multiplayer_messages)
            {
                uint16_t message_size = message->size();
                writeRaw(out, message_size);
                out.write(&message->front(), message_size);
            }
        }
        else if (data_type == "extra-properties")
        {
            uint32_t    length = r->extra_properties.size();
            SG_LOG(SG_SYSTEMS, SG_DEBUG, "data_type=" << data_type << " out.tellp()=" << out.tellp()
                    << " length=" << length);
            writeRaw(out, length);
            out.write(&r->extra_properties[0], length);
        }
        else
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "unrecognised data_type=" << data_type);
            assert(0);
        }
    }
    
}

bool continuousWriteFrame(
        Continuous& continuous,
        FGReplayData* r,
        std::ostream& out,
        SGPropertyNode_ptr config,
        FGTapeType tape_type
        )
{
    SG_LOG(SG_SYSTEMS, SG_BULK, "writing frame."
            << " out.tellp()=" << out.tellp()
            << " r->sim_time=" << r->sim_time
            );
    // Don't write frame if no data to write.
    //bool r_has_data = false;
    bool has_signals = false;
    bool has_multiplayer = false;
    bool has_extra_properties = false;
    for (auto data: config->getChildren("data"))
    {
        std::string data_type = data->getStringValue();
        if (data_type == "signals")
        {
            has_signals = true;
        }
        else if (data_type == "multiplayer")
        {
            if (!r->multiplayer_messages.empty())
            {
                has_multiplayer = true;
            }
        }
        else if (data_type == "extra-properties")
        {
            if (!r->extra_properties.empty())
            {
                has_extra_properties = true;
            }
        }
        else
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "unrecognised data_type=" << data_type);
            assert(0);
        }
    }
    if (!has_signals && !has_multiplayer && !has_extra_properties)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Not writing frame because no data to write");
        return true;
    }
    
    writeRaw(out, r->sim_time);
    
    if (tape_type == FGTapeType_CONTINUOUS && continuous.m_out_compression)
    {
        uint8_t flags = 0;
        if (has_signals)            flags |= 1;
        if (has_multiplayer)        flags |= 2;
        if (has_extra_properties)   flags |= 4;
        out.write((char*) &flags, sizeof(flags));
        
        /* We need to first write the size of the compressed data so compress
        to a temporary ostringstream first. */
        std::ostringstream  compressed;
        compression_ostream out_compressing(compressed, 1024, 1024);
        writeFrame2(r, out_compressing, config);
        out_compressing.flush();
        
        uint32_t compressed_size = compressed.str().size();
        out.write((char*) &compressed_size, sizeof(compressed_size));
        out.write((char*) compressed.str().c_str(), compressed.str().size());
    }
    else
    {
        writeFrame2(r, out, config);
    }
    bool ok = true;
    if (!out) ok = false;
    return ok;
}

SGPropertyNode_ptr continuousWriteHeader(
        Continuous&         continuous,
        FGFlightRecorder*   flight_recorder,
        std::ofstream&      out,
        const SGPath&       path,
        FGTapeType          tape_type
        )
{
    continuous.m_out_compression = fgGetInt("/sim/replay/record-continuous-compression");
    SGPropertyNode_ptr  config = saveSetup(NULL /*Extra*/, path, 0 /*Duration*/,
            tape_type, continuous.m_out_compression);
    SGPropertyNode* signals = config->getNode("signals", true /*create*/);
    flight_recorder->getConfig(signals);
    
    out.open(path.c_str(), std::ofstream::binary | std::ofstream::trunc);
    out.write(FlightRecorderFileMagic, strlen(FlightRecorderFileMagic)+1);
    PropertiesWrite(config, out);
    
    if (tape_type == FGTapeType_CONTINUOUS)
    {
        // Ensure that all recorded properties are written in first frame.
        //
        flight_recorder->resetExtraProperties();
    }
    
    if (!out)
    {
        out.close();
        config = nullptr;
    }
    return config;
}

/* Replays one frame from Continuous recording. <offset> and <offset_old> are
offsets in file of frames that are >= and < <time> respectively. <offset_old>
may be 0, in which case it is ignored.

We load the frame(s) from disc, omitting some data depending on
replay_signals, replay_multiplayer and replay_extra_properties. Then call
m_pRecorder->replay(), which updates the global state.

Returns true on success, otherwise we failed to read from Continuous recording.
*/
static bool replayContinuousInternal(
        Continuous& continuous,
        FGFlightRecorder* recorder,
        double time,
        size_t offset,
        size_t offset_old,
        bool replay_signals,
        bool replay_multiplayer,
        bool replay_extra_properties,
        int* xpos,
        int* ypos,
        int* xsize,
        int* ysize
        )
{
    std::shared_ptr<FGReplayData> replay_data = ReadFGReplayData(
            continuous,
            continuous.m_in,
            offset,
            continuous.m_in_config,
            replay_signals,
            replay_multiplayer,
            replay_extra_properties,
            continuous.m_in_compression
            );
    if (!replay_data)
    {
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Failed to read fgtape frame at offset=" << offset << " time=" << time);
        return false;
    }
    assert(replay_data.get());
    std::shared_ptr<FGReplayData> replay_data_old;
    if (offset_old)
    {
        replay_data_old = ReadFGReplayData(
                continuous,
                continuous.m_in,
                offset_old,
                continuous.m_in_config,
                replay_signals,
                replay_multiplayer,
                replay_extra_properties,
                continuous.m_in_compression
                );
    }
    if (replay_extra_properties) SG_LOG(SG_SYSTEMS, SG_DEBUG,
            "replay():"
            << " time=" << time
            << " offset=" << offset
            << " offset_old=" << offset_old
            << " replay_data_old=" << replay_data_old
            << " replay_data->raw_data.size()=" << replay_data->raw_data.size()
            << " replay_data->multiplayer_messages.size()=" << replay_data->multiplayer_messages.size()
            << " replay_data->extra_properties.size()=" << replay_data->extra_properties.size()
            << " replay_data->replay_extra_property_changes.size()=" << replay_data->replay_extra_property_changes.size()
            );
    recorder->replay(time, replay_data.get(), replay_data_old.get(), xpos, ypos, xsize, ysize);
    return true;
}

// fixme: this is duplicated in replay.cxx.
static void popupTip(const char* message, int delay)
{
    SGPropertyNode_ptr args(new SGPropertyNode);
    args->setStringValue("label", message);
    args->setIntValue("delay", delay);
    globals->get_commands()->execute("show-message", args);
}

void continuous_replay_video_end(Continuous& continuous)
{
    if (continuous.m_replay_create_video)
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Stopping replay create-video");
        auto view_mgr = globals->get_subsystem<FGViewMgr>();
        if (view_mgr)
        {
            view_mgr->video_stop();
        }
        continuous.m_replay_create_video = false;
    }
    if (continuous.m_replay_fixed_dt_prev != -1)
    {
        SG_LOG(SG_GENERAL, SG_ALERT, "Resetting fixed-dt to" << continuous.m_replay_fixed_dt_prev);
        fgSetDouble("/sim/time/fixed-dt", continuous.m_replay_fixed_dt_prev);
        continuous.m_replay_fixed_dt_prev = -1;
    }
}

bool replayContinuous(FGReplayInternal& self, double time)
{
    // We need to detect whether replay() updates the values for the main
    // window's position and size.
    int xpos0 = self.m_sim_startup_xpos->getIntValue();
    int ypos0 = self.m_sim_startup_xpos->getIntValue();
    int xsize0 = self.m_sim_startup_xpos->getIntValue();
    int ysize0 = self.m_sim_startup_xpos->getIntValue();

    int xpos = xpos0;
    int ypos = ypos0;
    int xsize = xsize0;
    int ysize = ysize0;

    double  multiplayer_recent = 3;

    // We replay all frames from just after the previously-replayed frame,
    // in order to replay extra properties and multiplayer aircraft
    // correctly.
    //
    double  t_begin = self.m_continuous->m_in_frame_time_last;

    if (time < self.m_continuous->m_in_time_last)
    {
        // We have gone backwards, e.g. user has clicked on the back
        // buttons in the Replay dialogue.
        //

        if (self.m_continuous->m_in_multiplayer)
        {
            // Continuous recording has multiplayer data, so replay recent
            // ones.
            //
            t_begin = time - multiplayer_recent;
        }

        if (self.m_continuous->m_in_extra_properties)
        {
            // Continuous recording has property changes. we need to replay
            // all property changes from the beginning.
            //
            t_begin = -1;
        }

        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Have gone backwards."
                << " m_in_time_last=" << self.m_continuous->m_in_time_last
                << " time=" << time
                << " t_begin=" << t_begin
                << " m_in_extra_properties=" << self.m_continuous->m_in_extra_properties
                );
    }

    // Prepare to replay signals from Continuoue recording file. We want
    // to find a pair of frames that straddle the requested <time> so that
    // we can interpolate.
    //
    auto p = self.m_continuous->m_in_time_to_frameinfo.lower_bound(time);
    bool ret = false;

    size_t  offset;
    size_t  offset_prev = 0;

    if (p == self.m_continuous->m_in_time_to_frameinfo.end())
    {
        // We are at end of recording; replay last frame.
        continuous_replay_video_end(*self.m_continuous);
        --p;
        offset = p->second.offset;
        ret = true;
    }
    else if (p->first > time)
    {
        // Look for preceding item.
        if (p == self.m_continuous->m_in_time_to_frameinfo.begin())
        {
            // <time> is before beginning of recording.
            offset = p->second.offset;
        }
        else
        {
            // Interpolate between pair of items that straddle <time>.
            auto prev = p;
            --prev;
            offset_prev = prev->second.offset;
            offset = p->second.offset;
        }
    }
    else
    {
        // Exact match.
        offset = p->second.offset;
    }

    // Before interpolating signals, we replay all property changes from
    // all frame times t satisfying t_prop_begin < t < time. We also replay
    // all recent multiplayer packets in this range, i.e. for which t >
    // time - multiplayer_recent.
    //
    // todo: figure out how to interpolate view position/direction, to
    // smooth things out if replay fps is different from record fps e.g.
    // with new fixed dt support.
    //
    for (auto p_before = self.m_continuous->m_in_time_to_frameinfo.upper_bound(t_begin);
            p_before != self.m_continuous->m_in_time_to_frameinfo.end();
            ++p_before)
    {
        if (p_before->first >= p->first)
        {
            break;
        }
        // Replaying a frame is expensive because we read frame data
        // from disc each time. So we only replay this frame if it has
        // extra_properties, or if it has multiplayer packets and we are
        // within <multiplayer_recent> seconds of current time.
        //
        bool    replay_this_frame = p_before->second.has_extra_properties;
        if (p_before->second.has_multiplayer && p_before->first > time - multiplayer_recent)
        {
            replay_this_frame = true;
        }

        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Looking at extra property changes."
                << " replay_this_frame=" << replay_this_frame
                << " m_continuous->m_in_time_last=" << self.m_continuous->m_in_time_last
                << " m_continuous->m_in_frame_time_last=" << self.m_continuous->m_in_frame_time_last
                << " time=" << time
                << " t_begin=" << t_begin
                << " p_before->first=" << p_before->first
                << " p_before->second=" << p_before->second
                );

        if (replay_this_frame)
        {
            size_t pos_prev = 0;
            if (p_before != self.m_continuous->m_in_time_to_frameinfo.begin())
            {
                auto p_before_prev = p_before;
                --p_before_prev;
                pos_prev = p_before_prev->second.offset;
            }
            bool ok = replayContinuousInternal(
                    *self.m_continuous,
                    self.m_flight_recorder.get(),
                    p_before->first,
                    p_before->second.offset,
                    pos_prev /*offset_old*/,
                    false /*replay_signals*/,
                    p_before->first > time - multiplayer_recent /*replay_multiplayer*/,
                    true /*replay_extra_properties*/,
                    &xpos,
                    &ypos,
                    &xsize,
                    &ysize
                    );
            if (!ok)
            {
                if (!self.m_replay_error->getBoolValue())
                {
                    SG_LOG(SG_SYSTEMS, SG_ALERT, "Replay failed: cannot read fgtape data");
                    popupTip("Replay failed: cannot read fgtape data", 10);
                    self.m_replay_error->setBoolValue(true);
                }
                return true;
            }
        }
    }

    /* Now replay signals, interpolating between frames atoffset_prev and
    offset. */
    bool ok = replayContinuousInternal(
            *self.m_continuous,
            self.m_flight_recorder.get(),
            time,
            offset,
            offset_prev /*offset_old*/,
            true /*replay_signals*/,
            true /*replay_multiplayer*/,
            true /*replay_extra_properties*/,
            &xpos,
            &ypos,
            &xsize,
            &ysize
            );
    if (!ok)
    {
        if (!self.m_replay_error->getBoolValue())
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "Replay failed: cannot read fgtape data");
            popupTip("Replay failed: cannot read fgtape data", 10);
            self.m_replay_error->setBoolValue(true);
        }
        return true;
    }
    if (0
            || xpos != xpos0
            || ypos != ypos0
            || xsize != xsize0
            || ysize != ysize0
            )
    {
        // Move/resize the main window to reflect the updated values.
        globals->get_props()->setIntValue("/sim/startup/xpos", xpos);
        globals->get_props()->setIntValue("/sim/startup/ypos", ypos);
        globals->get_props()->setIntValue("/sim/startup/xsize", xsize);
        globals->get_props()->setIntValue("/sim/startup/ysize", ysize);

        osgViewer::ViewerBase* viewer_base = globals->get_renderer()->getViewerBase();
        if (viewer_base)
        {
            std::vector<osgViewer::GraphicsWindow*> windows;
            viewer_base->getWindows(windows);
            osgViewer::GraphicsWindow* window = windows[0];

            // We use FGEventHandler::setWindowRectangle() to move the
            // window, because it knows how to convert from window work-area
            // coordinates to window-including-furniture coordinates.
            //
            flightgear::FGEventHandler* event_handler = globals->get_renderer()->getEventHandler();
            event_handler->setWindowRectangleInteriorWithCorrection(window, xpos, ypos, xsize, ysize);
        }
    }

    self.m_continuous->m_in_time_last = time;
    self.m_continuous->m_in_frame_time_last = p->first;

    return ret;
}

/* SGPropertyChangeListener callback for detecing when FDM is initialised and
for when continuous recording is started or stopped. */
void Continuous::valueChanged(SGPropertyNode * node)
{
    bool    prop_continuous = fgGetBool("/sim/replay/record-continuous");
    bool    prop_fdm = fgGetBool("/sim/signals/fdm-initialized");
    
    bool continuous = prop_continuous && prop_fdm;
    if (continuous == (m_out.is_open() ? true : false))
    {
        // No change.
        return;
    }
    
    if (m_out.is_open())
    {
        // Stop existing continuous recording.
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Stopping continuous recording");
        m_out.close();
        popupTip("Continuous record to file stopped", 5 /*delay*/);
    }
    
    if (continuous)
    {
        // Start continuous recording.
        SGPath  path_timeless;
        SGPath  path = makeSavePath(FGTapeType_CONTINUOUS, &path_timeless);
        m_out_config = continuousWriteHeader(
                *this,
                m_flight_recorder.get(),
                m_out,
                path,
                FGTapeType_CONTINUOUS
                );
        if (!m_out_config)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to start continuous recording");
            popupTip("Continuous record to file failed to start", 5 /*delay*/);
            return;
        }
        
        SG_LOG(SG_SYSTEMS, SG_ALERT, "Starting continuous recording");
        
        /* Make a convenience link to the recording. E.g.
        harrier-gr3-continuous.fgtape -> harrier-gr3-20201224-005034-continuous.fgtape.
        
        Link destination is in same directory as link so we use leafname
        path.file(). */
        path_timeless.remove();
        bool ok = path_timeless.makeLink(path.file());
        if (!ok)
        {
            SG_LOG(SG_SYSTEMS, SG_ALERT, "Failed to create link " << path_timeless.c_str() << " => " << path.file());
        }
        SG_LOG(SG_SYSTEMS, SG_DEBUG, "Starting continuous recording to " << path);
        if (m_out_compression)
        {
            popupTip("Continuous+compressed record to file started", 5 /*delay*/);
        }
        else
        {
            popupTip("Continuous record to file started", 5 /*delay*/);
        }
    }
}
