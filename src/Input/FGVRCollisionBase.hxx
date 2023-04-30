// VR Collision Basic/Generic Classes
//
// Copyright (C) 2022  James Hogan
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

#ifndef _FGVRCOLLISIONBASE_HXX
#define _FGVRCOLLISIONBASE_HXX

#include <osg/BoundingBox>

#include <cassert>
#include <list>
#include <memory>
#include <ostream>
#include <set>
#include <vector>
#if 0
#include <iostream> // FIXME
#endif

namespace FGVRCollision
{

class ShapeId
{
public:
    /// Test whether the shape ID is empty.
    bool empty() const
    {
        return _data.empty();
    }

    /// Initialise the ID to a particular type and return the pointer.
    template <typename T>
    typename T::Id* init()
    {
        _data.resize(sizeof(typename T::Id));
        return reinterpret_cast<typename T::Id*>(_data.data());
    }

    /// Read the shape ID as a particular type.
    template <typename T>
    const typename T::Id* as() const
    {
        if (_data.size() != sizeof(typename T::Id))
            return nullptr;
        return reinterpret_cast<const typename T::Id*>(_data.data());
    }

private:
    std::vector<unsigned char> _data;
};

#if 0
class ShapeIdFrameBase
{
public:
    class TopPtr
    {
    public:
        /// Default constructor.
        TopPtr() :
            _ptr(nullptr)
        {
        }

        operator bool() const
        {
            return _ptr;
        }

        const ShapeIdFrameBase* operator ->() const
        {
            return _ptr;
        }

        const ShapeIdFrameBase* push(ShapeIdFrameBase* newFrame)
        {
            auto* ret = _ptr;
            _ptr = newFrame;
            return ret;
        }

        void pop()
        {
            if (_ptr)
                _ptr = _ptr->getParent();
        }

    private:
        ShapeIdFrameBase const* _ptr;
    };

    ShapeIdFrameBase(TopPtr& topPtr) :
        _parent(topPtr.push(this)),
        _topPtr(&topPtr)
    {
    }

    virtual ~ShapeIdFrameBase()
    {
        _topPtr->pop();
    }

    const ShapeIdFrameBase* getParent() const
    {
        return _parent;
    }

    void fillShapeId(ShapeId& shapeId) const
    {
        if (_parent)
            _parent->fillShapeId(shapeId);
        writeTo(shapeId);
    }

protected:
    virtual void writeTo(ShapeId& shapeId) const = 0;

private:
    const ShapeIdFrameBase* _parent;
    TopPtr* _topPtr;
};

template <typename T>
class ShapeIdFrame : public ShapeIdFrameBase
{
public:
    ShapeIdFrame(TopPtr& topPtr) :
        ShapeIdFrameBase(topPtr)
    {
    }

    ShapeIdFrame(TopPtr& topPtr, const T& data) :
        ShapeIdFrameBase(topPtr),
        _data(data)
    {
    }

    // Dereference operators
    T& operator*()
    {
        return _data;
    }
    const T& operator*() const
    {
        return _data;
    }

    // Member of operators
    T* operator->()
    {
        return &_data;
    }
    const T* operator->() const
    {
        return &_data;
    }

protected:
    void writeTo(ShapeId& shapeId) const override
    {
        shapeId << _data;
    }

private:
    T _data;
};

template <typename SHAPE, typename SWEEP>
class ShapeIdFrames
{
public:
    template <typename INTERSECTIONS>
    ShapeIdFrames(INTERSECTIONS& intersections) :
        shape(intersections.getStaticFrame()),
        sweep(intersections.getSweepFrame())
    {
    }

    ShapeIdFrame<SHAPE> shape;
    ShapeIdFrame<SWEEP> sweep;
};
#endif

// ABSTRACT TYPES

class Shape
{
};

typedef struct {} EmptyData;

struct ShapeInfo
{
    typedef EmptyData FixedData;
    typedef EmptyData SweepData;
    typedef EmptyData Id;
    enum {
        shouldCheckBounds = 0,
    };
};

template <bool REF>
struct TRefHelper;

// Original value
template <>
struct TRefHelper<false>
{
    template <typename T>
    struct Types {
        // For storage
        typedef T ValOrRef;
        // For constructor arguments
        typedef const T& InitRef;
    };
};

// reference
template <>
struct TRefHelper<true>
{
    template <typename T>
    struct Types {
        // For storage
        typedef T& ValOrRef;
        // For constructor arguments
        typedef T& InitRef;
    };
};

template <typename INFO, bool REF = false>
class TShape : public Shape
{
    public:
        typedef INFO Info;
        typedef typename INFO::FixedData FixedData;
        typedef typename INFO::SweepData SweepData;
        typedef typename INFO::Id Id;
        enum {
            shouldCheckBounds = INFO::shouldCheckBounds,
        };

        typedef typename TRefHelper<REF>::template Types<FixedData>::ValOrRef FixedValOrRef;
        typedef typename TRefHelper<REF>::template Types<SweepData>::ValOrRef SweepValOrRef;
        typedef typename TRefHelper<REF>::template Types<FixedData>::InitRef FixedInitRef;
        typedef typename TRefHelper<REF>::template Types<SweepData>::InitRef SweepInitRef;

        // These are either stored by value or by reference depending on REF
        FixedValOrRef fixed;
        SweepValOrRef sweep;

        // A similar shape but with REF=true
        typedef TShape<INFO, true> ShapeRef;

        TShape(FixedInitRef fixed,
               SweepInitRef sweep) :
            fixed(fixed),
            sweep(sweep)
        {
        }

        // Implicitly create a shape reference
        operator ShapeRef()
        {
            return ShapeRef(fixed, sweep);
        }

        static auto getBounds(const FixedData& fixed, const SweepData& sweep)
        {
            return INFO::getBounds(fixed, sweep);
        }

        auto getBounds() const
        {
            return getBounds(fixed, sweep);
        }
};

template <typename INFO, bool REF = false>
class TCompound : public TShape<INFO, REF>
{
    public:
        typedef TShape<INFO, REF> Super;
        typedef typename INFO::FixedData FixedData;
        typedef typename INFO::SweepData SweepData;
        typedef typename INFO::Id Id;

        typedef typename Super::FixedInitRef FixedInitRef;
        typedef typename Super::SweepInitRef SweepInitRef;

        // A similar shape but with REF=true
        typedef TCompound<INFO, true> ShapeRef;

        TCompound(FixedInitRef fixed,
                  SweepInitRef sweep) :
            Super(fixed, sweep)
        {
        }

        // Implicitly create a shape reference
        operator ShapeRef()
        {
            return ShapeRef(this->fixed, this->sweep);
        }

        template <typename FUNCTOR>
        auto perPart(FUNCTOR &functor, Id* id) const
        {
            INFO::template perPart<FUNCTOR>(functor, this->fixed, this->sweep, id);
        }
};

template <typename INSTANT>
class TIntersection
{
    public:
        typedef INSTANT Instant;
        typedef typename Instant::Range Range;

        // ratioEntry <= ratioExit
        Instant entry;
        Instant exit;

        TIntersection(const Instant &newEntry, const Instant &newExit) :
            entry(newEntry),
            exit(newExit)
        {
        }

        void toRange(const Range& range)
        {
            entry.toRange(range);
            exit.toRange(range);
        }

        void combine(const TIntersection& other)
        {
            if (other.entry < entry)
                entry = other.entry;
            if (other.exit > exit)
                exit = other.exit;
        }

        bool overlaps(const TIntersection& other) const
        {
            return entry <= other.exit &&
                   exit >= other.entry;
        }

        bool operator <(const TIntersection& other) const
        {
            return exit < other.entry;
        }

        friend std::ostream& operator <<(std::ostream &ost, const TIntersection& intersection)
        {
            return ost << "[" << intersection.entry << " " << intersection.exit << "]";
        }
};

class Sweep : public Shape
{
    public:
        class Instant
        {
            public:
                float ratio;

                /// Shape ID of static shape.
                ShapeId staticId;
                /// Shape ID of sweeping shape.
                ShapeId sweepId;

                bool hasPosition = false;
                /// Intersection position.
                osg::Vec3f position;
                /// Intersection normal of shape at intersection with sweep.
                osg::Vec3f normal;

                const char* source = nullptr;
                //std::vector<osg::Vec3f> linestrip;

                typedef struct Range {
                    float minRatio = 0.0f;
                    float rangeRatio = 1.0f;

                    void toRange(const struct Range& range)
                    {
                        minRatio = range.minRatio + minRatio * range.rangeRatio;
                        rangeRatio *= range.rangeRatio;
                    }
                } Range;

                Instant(float newRatio) :
                    ratio(newRatio)
                {
                }

                Instant(float newRatio,
                        const char* source) :
                    ratio(newRatio),
                    source(source)
                {
                }

                Instant(float newRatio,
                        const osg::Vec3f& position,
                        const osg::Vec3f& normal,
                        const char* source) :
                    ratio(newRatio),
                    hasPosition(true),
                    position(position),
                    normal(normal),
                    source(source)
                {
                }

                void toRange(const Range& range)
                {
                    ratio = range.minRatio + ratio * range.rangeRatio;
                }

                bool atMin() const
                {
                    return ratio <= 0.0f;
                }
                bool atMax() const
                {
                    return ratio >= 1.0f;
                }

                bool operator <(const Instant& other) const
                {
                    return ratio < other.ratio;
                }
                bool operator >(const Instant& other) const
                {
                    return ratio > other.ratio;
                }
                bool operator <=(const Instant& other) const
                {
                    return ratio <= other.ratio;
                }
                bool operator >=(const Instant& other) const
                {
                    return ratio >= other.ratio;
                }

                friend std::ostream& operator <<(std::ostream &ost, const Instant& instant)
                {
                    return ost << instant.ratio;
                }
        };
        typedef TIntersection<Instant> Intersection;
};

class Strip : public Sweep
{
    public:
        class Instant : public Sweep::Instant
        {
            public:
                typedef Sweep::Instant Super;

                unsigned int segment;

                Instant(const Super& super, unsigned int seg) :
                    Super(super),
                    segment(seg)
                {
                }

                bool operator <(const Instant& other) const
                {
                    return segment+1 < other.segment ||
                        (segment+1 == other.segment && !(atMax() && other.atMin())) ||
                        (segment == other.segment && Super::operator <(other));
                }
                bool operator >(const Instant& other) const
                {
                    return segment > other.segment+1 ||
                        (segment == other.segment+1 && !(other.atMax() && atMin())) ||
                        (segment == other.segment && Super::operator >(other));
                }
                bool operator <=(const Instant& other) const
                {
                    return segment < other.segment ||
                        (segment == other.segment && Super::operator <=(other)) ||
                        (segment == other.segment+1 && other.atMax() && atMin());
                }
                bool operator >=(const Instant& other) const
                {
                    return segment > other.segment ||
                        (segment == other.segment && Super::operator >=(other));
                        (segment+1 == other.segment && atMax() && other.atMin());
                }

                friend std::ostream& operator <<(std::ostream &ost, const Instant& instant)
                {
                    return ost << instant.segment << ":" << (const Super&)instant;
                }
        };
        typedef TIntersection<Instant> Intersection;
};

template <typename INTERSECTION>
class TIntersections : public std::multiset<INTERSECTION>
{
    public:
        typedef INTERSECTION Intersection;
        typedef typename Intersection::Instant Instant;
        typedef typename Intersection::Range Range;

        TIntersections(const ShapeId* staticId = nullptr,
                       const ShapeId* sweepId = nullptr,
                       bool wantsNormals = true,
                       bool wantsPositions = true,
                       bool stopASAP = false) :
            _stopASAP(stopASAP),
            _wantsPositions(wantsPositions),
            _wantsNormals(wantsNormals),
            _staticId(staticId),
            _sweepId(sweepId)
        {
        }

        explicit TIntersections(const TIntersections& other) :
            _stopASAP(other.shouldStopASAP()),
            _wantsPositions(other.wantsPositions()),
            _wantsNormals(other.wantsNormals()),
            _staticId(other.getStaticId()),
            _sweepId(other.getSweepId())
        {
        }

        template <typename OTHER>
        explicit TIntersections(const TIntersections<OTHER>& other) :
            _stopASAP(other.shouldStopASAP()),
            _wantsPositions(other.wantsPositions()),
            _wantsNormals(other.wantsNormals()),
            _staticId(other.getStaticId()),
            _sweepId(other.getSweepId())
        {
        }

        /// Move constructor
        TIntersections(TIntersections&& other) = default;

#if 0
        void setIds(const ShapeId* staticId, const ShapeId* sweepId)
        {
            _staticId = staticId;
            _sweepId = sweepId;
        }

        void swapIds()
        {
            std::swap(_staticId, _sweepId);
        }
#endif

        const ShapeId* getStaticId() const
        {
            return _staticId;
        }
        const ShapeId* getSweepId() const
        {
            return _sweepId;
        }

        void insertIntersection(Intersection hit)
        {
            // Apply the current range
            hit.toRange(_range);

            // Copy the IDs
            if (_staticId && hit.exit.staticId.empty())
                hit.exit.staticId = hit.entry.staticId = *_staticId;
            if (_sweepId && hit.exit.sweepId.empty())
                hit.exit.sweepId = hit.entry.sweepId = *_sweepId;
#if 0
            if (_staticFrame) {
                if (hit.entry.staticId.empty())
                    _staticFrame->fillShapeId(hit.entry.staticId);
                if (hit.exit.staticId.empty())
                    _staticFrame->fillShapeId(hit.exit.staticId);
            }
            if (_sweepFrame) {
                if (hit.entry.sweepId.empty())
                    _sweepFrame->fillShapeId(hit.entry.sweepId);
                if (hit.exit.sweepId.empty())
                    _sweepFrame->fillShapeId(hit.exit.sweepId);
            }
#endif

            // Find overlaps
            auto [a, b] = this->equal_range(hit);
            if (a == b) {
                this->insert(hit);
                return;
            }
            // Combine new and existing overlapping intersections
            for (auto it = a; it != b; ++it)
                hit.combine(*it);
            // Remove redundant overlaps
            this->erase(a, b);
            // Insert new combined intersection
            this->insert(hit);
        }

        void insertIntersection(const Instant& entry, const Instant& exit)
        {
            insertIntersection(Intersection(entry, exit));
        }

        bool shouldStopASAP() const
        {
            return _stopASAP;
        }

        bool wantsPositions() const
        {
            return _wantsPositions;
        }

        // implies wantsPositions()
        bool wantsNormals() const
        {
            return _wantsNormals;
        }

        Range pushRange(const Range& range)
        {
            Range old = _range;
            _range = range;
            _range.toRange(old);
            return old;
        }

        void popRange(const Range& range)
        {
            _range = range;
        }

    protected:
        bool _stopASAP;
        bool _wantsPositions;
        bool _wantsNormals;
        Range _range;
        const ShapeId* _staticId;
        const ShapeId* _sweepId;
};

template <typename SHAPE, bool REF = false>
class TSweep : public Sweep
{
    public:
        typedef typename SHAPE::SweepData SingleSweepData;
        typedef typename SHAPE::FixedData FixedData;
        typedef typename SHAPE::Id Id;

        typedef typename TRefHelper<REF>::template Types<FixedData>::ValOrRef FixedValOrRef;
        typedef typename TRefHelper<REF>::template Types<SingleSweepData>::ValOrRef SingleSweepValOrRef;
        typedef typename TRefHelper<REF>::template Types<FixedData>::InitRef FixedInitRef;
        typedef typename TRefHelper<REF>::template Types<SingleSweepData>::InitRef SingleSweepInitRef;

        typedef struct SweepData {
            SingleSweepValOrRef start;
            SingleSweepValOrRef end;
        } SweepData;
        enum {
            shouldCheckBounds = SHAPE::shouldCheckBounds,
        };

        // These are either stored by value or by reference depending on REF
        FixedValOrRef fixed;
        SweepData sweep;

        TSweep(FixedInitRef fixed,
               const SweepData& sweep) :
            fixed(fixed),
            sweep(sweep)
        {
        }

        TSweep(FixedInitRef fixed,
               SingleSweepInitRef start, SingleSweepInitRef end) :
            fixed(fixed),
            sweep{start, end}
        {
        }

        TSweep(const SHAPE& start, const SHAPE& end) :
            fixed(start.fixed),
            sweep{start.sweep, end.sweep}
        {
            //assert(start.fixed == end.fixed);
        }

        SHAPE getStart() const
        {
            return SHAPE(fixed, sweep.start);
        }

        SHAPE getEnd() const
        {
            return SHAPE(fixed, sweep.end);
        }

        template <typename FUNCTOR>
        void perPart(FUNCTOR &functor, Id* id) const
        {
            SHAPE::Info::template perSweepPart<FUNCTOR>(functor, this->fixed,
                                                                 this->sweep.start,
                                                                 this->sweep.end,
                                                                 id);
        }

        static auto getBounds(const FixedData& fixed, const SweepData& sweep)
        {
            auto bb = SHAPE::getBounds(fixed, sweep.start);
            bb.expandBy(SHAPE::getBounds(fixed, sweep.end));
            return bb;
        }

        auto getBounds() const
        {
            return getBounds(fixed, sweep);
        }
};

template <typename SHAPE>
class TStrip : public Strip
{
    public:
        typedef typename SHAPE::SweepData SingleSweepData;
        typedef typename SHAPE::FixedData FixedData;
        typedef typename SHAPE::ShapeRef SingleShapeRef;
        typedef struct {
            std::vector<float> ratios;
            std::vector<SingleSweepData> strip;
            mutable bool boundingBoxSet = false;
            mutable osg::BoundingBox boundingBox;
        } SharedSweepData;
        typedef std::shared_ptr<SharedSweepData> SweepData;
        typedef typename SHAPE::Id SingleId;
        typedef struct Id {
            enum : unsigned int {
                NODE = 0,
                SEGMENT = 1,
            } type : 1;
            unsigned int index : 31;
            union {
                SingleId node;
                SingleId segment;
            };

            friend std::ostream& operator << (std::ostream& ost,
                                              const struct Id& id)
            {
                switch (id.type) {
                case NODE:
                    ost << "Strip{type:NODE,index:" << id.index << ",node:" << id.node << "}";
                    break;
                case SEGMENT:
                    ost << "Strip{type:SEGMENT,index:" << id.index << ",segment:" << id.segment << "}";
                    break;
                }
                return ost;
            }
        } Id;
        enum {
            shouldCheckBounds = SHAPE::shouldCheckBounds,
        };

        FixedData fixed;
        SweepData sweep;

        TStrip() :
            sweep(std::make_shared<SharedSweepData>())
        {
        }

        void addNode(float ratio, const SHAPE& node)
        {
            if (sweep->strip.empty())
                fixed = node.fixed;
            sweep->ratios.push_back(ratio);
            sweep->strip.push_back(node.sweep);
        }

        bool empty() const
        {
            return sweep->strip.empty();
        }
        unsigned int numNodes() const
        {
            return sweep->strip.size();
        }
        SingleShapeRef getNode(unsigned int index)
        {
            return SingleShapeRef(fixed, sweep->strip[index]);
        }
        const SingleShapeRef getNode(unsigned int index) const
        {
            return const_cast<TStrip*>(this)->getNode(index);
        }
        float getNodeRatio(unsigned int index) const
        {
            return sweep->ratios[index];
        }
        unsigned int numSegments() const
        {
            if (numNodes() < 2)
                return 0;
            return numNodes() - 1;
        }
        TSweep<SHAPE, true> getSegment(unsigned int index)
        {
            return TSweep<SHAPE, true>(fixed,
                                       sweep->strip[index],
                                       sweep->strip[index+1]);
        }
        const TSweep<SHAPE, true> getSegment(unsigned int index) const
        {
            // Necessary to pass non-constant references to constructor
            // It gets made const again afterwards so all's well
            return const_cast<TStrip*>(this)->getSegment(index);
        }

        float getMinRatio() const
        {
            assert(!empty());
            return sweep->ratios.front();
        }

        float getMaxRatio() const
        {
            assert(!empty());
            return sweep->ratios.back();
        }

        float getRatio(const Strip::Instant& instant) const
        {
            //return (instant.ratio + instant.segment) / numSegments();
            float ratio0 = sweep->ratios[instant.segment];
            float ratio1 = sweep->ratios[instant.segment+1];
            float ratioRange = ratio1 - ratio0;
            return ratio0 + instant.ratio * ratioRange;
        }

        static const osg::BoundingBox& getBounds(const FixedData& fixed, const SweepData& sweep)
        {
            if (!sweep->boundingBoxSet) {
                for (unsigned int i = 0; i < sweep->strip.size(); ++i)
                    sweep->boundingBox.expandBy(SHAPE::getBounds(fixed, sweep->strip[i]));
                sweep->boundingBoxSet = true;
            }
            return sweep->boundingBox;
        }

        auto getBounds() const
        {
            return getBounds(fixed, sweep);
        }
};

template <typename SHAPE>
struct TGroupInfo : public ShapeInfo
{
    typedef typename SHAPE::FixedData SingleFixedData;
    typedef typename SHAPE::SweepData SingleSweepData;
    typedef typename SHAPE::Id SingleId;
    typedef struct {
        std::vector<SingleFixedData> items;
    } FixedData;
    typedef struct {
        std::vector<SingleSweepData> items;
        osg::BoundingBox bb;
    } SweepData;
    typedef struct Id {
        unsigned int index;
        SingleId item;

        friend std::ostream& operator << (std::ostream& ost,
                                          const struct Id& id)
        {
            return ost << "Group{index:" << id.index << ",item:" << id.item << "}";
        }
    } Id;
    enum {
        shouldCheckBounds = SHAPE::shouldCheckBounds,
    };

    typedef typename SHAPE::ShapeRef SingleShapeRef;

    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor,
                        const FixedData& fixed,
                        const SweepData& sweep,
                        Id* id)
    {
        for (unsigned int i = 0; i < fixed.items.size(); ++i) {
            id->index = i;
            functor((const SingleShapeRef&)SingleShapeRef(const_cast<SingleFixedData&>(fixed.items[i]),
                                                          const_cast<SingleSweepData&>(sweep.items[i])), &id->item);
        }
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor,
                             const FixedData& fixed,
                             const SweepData& start,
                             const SweepData& end,
                             Id* id)
    {
        for (unsigned int i = 0; i < fixed.items.size(); ++i) {
            id->index = i;
            // Necessary to pass non-constant references to constructor
            // It gets made const again afterwards so all's well
            TSweep<SHAPE, true> shape(const_cast<SingleFixedData&>(fixed.items[i]),
                                      const_cast<SingleSweepData&>(start.items[i]),
                                      const_cast<SingleSweepData&>(end.items[i]));
            functor((const TSweep<SHAPE, true>&)shape, &id->item);
        }
    }

    static const osg::BoundingBox& getBounds(const FixedData& fixed,
                                             const SweepData& sweep)
    {
        return sweep.bb;
    }
};
template <typename SHAPE, bool REF = false>
class TGroup : public TCompound<TGroupInfo<SHAPE>, REF>
{
    public:
        typedef TCompound<TGroupInfo<SHAPE>, REF> Super;

        typedef typename Super::FixedInitRef FixedInitRef;
        typedef typename Super::SweepInitRef SweepInitRef;

        typedef typename SHAPE::ShapeRef SingleShapeRef;

        // A similar shape but with REF=true
        typedef TGroup<SHAPE, true> ShapeRef;

        TGroup() :
            Super({}, {})
        {
        }

        TGroup(FixedInitRef fixed,
               SweepInitRef sweep) :
            Super(fixed, sweep)
        {
        }

        void reserve(unsigned int items)
        {
            this->fixed.items.reserve(items);
            this->sweep.items.reserve(items);
        }

        unsigned int addItem(const SingleShapeRef& item)
        {
            unsigned int index = this->fixed.items.size();
            this->fixed.items.push_back(item.fixed);
            this->sweep.items.push_back(item.sweep);
            this->sweep.bb.expandBy(item.getBounds());

            return index;
        }

        void updateBounds()
        {
            for (unsigned int i = 0; i < numItems(); ++i)
                this->sweep.bb.expandBy(getItem(i).getBounds());
        }

        unsigned int numItems() const
        {
            return this->fixed.items.size();
        }
        SingleShapeRef getItem(unsigned int index)
        {
            return SingleShapeRef(this->fixed.items[index], this->sweep.items[index]);
        }
        const SingleShapeRef getItem(unsigned int index) const
        {
            // Necessary to pass non-constant references to constructor
            // It gets made const again afterwards so all's well
            return const_cast<TGroup*>(this)->getItem(index);
        }
};

template <typename SHAPE, typename METADATA>
struct TMetadataInfo : public ShapeInfo
{
    typedef typename SHAPE::FixedData ItemFixedData;
    typedef typename SHAPE::SweepData ItemSweepData;
    typedef METADATA Metadata;

    typedef struct {
        ItemFixedData data;
        Metadata meta;
    } FixedData;
    typedef ItemSweepData SweepData;
    typedef typename SHAPE::Id Id;

    typedef typename SHAPE::ShapeRef ItemShapeRef;

    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor,
                        const FixedData& fixed,
                        const SweepData& sweep,
                        Id* id)
    {
        functor((const ItemShapeRef&)ItemShapeRef(const_cast<ItemFixedData&>(fixed.data),
                                                  const_cast<ItemSweepData&>(sweep)), id);
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor,
                             const FixedData& fixed,
                             const SweepData& start,
                             const SweepData& end,
                             Id* id)
    {
        TSweep<SHAPE, true> shape(const_cast<ItemFixedData&>(fixed.data),
                                  const_cast<ItemSweepData&>(start),
                                  const_cast<ItemSweepData&>(end));
        functor((const TSweep<SHAPE, true>&)shape, id);
    }
};

template <typename SHAPE, typename METADATA, bool REF = false>
class TMetadata : public TCompound<TMetadataInfo<SHAPE, METADATA>, REF>
{
    public:
        typedef TCompound<TMetadataInfo<SHAPE, METADATA>, REF> Super;

        typedef METADATA Metadata;

        typedef typename Super::FixedInitRef FixedInitRef;
        typedef typename Super::SweepInitRef SweepInitRef;

        typedef typename SHAPE::ShapeRef ItemShapeRef;

        // A similar shape but with REF=true
        typedef TMetadata<SHAPE, METADATA, true> ShapeRef;

        TMetadata() :
            Super({}, {})
        {
        }
        TMetadata(const Metadata& meta) :
            Super({{}, meta}, {})
        {
        }
        TMetadata(const SHAPE& shape, const Metadata& meta) :
            Super({shape.fixed, meta}, shape.sweep)
        {
        }
        TMetadata(FixedInitRef fixed,
                  SweepInitRef sweep) :
            Super(fixed, sweep)
        {
        }

        // Implicitly create a shape reference
        operator ShapeRef()
        {
            return ShapeRef(this->fixed, this->sweep);
        }

        ItemShapeRef getItem()
        {
            return ItemShapeRef(this->fixed.data, this->sweep);
        }
        const ItemShapeRef getItem() const
        {
            // Necessary to pass non-constant references to constructor
            // It gets made const again afterwards so all's well
            return const_cast<TMetadata*>(this)->getItem();
        }

        Metadata& getMetadata()
        {
            return this->fixed.meta;
        }
        const Metadata& getMetadata() const
        {
            return this->fixed.meta;
        }
        void setMetadata(Metadata& meta)
        {
            this->fixed.meta = meta;
        }

        auto getBounds() const
        {
            return getItem().getBounds();
        }
};

// Anything x Strip -> [Anything x Sweep]
template <typename SHAPE, typename STRIP>
unsigned int rawIntersect(const SHAPE& shape, const TStrip<STRIP>& strip,
                          TIntersections<typename TStrip<STRIP>::Intersection>& intersections,
                          typename SHAPE::Id* shapeId, typename TStrip<STRIP>::Id* stripId)
{
    unsigned int total = 0;
    for (unsigned int i = 0; i < strip.numSegments(); ++i) {
        stripId->index = i;

        TIntersections<typename TSweep<STRIP>::Intersection> localIntersections(intersections);
        total += intersect(shape, strip.getSegment(i), localIntersections, shapeId, &stripId->segment);
        for (auto &hit: localIntersections)
            intersections.insertIntersection(typename TStrip<STRIP>::Instant(hit.entry, i),
                                             typename TStrip<STRIP>::Instant(hit.exit, i));
        // Stop early if we found an intersection
        if (intersections.shouldStopASAP() && total)
            return total;
    }
    return total;
}

// Anything x Sweep<Compound> -> [Anything x Sweep<Shape>]
template <typename SHAPE, typename SWEEP>
struct TCompoundSweepFunctor {
    const SHAPE& shape;
    TIntersections<typename TSweep<SWEEP>::Intersection>& intersections;
    typename SHAPE::Id* shapeId;
    unsigned int total;

    auto& getFrame()
    {
        return intersections.getSweepFrame();
    }

    template <typename T>
    void operator () (const T& part, typename T::Id* partId = nullptr)
    {
        total += intersect(shape, part, intersections, shapeId, partId);
    }
};
template <typename SHAPE, typename SWEEP, bool COMPOUND_REF, bool SWEEP_REF>
unsigned int rawIntersect(const SHAPE& shape, const TSweep<TCompound<SWEEP, COMPOUND_REF>, SWEEP_REF>& sweep,
                          TIntersections<typename TSweep<SWEEP>::Intersection>& intersections,
                          typename SHAPE::Id* shapeId, typename TSweep<TCompound<SWEEP, COMPOUND_REF>>::Id* sweepId)
{
    TCompoundSweepFunctor<SHAPE, SWEEP> functor{shape, intersections, shapeId, 0};
    sweep.template perPart(functor, sweepId);
    return functor.total;
}
// Shape x TSweep<TMetadata<TCompound>>
template <typename SHAPE, typename SWEEP, typename METADATA, bool COMPOUND_REF, bool SWEEP_REF, bool METADATA_REF>
unsigned int rawIntersect(const SHAPE& shape, const TSweep<TMetadata<TCompound<SWEEP, COMPOUND_REF>, METADATA, METADATA_REF>, SWEEP_REF>& sweep,
                          TIntersections<typename TSweep<SWEEP>::Intersection>& intersections,
                          typename SHAPE::Id* shapeId, typename TSweep<TCompound<SWEEP, COMPOUND_REF>>::Id* sweepId)
{
    TCompoundSweepFunctor<SHAPE, SWEEP> functor{shape, intersections, shapeId, 0};
    sweep.template perPart(functor, sweepId);
    return functor.total;
}

// A group is a fancy compound
// Anything x Sweep<Group<Shape>> -> [Anything x Sweep<Shape>]
template <typename SHAPE, typename SWEEP, bool SWEEP_REF, bool GROUP_REF>
unsigned int rawIntersect(const SHAPE& shape, const TSweep<TGroup<SWEEP, GROUP_REF>, SWEEP_REF>& sweep,
                          TIntersections<typename TSweep<SWEEP>::Intersection>& intersections,
                          typename SHAPE::Id* shapeId, typename TSweep<TGroup<SWEEP, GROUP_REF>>::Id* sweepId)
{
    TCompoundSweepFunctor<SHAPE, SWEEP> functor{shape, intersections, shapeId, 0};
    sweep.template perPart(functor, sweepId);
    return functor.total;
}

// Group<Shape> x Anything -> [Shape x Anything]
template <typename SWEEP>
struct TCompoundShapeFunctor {
    const SWEEP& sweep;
    TIntersections<typename SWEEP::Intersection>& intersections;
    typename SWEEP::Id* sweepId;
    unsigned int total;

    auto& getFrame()
    {
        return intersections.getStaticFrame();
    }

    template <typename T>
    void operator () (const T& part, typename T::Id* partId = nullptr)
    {
        total += intersect(part, sweep, intersections, partId, sweepId);
    }
};
template <typename SHAPE, bool SHAPE_REF, typename SWEEP>
unsigned int rawIntersect(const TCompound<SHAPE, SHAPE_REF>& compound, const SWEEP& sweep,
                          TIntersections<typename SWEEP::Intersection>& intersections,
                          typename TCompound<SHAPE, SHAPE_REF>::Id* compoundId, typename SWEEP::Id* sweepId)
{
    TCompoundShapeFunctor<SWEEP> functor{sweep, intersections, sweepId, 0};
    compound.template perPart(functor, compoundId);
    return functor.total;
}

// Strip<Shape1> x Sweep<Shape2> -> [Shape1 x Sweep<Shape2>]
// Sweeps of groups should be broken up first, above, hence TShape here to avoid
// ambiguity.
template <typename STRIP, typename SWEEPINFO, bool SWEEP_REF>
unsigned int rawIntersect(const TStrip<STRIP>& shape, const TSweep<TShape<SWEEPINFO>, SWEEP_REF>& sweep,
                          TIntersections<typename TSweep<TShape<SWEEPINFO>>::Intersection>& intersections,
                          typename TStrip<STRIP>::Id* shapeId, typename TSweep<TShape<SWEEPINFO>>::Id* sweepId)
{
    unsigned int total = 0;
    unsigned int i;
    //ShapeIdFrame<unsigned int> staticFrame(intersections.getStaticFrame());
    for (i = 0; i < shape.numSegments(); ++i) {
        shapeId->index = i;
        shapeId->type = TStrip<STRIP>::Id::NODE;
        total += intersect(shape.getNode(i), sweep, intersections, &shapeId->node, sweepId);
        shapeId->type = TStrip<STRIP>::Id::SEGMENT;
        total += intersect(shape.getSegment(i), sweep, intersections, &shapeId->segment, sweepId);
    }
    if (i < shape.numNodes()) {
        shapeId->index = i;
        shapeId->type = TStrip<STRIP>::Id::NODE;
        total += intersect(shape.getNode(i), sweep, intersections, &shapeId->node, sweepId);
    }
    return total;
}

template <int shouldCheckBounds>
struct boundsCheck;

template <>
struct boundsCheck<0>
{
    template <typename SHAPE, typename SWEEP>
    static bool check(const SHAPE& shape, const SWEEP& sweep)
    {
        return true;
    }
};

template <>
struct boundsCheck<1>
{
    template <typename SHAPE, typename SWEEP>
    static bool check(const SHAPE& shape, const SWEEP& sweep)
    {
        auto shapeBounds = shape.getBounds();
        auto sweepBounds = sweep.getBounds();
        bool ret = shapeBounds.intersects(sweepBounds);
        ++stats[ret?1:0];
        return ret;
    }

    static unsigned int stats[2];
};

template <typename SHAPE, typename SWEEP>
unsigned int intersect(const SHAPE& shape,
                       const SWEEP& sweep,
                       TIntersections<typename SWEEP::Intersection>& intersections,
                       typename SHAPE::Id* shapeId, typename SWEEP::Id* sweepId)
{
#if 1
    // Do a bounds check if possible
    if (!boundsCheck<SHAPE::shouldCheckBounds && SWEEP::shouldCheckBounds>::check(shape, sweep))
        return 0;
#endif
    unsigned int ret = rawIntersect(shape, sweep, intersections, shapeId, sweepId);
#if 0
    // Do a bounds check if possible
    if (!boundsCheck<SHAPE::shouldCheckBounds && SWEEP::shouldCheckBounds>::check(shape, sweep)) {
        if (ret) {
            std::cout << "Bounds check failed but intersection found" << std::endl;
            abort();
        }
    }
#endif
    return ret;
}

// Top level
#if 0
template <typename SHAPE, typename SWEEP, typename INTERSECTIONS>
unsigned int doIntersect(const SHAPE& shape,
                         const SWEEP& sweep,
                         INTERSECTIONS& intersections)
{
    ShapeId shapeId, sweepId;
    intersections.setIds(&shapeId, &sweepId);
    return intersect(shape, sweep, intersections,
                     shapeId.init<SHAPE>(),
                     sweepId.init<SWEEP>());
}
#endif

// Wrapper to return list of intersections
template <typename SHAPE, typename SWEEP>
TIntersections<typename SWEEP::Intersection> intersect(const SHAPE& shape,
                                                       const SWEEP& sweep)
{
    ShapeId shapeId, sweepId;
    TIntersections<typename SWEEP::Intersection> intersections(&shapeId, &sweepId);
    intersect(shape, sweep, intersections,
              shapeId.init<SHAPE>(),
              sweepId.init<SWEEP>());
    return intersections;
}

};

#endif
