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

namespace FGVRCollision
{

// ABSTRACT TYPES

class Shape
{
};

typedef struct {} EmptyData;

struct ShapeInfo
{
    typedef EmptyData FixedData;
    typedef EmptyData SweepData;
    enum {
        shouldCheckBounds = 0,
    };
};

template <typename INFO>
class TShape : public Shape
{
    public:
        typedef INFO Info;
        typedef typename INFO::FixedData FixedData;
        typedef typename INFO::SweepData SweepData;
        enum {
            shouldCheckBounds = INFO::shouldCheckBounds,
        };

        FixedData fixed;
        SweepData sweep;

        TShape(const FixedData& fixed,
               const SweepData& sweep) :
            fixed(fixed),
            sweep(sweep)
        {
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

template <typename INFO>
class TCompound : public TShape<INFO>
{
    public:
        typedef TShape<INFO> Super;
        typedef typename INFO::FixedData FixedData;
        typedef typename INFO::SweepData SweepData;

        TCompound(const FixedData& fixed, const SweepData& sweep) :
            Super(fixed, sweep)
        {
        }

        template <typename FUNCTOR>
        auto perPart(FUNCTOR &functor) const
        {
            INFO::template perPart<FUNCTOR>(functor, this->fixed, this->sweep);
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

        TIntersections(bool wantsNormals = true,
                       bool wantsPositions = true,
                       bool stopASAP = false) :
            _stopASAP(stopASAP),
            _wantsPositions(wantsPositions),
            _wantsNormals(wantsNormals)
        {
        }

        explicit TIntersections(const TIntersections& other) :
            _stopASAP(other.shouldStopASAP()),
            _wantsPositions(other.wantsPositions()),
            _wantsNormals(other.wantsNormals())
        {
        }

        template <typename OTHER>
        explicit TIntersections(const TIntersections<OTHER>& other) :
            _stopASAP(other.shouldStopASAP()),
            _wantsPositions(other.wantsPositions()),
            _wantsNormals(other.wantsNormals())
        {
        }

        void insertIntersection(Intersection hit)
        {
            // Apply the current range
            hit.toRange(_range);

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
};

template <typename SHAPE>
class TSweep : public Sweep
{
    public:
        typedef typename SHAPE::SweepData SingleSweepData;
        typedef typename SHAPE::FixedData FixedData;
        typedef struct SweepData {
            SingleSweepData start;
            SingleSweepData end;
        } SweepData;
        enum {
            shouldCheckBounds = SHAPE::shouldCheckBounds,
        };

        FixedData fixed;
        SweepData sweep;

        TSweep(const FixedData& fixed,
               const SweepData& sweep) :
            fixed(fixed),
            sweep(sweep)
        {
        }

        TSweep(const FixedData& fixed,
               const SingleSweepData& start, const SingleSweepData& end) :
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
        void perPart(FUNCTOR &functor) const
        {
            SHAPE::Info::template perSweepPart<FUNCTOR>(functor, this->fixed,
                                                                 this->sweep.start,
                                                                 this->sweep.end);
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
        typedef struct {
            std::vector<float> ratios;
            std::vector<SingleSweepData> strip;
            mutable bool boundingBoxSet = false;
            mutable osg::BoundingBox boundingBox;
        } SharedSweepData;
        typedef std::shared_ptr<SharedSweepData> SweepData;
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
        SHAPE getNode(unsigned int index) const
        {
            return SHAPE(fixed, sweep->strip[index]);
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
        TSweep<SHAPE> getSegment(unsigned int index) const
        {
            return TSweep<SHAPE>(fixed,
                                 sweep->strip[index], sweep->strip[index+1]);
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
    typedef struct {
        std::vector<SingleFixedData> items;
    } FixedData;
    typedef struct {
        std::vector<SingleSweepData> items;
        osg::BoundingBox bb;
    } SweepData;
    enum {
        shouldCheckBounds = SHAPE::shouldCheckBounds,
    };

    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor,
                        const FixedData& fixed,
                        const SweepData& sweep)
    {
        for (unsigned int i = 0; i < fixed.items.size(); ++i)
            functor(SHAPE(fixed.items[i], sweep.items[i]));
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor,
                             const FixedData& fixed,
                             const SweepData& start,
                             const SweepData& end)
    {
        for (unsigned int i = 0; i < fixed.items.size(); ++i)
            functor(TSweep<SHAPE>(fixed.items[i], start.items[i], end.items[i]));
    }

    static const osg::BoundingBox& getBounds(const FixedData& fixed,
                                             const SweepData& sweep)
    {
        return sweep.bb;
    }
};
template <typename SHAPE>
class TGroup : public TCompound<TGroupInfo<SHAPE>>
{
    public:
        typedef TCompound<TGroupInfo<SHAPE>> Super;

        TGroup() :
            Super({}, {})
        {
        }

        void reserve(unsigned int items)
        {
            this->fixed.items.reserve(items);
            this->sweep.items.reserve(items);
        }

        void addItem(const SHAPE& item)
        {
            this->fixed.items.push_back(item.fixed);
            this->sweep.items.push_back(item.sweep);
            this->sweep.bb.expandBy(item.getBounds());
        }

        unsigned int numItems() const
        {
            return this->fixed.items.size();
        }
        SHAPE getItem(unsigned int index) const
        {
            return SHAPE(this->fixed.items[index], this->sweep.items[index]);
        }
};

// Anything x Strip -> [Anything x Sweep]
template <typename SHAPE, typename STRIP>
unsigned int rawIntersect(const SHAPE& shape, const TStrip<STRIP>& strip,
                          TIntersections<typename TStrip<STRIP>::Intersection>& intersections)
{
    unsigned int total = 0;
    for (unsigned int i = 0; i < strip.numSegments(); ++i) {
        TIntersections<typename TSweep<STRIP>::Intersection> localIntersections(intersections);
        total += intersect(shape, strip.getSegment(i), localIntersections);
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
    unsigned int total;

    template <typename T>
    void operator () (const T& part)
    {
        total += intersect(shape, part, intersections);
    }
};
template <typename SHAPE, typename SWEEP>
unsigned int rawIntersect(const SHAPE& shape, const TSweep<TCompound<SWEEP>>& sweep,
                          TIntersections<typename TSweep<SWEEP>::Intersection>& intersections)
{
    TCompoundSweepFunctor<SHAPE, SWEEP> functor{shape, intersections, 0};
    sweep.template perPart(functor);
    return functor.total;
}

// A group is a fancy compound
// Anything x Sweep<Group<Shape>> -> [Anything x Sweep<Shape>]
template <typename SHAPE, typename SWEEP>
unsigned int rawIntersect(const SHAPE& shape, const TSweep<TGroup<SWEEP>>& sweep,
                          TIntersections<typename TSweep<SWEEP>::Intersection>& intersections)
{
    TCompoundSweepFunctor<SHAPE, SWEEP> functor{shape, intersections, 0};
    sweep.template perPart(functor);
    return functor.total;
}

// Group<Shape> x Anything -> [Shape x Anything]
template <typename SWEEP>
struct TCompoundShapeFunctor {
    const SWEEP& sweep;
    TIntersections<typename SWEEP::Intersection>& intersections;
    unsigned int total;

    template <typename T>
    void operator () (const T& part)
    {
        total += intersect(part, sweep, intersections);
    }
};
template <typename SHAPE, typename SWEEP>
unsigned int rawIntersect(const TCompound<SHAPE>& compound, const SWEEP& sweep,
                       TIntersections<typename SWEEP::Intersection>& intersections)
{
    TCompoundShapeFunctor<SWEEP> functor{sweep, intersections, 0};
    compound.template perPart(functor);
    return functor.total;
}
/*
template <typename SHAPE, typename SWEEP>
unsigned int rawIntersect(const TGroup<SHAPE>& group, const SWEEP& sweep,
                       TIntersections<typename SWEEP::Intersection>& intersections)
{
    TCompoundShapeFunctor<SWEEP> functor{sweep, intersections, 0};
    group.template perPart(functor);
    return functor.total;
}
*/

// Strip<Shape1> x Sweep<Shape2> -> [Shape1 x Sweep<Shape2>]
// Sweeps of groups should be broken up first, above, hence TShape here to avoid
// ambiguity.
template <typename STRIP, typename SWEEPINFO>
unsigned int rawIntersect(const TStrip<STRIP>& shape, const TSweep<TShape<SWEEPINFO>>& sweep,
                          TIntersections<typename TSweep<TShape<SWEEPINFO>>::Intersection>& intersections)
{
    unsigned int total = 0;
    unsigned int i;
    for (i = 0; i < shape.numSegments(); ++i) {
        total += intersect(shape.getNode(i), sweep, intersections);
        total += intersect(shape.getSegment(i), sweep, intersections);
    }
    if (i < shape.numNodes())
        total += intersect(shape.getNode(i), sweep, intersections);
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
                       TIntersections<typename SWEEP::Intersection>& intersections)
{
    // Do a bounds check if possible
    if (!boundsCheck<SHAPE::shouldCheckBounds && SWEEP::shouldCheckBounds>::check(shape, sweep))
        return 0;
    return rawIntersect(shape, sweep, intersections);
}

// Wrapper to return list of intersections
template <typename SHAPE, typename SWEEP>
TIntersections<typename SWEEP::Intersection> intersect(const SHAPE& shape,
                                                       const SWEEP& sweep)
{
    TIntersections<typename SWEEP::Intersection> intersections;
    intersect(shape, sweep, intersections);
    return intersections;
}

};

#endif
