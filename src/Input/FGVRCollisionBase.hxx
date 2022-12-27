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

#include <cassert>
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

template <typename INFO>
class TShape : public Shape
{
    public:
        typedef INFO Info;
        typedef typename INFO::FixedData FixedData;
        typedef typename INFO::SweepData SweepData;

        FixedData fixed;
        SweepData sweep;

        TShape(const FixedData& fixed, const SweepData& sweep) :
            fixed(fixed),
            sweep(sweep)
        {
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

        // ratioEntry <= ratioExit
        Instant entry;
        Instant exit;

        TIntersection(const Instant &newEntry, const Instant &newExit) :
            entry(newEntry),
            exit(newExit)
        {
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

                Instant(float newRatio) :
                    ratio(newRatio)
                {
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

        TIntersections(bool stopASAP = false) :
            _stopASAP(stopASAP)
        {
        }

        template <typename OTHER>
        explicit TIntersections(const TIntersections<OTHER>& other) :
            _stopASAP(other.shouldStopASAP())
        {
        }

        void insertIntersection(const Intersection& intersection)
        {
            // Find overlaps
            auto [a, b] = this->equal_range(intersection);
            if (a == b) {
                this->insert(intersection);
                return;
            }
            // Combine new and existing overlapping intersections
            Intersection hit(intersection);
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

    protected:
        bool _stopASAP;
};

template <typename SHAPE>
class TSweep : public Sweep
{
    public:
        typedef typename SHAPE::SweepData SingleSweepData;
        typedef typename SHAPE::FixedData FixedData;
        typedef struct {
            SingleSweepData start;
            SingleSweepData end;
        } SweepData;

        FixedData fixed;
        SweepData sweep;

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
            assert(start.fixed == end.fixed);
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
};

template <typename SHAPE>
class TStrip : public Strip
{
    public:
        typedef typename SHAPE::SweepData SingleSweepData;
        typedef typename SHAPE::FixedData FixedData;
        typedef struct {
            std::vector<SingleSweepData> strip;
        } SweepData;

        FixedData fixed;
        SweepData sweep;

        void addNode(const SHAPE& node)
        {
            if (sweep.strip.empty())
                fixed = node.fixed;
            sweep.strip.push_back(node.sweep);
        }

        unsigned int numNodes() const
        {
            return sweep.strip.size();
        }
        SHAPE getNode(unsigned int index) const
        {
            return SHAPE(fixed, sweep.strip[index]);
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
                                 sweep.strip[index], sweep.strip[index+1]);
        }

        float getOverallRatio(const Strip::Instant& instant) const
        {
            return (instant.ratio + instant.segment) / numSegments();
        }
};

template <typename SHAPE>
struct TGroupInfo
{
    typedef typename SHAPE::FixedData SingleFixedData;
    typedef typename SHAPE::SweepData SingleSweepData;
    typedef struct {
        std::vector<SingleFixedData> items;
    } FixedData;
    typedef struct {
        std::vector<SingleSweepData> items;
    } SweepData;

    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor, const FixedData& fixed, const SweepData& sweep)
    {
        for (unsigned int i = 0; i < fixed.items.size(); ++i)
            functor(SHAPE(fixed.items[i], sweep.items[i]));
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor, const FixedData& fixed, const SweepData& start, const SweepData& end)
    {
        for (unsigned int i = 0; i < fixed.items.size(); ++i)
            functor(TSweep<SHAPE>(fixed.items[i], start.items[i], end.items[i]));
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

        void addItem(const SHAPE& item)
        {
            this->fixed.items.push_back(item.fixed);
            this->sweep.items.push_back(item.sweep);
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
unsigned int intersect(const SHAPE& shape, const TStrip<STRIP>& strip,
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
struct TCompoundFunctor {
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
unsigned int intersect(const SHAPE& shape, const TSweep<TCompound<SWEEP>>& sweep,
                       TIntersections<typename TSweep<SWEEP>::Intersection>& intersections)
{
    TCompoundFunctor<SHAPE, SWEEP> functor{shape, intersections, 0};
    sweep.template perPart(functor);
    return functor.total;
}

// A group is a fancy compound
// Anything x Sweep<Group<Shape>> -> [Anything x Sweep<Shape>]
template <typename SHAPE, typename SWEEP>
unsigned int intersect(const SHAPE& shape, const TSweep<TGroup<SWEEP>>& sweep,
                       TIntersections<typename TSweep<SWEEP>::Intersection>& intersections)
{
    TCompoundFunctor<SHAPE, SWEEP> functor{shape, intersections, 0};
    sweep.template perPart(functor);
    return functor.total;
    /*
    unsigned int total = 0;
    for (unsigned int i = 0; i < sweep.fixed.items.size(); ++i) {
        TSweep<SWEEP> itemSweep(sweep.fixed.items[i],
                                sweep.sweep.start.items[i],
                                sweep.sweep.end.items[i]);
        total += intersect(shape, itemSweep, intersections);
    }
    return total;
    */
}

// Strip<Shape1> x Sweep<Shape2> -> [Shape1 x Sweep<Shape2>]
// Sweeps of groups should be broken up first, above, hence TShape here to avoid
// ambiguity.
template <typename STRIP, typename SWEEPINFO>
unsigned int intersect(const TStrip<STRIP>& shape, const TSweep<TShape<SWEEPINFO>>& sweep,
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
