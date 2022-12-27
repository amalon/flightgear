// VR Collision Concrete Classes
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

#ifndef _FGVRCOLLISION_HXX
#define _FGVRCOLLISION_HXX

#include "FGVRCollisionBase.hxx"

#include <osg/Vec3f>

#include <cmath>
#include <utility>

namespace FGVRCollision
{

typedef osg::Vec3f Position;

typedef struct {
    typedef EmptyData FixedData;
    typedef struct {
        Position position;
    } SweepData;
} PointInfo;
typedef TShape<PointInfo> RawPoint;
class Point : public RawPoint
{
    public:
        Point(const Position& position) :
            RawPoint({}, { position })
        {
        }
};

typedef TSweep<RawPoint> Line;
typedef TStrip<RawPoint> LineStrip;

typedef TSweep<Line> LineSweep;

typedef struct {
    typedef struct {
        float radius;
    } FixedData;
    typedef struct {
        Position position;
    } SweepData;
} SphereInfo;
typedef TShape<SphereInfo> RawSphere;
class Sphere : public RawSphere
{
    public:
        Sphere() :
            RawSphere({}, {})
        {
        }

        Sphere(float radius, const Position& position) :
            RawSphere({radius}, {position})
        {
        }

        void set(float radius, const Position& position)
        {
            this->fixed.radius = radius;
            this->sweep.position = position;
        }
};

typedef TSweep<RawSphere> RawSphereSweep;
class SphereSweep : public RawSphereSweep
{
    public:
        SphereSweep(float radius, const Position& start, const Position& end) :
            RawSphereSweep({radius}, {start}, {end})
        {
        }
};

struct RawCapsuleInfo {
    typedef struct {
        float radius[2];
    } FixedData;
    typedef struct {
        Position position[2];
    } SweepData;
};
template <bool startSphere, bool endSphere>
struct TCapsuleInfo : public RawCapsuleInfo {
    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor, const FixedData& fixed, const SweepData& sweep)
    {
        if (startSphere)
            functor(Sphere(fixed.radius[0], sweep.position[0]));
        if (startSphere || endSphere)
            functor(TShape<TCapsuleInfo<false, false>>(fixed, sweep));
        if (endSphere)
            functor(Sphere(fixed.radius[1], sweep.position[1]));
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor, const FixedData& fixed, const SweepData& start, const SweepData& end)
    {
        if (startSphere)
            functor(SphereSweep(fixed.radius[0], start.position[0], end.position[0]));
        if (startSphere || endSphere)
            functor(TSweep<TShape<TCapsuleInfo<false, false>>>(fixed, start, end));
        if (endSphere)
            functor(SphereSweep(fixed.radius[1], start.position[1], end.position[1]));
    }
};
template <typename SUPER>
class TCapsule : public SUPER
{
    public:
        TCapsule(const RawSphere& start, const RawSphere& end) :
            SUPER({ {start.fixed.radius, end.fixed.radius } },
                  { {start.sweep.position, end.sweep.position } })
        {
        }
};

// open both ends
typedef TCapsuleInfo<false, false> OpenCapsuleInfo;
typedef TShape<OpenCapsuleInfo> OpenCapsule;
// Sweep
typedef TSweep<OpenCapsule> OpenCapsuleSweep;

// only spherical at end, start open
typedef TCapsuleInfo<false, true> OneEndCapsuleInfo;
typedef TCompound<OneEndCapsuleInfo> RawOneEndCapsule;
typedef TCapsule<RawOneEndCapsule> OneEndCapsule;
// Sweep
typedef TSweep<RawOneEndCapsule> OneEndCapsuleSweep;
// Group sweeps/strips (like for fingers)
typedef TGroup<RawOneEndCapsule> OneEndCapsuleGroup;
typedef TSweep<OneEndCapsuleGroup> OneEndCapsuleGroupSweep;
typedef TStrip<OneEndCapsuleGroup> OneEndCapsuleGroupStrip;

// two ended capsule
typedef TCapsuleInfo<true, true> CapsuleInfo;
typedef TCompound<CapsuleInfo> RawCapsule;
typedef TCapsule<RawCapsule> Capsule;


typedef struct {
    typedef struct {
        float radius;
    } FixedData;
    typedef struct {
        Line line;
    } SweepData;
} CylinderData;
typedef TShape<CylinderData> Cylinder;

typedef TIntersections<Sweep::Intersection> SweepIntersections;
typedef TIntersections<Strip::Intersection> StripIntersections;

// Intersect a sphere sweep with a point
unsigned int intersect(const RawPoint& point,
                       const RawSphereSweep& sphereSweep,
                       SweepIntersections& intersections);

// Intersect a sphere sweep with a line (not considering end points)
unsigned int intersect(const Line& line,
                       const RawSphereSweep& sphereSweep,
                       SweepIntersections& intersections);

// Intersect a capsule with a sweeping point (line)
unsigned int intersect(const OpenCapsule& capsule,
                       const Line& line,
                       SweepIntersections& intersections);

// Intersect a point with a sweeping capsule
unsigned int intersect(const RawPoint& point,
                       const OpenCapsuleSweep& capsuleSweep,
                       SweepIntersections& intersections);

// Intersect a line with a sweeping capsule
inline unsigned int intersect(const Line& line,
                       const OpenCapsuleSweep& capsuleSweep,
                       SweepIntersections& intersections)
{
    // Since we intersect the ends of the lines with the capsule sweep, and the
    // line with the ends of the capsule sweep, we can get away without this for
    // now.
    return 0;
}

};

#endif
