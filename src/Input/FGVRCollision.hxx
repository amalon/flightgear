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

#include <osg/BoundingSphere>
#include <osg/Vec3f>

#include <cmath>
#include <map>
#include <memory>
#include <utility>

namespace FGVRCollision
{

// Points

typedef osg::Vec3f Position;
typedef osg::Vec3f Vector;

typedef struct : public ShapeInfo {
    typedef struct {
        Position position;
    } SweepData;

    enum {
        shouldCheckBounds = 1,
    };
    static osg::BoundingBox getBounds(const FixedData& fixed,
                                      const SweepData& sweep)
    {
        osg::BoundingBox bb;
        bb.expandBy(sweep.position);
        return bb;
    }
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

// Lines

typedef TSweep<RawPoint> Line;
typedef TStrip<RawPoint> LineStrip;

typedef TSweep<Line> LineSweep;

// Polygons (coplanar, convex)

struct PolygonInfo;
typedef struct PolygonInfo PolygonInfo;
typedef TShape<PolygonInfo> RawPolygon;
typedef TSweep<PolygonInfo> RawPolygonSweep;

struct PolygonInfo : public ShapeInfo {
    typedef struct {
        unsigned char numVertices;
    } FixedData;
    typedef struct {
        Position vertices[6];
        // cache
        mutable unsigned int cacheStep = 0;
        mutable Vector faceNormal;
        mutable Vector edgeNormals[6];
        mutable float edgeNormalOffsets[6];
        mutable bool boundingBoxSet = false;
        mutable osg::BoundingBox boundingBox;
    } SharedSweepData;
    typedef std::shared_ptr<SharedSweepData> SweepData;

    // When used in a TCompound, it expands into a raw polygon, edges and
    // vertices.

    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor,
                        const FixedData& fixed,
                        const SweepData& sweep)
    {
        for (unsigned int i = 0; i < fixed.numVertices; ++i)
            functor(Point(sweep->vertices[i]));
        for (unsigned int i = 1; i < fixed.numVertices; ++i)
            functor(Line(Point(sweep->vertices[i - 1]),
                         Point(sweep->vertices[i])));
        if (fixed.numVertices > 2) {
            functor(RawPolygon(fixed, sweep));
            functor(Line(Point(sweep->vertices[fixed.numVertices-1]),
                         Point(sweep->vertices[0])));
        }
    }

    enum {
        shouldCheckBounds = 1,
    };
    static const osg::BoundingBox& getBounds(const FixedData& fixed,
                                             const SweepData& sweep)
    {
        if (!sweep->boundingBoxSet) {
            for (unsigned int i = 0; i < fixed.numVertices; ++i)
                sweep->boundingBox.expandBy(sweep->vertices[i]);
            sweep->boundingBoxSet = true;
        }
        return sweep->boundingBox;
    }
};

// A polygon which splits into edges and points for the purposes of collision
// detection
typedef TCompound<PolygonInfo> CompoundPolygon;

// For convenient creation of polygons
class Polygon : public CompoundPolygon
{
    public:
        Polygon(unsigned int numVertices) :
            CompoundPolygon({(unsigned char)numVertices},
                            std::make_shared<PolygonInfo::SharedSweepData>())
        {
        }

        void setVertex(unsigned int index, const Position& pos)
        {
            this->sweep->vertices[index] = pos;
        }
};

// Something fancier would avoid duplicated vertex and edge intersections...
typedef TGroup<CompoundPolygon> PolygonGroup;

struct MeshInfo : public ShapeInfo
{
    typedef RawPoint::SweepData PointSweepData;
    typedef Line::SweepData LineSweepData;
    typedef RawPolygon::FixedData PolygonFixedData;
    typedef RawPolygon::SweepData PolygonSweepData;

    typedef struct {
        std::vector<PolygonFixedData> polygons;
    } FixedData;

    typedef struct {
        std::vector<PointSweepData> points;
        std::vector<LineSweepData> edges;
        std::vector<PolygonSweepData> polygons;
        osg::BoundingBox boundingBox;
    } SweepData;

    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor,
                        const FixedData& fixed,
                        const SweepData& sweep)
    {
        for (unsigned int i = 0; i < sweep.points.size(); ++i)
            functor(RawPoint({}, sweep.points[i]));
        for (unsigned int i = 0; i < sweep.edges.size(); ++i)
            functor(Line({}, sweep.edges[i]));
        for (unsigned int i = 0; i < fixed.polygons.size(); ++i)
            functor(RawPolygon(fixed.polygons[i], sweep.polygons[i]));
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor,
                             const FixedData& fixed,
                             const SweepData& start,
                             const SweepData& end)
    {
        for (unsigned int i = 0; i < start.points.size(); ++i)
            functor(Line({}, start.points[i], end.points[i]));
        for (unsigned int i = 0; i < start.edges.size(); ++i)
            functor(LineSweep({}, start.edges[i], end.edges[i]));
        for (unsigned int i = 0; i < fixed.polygons.size(); ++i)
            functor(TSweep<RawPolygon>(fixed.polygons[i],
                                       start.polygons[i],
                                       end.polygons[i]));
    }

    enum {
        shouldCheckBounds = 1,
    };
    static const osg::BoundingBox& getBounds(const FixedData& fixed,
                                             const SweepData& sweep)
    {
        return sweep.boundingBox;
    }
};
class Mesh : public TCompound<MeshInfo>
{
    public:
        typedef TCompound<MeshInfo> Super;

        Mesh() :
            Super({}, {})
        {
        }

        void addPolygon(const FGVRCollision::Polygon& polygon);

        void reservePolygons(unsigned int polygons)
        {
            fixed.polygons.reserve(polygons);
            sweep.polygons.reserve(polygons);
        }

        unsigned int numVertsUnique() const
        {
            return sweep.points.size();
        }

        unsigned int numVertsAdded() const
        {
            return _statsVerts;
        }

        unsigned int numEdgesUnique() const
        {
            return sweep.edges.size();
        }

        unsigned int numEdgesAdded() const
        {
            return _statsEdges;
        }

        unsigned int numPolygons() const
        {
            return fixed.polygons.size();
        }

    protected:
        typedef struct VertexInfo {
            Position position;

            bool operator <(const struct VertexInfo& other) const
            {
                if (position.x() < other.position.x())
                    return true;
                if (position.x() > other.position.x())
                    return false;
                if (position.y() < other.position.y())
                    return true;
                if (position.y() > other.position.y())
                    return false;
                return position.z() < other.position.z();
            }
        } VertexInfo;
        typedef struct EdgeInfo {
            unsigned int vertexIndices[2];

            bool operator <(const struct EdgeInfo& other) const
            {
                if (vertexIndices[0] < other.vertexIndices[0])
                    return true;
                if (vertexIndices[0] > other.vertexIndices[0])
                    return false;
                return vertexIndices[1] < other.vertexIndices[1];
            }
        } EdgeInfo;

        unsigned int addVertex(const Position& position);
        unsigned int addEdge(EdgeInfo edge);

        std::map<VertexInfo, unsigned int> _vertIndex;
        std::map<EdgeInfo, unsigned int> _edgeIndex;

        unsigned int _statsVerts = 0;
        unsigned int _statsEdges = 0;
};

// Spheres

typedef struct : public ShapeInfo {
    typedef struct {
        float radius;
    } FixedData;
    typedef struct {
        Position position;
    } SweepData;

    enum {
        shouldCheckBounds = 1,
    };
    static osg::BoundingBox getBounds(const FixedData& fixed,
                                      const SweepData& sweep)
    {
        osg::BoundingBox bb;
        bb.expandBy(osg::BoundingSphere(sweep.position,
                                        fixed.radius));
        return bb;
    }
} SphereInfo;
typedef TShape<SphereInfo> RawSphere;
class Sphere : public RawSphere
{
    public:
        Sphere() :
            RawSphere({0}, {})
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

// Capsules

struct RawCapsuleInfo : public ShapeInfo {
    typedef struct {
        float radius[2];
    } FixedData;
    typedef struct {
        Position position[2];
    } SweepData;

    enum {
        shouldCheckBounds = 1,
    };
    static osg::BoundingBox getBounds(const FixedData& fixed,
                                      const SweepData& sweep)
    {
        osg::BoundingBox bb;
        for (unsigned int i = 0; i < 2; ++i)
            bb.expandBy(osg::BoundingSphere(sweep.position[i],
                                            fixed.radius[i]));
        return bb;
    }
};
template <bool startSphere, bool endSphere>
struct TCapsuleInfo : public RawCapsuleInfo {
    template <typename FUNCTOR>
    static void perPart(FUNCTOR& functor,
                        const FixedData& fixed,
                        const SweepData& sweep)
    {
        if (startSphere)
            functor(Sphere(fixed.radius[0], sweep.position[0]));
        if (startSphere || endSphere)
            functor(TShape<TCapsuleInfo<false, false>>(fixed, sweep));
        if (endSphere)
            functor(Sphere(fixed.radius[1], sweep.position[1]));
    }

    template <typename FUNCTOR>
    static void perSweepPart(FUNCTOR& functor,
                             const FixedData& fixed,
                             const SweepData& start,
                             const SweepData& end)
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

        void setRadii(float radii)
        {
            this->fixed.radius[0] = this->fixed.radius[1] = radii;
        }

        float minRadius() const
        {
            return std::min(this->fixed.radius[0], this->fixed.radius[1]);
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

typedef TIntersections<Sweep::Intersection> SweepIntersections;
typedef TIntersections<Strip::Intersection> StripIntersections;

// Intersect a sweeping sphere with a point
unsigned int rawIntersect(const RawPoint& point,
                          const RawSphereSweep& sphereSweep,
                          SweepIntersections& intersections);

// Intersect a sweeping sphere with a line (not considering end points)
unsigned int rawIntersect(const Line& line,
                          const RawSphereSweep& sphereSweep,
                          SweepIntersections& intersections);

// Intersect a sweeping sphere with a polygon (not considering edges or points)
unsigned int rawIntersect(const RawPolygon& polygon,
                          const RawSphereSweep& sphereSweep,
                          SweepIntersections& intersections);

// Intersect a sweeping point (line) with a capsule
unsigned int rawIntersect(const OpenCapsule& capsule,
                          const Line& line,
                          SweepIntersections& intersections);

// Intersect a sweeping capsule with a point
unsigned int rawIntersect(const RawPoint& point,
                          const OpenCapsuleSweep& capsuleSweep,
                          SweepIntersections& intersections);

// Intersect a sweeping line with a capsule
unsigned int rawIntersect(const OpenCapsule& capsule,
                          const LineSweep& lineSweep,
                          SweepIntersections& intersections);

// Intersect a sweeping capsule with a line
unsigned int rawIntersect(const Line& line,
                          const OpenCapsuleSweep& capsuleSweep,
                          SweepIntersections& intersections);

// Intersect a sweeping sphere with a polygon (not considering edges or points)
inline unsigned int rawIntersect(const RawPolygon& polygon,
                                 const OpenCapsuleSweep& capsuleSweep,
                                 SweepIntersections& intersections)
{
    // Since we intersect the edges of the polygon and the points with the
    // capsule sweep, and the polygon with the ends of the capsule sweep, we can
    // get away without this for now.
    return 0;
}

};

#endif
