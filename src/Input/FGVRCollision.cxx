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

#include "FGVRCollision.hxx"

#include <osg/Quat>

using namespace FGVRCollision;

unsigned int FGVRCollision::boundsCheck<1>::stats[2] = {};

unsigned int Mesh::addVertex(const Position& position)
{
    ++_statsVerts;
    VertexInfo vinfo{position};
    auto it = _vertIndex.find(vinfo);
    if (it == _vertIndex.end()) {
        unsigned int ret = sweep.points.size();
        _vertIndex[vinfo] = ret;
        sweep.points.push_back({position});
        sweep.boundingBox.expandBy(position);
        return ret;
    } else {
        return (*it).second;
    }
}

unsigned int Mesh::addEdge(EdgeInfo edge)
{
    ++_statsEdges;
    if (edge.vertexIndices[0] > edge.vertexIndices[1])
        std::swap(edge.vertexIndices[0], edge.vertexIndices[1]);

    auto it = _edgeIndex.find(edge);
    if (it == _edgeIndex.end()) {
        unsigned int ret = sweep.edges.size();
        _edgeIndex[edge] = ret;
        sweep.edges.push_back({sweep.points[edge.vertexIndices[0]],
                               sweep.points[edge.vertexIndices[1]]});
        return ret;
    } else {
        return (*it).second;
    }
}

void Mesh::addPolygon(const FGVRCollision::Polygon& polygon)
{
    if (!polygon.fixed.numVertices)
        return;

    // add first vertex
    unsigned int vertIndex = addVertex(polygon.sweep->vertices[0]);
    unsigned int firstIndex = vertIndex;
    unsigned int lastIndex = vertIndex;
    for (unsigned int i = 1; i < polygon.fixed.numVertices; ++i) {
        // add next vertex and edge between them
        vertIndex = addVertex(polygon.sweep->vertices[i]);
        addEdge({{lastIndex, vertIndex}});
        lastIndex = vertIndex;
    }
    if (polygon.fixed.numVertices >= 3) {
        // complete the loop of edges
        addEdge({{firstIndex, lastIndex}});
        // add polygons
        fixed.polygons.push_back(polygon.fixed);
        sweep.polygons.push_back(polygon.sweep);
    }
}

// Much the same as a line with a sphere
unsigned int FGVRCollision::rawIntersect(const RawPoint& point,
                                         const RawSphereSweep& sphereSweep,
                                         SweepIntersections& intersections)
{
    // The sphere sweep line is defined as:
    //   sweep = start + ratio*(end - start)
    // The intersection with a point is where:
    //   radius = |sweep - point|
    //          = |(start - point) + ratio*(end - start)|
    //          = sqrt((start - point)² +
    //                 ratio*2*((start - point) * (end - start)) +
    //                 ratio²*(end - start)²)
    //   0 = (start - point)² - radius² +
    //       ratio*2*((start - point) * (end - start)) +
    //       ratio²*(end - start)²
    // Using the quadratic equation (where x=ratio):
    //   ax² + bx + c = 0
    //   a = (end - start)²
    //   b = 2*((start - point) * (end - start))
    //   c = (start - point)² - radius²
    //   x = (-b +- sqrt(b²-4ac)) / 2a

    // end - start
    osg::Vec3f lineVec = sphereSweep.sweep.end.position - sphereSweep.sweep.start.position;
    // start - point
    osg::Vec3f linePos = sphereSweep.sweep.start.position - point.sweep.position;
    // Use quadratic equation
    float a = lineVec.length2();
    float b = 2.0f * (linePos * lineVec);
    float c = linePos.length2() - sphereSweep.fixed.radius*sphereSweep.fixed.radius;
    float b2m4ac = b*b - 4.0f*a*c;

    // Any intersections?
    if (b2m4ac >= 0.0f) {
        // Only consider the first intersection (not point exiting sphere)
        float sqrt_b2m4ac = std::sqrt(b2m4ac);
        float mb = -b;
        float a2 = 2.0f * a;
        float ratio1 = (mb - sqrt_b2m4ac) / a2;
        float ratio2 = (mb + sqrt_b2m4ac) / a2;
        // But it only counts if it overlaps [0, 1]
        if (ratio2 >= 0.0f && ratio1 <= 1.0f) {
            if (ratio1 < 0.0f)
                ratio1 = 0.0f;
            if (ratio2 > 1.0f)
                ratio2 = 1.0f;
            osg::Vec3f norms[2];
            if (intersections.wantsNormals()) {
                norms[0] = linePos + lineVec*ratio1;
                norms[1] = linePos + lineVec*ratio2;
            }
            intersections.insertIntersection(Sweep::Instant(ratio1, point.sweep.position, norms[0], __PRETTY_FUNCTION__),
                                             Sweep::Instant(ratio2, point.sweep.position, norms[1], __PRETTY_FUNCTION__));
            return 1;
        }
    }
    return 0;
}

// Much the same as a line with a cylinder
unsigned int FGVRCollision::rawIntersect(const Line& line,
                                         const RawSphereSweep& sphereSweep,
                                         SweepIntersections& intersections)
{
    // The sphere sweep line is defined as:
    //   sweep_vec = sweep_end - sweep_start
    //   sweep = sweep_start + sweep_ratio*sweep_vec
    // The line is defined similarly:
    //   line_vec = line_end - line_start
    //   line = line_start + line_ratio*line_vec
    // The closest point on line to sweep is minimum distance or distance²:
    //   0 = d|line - sweep|²/d(line_ratio)
    //     = d|line_start - sweep + line_ratio*line_vec|²/d(line_ratio)
    //     = d((line_start - sweep)² +
    //         2*line_ratio*(line_start - sweep)*line_vec +
    //         line_ratio²*line_vec²)/d(line_ratio)
    //     = 2*(line_start - sweep)*line_vec +
    //       2*line_ratio*line_vec²
    //   line_ratio*line_vec² = -(line_start - sweep)*line_vec
    //   line_ratio = (sweep - line_start)*line_vec /
    //                line_vec²
    //              = (sweep - line_start) ./ line_vec
    //   line_closest = line_start + line_ratio*line_vec
    //   line_closest = line_start + ((sweep - line_start) ./ line_vec) * line_vec
    // The intersection of a sphere with the line happens when:
    //   radius = |line_closest - sweep|
    //   radius² = |line_start - sweep + ((sweep - line_start) ./ line_vec) * line_vec|²
    //           = |line_start - (sweep_start + sweep_ratio*sweep_vec) +
    //              ((sweep_start + sweep_ratio*sweep_vec - line_start) ./ line_vec) * line_vec|²
    //           = |(line_start - sweep_start) - sweep_ratio*sweep_vec +
    //              (((sweep_start - line_start) + sweep_ratio*sweep_vec) ./ line_vec) * line_vec|²
    //           = |(line_start - sweep_start) - sweep_ratio*sweep_vec +
    //              (((sweep_start - line_start) + sweep_ratio*sweep_vec) * line_vec) * line_vec/line_vec²|²
    //           = |(line_start - sweep_start) - sweep_ratio*sweep_vec +
    //              sumxyz(((sweep_start - line_start) + sweep_ratio*sweep_vec).i * line_vec.i) * line_vec/line_vec²|²
    //           = |(line_start - sweep_start) - sweep_ratio*sweep_vec +
    //              sumxyz((sweep_start - line_start).i * line_vec.i + sweep_ratio*sweep_vec.i * line_vec.i) * line_vec/line_vec²|²
    //           = |(line_start - sweep_start) + ((sweep_start - line_start) * line_vec) * line_vec/line_vec² +
    //              - sweep_ratio*sweep_vec +
    //              sweep_ratio * (sweep_vec * line_vec) * line_vec/line_vec²|²
    //           = |(line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec) +
    //              sweep_ratio * ((sweep_vec / line_vec) * line_vec - sweep_vec)|²
    //           = (line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec)² +
    //             sweep_ratio * 2*(line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec)*((sweep_vec / line_vec) * line_vec - sweep_vec) +
    //             sweep_ratio² * ((sweep_vec / line_vec) * line_vec - sweep_vec)²
    //   0 = (line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec)² - radius² +
    //       sweep_ratio * 2*(line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec)*((sweep_vec / line_vec) * line_vec - sweep_vec) +
    //       sweep_ratio² * ((sweep_vec / line_vec) * line_vec - sweep_vec)²
    // Using the quadratic equation where x=sweep_ratio:
    //   ax² + bx + c = 0
    //   a = ((sweep_vec / line_vec) * line_vec - sweep_vec)²
    //   b = 2*(line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec)*((sweep_vec / line_vec) * line_vec - sweep_vec)
    //   c = (line_start - sweep_start + ((sweep_start - line_start) / line_vec) * line_vec)² - radius²
    //   x = (-b +- sqrt(b²-4ac)) / 2a
    osg::Vec3f sweepVec = sphereSweep.sweep.end.position - sphereSweep.sweep.start.position;
    osg::Vec3f lineVec = line.sweep.end.position - line.sweep.start.position;
    osg::Vec3f lineToSweep = sphereSweep.sweep.start.position - line.sweep.start.position;
    float radius = sphereSweep.fixed.radius;

    float lineLen2 = lineVec.length2();
    osg::Vec3f rta = lineVec * ((sweepVec * lineVec) / lineLen2) - sweepVec;
    osg::Vec3f lineToSweepOnLine = lineVec * ((lineToSweep * lineVec) / lineLen2) - lineToSweep;

    float a = rta.length2();
    float b = 2.0f*(lineToSweepOnLine*rta);
    float c = lineToSweepOnLine.length2() - radius*radius;
    float b2m4ac = b*b - 4.0f*a*c;

    // Any intersections?
    if (b2m4ac >= 0.0f) {
        float sqrt_b2m4ac = std::sqrt(b2m4ac);
        float mb = -b;
        float a2 = 2.0f * a;
        float ratios[2] = {
            (mb - sqrt_b2m4ac) / a2,
            (mb + sqrt_b2m4ac) / a2
        };
        // It only counts if it overlaps [0, 1]
        if (ratios[1] >= 0.0f && ratios[0] <= 1.0f) {
            if (ratios[0] < 0.0f)
                ratios[0] = 0.0f;
            if (ratios[1] > 1.0f)
                ratios[1] = 1.0f;
            // Also check hit overlaps line range
            float sweepToLineConst = (lineToSweep * lineVec) / lineLen2;
            float sweepToLineLin = (sweepVec * lineVec) / lineLen2;
            float lineRatios[2];
            for (unsigned int i = 0; i < 2; ++i)
                lineRatios[i] = sweepToLineConst + sweepToLineLin*ratios[i];
            if (lineRatios[0] > lineRatios[1])
                std::swap(lineRatios[0], lineRatios[1]);
            if (lineRatios[1] >= 0.0f && lineRatios[0] <= 1.0f) {
                bool swapped = (sweepToLineLin < 0);
                if (lineRatios[0] < 0.0f) {
                    ratios[swapped?1:0] -= lineRatios[0] / sweepToLineLin;
                    lineRatios[0] = 0;
                }
                if (lineRatios[1] > 1.0f) {
                    ratios[swapped?0:1] -= (lineRatios[1] - 1.0f) / sweepToLineLin;
                    lineRatios[1] = 1.0f;
                }
                osg::Vec3f pos[2];
                osg::Vec3f norm[2];
                if (intersections.wantsPositions()) {
                    for (unsigned int i = 0; i < 2; ++i) {
                        pos[i] = line.sweep.start.position + lineVec*lineRatios[swapped?1:0];
                        if (intersections.wantsNormals())
                            norm[i] = sphereSweep.sweep.start.position + sweepVec*ratios[i] - pos[i];
                    }
                }
                intersections.insertIntersection(Sweep::Instant(ratios[0], pos[0], norm[0], __PRETTY_FUNCTION__),
                                                 Sweep::Instant(ratios[1], pos[1], norm[1], __PRETTY_FUNCTION__));
                return 1;
            }
        }
    }
    return 0;
}

unsigned int FGVRCollision::rawIntersect(const RawPolygon& polygon,
                                         const RawSphereSweep& sphereSweep,
                                         SweepIntersections& intersections)
{
    if (polygon.fixed.numVertices < 3)
        return 0;

    // Find the polygon normal
    osg::Vec3f& faceNorm = polygon.sweep->faceNormal;
    if (polygon.sweep->cacheStep == 0) {
        polygon.sweep->cacheStep = 1;
        for (unsigned int i = 2; i < polygon.fixed.numVertices; ++i) {
            osg::Vec3f vec0 = polygon.sweep->vertices[i-1] - polygon.sweep->vertices[0];
            osg::Vec3f vec1 = polygon.sweep->vertices[i] - polygon.sweep->vertices[0];
            faceNorm = vec0 ^ vec1;

            if (faceNorm.length2() > 0.0f)
                break;
        }
    }
    if (faceNorm.length2() == 0.0f)
        return 0;

    // Calculate edge normals and offsets
    osg::Vec3f* edgeNorms = polygon.sweep->edgeNormals;
    float* edgeNormOffsets = polygon.sweep->edgeNormalOffsets;
    if (polygon.sweep->cacheStep == 1) {
        polygon.sweep->cacheStep = 2;
        unsigned int last = polygon.fixed.numVertices - 1;
        for (unsigned int i = 0; i < polygon.fixed.numVertices; ++i) {
            osg::Vec3f vec = polygon.sweep->vertices[i] - polygon.sweep->vertices[last];
            edgeNorms[i] = vec ^ faceNorm;
            edgeNormOffsets[i] = edgeNorms[i] * polygon.sweep->vertices[i];
            last = i;
        }
    }
    for (unsigned int i = 0; i < polygon.fixed.numVertices; ++i) {
        // Cull now if sphere center remains on outer side of edge
        if (edgeNorms[i] * sphereSweep.sweep.start.position > edgeNormOffsets[i] &&
            edgeNorms[i] * sphereSweep.sweep.end.position > edgeNormOffsets[i]) {
            return 0;
        }
    }

    // Normalize the polygon plane so we can measure distances
    if (polygon.sweep->cacheStep == 2) {
        polygon.sweep->cacheStep = 3;
        faceNorm.normalize();
    }

    // Sweep:
    //   Sweep(ratio) = P0 + ratio*(P1 - P0)
    // Plane:
    //   dist(Pos) = (Norm/|Norm|)*(Pos - Z)
    //             = (Norm/|Norm|)*Pos - (Norm/|Norm|)*Z
    // Distance at sweep ratio:
    //   dist(Sweep(ratio)) = (Norm/|Norm|)*Sweep(ratio) - (Norm/|Norm|)*Z
    //                      = (Norm/|Norm|)*(P0 + ratio*(P1 - P0)) - (Norm/|Norm|)*Z
    //                      = (Norm/|Norm|)*P0 + (Norm/|Norm|)*ratio*(P1 - P0) - (Norm/|Norm|)*Z
    //                      = (Norm/|Norm|)*(P0 - Z) + ratio*(Norm/|Norm|)*(P1 - P0)
    //   ratio = (dist(Sweep(ratio)) - (Norm/|Norm|)*(P0 - Z)) / ((Norm/|Norm|)*(P1 - P0))

    osg::Vec3f sweepRel = sphereSweep.sweep.start.position - polygon.sweep->vertices[0];
    osg::Vec3f sweepVec = sphereSweep.sweep.end.position - sphereSweep.sweep.start.position;
    float radius = sphereSweep.fixed.radius;
    float sweepRelDist = sweepRel * faceNorm;
    float sweepVecDist = sweepVec * faceNorm;
    float ratios[2] = { 0.0f, 1.0f };
    if (sweepVecDist) {
        ratios[0] = (radius - sweepRelDist) / sweepVecDist;
        ratios[1] = (-radius - sweepRelDist) / sweepVecDist;
        if (ratios[0] > ratios[1])
            std::swap(ratios[0], ratios[1]);
    } else {
        // Sphere sweep is parallel to plane
        // No intersection if that fixed distance is outside of +-radius range
        if (std::abs(sweepRelDist) >= radius)
            return 0;
    }
    // No intersection if wholly outside of +-radius range
    if (ratios[0] >= 1.0f || ratios[1] <= 0.0f)
        return 0;

    // Clip ratios to 0..1
    if (ratios[0] < 0.0f)
        ratios[0] = 0.0f;
    if (ratios[1] > 1.0f)
        ratios[1] = 1.0f;

    // Ratio at edge plane:
    //   dist(Sweep(ratio)) = 0
    //   ratio = -((Norm/|Norm|)*(P0 - Z)) / ((Norm/|Norm|)*(P1 - P0))
    //         = -(Norm*(P0 - Z)) / (Norm*(P1 - P0))
    //         = (Norm*(Z - P0)) / (Norm*(P1 - P0))

    for (unsigned int i = 0; i < polygon.fixed.numVertices; ++i) {
        float edgeNormSweepVec = edgeNorms[i]*sweepVec;
        float edgeRatio = (edgeNorms[i]*(polygon.sweep->vertices[i] - sphereSweep.sweep.start.position)) /
                           edgeNormSweepVec;
        if (edgeNormSweepVec > 0.0f) {
            // sweep going outwards
            // No intersection if start would be beyond edge
            if (ratios[0] > edgeRatio)
                return 0;
            // Clip end ratio to edge
            if (ratios[1] > edgeRatio)
                ratios[1] = edgeRatio;
        } else {
            // Sweep going inwards
            // No intersection if end doesn't reach edge
            if (ratios[1] < edgeRatio)
                return 0;
            // Clip start ratio to edge
            if (ratios[0] < edgeRatio)
                ratios[0] = edgeRatio;
        }
    }

    osg::Vec3f pos[2];
    osg::Vec3f norms[2];
    if (intersections.wantsPositions()) {
        for (unsigned int i = 0; i < 2; ++i) {
            pos[i] = sweepRel + sweepVec*ratios[i];
            float dist = pos[i] * faceNorm;
            pos[i] = polygon.sweep->vertices[0] + pos[i] - faceNorm*dist;
            if (dist && intersections.wantsNormals())
                norms[i] = (dist > 0.0f) ? faceNorm : - faceNorm;
        }
    }
    intersections.insertIntersection(Sweep::Instant(ratios[0], pos[0], norms[0], __PRETTY_FUNCTION__),
                                     Sweep::Instant(ratios[1], pos[1], norms[1], __PRETTY_FUNCTION__));
    return 1;
}

unsigned int FGVRCollision::rawIntersect(const OpenCapsule& capsule,
                                         const Line& line,
                                         SweepIntersections& intersections)
{
    // The capsule center line is:
    //   capsule_vec = capsule_end - capsule_start
    //   capsule(ratio) = capsule_start + ratio*capsule_vec
    // The radius of the capsule is:
    //   delta_r = capsule_end.radius - capsule_start.radius;
    //   capsule_radius(ratio) = capsule_start.radius + ratio*delta_r
    // The line is:
    //   line_vec = line_end - line_start
    //   line(ratio) = line_start + ratio*line_vec
    // The closest point on capsule to line is minimum distance or distance²:
    //   capsule_closest(capsule_ratio) = capsule_start + capsule_ratio*capsule_vec
    //     capsule_ratio(line_ratio) = (line(line_ratio) - capsule_start)/capsule_vec
    //                               = (line_start - capsule_start + line_ratio*line_vec)/capsule_vec
    //   capsule_closest(line_ratio) = capsule_start + ((line_start - capsule_start + line_ratio*line_vec)/capsule_vec)*capsule_vec
    // Intersection is at [line] ratio where:
    //   |capsule_closest(ratio) - (line_start + ratio*line_vec)| = capsule_radius(capsule_ratio(ratio))
    //   |capsule_closest(ratio) - (line_start + ratio*line_vec)|² = capsule_radius(capsule_ratio(ratio))²
    //   0 = capsule_radius(capsule_ratio(ratio))²
    //       -|capsule_closest(ratio) - (line_start + ratio*line_vec)|²
    //     = (capsule_start.radius + capsule_ratio(ratio)*delta_r)²
    //       -|capsule_start + ((line_start - capsule_start + ratio*line_vec)/capsule_vec)*capsule_vec - (line_start + ratio*line_vec)|²
    //     = (capsule_start.radius + ((line_start - capsule_start + ratio*line_vec)/capsule_vec)*delta_r)²
    //       -|capsule_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec + ((ratio*line_vec)/capsule_vec)*capsule_vec - (line_start + ratio*line_vec)|²
    //     = ((capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r) + ratio*((line_vec/capsule_vec)*delta_r))²
    //       -|(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec) + ratio*((line_vec/capsule_vec)*capsule_vec - line_vec)|²
    //     = (capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r)²
    //       +2*ratio*(capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r)*((line_vec/capsule_vec)*delta_r)
    //       +ratio²*((line_vec/capsule_vec)*delta_r)²
    //       -(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec)²
    //       -2*ratio*((line_vec/capsule_vec)*capsule_vec - line_vec)*(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec)
    //       -ratio²*((line_vec/capsule_vec)*capsule_vec - line_vec)²
    //     = (capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r)²
    //       -(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec)²
    //       +ratio*2*((capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r)*((line_vec/capsule_vec)*delta_r)
    //                 -((line_vec/capsule_vec)*capsule_vec - line_vec)*(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec))
    //       +ratio²*(((line_vec/capsule_vec)*delta_r)² - ((line_vec/capsule_vec)*capsule_vec - line_vec)²)
    //
    // Using the quadratic equation where x=ratio:
    //   ax² + bx + c = 0
    //   a = ((line_vec/capsule_vec)*delta_r)² - ((line_vec/capsule_vec)*capsule_vec - line_vec)²
    //   b = 2*((capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r)*((line_vec/capsule_vec)*delta_r)
    //         -((line_vec/capsule_vec)*capsule_vec - line_vec)*(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec))
    //   c = (capsule_start.radius + (line_start - capsule_start)/capsule_vec * delta_r)²
    //       -(capsule_start - line_start + ((line_start - capsule_start)/capsule_vec)*capsule_vec)²
    //   x = (-b +- sqrt(b²-4ac))/2a
    const osg::Vec3f& capsuleStart = capsule.sweep.position[0];

    osg::Vec3f lineVec = line.sweep.end.position - line.sweep.start.position;
    osg::Vec3f capsuleVec = capsule.sweep.position[1] - capsuleStart;
    float deltaRadius = capsule.fixed.radius[1] - capsule.fixed.radius[0];
    float capsuleLen2 = capsuleVec.length2();
    float lineRatioCapsule = (lineVec * capsuleVec) / capsuleLen2;
    float a1 = lineRatioCapsule*deltaRadius;
    osg::Vec3f a2 = capsuleVec*lineRatioCapsule - lineVec;
    float a = a1*a1 - a2.length2();

    osg::Vec3f capsuleToLine = line.sweep.start.position - capsuleStart;
    float capsuleToLineRatioCapsule = (capsuleToLine * capsuleVec) / capsuleLen2;
    float capsuleRadiusBase = capsule.fixed.radius[0] + capsuleToLineRatioCapsule*deltaRadius;
    osg::Vec3f capsuleBase = capsuleVec*capsuleToLineRatioCapsule - capsuleToLine;
    float b = 2.0f*(capsuleRadiusBase*lineRatioCapsule*deltaRadius
                    -(a2*capsuleBase));
    float c = capsuleRadiusBase*capsuleRadiusBase - capsuleBase*capsuleBase;
    float b2m4ac = b*b - 4.0f*a*c;

    // Any intersections?
    if (b2m4ac >= 0.0f) {
        float sqrt_b2m4ac = std::sqrt(b2m4ac);
        // ratio1 should be the lower ratio
        if (a < 0)
            sqrt_b2m4ac = -sqrt_b2m4ac;
        float mb = -b;
        float a2 = 2.0f * a;
        float ratio[2] = {
            (mb - sqrt_b2m4ac) / a2,
            (mb + sqrt_b2m4ac) / a2
        };

        // It only counts if it overlaps the range [0, 1]
        if (ratio[1] >= 0.0f && ratio[0] <= 1.0f) {
            // If radii differ, there is a focus beyond which the cone diverges
            // again. If the line intersects both ends of the code, we have to be
            // extra careful.
            float capsuleEnds[2] = { 0.0f, 1.0f };
            bool swapped = lineRatioCapsule < 0;
            if (deltaRadius) {
                // Offset capsule ends depending on conicity
                float capsuleOffsetFactor = -deltaRadius / capsuleLen2;
                capsuleEnds[0] += capsule.fixed.radius[0] * capsuleOffsetFactor;
                capsuleEnds[1] += capsule.fixed.radius[1] * capsuleOffsetFactor;

                // Find capsule ratios of pre-clip intersection line ratios
                float preClipCapsuleRatio[2] = {
                    capsuleToLineRatioCapsule + ratio[swapped?1:0] * lineRatioCapsule,
                    capsuleToLineRatioCapsule + ratio[swapped?0:1] * lineRatioCapsule
                };

                // Find focus
                float focusCapsuleRatio = -capsule.fixed.radius[0]/deltaRadius;
                bool focusBefore = focusCapsuleRatio < 0.0f;
                bool crossesFocus;
                if (focusBefore)
                    crossesFocus = preClipCapsuleRatio[0] < focusCapsuleRatio;
                else
                    crossesFocus = preClipCapsuleRatio[1] > focusCapsuleRatio;

                if (crossesFocus) {
                    if (focusBefore) {
                        // If the line passes outside of the capsule, discard
                        if (preClipCapsuleRatio[1] > capsuleEnds[1])
                            return 0;
                        // Recalculate bad ratio
                        ratio[swapped?1:0] = ratio[swapped?0:1];
                        ratio[swapped?0:1] = (capsuleEnds[1] - capsuleToLineRatioCapsule) / lineRatioCapsule;
                    } else {
                        // If the line passes outside of the capsule, discard
                        if (preClipCapsuleRatio[0] < capsuleEnds[0])
                            return 0;
                        // Recalculate bad ratio
                        ratio[swapped?0:1] = ratio[swapped?1:0];
                        ratio[swapped?1:0] = (capsuleEnds[0] - capsuleToLineRatioCapsule) / lineRatioCapsule;
                    }
                }
            }
            // Clip ratios into range of line
            if (ratio[0] < 0.0f)
                ratio[0] = 0.0f;
            if (ratio[1] > 1.0f)
                ratio[1] = 1.0f;
            // Also check hit overlaps ends of capsule
            float capsuleRatio[2] = {
                capsuleToLineRatioCapsule + ratio[swapped?1:0] * lineRatioCapsule,
                capsuleToLineRatioCapsule + ratio[swapped?0:1] * lineRatioCapsule
            };
            if (capsuleRatio[1] >= capsuleEnds[0] && capsuleRatio[0] <= capsuleEnds[1]) {
                // If either end extends beyond capsule, clip to flat ends
                float normDir[2] = {};
                if (capsuleRatio[0] < capsuleEnds[0])  {
                    ratio[swapped?1:0] = (capsuleEnds[0] - capsuleToLineRatioCapsule) / lineRatioCapsule;
                    normDir[swapped?1:0] = -1.0f;
                }
                if (capsuleRatio[1] > capsuleEnds[1]) {
                    ratio[swapped?0:1] = (capsuleEnds[1] - capsuleToLineRatioCapsule) / lineRatioCapsule;
                    normDir[swapped?1:0] = 1.0f;
                }
                osg::Vec3f pos[2];
                osg::Vec3f norms[2];
                if (intersections.wantsPositions()) {
                    for (unsigned int i = 0; i < 2; ++i) {
                        pos[i] = line.sweep.start.position + lineVec*ratio[i];
                        if (intersections.wantsNormals()) {
                            if (normDir[i])
                                // flat end of capsule
                                norms[i] = capsuleVec*normDir[i];
                            else
                                // conic part of capsule
                                norms[i] = pos[i] - (capsuleStart + capsuleVec*capsuleRatio[swapped?(1-i):i]);
                        }
                    }
                    intersections.insertIntersection(Sweep::Instant(ratio[0], pos[0], norms[0], __PRETTY_FUNCTION__),
                                                     Sweep::Instant(ratio[1], pos[1], norms[1], __PRETTY_FUNCTION__));
                } else {
                    intersections.insertIntersection(Sweep::Instant(ratio[0], __PRETTY_FUNCTION__),
                                                     Sweep::Instant(ratio[1], __PRETTY_FUNCTION__));
                }
                return 1;
            }
        }
    }
    return 0;
}

unsigned int FGVRCollision::rawIntersect(const RawPoint& point,
                                         const OpenCapsuleSweep& capsuleSweep,
                                         SweepIntersections& intersections)
{
    // perform intersection in frame of capsule
    // i.e. between a point sweep (line) and static capsule

    // Construct an inverse rotation of capsule over the sweep
    const osg::Vec3f &capAT0 = capsuleSweep.sweep.start.position[0];
    const osg::Vec3f &capBT0 = capsuleSweep.sweep.start.position[1];
    const osg::Vec3f &capAT1 = capsuleSweep.sweep.end.position[0];
    const osg::Vec3f &capBT1 = capsuleSweep.sweep.end.position[1];
    osg::Vec3f capVecT0 = capBT0 - capAT0;
    osg::Vec3f capVecT1 = capBT1 - capAT1;
    osg::Quat rotCap1to0;
    rotCap1to0.makeRotate(capVecT1, capVecT0);

    // Transform point at start and end into frame of capsule sweep start
    const osg::Vec3f &p = point.sweep.position;
    osg::Vec3f pT1 = capAT0 + rotCap1to0 * (p - capAT1);

    // Now we need to intersect the point sweep p..pT1 with the start capsule.
    // Note, this assumes the capsule length is constant
    SweepIntersections tempIntersections(false, false);
    unsigned int ret = rawIntersect(capsuleSweep.getStart(),
                                    Line(point, Point(pT1)),
                                    tempIntersections);
    if (intersections.wantsPositions()) {
        // Recalculate intersection positions and normals
        for (auto hit: tempIntersections) {
            hit.entry.hasPosition = true;
            hit.entry.position = p;
            hit.exit.hasPosition = true;
            hit.exit.position = p;
            if (intersections.wantsNormals()) {
                osg::Vec3f capStart = capAT0 + (capAT1-capAT0)*hit.entry.ratio;
                osg::Vec3f capEnd   = capBT0 + (capBT1-capBT0)*hit.entry.ratio;
                osg::Vec3f capNorm = capEnd - capStart;
                float capRatio = ((p - capStart) * capNorm) / capNorm.length2();
                hit.entry.normal = capStart + capNorm*capRatio - p;

                capStart = capAT0 + (capAT1-capAT0)*hit.exit.ratio;
                capEnd   = capBT0 + (capBT1-capBT0)*hit.exit.ratio;
                capNorm = capEnd - capStart;
                capRatio = ((p - capStart) * capNorm) / capNorm.length2();
                hit.exit.normal = capStart + capNorm*capRatio - p;
            }
            intersections.insertIntersection(hit);
        }
    }
    return ret;
}

static osg::Vec3f bilinear(const LineSweep& lineSweep, float lineRatio, float timeRatio)
{
    float invLineRatio = 1.0f - lineRatio;
    float invTimeRatio = 1.0f - timeRatio;
    return lineSweep.sweep.start.start.position * (invLineRatio*invTimeRatio)
         + lineSweep.sweep.start.end.position   * (   lineRatio*invTimeRatio)
         + lineSweep.sweep.end.start.position   * (invLineRatio*   timeRatio)
         + lineSweep.sweep.end.end.position     * (   lineRatio*   timeRatio);
}

unsigned int FGVRCollision::rawIntersect(const OpenCapsule& capsule,
                                         const LineSweep& lineSweep,
                                         SweepIntersections& intersections)
{
    // Find the line along the sweep that is closest to the capsule centerline
    //   Line(t, ratio) = (AT0+t*(AT1-AT0)) + ratio*((BT0+t*(BT1-BT0))-(AT0+t*(AT1-AT0)))
    //   Capsule(ratio) = C0 + ratio*(C1-C0)
    //   CapsuleClosest(t) = Capsule(0) + ((Line(t,ratio) - Capsule(0)) / CapsuleVec) * CapsuleVec

    // Quick calculate the normals of the lines.
    osg::Vec3f capsuleVec = capsule.sweep.position[1] - capsule.sweep.position[0];
    osg::Vec3f lineT0Vec = lineSweep.sweep.start.end.position - lineSweep.sweep.start.start.position;
    osg::Vec3f lineT1Vec = lineSweep.sweep.end.end.position - lineSweep.sweep.end.start.position;
    osg::Vec3f lineT0Rel = lineSweep.sweep.start.start.position - capsule.sweep.position[0];
    osg::Vec3f lineT1Rel = lineSweep.sweep.end.start.position - capsule.sweep.position[0];
    float aT0 = lineT0Vec.length2();
    float bT0 = lineT0Vec*capsuleVec;
    float c = capsuleVec.length2();
    float dT0 = lineT0Vec*lineT0Rel;
    float eT0 = capsuleVec*lineT0Rel;
    float denominatorT0 = aT0*c - bT0*bT0;
    if (denominatorT0 == 0.0f) {
        // If they're parallel and coplanar, then other part's intersection
        // routines should cover this.
        return 0;
    }
    float aT1 = lineT1Vec.length2();
    float bT1 = lineT1Vec*capsuleVec;
    float dT1 = lineT1Vec*lineT1Rel;
    float eT1 = capsuleVec*lineT1Rel;
    float denominatorT1 = aT1*c - bT1*bT1;
    if (denominatorT1 == 0.0f) {
        // If they're parallel and coplanar, then other part's intersection
        // routines should cover this.
        return 0;
    }

    float closestRatioT0 = (bT0*eT0 - c*dT0) / denominatorT0;
    float closestRatioT1 = (bT1*eT1 - c*dT1) / denominatorT1;

    bool lineT0Above = (closestRatioT0 > 1.0f);
    bool lineT0Below = (closestRatioT0 < 0.0f);
    bool lineT1Above = (closestRatioT1 > 1.0f);
    bool lineT1Below = (closestRatioT1 < 0.0f);
#if 0
    if (!lineT0Above && !lineT0Below && !lineT1Above && !lineT1Below)
        // Fully inside the line sweep
        // Just intersect the line of closest distance to capsule
        return intersect(capsule,
                         Line(Point(lineSweep.sweep.start.start.position + lineT0Vec*closestRatioT0),
                              Point(lineSweep.sweep.end.start.position   + lineT0Vec*closestRatioT0)),
                         intersections);
    if (lineT0Above && lineT1Above)
        // Fully outside the line sweep
        // Just intersect the sweep of end point of line
        return intersect(capsule,
                         Line(Point(lineSweep.sweep.start.end.position),
                              Point(lineSweep.sweep.end.end.position)),
                         intersections);
    if (lineT0Below && lineT1Below)
        // Fully outside the line sweep
        // Just intersect the sweep of start point of line
        return intersect(capsule,
                         Line(Point(lineSweep.sweep.start.start.position),
                              Point(lineSweep.sweep.end.start.position)),
                         intersections);
#endif

    unsigned int ret = 0;

    float lineRatio[4] = { closestRatioT0, closestRatioT0, closestRatioT1, closestRatioT1 };
    float timeRatio[4] = { 0.0f, 0.0f, 1.0f, 1.0f };

    if (lineT0Below) {
        lineRatio[0] = 0.0f;
        lineRatio[1] = 0.0f;
    } else if (lineT0Above) {
        lineRatio[0] = 1.0f;
        lineRatio[1] = 1.0f;
    }
    if (lineT1Below) {
        lineRatio[2] = 0.0f;
        lineRatio[3] = 0.0f;
    } else if (lineT1Above) {
        lineRatio[2] = 1.0f;
        lineRatio[3] = 1.0f;
    }
    if (lineT0Below != lineT1Below) {
        float timeIntersect = closestRatioT0/(closestRatioT0 - closestRatioT1);
        if (lineT0Below)
            timeRatio[1] = timeIntersect;
        else
            timeRatio[2] = timeIntersect;
    }
    if (lineT0Above != lineT1Above) {
        float timeIntersect = (closestRatioT0 - 1.0f)/(closestRatioT0 - closestRatioT1);
        if (lineT0Above)
            timeRatio[1] = timeIntersect;
        else
            timeRatio[2] = timeIntersect;
    }

    for (unsigned int i = 0; i < 3; ++i) {
        if (timeRatio[i] == timeRatio[i + 1])
            continue;
        auto range = intersections.pushRange({timeRatio[i], timeRatio[i + 1] - timeRatio[i]});
        ret += rawIntersect(capsule,
                            Line(Point(bilinear(lineSweep, lineRatio[i], timeRatio[i])),
                                 Point(bilinear(lineSweep, lineRatio[i + 1], timeRatio[i + 1]))),
                            intersections);
        intersections.popRange(range);
    }

    return ret;
}

unsigned int FGVRCollision::rawIntersect(const Line& line,
                                         const OpenCapsuleSweep& capsuleSweep,
                                         SweepIntersections& intersections)
{
    // perform intersection in frame of capsule
    // i.e. between a line sweep and static capsule

    // Construct an inverse rotation of capsule over the sweep
    const osg::Vec3f &capAT0 = capsuleSweep.sweep.start.position[0];
    const osg::Vec3f &capBT0 = capsuleSweep.sweep.start.position[1];
    const osg::Vec3f &capAT1 = capsuleSweep.sweep.end.position[0];
    const osg::Vec3f &capBT1 = capsuleSweep.sweep.end.position[1];
    osg::Vec3f capVecT0 = capBT0 - capAT0;
    osg::Vec3f capVecT1 = capBT1 - capAT1;
    osg::Quat rotCap1to0;
    rotCap1to0.makeRotate(capVecT1, capVecT0);

    // Transform points at start and end into frame of capsule sweep start
    const osg::Vec3f &aT0 = line.sweep.start.position;
    const osg::Vec3f &bT0 = line.sweep.end.position;
    osg::Vec3f aT1 = capAT0 + rotCap1to0 * (aT0 - capAT1);
    osg::Vec3f bT1 = capAT0 + rotCap1to0 * (bT0 - capAT1);

    // Now we need to intersect the line sweep [aT0,bT0]..[aT1,bT1] with the
    // start capsule. Note, this assumes the capsule length is constant
    SweepIntersections tempIntersections(intersections);
    unsigned int ret = rawIntersect(capsuleSweep.getStart(),
                                    LineSweep(line, Line(Point(aT1), Point(bT1))),
                                    tempIntersections);
    if (intersections.wantsPositions() && !tempIntersections.empty()) {
        // Recalculate intersection positions and normals
#if 0
        osg::Quat rotCap0to1 = rotCap1to0.inverse();
#endif
        osg::Quat rotCap0toN;
        for (auto hit: tempIntersections) {
            for (auto* subhit: { &hit.entry, &hit.exit }) {
                if (subhit->hasPosition) {
                    float ratio = subhit->ratio;
                    float invRatio = 1.0f - ratio;
                    osg::Vec3f capATn = capAT0 * invRatio + capAT1 * ratio;
                    // Construct a rotation of capsule from time 0 to time ratio
#if 0
                    rotCap0toN.slerp(ratio, osg::Quat(), rotCap0to1);
#else
                    osg::Vec3f capBTn = capBT0 * invRatio + capBT1 * ratio;
                    osg::Vec3f capVecTN = capBTn - capATn;
                    rotCap0toN.makeRotate(capVecT0, capVecTN);
#endif
                    // vT0 = capATn + rotCap0toN * (vTn - capAT0)
                    subhit->position = capATn + rotCap0toN * (subhit->position - capAT0);
#if 0
                    subhit->linestrip.clear();
                    subhit->linestrip.push_back(capATn);
                    subhit->linestrip.push_back(capBTn);
#endif
                    if (intersections.wantsNormals()) {
                        // FIXME still sometimes sideways normals
                        // Normal needs to be relative to the line, so invert
                        subhit->normal = rotCap0toN * -subhit->normal;
                    }
                }
            }
            intersections.insertIntersection(hit);
        }
    }
    return ret;
}
