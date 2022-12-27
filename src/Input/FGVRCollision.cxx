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

// Intersect a sphere sweep with a point
// Much the same as a line with a sphere
unsigned int FGVRCollision::intersect(const RawPoint& point,
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
            intersections.insertIntersection(ratio1, ratio2);
            return 1;
        }
    }
    return 0;
}

// Intersect a sphere sweep with a line (not considering end points)
// Much the same as a line with a cylinder
unsigned int FGVRCollision::intersect(const Line& line,
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
        float ratio1 = (mb - sqrt_b2m4ac) / a2;
        float ratio2 = (mb + sqrt_b2m4ac) / a2;
        // It only counts if it overlaps [0, 1]
        if (ratio2 >= 0.0f && ratio1 <= 1.0f) {
            if (ratio1 < 0.0f)
                ratio1 = 0.0f;
            if (ratio2 > 1.0f)
                ratio2 = 1.0f;
            // Also check hit overlaps line range
            float sweepToLineConst = (lineToSweep * lineVec) / lineLen2;
            float sweepToLineLin = (sweepVec * lineVec) / lineLen2;
            float lineRatio1 = sweepToLineConst + sweepToLineLin*ratio1;
            float lineRatio2 = sweepToLineConst + sweepToLineLin*ratio2;
            if (lineRatio1 > lineRatio2)
                std::swap(lineRatio1, lineRatio2);
            if (lineRatio2 >= 0.0f && lineRatio1 <= 1.0f) {
                intersections.insertIntersection(ratio1, ratio2);
                return 1;
            }
        }
    }
    return 0;
}

// Intersect a capsule with a sweeping point (line)
unsigned int FGVRCollision::intersect(const OpenCapsule& capsule,
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
                if (capsuleRatio[0] < capsuleEnds[0])
                    ratio[swapped?1:0] = (capsuleEnds[0] - capsuleToLineRatioCapsule) / lineRatioCapsule;
                if (capsuleRatio[1] > capsuleEnds[1])
                    ratio[swapped?0:1] = (capsuleEnds[1] - capsuleToLineRatioCapsule) / lineRatioCapsule;
                intersections.insertIntersection(ratio[0], ratio[1]);
                return 1;
            }
        }
    }
    return 0;
}

// Intersect a point with a sweeping capsule
unsigned int FGVRCollision::intersect(const RawPoint& point,
                                      const OpenCapsuleSweep& capsuleSweep,
                                      SweepIntersections& intersections)
{
    // perform intersection in frame of capsule
    // i.e. between a point sweep (line) and static capsule

    // Construct an inverse rotation of capsule over the sweep
    const osg::Vec3f &capAT0 = capsuleSweep.sweep.start.position[0];
    const osg::Vec3f &capBT0   = capsuleSweep.sweep.start.position[1];
    const osg::Vec3f &capAT1 = capsuleSweep.sweep.end.position[0];
    const osg::Vec3f &capBT1   = capsuleSweep.sweep.end.position[1];
    osg::Vec3f capVecT0 = capBT0 - capAT0;
    osg::Vec3f capVecT1 = capBT1 - capAT1;
    osg::Quat rotCap1to0;
    rotCap1to0.makeRotate(capVecT1, capVecT0);

    // Transform point at start and end into frame of capsule sweep start
    const osg::Vec3f &p = point.sweep.position;
    osg::Vec3f pT1 = capAT0 + rotCap1to0 * (p - capAT1);

    // Now we need to intersect the point sweep p..pT1 with the start capsule.
    // Note, this assumes the capsule length is constant
    return intersect(capsuleSweep.getStart(), Line(point, Point(pT1)),
                     intersections);
}
