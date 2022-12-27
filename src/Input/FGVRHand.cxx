// Interacting VR Hand Pose
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

#include "FGVRHand.hxx"

#include "FGVRCollision.hxx"

#include <Viewer/CameraGroup.hxx>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Point>

#define NUM_INTERSECTION_SEGMENTS 8

// Debug
#include <iostream>

// Shows finger-plane vs scene-geometry intersections
//#define DRAW_PLANE_INTERSECTIONS (finger == 1)

// Shows finger-squeeze vs plane-intersection intersection ranges
//#define DRAW_SQUEEZE_INTERSECTIONS (finger == 1)

// Log finger movement decisions
//#define DEBUG_FINGER_MOVEMENT (finger == 1)

// Debug defaults
#ifndef DEBUG_FINGER_MOVEMENT
#define DEBUG_FINGER_MOVEMENT false
#endif

using flightgear::CameraGroup;

FGVRHand::FGVRHand(osg::MatrixTransform* localSpaceGroup,
                   const std::shared_ptr<osgXR::HandPose>& parent) :
    _parent(parent),
    _fingersOverriding{},
    _localSpaceGroup(localSpaceGroup)
{
}

FGVRHand::~FGVRHand()
{
    if (_debugGeode.valid())
        _localSpaceGroup->removeChild(_debugGeode);
}

void FGVRHand::advance(float dt)
{
    // Ensure parent pose tracking is up to date
    _parent->update();

    // Copy the pose from the parent pose
    *(osgXR::HandPose*)this = *_parent;

    if (_parent->isActive()) {
        // Create debug geometry
        osg::Vec3Array* vertices = new osg::Vec3Array();
        osg::Geometry* debugGeom = new osg::Geometry;
        debugGeom->setVertexArray(vertices);
        debugGeom->setUseDisplayList(false);
        if (_debugGeode.valid())
            _localSpaceGroup->removeChild(_debugGeode);
        _debugGeode = new osg::Geode;
        _debugGeode->addDrawable(debugGeom);

        // Hard code hit & miss style for now
        int forceOff = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
        osg::StateSet* ss = new osg::StateSet;
        // It should look like some sort of laser
        ss->setMode(GL_LIGHTING, forceOff);
        ss->setMode(GL_DEPTH_TEST, forceOff);
        ss->setAttribute(new osg::LineWidth(2.0f));
        // Material setup
        osg::Material* material = new osg::Material;
        material->setColorMode(osg::Material::OFF);
        material->setDiffuse(osg::Material::FRONT_AND_BACK,
                             osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
        ss->setAttribute(material);
        ss->setAttribute(new osg::Point(4.0f));
        _debugGeode->setStateSet(ss);

        _localSpaceGroup->addChild(_debugGeode);

        // get bounding box in scene space
        auto& invMatrix = _localSpaceGroup->getMatrix();

        static const unsigned int fingerMasks[5] = {
            osgXR::HandPose::JOINT_THUMB_BITS,
            osgXR::HandPose::JOINT_INDEX_BITS  & ~osgXR::HandPose::JOINT_METACARPAL_BITS,
            osgXR::HandPose::JOINT_MIDDLE_BITS & ~osgXR::HandPose::JOINT_METACARPAL_BITS,
            osgXR::HandPose::JOINT_RING_BITS   & ~osgXR::HandPose::JOINT_METACARPAL_BITS,
            osgXR::HandPose::JOINT_LITTLE_BITS & ~osgXR::HandPose::JOINT_METACARPAL_BITS,
        };
        static const osgXR::HandPose::Joint fingerJoints[5][4] = {
            {
                osgXR::HandPose::JOINT_THUMB_METACARPAL,
                osgXR::HandPose::JOINT_THUMB_PROXIMAL,
                osgXR::HandPose::JOINT_THUMB_DISTAL,
                osgXR::HandPose::JOINT_THUMB_TIP,
            },
            {
                osgXR::HandPose::JOINT_INDEX_PROXIMAL,
                osgXR::HandPose::JOINT_INDEX_INTERMEDIATE,
                osgXR::HandPose::JOINT_INDEX_DISTAL,
                osgXR::HandPose::JOINT_INDEX_TIP,
            },
            {
                osgXR::HandPose::JOINT_MIDDLE_PROXIMAL,
                osgXR::HandPose::JOINT_MIDDLE_INTERMEDIATE,
                osgXR::HandPose::JOINT_MIDDLE_DISTAL,
                osgXR::HandPose::JOINT_MIDDLE_TIP,
            },
            {
                osgXR::HandPose::JOINT_RING_PROXIMAL,
                osgXR::HandPose::JOINT_RING_INTERMEDIATE,
                osgXR::HandPose::JOINT_RING_DISTAL,
                osgXR::HandPose::JOINT_RING_TIP,
            },
            {
                osgXR::HandPose::JOINT_LITTLE_PROXIMAL,
                osgXR::HandPose::JOINT_LITTLE_INTERMEDIATE,
                osgXR::HandPose::JOINT_LITTLE_DISTAL,
                osgXR::HandPose::JOINT_LITTLE_TIP,
            },
        };

        osgXR::HandPose::HandDimentions dim(*this);
        _ranges.extend(*this);
        // Allow fingers to stretch back at the proximal joint
        _ranges.extendX(osgXR::HandPose::JOINT_INDEX_PROXIMAL, M_PI * 30 / 180);
        _ranges.extendX(osgXR::HandPose::JOINT_MIDDLE_PROXIMAL, M_PI * 30 / 180);
        _ranges.extendX(osgXR::HandPose::JOINT_RING_PROXIMAL, M_PI * 30 / 180);
        _ranges.extendX(osgXR::HandPose::JOINT_LITTLE_PROXIMAL, M_PI * 30 / 180);

        osgXR::HandPose::SqueezeValues trackedSqueeze(*_parent, _ranges);

        // Find positions of important finger joints we can use for collision
        // detection
        osgXR::HandPose tempPose(*_parent);
        osgXR::HandPose::SqueezeValues squeeze;
        osg::Vec3f pos[NUM_INTERSECTION_SEGMENTS][5][4];
        FGVRCollision::OneEndCapsuleGroupStrip fingersStrip[5];
        for (unsigned int i = 0; i < NUM_INTERSECTION_SEGMENTS; ++i) {
            squeeze.setFingersSqueeze(osgXR::HandPose::FINGER_ALL_BITS,
                                      (float)i / (NUM_INTERSECTION_SEGMENTS - 1));
            tempPose.setPose(squeeze, _ranges, &dim);
            for (unsigned int finger = 0; finger < 5; ++finger) {
                FGVRCollision::OneEndCapsuleGroup fingerBones;
                FGVRCollision::Sphere joints[4];
                for (unsigned int j = 0; j < 4; ++j) {
                    auto &loc = tempPose.getJointLocation(fingerJoints[finger][j]);
                    joints[j].set(loc.getRadius(), loc.getPosition());
                    if (/*j==1 &&*/ j > 0) {
                        FGVRCollision::OneEndCapsule bone(joints[j-1], joints[j]);
                        // Treat bones as fixed radius, to match osgXR::Hand
                        // rendering.
                        bone.fixed.radius[0] = bone.fixed.radius[1] = std::min(bone.fixed.radius[0], bone.fixed.radius[1]);
                        fingerBones.addItem(bone);
                    }
                    pos[i][finger][j] = loc.getPosition()/* + loc.getOrientation() * osg::Vec3f(0.0f, -loc.getRadius(), 0.0f)*/;
                }
                fingersStrip[finger].addNode(fingerBones);
            }
        }

        unsigned int jointMask = 0;
        for (unsigned int finger = 0; finger < 5; ++finger) {
            // and get plane of finger motion
            auto& loc1 = _parent->getJointLocation(fingerJoints[finger][0]);
            auto& loc4 = _parent->getJointLocation(fingerJoints[finger][3]);
            auto pos1 = (osg::Vec3d)loc1.getPosition() * invMatrix;
            //osg::Matrixd ori1(loc1.getOrientation());
            //auto vec3_4 = osg::Vec4d(0.0f, -1.0f, 0.0f, 0.0f) * ori1;
            //osg::Vec3d vec3(vec3_4.x(), vec3_4.y(), vec3_4.z());
            //auto pos3 = (osg::Vec3d)(loc2.getPosition() + ori1*osg::Vec3d(0.0f, -1.0f, 0.0f)) * invMatrix;

#if 0
            if (finger == 4) {
                osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                            vertices->size(),
                                                            NUM_INTERSECTION_SEGMENTS);
                for (unsigned int i = 0; i < NUM_INTERSECTION_SEGMENTS; ++i) {
                    auto vertTransformed = pos[i][finger][1];
                    vertices->push_back(vertTransformed);
                }
                debugGeom->addPrimitiveSet(prim);
            }
#endif

            osg::Polytope polytope;
            osg::BoundingBoxd bb;// = _parent->getBoundingBox(invMatrix);
            const double grow = dim.getJointLength(fingerJoints[finger][1])
                              + dim.getJointLength(fingerJoints[finger][2])
                              + dim.getJointLength(fingerJoints[finger][3])
                              + loc4.getRadius();
            bb.expandBy(pos1);
            bb.expandBy(bb.xMin()-grow, bb.yMin()-grow, bb.zMin()-grow);
            bb.expandBy(bb.xMax()+grow, bb.yMax()+grow, bb.zMax()+grow);
            // FIXME The use of OSG_USE_FLOAT_BOUNDINGBOX is dumb
            //polytope.setToBoundingBox(bb);
            osg::Polytope::PlaneList planes;
            planes.push_back(osg::Plane(1.0,0.0,0.0,-bb.xMin())); // left plane.
            planes.push_back(osg::Plane(-1.0,0.0,0.0,bb.xMax())); // right plane.
            planes.push_back(osg::Plane(0.0,1.0,0.0,-bb.yMin())); // bottom plane.
            planes.push_back(osg::Plane(0.0,-1.0,0.0,bb.yMax())); // top plane.
            planes.push_back(osg::Plane(0.0,0.0,1.0,-bb.zMin())); // near plane
            planes.push_back(osg::Plane(0.0,0.0,-1.0,bb.zMax())); // far plane
            polytope.set(planes);

            bool overriding = false;
            float curSqueeze = _squeeze.getFingerSqueeze((osgXR::HandPose::Finger)finger);
            float targetSqueeze = trackedSqueeze.getFingerSqueeze((osgXR::HandPose::Finger)finger);

            auto planePos1 = (osg::Vec3d)pos[0][finger][3] * invMatrix;
            auto planePos2 = (osg::Vec3d)pos[NUM_INTERSECTION_SEGMENTS>>1][finger][3] * invMatrix;
            auto planePos3 = (osg::Vec3d)pos[NUM_INTERSECTION_SEGMENTS-1][finger][3] * invMatrix;
            osg::Plane plane(planePos1, planePos2, planePos3);
            osgUtil::PlaneIntersector::Intersections planeIntersections;
            if (computeSceneIntersections(CameraGroup::getDefault(), plane, polytope,
                                          planeIntersections)) {
                unsigned int verts = 0;
                for (auto &hit: planeIntersections)
                    verts += hit.polyline.size();
                if (verts) {
#if 1
                    osg::Matrix mat;
                    mat.invert(invMatrix);
#endif
                    FGVRCollision::StripIntersections intersections;
                    for (auto &hit: planeIntersections) {
                        FGVRCollision::LineStrip linestrip;
#ifdef DRAW_PLANE_INTERSECTIONS
                        osg::ref_ptr<osg::DrawArrays> prim;
                        if (DRAW_PLANE_INTERSECTIONS)
                            prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                       vertices->size(),
                                                       hit.polyline.size());
#endif
                        for (auto &vert: hit.polyline) {
                            auto vertTransformed = vert * *hit.matrix * mat;
#ifdef DRAW_PLANE_INTERSECTIONS
                            if (prim.valid())
                                vertices->push_back(vertTransformed);
#endif
                            linestrip.addNode(FGVRCollision::Point(vertTransformed));
                        }
#ifdef DRAW_PLANE_INTERSECTIONS
                        if (prim.valid())
                            debugGeom->addPrimitiveSet(prim);
#endif

                        FGVRCollision::intersect(linestrip, fingersStrip[finger], intersections);
                    }

#ifdef DRAW_SQUEEZE_INTERSECTIONS
                    unsigned int primStart = vertices->size();
#endif
                    if (DEBUG_FINGER_MOVEMENT)
                        std::cout << "cur " << curSqueeze << std::endl;
                    // Extent of current clearance
                    // Ratios are reversed if inside intersection
                    float clearanceRatio[2] = { 0.0f, 1.0f };
                    for (auto& hit: intersections) {
                        float intersectionEntry = fingersStrip[finger].getOverallRatio(hit.entry);
                        float intersectionExit = fingersStrip[finger].getOverallRatio(hit.exit);
#ifdef DRAW_SQUEEZE_INTERSECTIONS
                        if (DRAW_SQUEEZE_INTERSECTIONS) {
                            unsigned int stripStart = vertices->size();
                            for (unsigned int j = 3; j < 4; ++j) {
                                vertices->push_back(pos[hit.entry.segment][finger][j] * (1.0f - hit.entry.ratio) +
                                                    pos[hit.entry.segment + 1][finger][j] * hit.entry.ratio);
                                for (unsigned int i = hit.entry.segment+1; i <= hit.exit.segment; ++i)
                                    vertices->push_back(pos[i][finger][j]);
                                vertices->push_back(pos[hit.exit.segment][finger][j] * (1.0f - hit.exit.ratio) +
                                                    pos[hit.exit.segment + 1][finger][j] * hit.exit.ratio);
                            }
                            osg::DrawArrays* prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,
                                                                        stripStart,
                                                                        vertices->size() - stripStart);
                            debugGeom->addPrimitiveSet(prim);
                        }
#endif
                        if (DEBUG_FINGER_MOVEMENT)
                            std::cout << " [" << intersectionEntry << "-" << intersectionExit << "]" << std::endl;

                        if (intersectionExit <= curSqueeze) {
                            // curSqueeze after intersection
                            clearanceRatio[0] = intersectionExit;
                        } else if (intersectionEntry < curSqueeze) {
                            // curSqueeze inside intersection
                            float downward = intersectionExit - curSqueeze;
                            float upward = curSqueeze - intersectionEntry;
                            static const float maxFingerJump = 0.2f;
                            if (intersectionExit < 1.0f && downward <= upward) {
                                clearanceRatio[0] = intersectionExit;
                                // Don't jump too far
                                if (downward < maxFingerJump) {
                                    // push down
                                    curSqueeze = intersectionExit;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  down out of intersection" << std::endl;
                                } else {
                                    // Stuck!
                                    clearanceRatio[1] = intersectionEntry;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  stuck in intersection (down)" << std::endl;
                                    break;
                                }
                            } else if (intersectionEntry > 0.0f){
                                clearanceRatio[1] = intersectionEntry;
                                // Don't jump too far
                                if (upward < maxFingerJump) {
                                    // push up
                                    curSqueeze = intersectionEntry;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  up out of intersection" << std::endl;
                                    // no further intersections are relevant
                                    break;
                                } else {
                                    // Stuck!
                                    clearanceRatio[0] = intersectionExit;
                                    if (DEBUG_FINGER_MOVEMENT)
                                        std::cout << "  stuck in intersection (up)" << std::endl;
                                    break;
                                }
                            }
                        } else if (clearanceRatio[1] >= 1.0f) {
                            clearanceRatio[1] = intersectionEntry;
                        }
                    }
                    if (clearanceRatio[0] > clearanceRatio[1]) {
                        // If stuck, target current squeeze (don't change it)
                        targetSqueeze = curSqueeze;
                        overriding = true;
                    } else if (clearanceRatio[1] < 1.0f) {
                        // push down if there's anything to touch
                        targetSqueeze = clearanceRatio[1];
                        overriding = true;
                    } else if (targetSqueeze < clearanceRatio[0]) {
                        // don't target upwards beyond clearance
                        targetSqueeze = clearanceRatio[0];
                        overriding = true;
                    } else if (targetSqueeze > clearanceRatio[1]) {
                        // don't target downwards beyond clearance
                        targetSqueeze = clearanceRatio[1];
                        overriding = true;
                    }
#ifdef DRAW_SQUEEZE_INTERSECTIONS
                    if (vertices->size() > primStart)  {
                        osg::DrawArrays* primPoints = new osg::DrawArrays(osg::PrimitiveSet::POINTS,
                                                                    primStart,
                                                                    vertices->size() - primStart);
                        debugGeom->addPrimitiveSet(primPoints);
                    }
#endif
                }
            }

            float fingerSpeed = 2.0f;
            // still move fingers if paused
            if (dt == 0.0f)
                dt = 0.01f;
            if (overriding)
                _fingersOverriding[finger] = true;
            if (!_fingersOverriding[finger])
                fingerSpeed = 10.0f;
            if (curSqueeze < targetSqueeze) {
                curSqueeze += dt*fingerSpeed;
                if (curSqueeze > targetSqueeze) {
                    curSqueeze = targetSqueeze;
                    if (!overriding)
                        _fingersOverriding[finger] = false;
                }
            } else if (curSqueeze > targetSqueeze) {
                curSqueeze -= dt*fingerSpeed;
                if (curSqueeze < targetSqueeze) {
                    curSqueeze = targetSqueeze;
                    if (!overriding)
                        _fingersOverriding[finger] = false;
                }
            }
            _squeeze.setFingerSqueeze((osgXR::HandPose::Finger)finger, curSqueeze);
            jointMask |= fingerMasks[finger];
        }
        if (jointMask)
            setPose(_squeeze, _ranges, &dim, jointMask);
    }
}
